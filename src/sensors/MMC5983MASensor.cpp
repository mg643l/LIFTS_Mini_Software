#include "sensors/MMC5983MASensor.hpp"
#include <stdio.h>

MMC5983MASensor::MMC5983MASensor(uint cs_pin)
    : cs_pin_(cs_pin) {}

bool MMC5983MASensor::init() {
    // Perform software reset
    writeRegister(REG_MMC5983MA_CONTROL_1, CTRL1_SW_RST);
    sleep_ms(15); // Allow time for reset sequence

    if (!powerUpCheck()) {
        return false;
    }

    // Apply high-current SET pulse to restore initial magnetization state
    writeRegister(REG_MMC5983MA_CONTROL_0, CTRL0_SET);
    sleep_ms(2);

    return true;
}

uint8_t MMC5983MASensor::readRegister(uint8_t reg_addr) {
    // MMC5983MA uses SPI Read Bit (0x80) on the command byte.
    uint8_t tx[2] = {
        static_cast<uint8_t>((reg_addr & 0x3F) | MMC5983MA_SPI_READ_BIT),
        0x00
    };
    uint8_t rx[2] = {0};

    csSelect();
    spi_write_read_blocking(spi0, tx, rx, 2);
    csDeselect();

    return rx[1];
}

void MMC5983MASensor::writeRegister(uint8_t reg_addr, uint8_t data) {
    // Write bit is indicated by clearing MSB
    uint8_t tx[2] = {
        static_cast<uint8_t>(reg_addr & 0x3F),
        data
    };
    uint8_t rx[2] = {0};

    csSelect();
    spi_write_read_blocking(spi0, tx, rx, 2);
    csDeselect();
}

bool MMC5983MASensor::powerUpCheck() {
    uint8_t prod_id = readRegister(REG_MMC5983MA_PRODUCT_ID);
    return (prod_id == ExpectedProductId);
}

void MMC5983MASensor::configure(ODR odr) {
    // 1. Set measurement bandwidth filter (BW0=0, BW1=0 yields 100Hz bandwidth / 8ms measurement duration)
    writeRegister(REG_MMC5983MA_CONTROL_1, CTRL1_BW_100HZ);

    // 2. Enable automatic Set/Reset internally and periodic set
    writeRegister(REG_MMC5983MA_CONTROL_0, CTRL0_AUTO_SR_EN);

    // 3. Configure Continuous Measurement Mode ODR & Enable CMM & Periodic Set
    uint8_t ctrl2 = (odr & 0x07) | CTRL2_CMM_EN | CTRL2_EN_PRD_SET;
    writeRegister(REG_MMC5983MA_CONTROL_2, ctrl2);
}

bool MMC5983MASensor::readData(Data &out_data) {
    // Multi-byte burst read starting from 0x00 through 0x07:
    // [0-1]: Xout, [2-3]: Yout, [4-5]: Zout, [6]: XYZout2 (18-bit LSBs), [7]: Tout
    uint8_t tx[9] = {
        static_cast<uint8_t>((REG_MMC5983MA_XOUT_0 & 0x3F) | MMC5983MA_SPI_READ_BIT)
    };
    uint8_t rx[9] = {0};

    csSelect();
    spi_write_read_blocking(spi0, tx, rx, 9);
    csDeselect();

    // Parse 18-bit raw unsigned integers:
    // X [17..10] = rx[1], X [9..2] = rx[2], X [1..0] = rx[7][7..6]
    uint32_t raw_x = (static_cast<uint32_t>(rx[1]) << 10) | 
                     (static_cast<uint32_t>(rx[2]) << 2)  | 
                     ((rx[7] >> 6) & 0x03);

    uint32_t raw_y = (static_cast<uint32_t>(rx[3]) << 10) | 
                     (static_cast<uint32_t>(rx[4]) << 2)  | 
                     ((rx[7] >> 4) & 0x03);

    uint32_t raw_z = (static_cast<uint32_t>(rx[5]) << 10) | 
                     (static_cast<uint32_t>(rx[6]) << 2)  | 
                     ((rx[7] >> 2) & 0x03);

    // Convert raw 18-bit unsigned values to Gauss (Offset: 2^17 = 131072, Sensitivity: 16384 LSB/Gauss)
    out_data.mag_x_g = (static_cast<float>(raw_x) - 131072.0f) / 16384.0f;
    out_data.mag_y_g = (static_cast<float>(raw_y) - 131072.0f) / 16384.0f;
    out_data.mag_z_g = (static_cast<float>(raw_z) - 131072.0f) / 16384.0f;

    return true;
}