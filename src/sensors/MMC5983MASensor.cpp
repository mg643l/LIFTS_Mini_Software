#include "sensors/MMC5983MASensor.hpp"

#include <stdio.h>

MMC5983MASensor::MMC5983MASensor(uint cs_pin)
    : cs_pin_(cs_pin) {}

bool MMC5983MASensor::init() {
    // Perform a software reset before checking the sensor identity.
    writeRegister(REG_MMC5983MA_CONTROL_1, CTRL1_SW_RST);
    sleep_ms(MMC5983_RESET_DELAY_MS);

    // Verify that the connected device is the expected MMC5983MA part.
    if (!powerUpCheck()) {
        return false;
    }

    // Apply the high-current set pulse to restore the initial magnetisation state.
    writeRegister(REG_MMC5983MA_CONTROL_0, CTRL0_SET);
    sleep_ms(2);

    // Ensure the SPI bus is configured in the expected framing mode.
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    return true;
}

uint8_t MMC5983MASensor::readRegister(uint8_t reg_addr) {
    // MMC5983MA uses the SPI read bit on the command byte.
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
    // The write operation is indicated by clearing the MSB of the command byte.
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
    // Read the device product ID to confirm that the expected sensor is attached.
    uint8_t prod_id = readRegister(REG_MMC5983MA_PRODUCT_ID);
    return (prod_id == ExpectedProductId);
}

void MMC5983MASensor::applyConfiguration(ODR odr) {
    // Set the measurement bandwidth filter for the selected output data rate.
    writeRegister(REG_MMC5983MA_CONTROL_1, CTRL1_BW_100HZ);

    // Re-apply the magnetic reset and auto set-reset configuration for a stable
    // continuous measurement state after a power-up or software reset.
    writeRegister(REG_MMC5983MA_CONTROL_0, CTRL0_SET | CTRL0_AUTO_SR_EN);
    sleep_ms(2);

    // Configure continuous measurement mode with the requested ODR.
    uint8_t ctrl2 = (odr & 0x07) | CTRL2_CMM_EN | CTRL2_EN_PRD_SET;
    writeRegister(REG_MMC5983MA_CONTROL_2, ctrl2);

    // Restore the bus mode explicitly after any reset event.
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
}

void MMC5983MASensor::recoverFromTimeout(ODR odr) {
    // Release the shared SPI bus before resetting the sensor to keep the BMP585
    // responsive to the other device on the same bus.
    csDeselect();

    // Trigger a full software reset on the magnetometer.
    writeRegister(REG_MMC5983MA_CONTROL_1, CTRL1_SW_RST);
    sleep_ms(MMC5983_RESET_DELAY_MS);

    // Re-apply the sensor configuration after the reset.
    applyConfiguration(odr);
}

MMC5983MASensor::Status MMC5983MASensor::waitForMeasurementReady() {
    uint32_t start_ms = to_ms_since_boot();

    while ((to_ms_since_boot() - start_ms) < MMC5983_POLL_TIMEOUT_MS) {
        uint8_t status = readRegister(REG_MMC5983MA_STATUS);
        if ((status & STATUS_MEAS_M_DONE) != 0U) {
            return MMC5983_OK;
        }
        sleep_us(100);
    }

    return MMC5983_ERROR_TIMEOUT;
}

void MMC5983MASensor::configure(ODR odr) {
    applyConfiguration(odr);
}

MMC5983MASensor::Status MMC5983MASensor::readData(Data &out_data) {
    Status ready_status = waitForMeasurementReady();
    if (ready_status != MMC5983_OK) {
        // Guarantee the device is released from the shared SPI bus before the
        // software reset sequence is attempted.
        gpio_put(cs_pin_, 1);
        recoverFromTimeout(ODR_50_HZ);
        return ready_status;
    }

    // Burst-read the X, Y, Z, and temperature registers starting at 0x00.
    // [0-1]: Xout, [2-3]: Yout, [4-5]: Zout, [6]: XYZout2, [7]: Tout
    uint8_t tx[9] = {
        static_cast<uint8_t>((REG_MMC5983MA_XOUT_0 & 0x3F) | MMC5983MA_SPI_READ_BIT)
    };
    uint8_t rx[9] = {0};

    csSelect();
    spi_write_read_blocking(spi0, tx, rx, 9);
    csDeselect();

    // Parse the 18-bit raw values from the returned data bytes.
    uint32_t raw_x = (static_cast<uint32_t>(rx[1]) << 10) |
                     (static_cast<uint32_t>(rx[2]) << 2) |
                     ((rx[7] >> 6) & 0x03);

    uint32_t raw_y = (static_cast<uint32_t>(rx[3]) << 10) |
                     (static_cast<uint32_t>(rx[4]) << 2) |
                     ((rx[7] >> 4) & 0x03);

    uint32_t raw_z = (static_cast<uint32_t>(rx[5]) << 10) |
                     (static_cast<uint32_t>(rx[6]) << 2) |
                     ((rx[7] >> 2) & 0x03);

    // Convert the raw 18-bit values to Gauss using the MMC5983MA offset and
    // sensitivity values.
    out_data.mag_x_g = (static_cast<float>(raw_x) - 131072.0f) / 16384.0f;
    out_data.mag_y_g = (static_cast<float>(raw_y) - 131072.0f) / 16384.0f;
    out_data.mag_z_g = (static_cast<float>(raw_z) - 131072.0f) / 16384.0f;

    return MMC5983_OK;
}