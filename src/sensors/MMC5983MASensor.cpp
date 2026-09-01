#include "sensors/MMC5983MASensor.hpp"

#include <stdio.h>

#include "pico/time.h"

#define MMC5983MA_DISABLE_AUTO_SR 0

static uint16_t mmc5983ma_prd_set_measurements(uint8_t prd_set_bits) {
    switch (prd_set_bits & 0x07) {
        case 0x00: return 1;
        case 0x01: return 25;
        case 0x02: return 75;
        case 0x03: return 100;
        case 0x04: return 250;
        case 0x05: return 500;
        case 0x06: return 1000;
        case 0x07: return 2000;
        default:   return 0;
    }
}

MMC5983MASensor::MMC5983MASensor(uint cs_pin)
    : cs_pin_(cs_pin) {}

bool MMC5983MASensor::init() {
    // Perform a software reset before checking the sensor identity.
    writeRegister(REG_MMC5983MA_CONTROL_1, CTRL1_SW_RST);
    sleep_ms(15);

    // Verify that the connected device is the expected MMC5983MA part.
    if (!powerUpCheck()) {
        return false;
    }

    // Apply the high-current set pulse to restore the initial magnetisation state.
    writeRegister(REG_MMC5983MA_CONTROL_0, CTRL0_SET);
    sleep_ms(2);

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

void MMC5983MASensor::configure(ODR odr, PeriodicSet periodic_set) {
    // Set the measurement bandwidth filter for the selected output data rate.
    writeRegister(REG_MMC5983MA_CONTROL_1, CTRL1_BW_100HZ);

    // Diagnostic-only configuration log: keep the operating mode otherwise
    // unchanged, but allow isolating the AUTO_SR coil pulsing path.
    uint8_t ctrl0 = CTRL0_AUTO_SR_EN;
#if MMC5983MA_DISABLE_AUTO_SR
    ctrl0 = 0x00;
#endif

    // Configure continuous measurement mode with the requested ODR and a
    // periodic SET interval that avoids firing on every sample.
    uint8_t ctrl2 = (odr & 0x07) |
                    (static_cast<uint8_t>(periodic_set) << 4) |
                    CTRL2_CMM_EN |
                    CTRL2_EN_PRD_SET;
    uint8_t prd_set_bits = (ctrl2 >> 4) & 0x07;
    uint16_t measurements_between_sets = mmc5983ma_prd_set_measurements(prd_set_bits);

    printf("MMC5983MA configure: write CTRL0[0x09]=0x%02X, CTRL2[0x0B]=0x%02X, Prd_set[2:0]=0x%X (%u measurements between SETs)\n",
           ctrl0,
           ctrl2,
           prd_set_bits,
           measurements_between_sets);

    writeRegister(REG_MMC5983MA_CONTROL_0, ctrl0);
    writeRegister(REG_MMC5983MA_CONTROL_2, ctrl2);
}

bool MMC5983MASensor::readData(Data &out_data) {
    // The MMC5983MA reports a new sample only after the measurement-complete
    // flag in STATUS is set. If we read data too early, the sensor keeps
    // returning the same stale sample forever.
    uint8_t status = 0;
    for (int attempt = 0; attempt < 20; ++attempt) {
        status = readRegister(REG_MMC5983MA_STATUS);
        if (status & STATUS_M_DONE) {
            break;
        }
        sleep_ms(1);
    }

    if (!(status & STATUS_M_DONE)) {
        printf("MMC5983MA not ready first pass: status=0x%02X\n", status);

        // The sensor can occasionally get stuck in a stale state after a long
        // run. Re-arming the set/reset pulse gives it a fresh measurement cycle.
        writeRegister(REG_MMC5983MA_CONTROL_0, CTRL0_SET);
        sleep_ms(2);

        for (int attempt = 0; attempt < 20; ++attempt) {
            status = readRegister(REG_MMC5983MA_STATUS);
            if (status & STATUS_M_DONE) {
                break;
            }
            sleep_ms(1);
        }

        if (!(status & STATUS_M_DONE)) {
            printf("MMC5983MA still not ready after rearm: status=0x%02X, elapsed_ms=%lu\n",
                   status,
                   to_ms_since_boot(get_absolute_time()));
            return false;
        }
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

    return true;
}