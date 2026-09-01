#include "BMP585Sensor.hpp"

#include <stdio.h>

BMP585Sensor::BMP585Sensor(uint cs_pin)
    : cs_pin_(cs_pin) {}

bool BMP585Sensor::init() {
    // Perform a dummy SPI transaction to initialise the sensor's SPI interface.
    uint8_t dummy_tx[2] = {0x00, 0x00};
    uint8_t dummy_rx[2] = {0};

    csSelect();
    spi_write_read_blocking(spi0, dummy_tx, dummy_rx, 2);
    csDeselect();

    // Verify that the sensor has powered up and is reporting the expected
    // identification and status values.
    if (!powerUpCheck()) {
        // printf("BMP585 power-up check failed!\n");
        return false;
    }

    // printf("BMP585 power-up check passed.\n");
    return true;
}

uint8_t BMP585Sensor::readRegister(uint8_t reg_addr) {
    // Set the MSB to indicate an SPI register read. The second byte clocks
    // the register contents back from the sensor.
    uint8_t tx[2] = {
        static_cast<uint8_t>(reg_addr | BMP585_SPI_READ_BIT),
        0x00
    };
    uint8_t rx[2] = {0};

    csSelect();
    spi_write_read_blocking(spi0, tx, rx, 2);
    csDeselect();

    return rx[1];
}

void BMP585Sensor::writeRegister(uint8_t reg_addr, uint8_t data) {
    // Clear the MSB to indicate an SPI register write.
    uint8_t tx[2] = {
        static_cast<uint8_t>(reg_addr & 0x7F),
        data
    };
    uint8_t rx[2] = {0};

    csSelect();
    spi_write_read_blocking(spi0, tx, rx, 2);
    csDeselect();
}

bool BMP585Sensor::powerUpCheck() {
    // Read the registers required to verify sensor identity and power-up state.
    uint8_t chip_id = readRegister(REG_BMP585_CHIP_ID);
    uint8_t rev_id = readRegister(REG_BMP585_REV_ID);
    uint8_t int_status = readRegister(REG_BMP585_INT_STATUS);
    uint8_t status = readRegister(REG_BMP585_STATUS);

    // Verify that the connected device is the expected BMP585 revision.
    bool chip_id_ok = (chip_id == ChipIdExpected);
    bool rev_id_ok = (rev_id == RevIdExpected);

    // Confirm that NVM is ready and that no NVM error has been reported.
    bool nvm_ok = (status & StatusNvmRdyMask) &&
                 !(status & StatusNvmErrMask);

    // Confirm that the power-on-reset event has occurred.
    bool por_ok = (int_status & IntStatusPorMask);

    return chip_id_ok && rev_id_ok && nvm_ok && por_ok;
}

void BMP585Sensor::configure() {
    // Place the sensor in standby before changing its configuration.
    writeRegister(REG_BMP585_ODR_CONFIG, MODE_STANDBY);
    sleep_ms(3);

    // Enable pressure measurements and configure the pressure and temperature
    // oversampling ratios.
    uint8_t press_en = 1;
    uint8_t osr_p = OSR_16X;
    uint8_t osr_t = OSR_2X;
    uint8_t osr_config = (press_en << 6) | (osr_p << 3) | (osr_t << 0);
    writeRegister(REG_BMP585_OSR_CONFIG, osr_config);

    // Configure the digital IIR filters for pressure and temperature data.
    uint8_t iir_p = IIR_COEFF_3;
    uint8_t iir_t = IIR_COEFF_1;
    uint8_t iir_config = (iir_p << 3) | (iir_t << 0);
    writeRegister(REG_BMP585_DSP_IIR, iir_config);

    // Disable deep-sleep mode, select the 50 Hz output data rate, and place
    // the sensor into normal measurement mode.
    uint8_t deep_dis = 1;
    uint8_t odr = ODR_50_HZ;
    uint8_t odr_config = (deep_dis << 7) | (odr << 2) | (MODE_NORMAL & 0x03);
    writeRegister(REG_BMP585_ODR_CONFIG, odr_config);

    // Allow the sensor time to enter normal operating mode.
    sleep_ms(5);
}

bool BMP585Sensor::readData(Data &out_data) {
    // Read the three temperature bytes followed by the three pressure bytes
    // in a single SPI transaction.
    uint8_t tx[7] = {
        static_cast<uint8_t>(REG_BMP585_DATA_START | BMP585_SPI_READ_BIT)
    };
    uint8_t rx[7] = {0};

    csSelect();
    spi_write_read_blocking(spi0, tx, rx, 7);
    csDeselect();

    // Reconstruct the 24-bit little-endian raw temperature value.
    uint32_t raw_temp = (static_cast<uint32_t>(rx[3]) << 16) |
                        (static_cast<uint32_t>(rx[2]) << 8) |
                        static_cast<uint32_t>(rx[1]);

    // Reconstruct the 24-bit little-endian raw pressure value.
    uint32_t raw_press = (static_cast<uint32_t>(rx[6]) << 16) |
                         (static_cast<uint32_t>(rx[5]) << 8) |
                         static_cast<uint32_t>(rx[4]);

    // Sign-extend the 24-bit two's-complement temperature value to 32 bits.
    int32_t signed_temp = static_cast<int32_t>(raw_temp);
    if (signed_temp & 0x00800000) {
        signed_temp |= 0xFF000000;
    }

    // Convert the raw sensor values into physical units using the BMP585
    // scaling factors.
    out_data.temperature_c = static_cast<float>(signed_temp) / 65536.0f;
    out_data.pressure_pa = static_cast<float>(raw_press) / 64.0f;

    return true;
};