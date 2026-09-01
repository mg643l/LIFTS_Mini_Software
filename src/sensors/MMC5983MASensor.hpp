#pragma once

#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"

// MMC5983MA magnetometer sensor driver interface.
class MMC5983MASensor {
public:
    // Contains the magnetic field values calculated from the sensor output.
    struct Data {
        float mag_x_g;
        float mag_y_g;
        float mag_z_g;
    };

    // Output data rates supported in continuous measurement mode.
    enum ODR : uint8_t {
        ODR_OFF    = 0x00,
        ODR_1_HZ   = 0x01,
        ODR_10_HZ  = 0x02,
        ODR_20_HZ  = 0x03,
        ODR_50_HZ  = 0x04,
        ODR_100_HZ = 0x05,
        ODR_200_HZ = 0x06,
        ODR_1000_HZ = 0x07
    };

    // Creates a sensor interface using the specified chip-select GPIO.
    explicit MMC5983MASensor(uint cs_pin);

    // Initialises the sensor and verifies that it is responding correctly.
    bool init();

    // Reads a single byte from the specified sensor register.
    uint8_t readRegister(uint8_t reg_addr);

    // Writes a single byte to the specified sensor register.
    void writeRegister(uint8_t reg_addr, uint8_t data);

    // Verifies the sensor's product ID and power-up state.
    bool powerUpCheck();

    // Configures the magnetometer for continuous sampling.
    void configure(ODR odr = ODR_50_HZ);

    // Reads the latest X, Y, and Z magnetic field values.
    bool readData(Data &out_data);

private:
    // Register mapping for the MMC5983MA magnetometer.
    static constexpr uint8_t REG_MMC5983MA_XOUT_0     = 0x00;
    static constexpr uint8_t REG_MMC5983MA_TOUT       = 0x07;
    static constexpr uint8_t REG_MMC5983MA_STATUS     = 0x08;
    static constexpr uint8_t REG_MMC5983MA_CONTROL_0  = 0x09;
    static constexpr uint8_t REG_MMC5983MA_CONTROL_1  = 0x0A;
    static constexpr uint8_t REG_MMC5983MA_CONTROL_2  = 0x0B;
    static constexpr uint8_t REG_MMC5983MA_PRODUCT_ID = 0x2F;

    // Bit masks and constants used for SPI transactions and configuration.
    static constexpr uint8_t MMC5983MA_SPI_READ_BIT = 0x80;
    static constexpr uint8_t ExpectedProductId      = 0x30;

    // Control bits for the MMC5983MA configuration registers.
    static constexpr uint8_t CTRL0_TM_MAG    = (1 << 0);
    static constexpr uint8_t CTRL0_TM_TEMP   = (1 << 1);
    static constexpr uint8_t CTRL0_SET       = (1 << 3);
    static constexpr uint8_t CTRL0_RESET     = (1 << 4);
    static constexpr uint8_t CTRL0_AUTO_SR_EN = (1 << 5);

    static constexpr uint8_t CTRL1_BW_100HZ = 0x00;
    static constexpr uint8_t CTRL1_SW_RST   = (1 << 7);

    static constexpr uint8_t CTRL2_CMM_EN    = (1 << 3);
    static constexpr uint8_t CTRL2_EN_PRD_SET = (1 << 7);

    static constexpr uint8_t STATUS_M_DONE = (1 << 0);
    static constexpr uint8_t STATUS_T_DONE = (1 << 1);

    // GPIO used to select this sensor on the shared SPI bus.
    uint cs_pin_;

    // Sets the sensor's chip-select line before an SPI transaction.
    inline void csSelect() const {
        asm volatile("nop \n nop \n nop");
        gpio_put(cs_pin_, 0);
        asm volatile("nop \n nop \n nop");
    }

    // Clears the sensor's chip-select line after an SPI transaction.
    inline void csDeselect() const {
        asm volatile("nop \n nop \n nop");
        gpio_put(cs_pin_, 1);
        asm volatile("nop \n nop \n nop");
    }
};