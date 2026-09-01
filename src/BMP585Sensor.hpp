#pragma once

#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"

// Provides an interface for communicating with a Bosch BMP585 pressure sensor over SPI.
class BMP585Sensor {
public:
    // Contains the temperature and pressure measurements returned by the sensor.
    struct Data {
        float temperature_c;
        float pressure_pa;
    };

    // Pressure and temperature oversampling ratios supported by the BMP585.
    enum OSR : uint8_t {
        OSR_1X   = 0x0,
        OSR_2X   = 0x1,
        OSR_4X   = 0x2,
        OSR_8X   = 0x3,
        OSR_16X  = 0x4,
        OSR_32X  = 0x5,
        OSR_64X  = 0x6,
        OSR_128X = 0x7
    };

    // Output data rate sensor configurations.
    enum ODR : uint8_t {
        ODR_240_HZ = 0x00,
        ODR_100_HZ = 0x07,
        ODR_50_HZ  = 0x0A,
        ODR_10_HZ  = 0x10,
        ODR_1_HZ   = 0x15
    };

    // Sensor operating modes.
    enum PowerMode : uint8_t {
        MODE_STANDBY    = 0x00,
        MODE_NORMAL     = 0x01,
        MODE_FORCED     = 0x02,
        MODE_CONTINUOUS = 0x03
    };

    // Internal sensor digital IIR filter coefficients.
    enum IIRFilter : uint8_t {
        IIR_BYPASS    = 0x0,
        IIR_COEFF_1   = 0x1,
        IIR_COEFF_3   = 0x2,
        IIR_COEFF_7   = 0x3,
        IIR_COEFF_15  = 0x4,
        IIR_COEFF_31  = 0x5,
        IIR_COEFF_63  = 0x6,
        IIR_COEFF_127 = 0x7
    };

    // Creates a sensor interface using the specified chip-select GPIO.
    explicit BMP585Sensor(uint cs_pin);

    // Initialises the sensor and verifies that it is responding correctly.
    bool init();

    // Reads a single byte from the specified sensor register.
    uint8_t readRegister(uint8_t reg_addr);

    // Writes a single byte to the specified sensor register.
    void writeRegister(uint8_t reg_addr, uint8_t data);

    // Verifies the sensor identity and power-up status.
    bool powerUpCheck();

    // Applies the required sensor measurement configuration.
    void configure();

    // Reads the latest temperature and pressure measurements.
    bool readData(Data &out_data);

private:
    // BMP585 register addresses used by this driver.
    static constexpr uint8_t REG_BMP585_CHIP_ID    = 0x01;
    static constexpr uint8_t REG_BMP585_REV_ID     = 0x02;
    static constexpr uint8_t REG_BMP585_INT_STATUS = 0x27;
    static constexpr uint8_t REG_BMP585_STATUS     = 0x28;
    static constexpr uint8_t REG_BMP585_DSP_IIR    = 0x31;
    static constexpr uint8_t REG_BMP585_OSR_CONFIG = 0x36;
    static constexpr uint8_t REG_BMP585_ODR_CONFIG = 0x37;
    static constexpr uint8_t REG_BMP585_DATA_START = 0x1D;

    // Bit set in the register address to indicate an SPI read operation.
    static constexpr uint8_t BMP585_SPI_READ_BIT = 0x80;

    // Expected values and status masks used during sensor initialisation.
    static constexpr uint8_t ChipIdExpected   = 0x51;
    static constexpr uint8_t RevIdExpected    = 0x32;
    static constexpr uint8_t StatusNvmRdyMask = (1 << 1);
    static constexpr uint8_t StatusNvmErrMask = (1 << 2);
    static constexpr uint8_t IntStatusPorMask = (1 << 4);

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