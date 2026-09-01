#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"

#include "sensors/BMP585Sensor.hpp"
#include "sensors/MMC5983MASensor.hpp"

// SPI bus pin assignments.
constexpr uint PIN_MISO = 20;
constexpr uint PIN_MOSI = 19;
constexpr uint PIN_SCK = 22;

// Chip-select pins for devices connected to the SPI bus.
constexpr uint PIN_BMP585_CS = 11;
constexpr uint PIN_LSM6DSO32TR_CS = 3;
constexpr uint PIN_MMC5983MA_CS = 5;
constexpr uint PIN_W25Q128JVSIQ_CS = 17;

void init();

int main() {
    // Initialise the standard I/O and SPI peripherals.
    init();

    // Create and initialise the BMP585 pressure sensor.
    BMP585Sensor bmp585(PIN_BMP585_CS);

    if (!bmp585.init()) {
        printf("Sensor initialisation failed.\n");
        while (true) tight_loop_contents();
    }

    // Create and initialise the MMC5983MA magnetometer sensor.
    MMC5983MASensor mmc5983(PIN_MMC5983MA_CS);

    if (!mmc5983.init()) {
        printf("MMC5983MA power-up check failed!\n");
        while (true) tight_loop_contents();
    }

    // Configure the sensor's measurement settings.
    bmp585.configure();

    BMP585Sensor::Data bmp585_sensor_data;

    // Configure for 50 Hz continuous mode with automatic set/reset enabled.
    mmc5983.configure(MMC5983MASensor::ODR_50_HZ);

    MMC5983MASensor::Data mmc5983ma_sensor_data;

    while (true) {
        // Read the latest temperature and pressure measurements.
        if (bmp585.readData(bmp585_sensor_data)) {
            printf("Temp: %.2f degC | Pressure: %.2f hPa\n",
                   bmp585_sensor_data.temperature_c,
                   bmp585_sensor_data.pressure_pa / 100.0f);
        }

        // Read the latest magnetic field values from the MMC5983MA.
        if (mmc5983.readData(mmc5983ma_sensor_data)) {
            printf("Mag X: %7.3f G | Y: %7.3f G | Z: %7.3f G\n",
                   mmc5983ma_sensor_data.mag_x_g,
                   mmc5983ma_sensor_data.mag_y_g,
                   mmc5983ma_sensor_data.mag_z_g);
        }

        // Wait for 100 ms before taking the next measurement.
        sleep_ms(100);
    }

    return 0;
}

// Initialises standard I/O, the SPI bus, and chip-select GPIOs.
void init() {
    stdio_init_all();

    // Configure SPI0 for communication with the sensors at 500 kHz.
    spi_init(spi0, 500 * 1000);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Set all SPI device chip-select lines inactive.
    uint cs_pins[] = {
        PIN_BMP585_CS,
        PIN_LSM6DSO32TR_CS,
        PIN_MMC5983MA_CS,
        PIN_W25Q128JVSIQ_CS
    };

    for (uint pin : cs_pins) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 1);
    }
}