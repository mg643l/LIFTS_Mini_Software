#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// Pin definitions
constexpr uint PIN_MISO = 20;
constexpr uint PIN_MOSI = 19;
constexpr uint PIN_SCK = 22;
constexpr uint PIN_BMP585_CS = 11;

// BMP585 Register map
constexpr uint8_t REG_CHIP_ID = 0x01;
constexpr uint8_t SPI_READ_BIT = 0x80;

int main()
{
    stdio_init_all();

    // Initialise SPI0 at 500 kHz
    spi_init(spi0, 500 * 1000);
    // Set pin functions for SPI
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Initialise BMP585 CS pin
    gpio_init(PIN_BMP585_CS);
    gpio_set_dir(PIN_BMP585_CS, GPIO_OUT);
    gpio_put(PIN_BMP585_CS, 1);

    // Send a dummy byte to set BMP585 to SPI mode
    uint8_t dummy_tx[2] = {0x00, 0x00};
    uint8_t dummy_rx[2];

    gpio_put(PIN_BMP585_CS, 0);
    spi_write_blocking(spi0, dummy_tx, 2);
    gpio_put(PIN_BMP585_CS, 1);

    // Read the chip ID from BMP585
    uint8_t tx[2] = {REG_CHIP_ID | SPI_READ_BIT, 0x00};
    uint8_t rx[2] = {0};

    gpio_put(PIN_BMP585_CS, 0);
    spi_write_blocking(spi0, tx, 1);
    spi_read_blocking(spi0, 0x00, rx, 1);
    gpio_put(PIN_BMP585_CS, 1);

    // Print the chip ID in an infinite loop
    while (true) {
        printf("Chip ID: 0x%02X\n", rx[0]);
        sleep_ms(1000);
    }
}