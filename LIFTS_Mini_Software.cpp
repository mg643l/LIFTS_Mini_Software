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

void Init(){
    // Initialise standard I/O
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
}

void BMP585_Init() {
    // Send a dummy byte on CS toggle to switch BMP585 interface to SPI mode
    uint8_t dummy_tx[2] = {0x00, 0x00};
    uint8_t dummy_rx[2] = {0};

    // Write dummy bytes to BMP585 for 16 SCK cycles
    gpio_put(PIN_BMP585_CS, 0);
    spi_write_read_blocking(spi0, dummy_tx, dummy_rx, 2);
    gpio_put(PIN_BMP585_CS, 1);
}

uint8_t BMP585_ReadRegister(uint8_t Reg_Addr) {
    // 2 Bytes: [0] Reg Addr | Read Bit, [1] Data Output Clocking
    uint8_t tx[2] = {static_cast<uint8_t>(Reg_Addr | SPI_READ_BIT), 0x00};
    uint8_t rx[2] = {0};

    // SPI read transaction
    gpio_put(PIN_BMP585_CS, 0);
    spi_write_read_blocking(spi0, tx, rx, 2);
    gpio_put(PIN_BMP585_CS, 1);

    // Byte 0 corresponds to TX command, Byte 1 holds register data
    return rx[1];
}

int main()
{
    Init();

    BMP585_Init();

    // Continuously read and print chip ID (Expected: 0x51)
    while (true) {
        uint8_t chip_id = BMP585_ReadRegister(REG_CHIP_ID);
        printf("Chip ID: 0x%02X\n", chip_id);
        sleep_ms(1000);
    }

    return 0;
}