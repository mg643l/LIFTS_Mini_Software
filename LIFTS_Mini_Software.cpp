#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// Pin definitions
constexpr uint PIN_MISO = 20;
constexpr uint PIN_MOSI = 19;
constexpr uint PIN_SCK = 22;
constexpr uint PIN_BMP585_CS = 11;
constexpr uint PIN_LSM6DSO32TR_CS = 3;
constexpr uint PIN_MMC5983MA_CS = 5;
constexpr uint PIN_W25Q128JVSIQ_CS = 17;

// BMP585 Register map
constexpr uint8_t REG_BMP585_CHIP_ID = 0x01;
constexpr uint8_t REG_BMP585_REV_ID = 0x02;
constexpr uint8_t REG_BMP585_INT_STATUS = 0x27;
constexpr uint8_t REG_BMP585_STATUS = 0x28;

// Bitwise ORed with the register address 
constexpr uint8_t BMP585_SPI_READ_BIT = 0x80;

// BMP585 Register bit masks
namespace BMP585 {
    constexpr uint8_t ChipIdExpected   = 0x51;
    constexpr uint8_t RevIdExpected    = 0x32;

    constexpr uint8_t StatusNvmRdyMask  = (1 << 1);
    constexpr uint8_t StatusNvmErrMask  = (1 << 2);
    constexpr uint8_t IntStatusPorMask  = (1 << 4);
}


void Init();
void BMP585_Init();
uint8_t BMP585_ReadRegister(uint8_t Reg_Addr);
bool BMP585_Power_Up_Check();
static inline void CS_Select(uint cs_pin);
static inline void CS_Deselect(uint cs_pin);

int main()
{
    Init();

    sleep_ms(5000);

    BMP585_Init();

    // Continuously read and print chip ID (Expected: 0x51) and revision ID (Expected: 0x32)
    while (true) {
        uint8_t chip_id = BMP585_ReadRegister(REG_BMP585_CHIP_ID);
        printf("Chip ID: 0x%02X\n", chip_id);
        sleep_ms(1000);
        uint8_t rev_id = BMP585_ReadRegister(REG_BMP585_REV_ID);
        printf("Revision ID: 0x%02X\n", rev_id);
        sleep_ms(1000);
    }

    return 0;
}

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

    // Initialise LSM6DSO32TR CS pin
    gpio_init(PIN_LSM6DSO32TR_CS);
    gpio_set_dir(PIN_LSM6DSO32TR_CS, GPIO_OUT);
    gpio_put(PIN_LSM6DSO32TR_CS, 1);

    // Initialise MMC5983MA CS pin
    gpio_init(PIN_MMC5983MA_CS);
    gpio_set_dir(PIN_MMC5983MA_CS, GPIO_OUT);
    gpio_put(PIN_MMC5983MA_CS, 1);

    // Initialise W25Q128JVSIQ CS pin
    gpio_init(PIN_W25Q128JVSIQ_CS);
    gpio_set_dir(PIN_W25Q128JVSIQ_CS, GPIO_OUT);
    gpio_put(PIN_W25Q128JVSIQ_CS, 1);
}

void BMP585_Init() {
    // Send a dummy byte on CS toggle to switch BMP585 interface to SPI mode
    uint8_t dummy_tx[2] = {0x00, 0x00};
    uint8_t dummy_rx[2] = {0};

    // Write dummy bytes to BMP585 for 16 SCK cycles
    CS_Select(PIN_BMP585_CS);
    spi_write_read_blocking(spi0, dummy_tx, dummy_rx, 2);
    CS_Deselect(PIN_BMP585_CS);

    if (!BMP585_Power_Up_Check()) {
        printf("BMP585 power-up check failed!\n");
    } else {
        printf("BMP585 power-up check passed.\n");
    }
}

uint8_t BMP585_ReadRegister(uint8_t Reg_Addr) {
    // 2 Bytes: [0] Reg Addr | Read Bit, [1] Data Output Clocking
    uint8_t tx[2] = {static_cast<uint8_t>(Reg_Addr | BMP585_SPI_READ_BIT), 0x00};
    uint8_t rx[2] = {0};

    // SPI read transaction
    CS_Select(PIN_BMP585_CS);
    spi_write_read_blocking(spi0, tx, rx, 2);
    CS_Deselect(PIN_BMP585_CS);

    // Byte 0 corresponds to TX command garbage, Byte 1 holds register data
    return rx[1];
}

bool BMP585_Power_Up_Check() {
    uint8_t chip_id    = BMP585_ReadRegister(REG_BMP585_CHIP_ID);
    uint8_t rev_id     = BMP585_ReadRegister(REG_BMP585_REV_ID);
    uint8_t int_status = BMP585_ReadRegister(REG_BMP585_INT_STATUS);
    uint8_t status     = BMP585_ReadRegister(REG_BMP585_STATUS);

    bool chip_id_ok = (chip_id == BMP585::ChipIdExpected);
    bool rev_id_ok  = (rev_id  == BMP585::RevIdExpected);
    bool nvm_ok     = (status & BMP585::StatusNvmRdyMask) && !(status & BMP585::StatusNvmErrMask);
    bool por_ok     = (int_status & BMP585::IntStatusPorMask);

    return chip_id_ok && rev_id_ok && nvm_ok && por_ok;
}

// Chip Select control functions with nop for timing adjustment 
static inline void CS_Select(uint cs_pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void CS_Deselect(uint cs_pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop");
}