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

// BMP585 Registers
constexpr uint8_t REG_BMP585_CHIP_ID     = 0x01;
constexpr uint8_t REG_BMP585_REV_ID      = 0x02;
constexpr uint8_t REG_BMP585_INT_STATUS  = 0x27;
constexpr uint8_t REG_BMP585_STATUS      = 0x28;
constexpr uint8_t REG_BMP585_DSP_IIR     = 0x31;
constexpr uint8_t REG_BMP585_OSR_CONFIG  = 0x36;
constexpr uint8_t REG_BMP585_ODR_CONFIG  = 0x37;

constexpr uint8_t BMP585_SPI_READ_BIT    = 0x80;

namespace BMP585 {
    constexpr uint8_t ChipIdExpected    = 0x51;
    constexpr uint8_t RevIdExpected     = 0x32;
    constexpr uint8_t StatusNvmRdyMask  = (1 << 1);
    constexpr uint8_t StatusNvmErrMask  = (1 << 2);
    constexpr uint8_t IntStatusPorMask  = (1 << 4);

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

    enum ODR : uint8_t {
        ODR_240_HZ = 0x00,
        ODR_100_HZ = 0x07,
        ODR_50_HZ  = 0x0A,
        ODR_10_HZ  = 0x10,
        ODR_1_HZ   = 0x15
    };

    enum PowerMode : uint8_t {
        MODE_STANDBY    = 0x00,
        MODE_NORMAL     = 0x01,
        MODE_FORCED     = 0x02,
        MODE_CONTINUOUS = 0x03
    };

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
}

void Init();
void BMP585_Init();
uint8_t BMP585_ReadRegister(uint8_t Reg_Addr);
void BMP585_WriteRegister(uint8_t Reg_Addr, uint8_t Data);
bool BMP585_Power_Up_Check();
void BMP585_Configure();

static inline void CS_Select(uint cs_pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void CS_Deselect(uint cs_pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop");
}

int main() {
    Init();
    sleep_ms(5000);

    BMP585_Init();
    BMP585_Configure();

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

void Init() {
    stdio_init_all();

    spi_init(spi0, 500 * 1000);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    uint cs_pins[] = {PIN_BMP585_CS, PIN_LSM6DSO32TR_CS, PIN_MMC5983MA_CS, PIN_W25Q128JVSIQ_CS};
    for (uint pin : cs_pins) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 1);
    }
}

void BMP585_Init() {
    uint8_t dummy_tx[2] = {0x00, 0x00};
    uint8_t dummy_rx[2] = {0};

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
    uint8_t tx[2] = {static_cast<uint8_t>(Reg_Addr | BMP585_SPI_READ_BIT), 0x00};
    uint8_t rx[2] = {0};

    CS_Select(PIN_BMP585_CS);
    spi_write_read_blocking(spi0, tx, rx, 2);
    CS_Deselect(PIN_BMP585_CS);

    return rx[1];
}

void BMP585_WriteRegister(uint8_t Reg_Addr, uint8_t Data) {
    uint8_t tx[2] = {static_cast<uint8_t>(Reg_Addr & 0x7F), Data};
    uint8_t rx[2] = {0};

    CS_Select(PIN_BMP585_CS);
    spi_write_read_blocking(spi0, tx, rx, 2);
    CS_Deselect(PIN_BMP585_CS);
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

void BMP585_Configure() {
    // Force STANDBY mode before editing configuration registers
    BMP585_WriteRegister(REG_BMP585_ODR_CONFIG, BMP585::MODE_STANDBY);
    sleep_ms(3);

    // Configure OSR_CONFIG (0x36): press_en [6], osr_p [5:3], osr_t [2:0]
    uint8_t press_en = 1;
    uint8_t osr_p = BMP585::OSR_16X;
    uint8_t osr_t = BMP585::OSR_2X;
    uint8_t osr_config = (press_en << 6) | (osr_p << 3) | (osr_t << 0);
    BMP585_WriteRegister(REG_BMP585_OSR_CONFIG, osr_config);

    // Configure DSP_IIR (0x31): set_iir_p [5:3], set_iir_t [2:0]
    uint8_t iir_p = BMP585::IIR_COEFF_3;
    uint8_t iir_t = BMP585::IIR_COEFF_1;
    uint8_t iir_config = (iir_p << 3) | (iir_t << 0);
    BMP585_WriteRegister(REG_BMP585_DSP_IIR, iir_config);

    // Configure ODR_CONFIG (0x37): deep_dis [7], odr [6:2], pwr_mode [1:0]
    // Transitions device directly into NORMAL (continuous sampling) mode
    uint8_t deep_dis = 1;
    uint8_t odr = BMP585::ODR_50_HZ;
    uint8_t odr_config = (deep_dis << 7) | (odr << 2) | (BMP585::MODE_NORMAL & 0x03);
    BMP585_WriteRegister(REG_BMP585_ODR_CONFIG, odr_config);

    sleep_ms(5); // Allow power state transition to complete
}