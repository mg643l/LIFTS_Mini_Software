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

void Init();

class BMP585Sensor {
public:
    struct Data {
        float temperature_c;
        float pressure_pa;
    };

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

    explicit BMP585Sensor(uint cs_pin)
        : cs_pin_(cs_pin) {}

    bool init() {
        uint8_t dummy_tx[2] = {0x00, 0x00};
        uint8_t dummy_rx[2] = {0};

        csSelect();
        spi_write_read_blocking(spi0, dummy_tx, dummy_rx, 2);
        csDeselect();

        if (!powerUpCheck()) {
            printf("BMP585 power-up check failed!\n");
            return false;
        }

        printf("BMP585 power-up check passed.\n");
        return true;
    }

    uint8_t readRegister(uint8_t reg_addr) {
        uint8_t tx[2] = {static_cast<uint8_t>(reg_addr | BMP585_SPI_READ_BIT), 0x00};
        uint8_t rx[2] = {0};

        csSelect();
        spi_write_read_blocking(spi0, tx, rx, 2);
        csDeselect();

        return rx[1];
    }

    void writeRegister(uint8_t reg_addr, uint8_t data) {
        uint8_t tx[2] = {static_cast<uint8_t>(reg_addr & 0x7F), data};
        uint8_t rx[2] = {0};

        csSelect();
        spi_write_read_blocking(spi0, tx, rx, 2);
        csDeselect();
    }

    bool powerUpCheck() {
        uint8_t chip_id    = readRegister(REG_BMP585_CHIP_ID);
        uint8_t rev_id     = readRegister(REG_BMP585_REV_ID);
        uint8_t int_status = readRegister(REG_BMP585_INT_STATUS);
        uint8_t status     = readRegister(REG_BMP585_STATUS);

        bool chip_id_ok = (chip_id == ChipIdExpected);
        bool rev_id_ok  = (rev_id  == RevIdExpected);
        bool nvm_ok     = (status & StatusNvmRdyMask) && !(status & StatusNvmErrMask);
        bool por_ok     = (int_status & IntStatusPorMask);

        return chip_id_ok && rev_id_ok && nvm_ok && por_ok;
    }

    void configure() {
        // Force STANDBY mode before editing configuration registers
        writeRegister(REG_BMP585_ODR_CONFIG, MODE_STANDBY);
        sleep_ms(3);

        // Configure OSR_CONFIG (0x36): press_en [6], osr_p [5:3], osr_t [2:0]
        uint8_t press_en = 1;
        uint8_t osr_p = OSR_16X;
        uint8_t osr_t = OSR_2X;
        uint8_t osr_config = (press_en << 6) | (osr_p << 3) | (osr_t << 0);
        writeRegister(REG_BMP585_OSR_CONFIG, osr_config);

        // Configure DSP_IIR (0x31): set_iir_p [5:3], set_iir_t [2:0]
        uint8_t iir_p = IIR_COEFF_3;
        uint8_t iir_t = IIR_COEFF_1;
        uint8_t iir_config = (iir_p << 3) | (iir_t << 0);
        writeRegister(REG_BMP585_DSP_IIR, iir_config);

        // Configure ODR_CONFIG (0x37): deep_dis [7], odr [6:2], pwr_mode [1:0]
        // Transitions device directly into NORMAL (continuous sampling) mode
        uint8_t deep_dis = 1;
        uint8_t odr = ODR_50_HZ;
        uint8_t odr_config = (deep_dis << 7) | (odr << 2) | (MODE_NORMAL & 0x03);
        writeRegister(REG_BMP585_ODR_CONFIG, odr_config);

        sleep_ms(5); // Allow power state transition to complete
    }

    bool readData(Data &out_data) {
        // 1 command byte + 6 data bytes (TEMP_XLSB..PRESS_MSB) = 7 total bytes.
        // BMP585 has NO dummy byte in its SPI read frame so
        // the first data byte arrives immediately after the command byte.
        uint8_t tx[7] = {static_cast<uint8_t>(REG_BMP585_DATA_START | BMP585_SPI_READ_BIT)};
        uint8_t rx[7] = {0};

        csSelect();
        spi_write_read_blocking(spi0, tx, rx, 7);
        csDeselect();

        // rx[0] is the address-echo byte (invalid); real data starts at rx[1].
        uint32_t raw_temp = (static_cast<uint32_t>(rx[3]) << 16) |
                            (static_cast<uint32_t>(rx[2]) << 8)  |
                             static_cast<uint32_t>(rx[1]);

        uint32_t raw_press = (static_cast<uint32_t>(rx[6]) << 16) |
                             (static_cast<uint32_t>(rx[5]) << 8)  |
                              static_cast<uint32_t>(rx[4]);

        int32_t signed_temp = static_cast<int32_t>(raw_temp);
        if (signed_temp & 0x00800000) {
            signed_temp |= 0xFF000000;
        }

        out_data.temperature_c = static_cast<float>(signed_temp) / 65536.0f; // raw / 2^16
        out_data.pressure_pa   = static_cast<float>(raw_press) / 64.0f;     // raw / 2^6

        return true;
    }

private:
    // BMP585 Registers
    static constexpr uint8_t REG_BMP585_CHIP_ID    = 0x01;
    static constexpr uint8_t REG_BMP585_REV_ID     = 0x02;
    static constexpr uint8_t REG_BMP585_INT_STATUS = 0x27;
    static constexpr uint8_t REG_BMP585_STATUS     = 0x28;
    static constexpr uint8_t REG_BMP585_DSP_IIR    = 0x31;
    static constexpr uint8_t REG_BMP585_OSR_CONFIG = 0x36;
    static constexpr uint8_t REG_BMP585_ODR_CONFIG = 0x37;
    static constexpr uint8_t REG_BMP585_DATA_START = 0x1D;

    static constexpr uint8_t BMP585_SPI_READ_BIT   = 0x80;

    // Expected HW Constants & Bitmasks
    static constexpr uint8_t ChipIdExpected   = 0x51;
    static constexpr uint8_t RevIdExpected    = 0x32;
    static constexpr uint8_t StatusNvmRdyMask = (1 << 1);
    static constexpr uint8_t StatusNvmErrMask = (1 << 2);
    static constexpr uint8_t IntStatusPorMask = (1 << 4);

    uint cs_pin_;

    inline void csSelect() const {
        asm volatile("nop \n nop \n nop");
        gpio_put(cs_pin_, 0);
        asm volatile("nop \n nop \n nop");
    }

    inline void csDeselect() const {
        asm volatile("nop \n nop \n nop");
        gpio_put(cs_pin_, 1);
        asm volatile("nop \n nop \n nop");
    }
};

int main() {
    Init();
    sleep_ms(5000);

    BMP585Sensor bmp585(PIN_BMP585_CS);
    if (!bmp585.init()) {
        printf("Sensor initialization failed.\n");
        while (true) tight_loop_contents();
    }

    bmp585.configure();

    BMP585Sensor::Data sensor_data;

    while (true) {
        if (bmp585.readData(sensor_data)) {
            printf("Temp: %.2f degC  |  Pressure: %.2f Pa (%.2f hPa)\n",
                   sensor_data.temperature_c,
                   sensor_data.pressure_pa,
                   sensor_data.pressure_pa / 100.0f);
        }
        sleep_ms(500); // Poll at 50 Hz ODR configuration
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