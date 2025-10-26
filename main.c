// -----------------------------------------------------------------------------
// IGR, IGDS, DFO/MFO for PlayStation 1
// -----------------------------------------------------------------------------
// IGR  = In-Game Reset
// IGDS = In-Game Disc Switch
// DFO  = Dual Frequency Oscillator
// MFO  = Multiple Frequency Oscillator
// -----------------------------------------------------------------------------
// Pinout:
//  1 -> IGR (reset signal)
//  2 -> IGDS (pulse lid signal)
//  4 <- PlayStation Port 1, Pin 1 (Data (DAT) / RXD)
//  5 <- PlayStation Port 1, Pin 6 (Attention (ATT) / Chip Select (CS) / Slave Select (SS) / DTR)
//  6 <- PlayStation Port 1, Pin 7 (Clock (SCK))
// 12 -> Si5351 SDA
// 13 -> Si5351 SCL
// 14 <- PAL(0) / NTSC(1) select
// -----------------------------------------------------------------------------
// Si5351 CLK mapping:
// * CLK0 : 53.2034 MHz (PAL) / 53.6931 MHz (NTSC)
// * CLK1 : 53.6931 MHz (NTSC)
// * CLK2 : 4.43361875 MHz (PAL) / 3.579545 MHz MHz (NTSC)
// Short GPIO_14 to GND or GPIO_15 => CLK0 fixed on 53.2034 MHz (PAL)
// Short GPIO_11 to GND or GPIO_10 => CLK2 fixed on 4.43361875 MHz (PAL)
// -----------------------------------------------------------------------------



// -----------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "ws2812.h"
// -----------------------------------------------------------------------------
//#define WS2812_PIN 23     // rp2040 black
#define WS2812_PIN 16       // rp2040 zero
// -----------------------------------------------------------------------------



// -----------------------------------------------------------------------------
// IGR specific defines
// -----------------------------------------------------------------------------
#define IGR_ENABLED

// Comment to disable Long Reset function (IGR for XStation / PicoStation)
#define IGR_XSTATION
// Comment to disable Pulse Lid function (IGDS)
#define IGR_PULSE_LID

// Cooldown time after performing IGR
#define IGR_COOLDOWN_TIME 10000
// Cooldown time after performing IGDS
#define IGDS_COOLDOWN_TIME 5000

// Short IGR signal duration in milliseconds
#define SHORT_RESET_DELAY 500
// Long IGR signal duration in milliseconds
#define LONG_RESET_DELAY  2000
// Open Lid signal (IGDS) duration in milliseconds
#define OPEN_LID_DELAY 5000

// Amount of milliseconds to prevent short-term hot-buttons down.
// Comment to enable immediate hot-buttons.
#define HOT_BUTTONS_HOLDING_TIME 500
// -----------------------------------------------------------------------------
// GPIO for Reset signal
#define GPIO_IGR1 1
// GPIO for Open Lid signal
#define GPIO_IGR2 2

// GPIO's for reading gamepad data
#define GPIO_DAT 4 /* [SPI0 RX]  PlayStation Port 1, Pin 1 - Data / RXD */
#define GPIO_ATT 5 /* [SPI0 CS]  PlayStation Port 1, Pin 6 - Attention / Slave Select (SS) / DTR */
#define GPIO_CLK 6 /* [SPI0 CLK] PlayStation Port 1, Pin 7 - Clock / SCK */

// SPI instance mapping
#define SPI_DAT spi0
// -----------------------------------------------------------------------------



// -----------------------------------------------------------------------------
// DFO/MFO specific defines
// -----------------------------------------------------------------------------
#define OSC_ENABLED

// Video Mode input: 0 - PAL, 1 - NTSC
#define GPIO_VIDEO_MODE 14

// GPIO's for Si5351 control via i2c
#define OSC_SDA 12
#define OSC_SCL 13

#define OSC_PRESET_PAL  0
#define OSC_PRESET_NTSC 1
// -----------------------------------------------------------------------------



// -----------------------------------------------------------------------------
// IGR & IGDS specific configuration
// -----------------------------------------------------------------------------
// PlayStation controller buttons masks
#define BTN_SELECT   (1 << 0)
#define BTN_L3       (1 << 1)
#define BTN_R3       (1 << 2)
#define BTN_START    (1 << 3)
#define BTN_UP       (1 << 4)
#define BTN_RIGHT    (1 << 5)
#define BTN_DOWN     (1 << 6)
#define BTN_LEFT     (1 << 7)
#define BTN_L2       (1 << 8)
#define BTN_R2       (1 << 9)
#define BTN_L1       (1 << 10)
#define BTN_R1       (1 << 11)
#define BTN_TRIANGLE (1 << 12)
#define BTN_CIRCLE   (1 << 13)
#define BTN_CROSS    (1 << 14)
#define BTN_SQUARE   (1 << 15)
// -----------------------------------------------------------------------------
struct table_entry
{
    uint16_t buttons;
    uint32_t gpio;
    uint32_t duration;
    uint32_t cooldown;
};
// -----------------------------------------------------------------------------
struct table_entry hot_buttons_map[] =
{
#if defined(IGR_XSTATION)
    // Long Reset signal
    { BTN_SELECT | BTN_CROSS  | BTN_L1 | BTN_R1, GPIO_IGR1, LONG_RESET_DELAY,  IGR_COOLDOWN_TIME },
    { BTN_SELECT | BTN_CROSS  | BTN_L2 | BTN_R2, GPIO_IGR1, LONG_RESET_DELAY,  IGR_COOLDOWN_TIME },
#endif

    // Short Reset signal
    { BTN_SELECT | BTN_START  | BTN_L1 | BTN_R1, GPIO_IGR1, SHORT_RESET_DELAY, IGR_COOLDOWN_TIME },
    { BTN_SELECT | BTN_START  | BTN_L2 | BTN_R2, GPIO_IGR1, SHORT_RESET_DELAY, IGR_COOLDOWN_TIME },

#if defined(IGR_PULSE_LID)
    // Open Lid signal
    { BTN_SELECT | BTN_CIRCLE | BTN_L1 | BTN_R1, GPIO_IGR2, OPEN_LID_DELAY, IGDS_COOLDOWN_TIME },
    { BTN_SELECT | BTN_CIRCLE | BTN_L2 | BTN_R2, GPIO_IGR2, OPEN_LID_DELAY, IGDS_COOLDOWN_TIME },
#endif
};

const int32_t hot_buttons_count = count_of(hot_buttons_map);
// -----------------------------------------------------------------------------
#define GUNCON_IGR_BUTTONS 0x9FF7 /* 1001 1111 1111 0111 */
// -----------------------------------------------------------------------------
#define PID_DIGITAL     0x41
#define PID_DUAL_ANALOG 0x53
#define PID_GUNCON      0x63
#define PID_ANALOGUE    0x73
// -----------------------------------------------------------------------------
struct __attribute__((aligned(1), packed)) ps1_dat
{
    uint8_t size;
    uint8_t ready;
    union __attribute__((aligned(1), packed))
    {
        uint8_t raw[10];
        struct __attribute__((aligned(1), packed))
        {
            uint8_t  ff;      // Always 0xFF
            uint8_t  pid;     // Peripheral ID (0x41 - Digital, 0x53 - Dual Analog, 0x63 - GunCon, 0x73 - Analogue)
            uint8_t  sid;     // Source ID (for controller = 0x5A)
            uint16_t buttons;
        };
    };
};
// -----------------------------------------------------------------------------



// -----------------------------------------------------------------------------
// helper routines
// -----------------------------------------------------------------------------
bool read_ctrl_data(struct ps1_dat* data);
// -----------------------------------------------------------------------------
void osc_init(uint8_t initial_preset);
void osc_select(uint8_t preset);
// -----------------------------------------------------------------------------
inline uint32_t millis(void)
{
    return (uint32_t)(time_us_64() / 1000);
}
// -----------------------------------------------------------------------------
#define _uart_puts(s) uart_puts(uart0, s);
void _uart_printf(const char* fmt, ...);
// -----------------------------------------------------------------------------





// -----------------------------------------------------------------------------
#if defined(OSC_ENABLED)
// -----------------------------------------------------------------------------
//#define OSC_DEBUG
#define GPIO_BOOTSTRAP_BASE1 15
#define GPIO_BOOTSTRAP_BASE2 10
#define GPIO_LOCK_CLK2  11
// -----------------------------------------------------------------------------
#endif /* OSC_ENABLED */
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
#if defined(IGR_ENABLED)
// -----------------------------------------------------------------------------
//#define IGR_DEBUG
// -----------------------------------------------------------------------------
#endif /* IGR_ENABLED */
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// main routine
// -----------------------------------------------------------------------------
[[noreturn]] void main(void)
{
    // -------------------------------------------------------------------------
    //
    // setup system clock
    //
    set_sys_clock_pll(864 * MHZ, 6, 6); // 24 MHz
    //set_sys_clock_pll(750 * MHZ, 7, 7); // ~15 MHz

#if defined(OSC_ENABLED)
    //
    // initialize GPIO - input
    //
    gpio_init(GPIO_VIDEO_MODE);
    gpio_set_dir(GPIO_VIDEO_MODE, GPIO_IN);

    //
    // initialize bootstraps
    //
    gpio_init(GPIO_BOOTSTRAP_BASE1);
    gpio_set_dir(GPIO_BOOTSTRAP_BASE1, GPIO_OUT);
    gpio_put(GPIO_BOOTSTRAP_BASE1, 0);

    gpio_init(GPIO_BOOTSTRAP_BASE2);
    gpio_set_dir(GPIO_BOOTSTRAP_BASE2, GPIO_OUT);
    gpio_put(GPIO_BOOTSTRAP_BASE2, 0);

    gpio_init(GPIO_LOCK_CLK2);
    gpio_set_dir(GPIO_LOCK_CLK2, GPIO_IN);
    gpio_pull_up(GPIO_LOCK_CLK2);

    //
    // initialize i2c channel
    //
    i2c_init(i2c0, 100000);
    gpio_set_function(OSC_SDA, GPIO_FUNC_I2C);
    gpio_set_function(OSC_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(OSC_SDA);
    gpio_pull_up(OSC_SCL);

    //
    // setup initial frequencies
    //
    bool last_mode = gpio_get(GPIO_VIDEO_MODE);
    osc_init((uint8_t)last_mode);
#endif /* OSC_ENABLED */

    // -------------------------------------------------------------------------

#if defined(IGR_ENABLED)
    //
    // init SPI channels
    //
    spi_init(SPI_DAT, 1 * MHZ);
    spi_set_slave(SPI_DAT, 1);
    spi_set_format(SPI_DAT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(GPIO_DAT, GPIO_FUNC_SPI);
    gpio_set_function(GPIO_ATT, GPIO_FUNC_SPI);
    gpio_set_function(GPIO_CLK, GPIO_FUNC_SPI);

    //
    // initialize GPIO - outputs
    //
    gpio_init(GPIO_IGR1);
    gpio_pull_up(GPIO_IGR1);
    gpio_put(GPIO_IGR1, 1);
    gpio_set_dir(GPIO_IGR1, GPIO_IN);

#if defined(IGR_PULSE_LID)
    gpio_init(GPIO_IGR2);
    gpio_pull_up(GPIO_IGR2);
    gpio_put(GPIO_IGR2, 1);
    gpio_set_dir(GPIO_IGR2, GPIO_IN);
#endif /* IGR_PULSE_LID */

#endif /* IGR_ENABLED*/

    // -------------------------------------------------------------------------

#if defined(OSC_DEBUG) || defined(IGR_DEBUG)
    //
    // init UART
    //
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);

    //
    // init LED pin
    //
    //gpio_init(PICO_DEFAULT_LED_PIN);
    //gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

#endif /* OSC_DEBUG || IGR_DEBUG */
#if defined(OSC_DEBUG)
    //gpio_put(PICO_DEFAULT_LED_PIN, last_mode);
#endif

    ws2812_init(WS2812_PIN);
    ws2812_set_rgb(7, 0, 0);

    // -------------------------------------------------------------------------
    //
    // local variables
    //
#if defined(OSC_ENABLED)
    bool mode;
#endif

#if defined(IGR_ENABLED)
    struct ps1_dat dat;
    dat.size = 0;
    dat.ready = 0;

    uint16_t buttons;

    uint32_t t;
    uint32_t target_gpio;
    uint32_t tmo_gpio = 0;
    uint32_t tmo_cldn = millis() + IGR_COOLDOWN_TIME;

    bool flag_led_off = 1;
    bool flag_next = 1;

    int32_t i;
    int32_t hot_button_idx = -1;

#if defined(IGR_DEBUG)
    char str[512];
    char str_size = 0;
#endif

#if defined(HOT_BUTTONS_HOLDING_TIME)
    uint32_t tmo_btn = 0;
    uint16_t last_btn = 0;
#endif /* HOT_BUTTONS_HOLDING_TIME */
#endif /* IGR_ENABLED */

#if defined(OSC_DEBUG) || defined(IGR_DEBUG)
    _uart_printf("[%.10d] Start\n", t);
#endif

    // -------------------------------------------------------------------------
    //
    // main loop
    //
    while (1)
    {
#if defined(OSC_ENABLED)
        //
        // handle video mode changing
        //
        mode = gpio_get(GPIO_VIDEO_MODE);

        if (last_mode != mode)
        {
            last_mode = mode;

            osc_select((uint8_t)mode);

#if defined(OSC_DEBUG)
            gpio_put(PICO_DEFAULT_LED_PIN, mode);
#endif
        }
#endif /* OSC_ENABLED */

        // ---------------------------------------------------------------------

#if defined(IGR_ENABLED)
        //
        // read gamepad data and process
        //
        read_ctrl_data(&dat);

        t = millis();

        if (tmo_cldn < t)
        {
            if (flag_led_off)
            {
                flag_led_off = 0;
                ws2812_set_rgb(0, 0, 0);
#if defined(IGR_DEBUG)
                _uart_printf("[%.10d] IGR ready\n", t);
#endif
            }

            if (dat.ready)
            {
                hot_button_idx = -1;

                switch (dat.pid)
                {
                    case PID_ANALOGUE:
                    case PID_DIGITAL:
                    case PID_DUAL_ANALOG:
                    {
                        buttons = ~dat.buttons;

                        for (i = 0; i < hot_buttons_count; i++)
                        {
                            if (buttons == hot_buttons_map[i].buttons)
                            {
                                hot_button_idx = i;
                                break;
                            }
                        }

                        flag_next = 0;

                        break;
                    }

                    case PID_GUNCON:
                    {
                        buttons = ~dat.buttons;

                        if (buttons == GUNCON_IGR_BUTTONS)
                        {
                            hot_button_idx = 0;
                        }

                        flag_next = 0;

                        break;
                    }

                    default:
                        flag_next = 1;
                        break;
                }

                
                if (hot_button_idx >= 0)
                {
#if defined(HOT_BUTTONS_HOLDING_TIME)
                    if (last_btn != buttons)
                    {
                        tmo_btn = 0;
                        last_btn = buttons;
                    }
                    else if (!tmo_btn)
                    {
                        tmo_btn = t + HOT_BUTTONS_HOLDING_TIME;
                    }
                    else if (tmo_btn < t)
                    {
#endif
                        target_gpio = hot_buttons_map[hot_button_idx].gpio;

                        gpio_set_dir(target_gpio, GPIO_OUT);
                        gpio_put(target_gpio, 0);

                        tmo_gpio = t        + hot_buttons_map[hot_button_idx].duration;
                        tmo_cldn = tmo_gpio + hot_buttons_map[hot_button_idx].cooldown;

                        if (target_gpio == GPIO_IGR1)
                            ws2812_set_rgb(0, 0, 15);
                        else
                            ws2812_set_rgb(0, 15, 0);
                        flag_led_off = 1;

#if defined(IGR_DEBUG)
                        //gpio_put(PICO_DEFAULT_LED_PIN, 1);
                        _uart_printf("[%.10d] IGR push GPIO=%d\n", t, target_gpio);
#endif

#if defined(HOT_BUTTONS_HOLDING_TIME)
                        tmo_btn  = 0;
                        last_btn = 0;
                    }
                } /* if (hot_button_idx >= 0) */
                else if (!flag_next)
                {
                    tmo_btn = 0;
                    last_btn = 0;
#endif
                }
            } /* if (!read_ctrl_data(&dat)) */
        } /* if (tmo_cldn < t) */

        // ---------------------------------------------------------------------

        if (tmo_gpio && (tmo_gpio < t))
        {
            gpio_put(target_gpio, 1);
            gpio_set_dir(target_gpio, GPIO_IN);

            ws2812_set_rgb(7, 0, 0);

            tmo_gpio = 0;

#if defined(IGR_DEBUG)
            //gpio_put(PICO_DEFAULT_LED_PIN, 0);
            _uart_printf("[%.10d] IGR release GPIO=%d\n", t, target_gpio);
#endif
        }

        // ---------------------------------------------------------------------

#endif /* IGR_ENABLED */
    }

    __unreachable();
}
// -----------------------------------------------------------------------------





// -----------------------------------------------------------------------------
// helper routines implementation
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// printf to UART
// -----------------------------------------------------------------------------
void _uart_printf(const char* fmt, ...)
{
    const int buffer_size = 2048;
    char buffer[buffer_size + 1];

    va_list args;
    va_start(args, fmt);

    if (vsnprintf(buffer, buffer_size, fmt, args) > buffer_size)
        buffer[buffer_size] = '\0';

    _uart_puts(buffer);

    va_end(args);
}
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// read playstation controller data
// returns 1, when entire data packet is ready
// -----------------------------------------------------------------------------
bool __not_in_flash_func(read_ctrl_data)(struct ps1_dat* data)
{
    const uint8_t max_size = sizeof(data->raw);
    uint8_t size = 0;
    uint8_t v;
    int cnt = 0;

    spi_hw_t* spi = (spi_hw_t*)(SPI_DAT);

    while (spi->sr & SPI_SSPSR_RNE_BITS)
    {
        v = (uint8_t)(spi->dr);

        if (size < max_size)
        {
            // reverse bits order (LSB to MSB)
            v = ((v & 0xF0) >> 4) | ((v & 0x0F) << 4);
            v = ((v & 0xCC) >> 2) | ((v & 0x33) << 2);
            v = ((v & 0xAA) >> 1) | ((v & 0x55) << 1);

            data->raw[size++] = v;
        }

        sleep_us(60);
    }

    data->size = size;

    if (size >= 5)
    {
        if ( (data->sid == 0x5A) &&
             ((data->ff == 0xFF) || (data->ff == 0x01)) )
        {
            data->ready = 1;
            return 1;
        }
    }

    data->ready = 0;
    return 0;
}
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// Si5351 control
// -----------------------------------------------------------------------------
#define SI5351_ADDR 0x60

// Si5351 registers
#define SI_CLK_OE    3
#define SI_CLK0_CTL  16
#define SI_CLK1_CTL  17
#define SI_CLK2_CTL  18
#define SI_PLLA      26
#define SI_PLLB      34
#define SI_MS0       42
#define SI_MS1       50
#define SI_MS2       58
#define SI_SS_EN     149
#define SI_PLL_RESET 177

// CLK_OE register 3 masks
#define SI_CLK0_DISABLE 0b00000001 /* Disable clock 0 output */
#define SI_CLK1_DISABLE 0b00000010 /* Disable clock 1 output */
#define SI_CLK2_DISABLE 0b00000100 /* Disable clock 2 output */

// CLKi_CTL register 16, 17, 18 values
#define SI_CLK_BASE     0b00001100
#define SI_CLK_INT      0b01000000 /* Set integer mode */
#define SI_CLK_PLLA     0b00000000 /* Select PLL A as MS source */
#define SI_CLK_PLLB     0b00100000 /* Select PLL B as MS source (default 0 = PLL A) */
#define SI_CLK_2MA      0b00000000 /* Select output drive 2 mA */
#define SI_CLK_4MA      0b00000001 /* Select output drive 4 mA */
#define SI_CLK_6MA      0b00000010 /* Select output drive 6 mA */
#define SI_CLK_8MA      0b00000011 /* Select output drive 8 mA */

// PLL_RESET register 177 values
#define SI_PLLA_RST     0b00100000 /* Reset PLL A */
#define SI_PLLB_RST     0b10000000 /* Reset PLL B */
// -----------------------------------------------------------------------------
// NTSC: 53.6931 MHz / 14.318 MHz / 3.579545 MHz
// PAL:  53.2034 MHz / 17.734 MHz / 4.43361875 MHz
#define GPU_CLK_PAL  53203400L
#define GPU_CLK_NTSC 53693100L

#define SUB_CLK_PAL  4433618L
#define SUB_CLK_NTSC 3579545L
// -----------------------------------------------------------------------------
//#define xtal_correction -408.0
//#define xtal_correction -500.0
//#define xtal_correction 1260.0

uint8_t clk0_preset[2][9];
uint8_t clk2_preset[2][9];
// -----------------------------------------------------------------------------
inline int i2c_put_data(const uint8_t* src, size_t len, bool nostop)
{
    return i2c_write_blocking(i2c0, SI5351_ADDR, src, len, nostop);
}
// -----------------------------------------------------------------------------
void calc_preset(uint8_t addr, int32_t freq, uint8_t* data)
{
    // Apply correction, _after_ determining rdiv.
#if defined(xtal_correction)
    freq = freq - (int32_t)((((double)freq) / 10000000.0) * xtal_correction);
#endif

    int32_t pll_freq = 900000000;
    int32_t t = (freq >> 20) + 1;

    int32_t div   = (pll_freq / freq);
    int32_t num   = (pll_freq % freq) / t;

    int32_t P3 = freq / t;
    int32_t P2 = (128 * num) % P3;
    int32_t P1 = (128 * div) + ((128 * num) / P3) - 512;

    data[0] = addr;
    data[1] = (P3 >> 8) & 0xFF;
    data[2] = P3 & 0xFF;
    data[3] = ((P1 >> 16) & 0x3);
    data[4] = (P1 >> 8) & 0xFF;
    data[5] = P1 & 0xFF;
    data[6] = ((P3 >> 12) & 0xF0) | ((P2 >> 16) & 0xF);
    data[7] = (P2 >> 8) & 0xFF;
    data[8] = P2 & 0xFF;
}
// ----------------------------------------------------------------------------
void calc_pll(uint8_t addr, uint8_t* data)
{
    int32_t mult = 36;
    int32_t num  = 0;

    int32_t P3 = 1;
    int32_t P2 = (128 * num) % P3;
    int32_t P1 = (128 * mult) + ((128 * num)) / P3 - 512;

    data[0] = addr;
    data[1] = (P3 >> 8) & 0xFF;
    data[2] = P3 & 0xFF;
    data[3] = ((P1 >> 16) & 0x3);
    data[4] = (P1 >> 8) & 0xFF;
    data[5] = P1 & 0xFF;
    data[6] = ((P3 >> 12) & 0xF0) | ((P2 >> 16) & 0xF);
    data[7] = (P2 >> 8) & 0xFF;
    data[8] = P2 & 0xFF;
}
// ----------------------------------------------------------------------------
void osc_init(uint8_t initial_preset)
{
    uint8_t data[9];

    // Disable Spread Spectrum
    data[0] = SI_SS_EN;
    data[1] = 0x00;
    i2c_put_data(data, 2, 0);

    // Disable all CLKx
    data[0] = SI_CLK_OE;
    data[1] = 0xFF;
    i2c_put_data(data, 2, 0);

    // Power down all CLK3 - CLK7
    data[0] = SI_CLK0_CTL;
    data[1] = SI_CLK_BASE | SI_CLK_PLLA | SI_CLK_8MA; // CLK0
    data[2] = SI_CLK_BASE | SI_CLK_PLLA | SI_CLK_8MA; // CLK1
    data[3] = SI_CLK_BASE | SI_CLK_PLLA | SI_CLK_8MA; // CLK2
    data[4] = 0x80; // CLK3
    data[5] = 0x80; // CLK4
    data[6] = 0x80; // CLK5
    data[7] = 0x80; // CLK6
    data[8] = 0x80; // CLK7
    i2c_put_data(data, 9, 0);

    // Init Multisynth Deviders config presets
    calc_preset(SI_MS0, GPU_CLK_PAL,  clk0_preset[OSC_PRESET_PAL]);
    calc_preset(SI_MS0, GPU_CLK_NTSC, clk0_preset[OSC_PRESET_NTSC]);
    calc_preset(SI_MS2, SUB_CLK_PAL,  clk2_preset[OSC_PRESET_PAL]);
    calc_preset(SI_MS2, SUB_CLK_NTSC, clk2_preset[OSC_PRESET_NTSC]);

    // Write Multisynth Deviders config
    data[0] = SI_MS1;
    memcpy(&data[1], &clk0_preset[OSC_PRESET_NTSC][1], 8);
    i2c_put_data(data, 9, 0);

    i2c_put_data(clk0_preset[initial_preset], 9, 0);
    i2c_put_data(clk2_preset[gpio_get(GPIO_LOCK_CLK2) ? initial_preset : OSC_PRESET_PAL], 9, 0);

    // Init PLLA
    calc_pll(SI_PLLA, data);
    i2c_put_data(data, 9, 0);

    // Reset PLLA
    data[0] = SI_PLL_RESET;
    data[1] = SI_PLLA_RST;
    i2c_put_data(data, 2, 0);

    // Enable CLK0, CLK1, CLK2
    data[0] = SI_CLK_OE;
    data[1] = 0b11111000;
    i2c_put_data(data, 2, 0);
}
// -----------------------------------------------------------------------------
/**
  * preset:
  *   0 - PAL
  *   1 - NTSC
  */
void osc_select(uint8_t preset)
{
    i2c_put_data(clk0_preset[preset], 9, 0);
    i2c_put_data(clk2_preset[gpio_get(GPIO_LOCK_CLK2) ? preset : OSC_PRESET_PAL], 9, 0);
}
// -----------------------------------------------------------------------------
