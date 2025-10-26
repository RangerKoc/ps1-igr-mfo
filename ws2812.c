// -----------------------------------------------------------------------------
// WS2812 LED driver
// -----------------------------------------------------------------------------
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
// -----------------------------------------------------------------------------
#define ws2812c_wrap_target 0
#define ws2812c_wrap 4
#define ws2812_sm 0
#define ws2812_dma_chan 0
#define ws2812_pio pio0
// -----------------------------------------------------------------------------
static dma_channel_config ws2812_dma_ch0;
// -----------------------------------------------------------------------------
static const uint16_t ws2812c_program_instructions[] =
{
            //     .wrap_target
    0x80a0, //  0: pull   block                      
    0x6068, //  1: out    null, 8                    
    0xa00b, //  2: mov    pins, !null                
    0x6001, //  3: out    pins, 1                    
    0x10e2, //  4: jmp    !osre, 2        side 0     
            //     .wrap
};
// -----------------------------------------------------------------------------
static const struct pio_program ws2812c_program =
{
    .instructions = ws2812c_program_instructions,
    .length = 5,
    .origin = -1,
};
// -----------------------------------------------------------------------------
static inline pio_sm_config ws2812c_program_get_default_config(uint32_t offset)
{
    pio_sm_config c = pio_get_default_sm_config();

    sm_config_set_wrap(&c, offset + ws2812c_wrap_target, offset + ws2812c_wrap);
    sm_config_set_sideset(&c, 2, 1, 0);

    return c;
}
// -----------------------------------------------------------------------------
void ws2812c_program_init(PIO pio, uint32_t sm, uint32_t offset, uint32_t pin)
{
    pio_sm_config cfg = ws2812c_program_get_default_config(offset);
    sm_config_set_out_shift(&cfg, 0, 0, 32);
    sm_config_set_clkdiv(&cfg, (clock_get_hz(clk_sys) / 1e6f) * 1.25f / 3.0f);  //1.25us / 3
    sm_config_set_out_pins(&cfg, pin, 1);
    sm_config_set_sideset_pins(&cfg, pin);

    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, 1);
    pio_sm_init(pio, sm, offset, &cfg);
}
// -----------------------------------------------------------------------------
void ws2812_init(uint32_t gpio)
{
    uint32_t offset = pio_add_program(ws2812_pio, &ws2812c_program);
    ws2812c_program_init(ws2812_pio, ws2812_sm, offset, gpio);

    // initialise the used GPIO pin to LOW
    gpio_put(gpio, 0);

    // configure DMA to copy the LED buffer to the PIO state machine's FIFO
    channel_config_set_transfer_data_size(&ws2812_dma_ch0, DMA_SIZE_32);
    channel_config_set_read_increment(&ws2812_dma_ch0, 1);
    channel_config_set_write_increment(&ws2812_dma_ch0, 0);
    channel_config_set_dreq(&ws2812_dma_ch0, DREQ_PIO0_TX0);

    //run the state machine
    pio_sm_set_enabled(ws2812_pio, ws2812_sm, 1);
    ws2812_dma_ch0 = dma_channel_get_default_config(ws2812_dma_chan);
}
// -----------------------------------------------------------------------------
void ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    static uint32_t ws2812_buffer;

    ws2812_buffer = (g << 16) | (r << 8) | b;

    dma_channel_configure(ws2812_dma_chan, &ws2812_dma_ch0, &ws2812_pio->txf[ws2812_sm], &ws2812_buffer, 1, 1);
}
// -----------------------------------------------------------------------------
//  hue = 0-30-60-90-120-150-180-210-240
//  sat = saturation
//  bri = brightness
void ws2812_set_hsv(uint8_t hue, uint8_t sat, uint8_t bri)
{
    uint8_t red, green, blue;

    if (!sat)
    {
        red = green = blue = bri;
    }
    else
    {
        float h = (float)hue / 255.0f;
        float s = (float)sat / 255.0f;
        float b = (float)bri / 255.0f;

        int32_t i = (int32_t)(h * 6.0f);
        float f = h * 6.0f - (float)i;

        uint8_t p = (uint8_t)(b * (1.0f - s) * 255.0f);
        uint8_t q = (uint8_t)(b * (1.0f - f * s) * 255.0f);
        uint8_t t = (uint8_t)(b * (1.0f - (1.0f - f) * s) * 255.0f);

        switch (i % 6)
        {
            case 0:
                red   = bri;
                green = t;
                blue  = p;
                break;

            case 1:
                red   = q;
                green = bri;
                blue  = p;
                break;
            
            case 2:
                red   = p;
                green = bri;
                blue  = t;
                break;

            case 3:
                red   = p;
                green = q;
                blue  = bri;
                break;

            case 4:
                red   = t;
                green = p;
                blue  = bri;
                break;

            default:
                red   = bri;
                green = p;
                blue  = q;
                break;
        }
    }

    ws2812_set_rgb(red, green, blue);
}
// -----------------------------------------------------------------------------
