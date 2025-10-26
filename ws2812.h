// -----------------------------------------------------------------------------
// WS2812 LED driver
// -----------------------------------------------------------------------------
#define WS2812_RED     0
#define WS2812_YELLOW  30
#define WS2812_GREEN   70
#define WS2812_CYAN    105
#define WS2812_BLUE    170
#define WS2812_PURPLE  220
// -----------------------------------------------------------------------------
void ws2812_init(uint32_t gpio);

void ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b);

//  hue = 0-30-60-90-120-150-180-210-240
//  sat = saturation
//  bri = brightness
void ws2812_set_hsv(uint8_t hue, uint8_t sat, uint8_t bri);
// -----------------------------------------------------------------------------
