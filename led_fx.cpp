#include "hardware.h"
#include "robot.h"
#include "logger.h"
#include <math.h>

// Color Constants
const uint32_t COLOR_OFF = 0;
const uint32_t COLOR_RED = pixels.Color(255, 0, 0);
const uint32_t COLOR_GREEN = pixels.Color(0, 255, 0);
const uint32_t COLOR_BLUE = pixels.Color(0, 0, 255);
const uint32_t COLOR_YELLOW = pixels.Color(255, 255, 0);
const uint32_t COLOR_ORANGE = pixels.Color(255, 100, 0);
const uint32_t COLOR_WHITE = pixels.Color(255, 255, 255);
const uint32_t COLOR_PURPLE = pixels.Color(128, 0, 128);

// Internal state for animations
static int scanner_pos = 1;
static int scanner_dir = 1;
static unsigned long last_anim_tick = 0;

void led_fx_init() {
    LOG_INFO("LED FX: Starting NeoPixel initialization on pin %d with %d LEDs", NEOPIXEL_PIN, NEOPIXEL_COUNT);
    pixels.begin();
    LOG_INFO("LED FX: pixels.begin() completed");
    
    pixels.setBrightness(100); // INCREASED from 50 to make LEDs more visible
    LOG_INFO("LED FX: Brightness set to 100 (was 50)");
    
    pixels.clear();
    LOG_INFO("LED FX: Buffer cleared");
    
    pixels.show();
    LOG_INFO("LED FX: NeoPixels initialized and displayed (should be OFF)");
}

// Helper to set color on the 3 visible LEDs only (indices 1, 2, 3)
void led_fx_set_visible_only(uint32_t color) {
    for(int i = 1; i < NEOPIXEL_COUNT; i++) {
        pixels.setPixelColor(i, color);
    }
}

void led_fx_off() {
    pixels.clear();
    pixels.show();
}

void led_fx_set_all(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t color = pixels.Color(r, g, b);
    for(int i=0; i < NEOPIXEL_COUNT; i++) {
        pixels.setPixelColor(i, color);
    }
    pixels.show();
}

// --- Specific Effect Helpers ---

void effect_breathing(uint32_t base_color, float speed = 2000.0) {
    float breath = (sin(millis() / speed * 2 * PI) + 1) / 2;
    uint8_t r = (base_color >> 16) & 0xFF;
    uint8_t g = (base_color >> 8) & 0xFF;
    uint8_t b = base_color & 0xFF;
    led_fx_set_visible_only(pixels.Color(r * breath, g * breath, b * breath));
}

void effect_flashing(uint32_t color1, uint32_t color2, int period) {
    uint32_t color = (millis() % period < period / 2) ? color1 : color2;
    led_fx_set_visible_only(color);
}

void effect_scanner(uint32_t color, int speed_ms) {
    if (millis() - last_anim_tick > speed_ms) {
        scanner_pos += scanner_dir;
        if (scanner_pos >= NEOPIXEL_COUNT - 1 || scanner_pos <= 1) {
            scanner_dir *= -1;
        }
        last_anim_tick = millis();
    }
    pixels.setPixelColor(scanner_pos, color);
}

void effect_rainbow() {
    for(int i=1; i<NEOPIXEL_COUNT; i++) {
        uint16_t hue = (((i * 65536 / (NEOPIXEL_COUNT-1)) + (millis()/10)) % 65536);
        pixels.setPixelColor(i, pixels.gamma32(pixels.ColorHSV(hue, 255, 255)));
    }
}

// --- System LED Handling ---

void update_system_led(const Robot& robot) {
    // LED 0 is the system status LED
    if (robot.batteryIsCritical) {
        pixels.setPixelColor(0, (millis() % 200 < 100) ? COLOR_RED : COLOR_OFF);
    } else {
        // Heartbeat green
        float pulse = (sin(millis() / 1000.0 * 2 * PI) + 1) / 2;
        pixels.setPixelColor(0, pixels.Color(0, 50 * pulse, 0));
    }
}

void led_fx_update(const Robot& robot) {
    static unsigned long last_update = 0;
    const int update_interval = 30; // ms, slightly faster for smoother anims

    if (millis() - last_update < update_interval) {
        return;
    }
    last_update = millis();

    pixels.clear(); 
    
    // 1. Update System LED (Pixel 0)
    update_system_led(robot);

    // 2. Determine Priority State for Visible LEDs (1-3)
    
    // Priority 1: Critical battery
    if (robot.batteryIsCritical) {
        effect_flashing(COLOR_RED, COLOR_OFF, 400);
        pixels.show();
        return;
    }
    
    // Priority 2: Cliff detected
    if (robot.currentState == CLIFF_DETECTED) {
        effect_flashing(COLOR_ORANGE, COLOR_OFF, 600);
        pixels.show();
        return;
    }
    
    // Priority 3: Emergency evasion (Bumper)
    if (robot.currentState == EMERGENCY_EVASION) {
        effect_flashing(COLOR_WHITE, COLOR_RED, 200);
        pixels.show();
        return;
    }

    // 3. Standard state-based effects
    switch (robot.currentState) {
        case IDLE:
            effect_breathing(pixels.Color(0, 0, 150));
            break;

        case MOVING_FORWARD:
        case FOLLOW_HEADING:
        case MAINTAIN_HEADING:
        case SMART_AVOIDANCE:
            led_fx_set_visible_only(COLOR_GREEN);
            break;

        case MOVING_BACKWARD:
            led_fx_set_visible_only(COLOR_ORANGE);
            break;

        case TURNING_LEFT:
            pixels.setPixelColor(3, COLOR_YELLOW);
            break;

        case TURNING_RIGHT:
            pixels.setPixelColor(1, COLOR_YELLOW);
            break;

        case OBSTACLE_AVOIDANCE:
            effect_scanner(COLOR_RED, 80);
            break;

        case SENTRY_MODE:
            if(robot.sentryState == SENTRY_TRACKING || robot.sentryState == SENTRY_ALARM) {
                 effect_flashing(COLOR_RED, COLOR_YELLOW, 200);
            } else { // SENTRY_SCAN
                effect_scanner(COLOR_BLUE, 100);
            }
            break;

        case CALIBRATING_COMPASS:
            effect_rainbow();
            break;

        default:
            if (robot.batteryIsLow) {
                effect_breathing(COLOR_YELLOW, 3000.0);
            }
            break;
    }
    
    pixels.show();
}

// --- Startup LED Test ---
void led_fx_startup_test() {
    LOG_INFO("LED FX: Starting LED startup test sequence...");
    
    // Test 1: All LEDs RED (400ms)
    LOG_INFO("LED FX: Test 1 - All LEDs RED");
    led_fx_set_all(255, 0, 0);
    delay(400);
    
    // Test 2: All LEDs GREEN (400ms)
    LOG_INFO("LED FX: Test 2 - All LEDs GREEN");
    led_fx_set_all(0, 255, 0);
    delay(400);
    
    // Test 3: All LEDs BLUE (400ms)
    LOG_INFO("LED FX: Test 3 - All LEDs BLUE");
    led_fx_set_all(0, 0, 255);
    delay(400);
    
    // Test 4: All LEDs YELLOW (400ms)
    LOG_INFO("LED FX: Test 4 - All LEDs YELLOW");
    led_fx_set_all(255, 255, 0);
    delay(400);
    
    // Test 5: Individual LED test - each LED lights up sequentially (200ms each)
    LOG_INFO("LED FX: Test 5 - Individual LEDs (each 200ms)");
    for (int i = 0; i < NEOPIXEL_COUNT; i++) {
        pixels.clear();
        pixels.setPixelColor(i, COLOR_WHITE);
        pixels.show();
        delay(200);
    }
    
    // Test 6: Rainbow effect (1s)
    LOG_INFO("LED FX: Test 6 - Rainbow effect");
    unsigned long rainbow_start = millis();
    while (millis() - rainbow_start < 1000) {
        for(int i=0; i<NEOPIXEL_COUNT; i++) {
            uint16_t hue = (((i * 65536 / NEOPIXEL_COUNT) + (millis()/10)) % 65536);
            pixels.setPixelColor(i, pixels.gamma32(pixels.ColorHSV(hue, 255, 255)));
        }
        pixels.show();
        delay(30);
    }
    
    // Turn off all LEDs
    led_fx_off();
    LOG_INFO("LED FX: Startup test sequence complete");
}
