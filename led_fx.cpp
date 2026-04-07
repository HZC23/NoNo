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
const uint32_t COLOR_CYAN = pixels.Color(0, 255, 255);
const uint32_t COLOR_MAGENTA = pixels.Color(255, 0, 255);

// Internal state for animations
static unsigned long last_anim_tick = 0;
static int scanner_pos = 0;
static int scanner_dir = 1;

void led_fx_init() {
    LOG_INFO("LED FX: Starting NeoPixel initialization on pin %d with %d LEDs", NEOPIXEL_PIN, NEOPIXEL_COUNT);
    pixels.begin();
    pixels.setBrightness(100); 
    pixels.clear();
    pixels.show();
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

// --- Animation Helpers ---

void effect_breathing(int led_idx, uint32_t base_color, float speed = 2000.0) {
    float breath = (sin(millis() / speed * 2 * PI) + 1) / 2;
    uint8_t r = (base_color >> 16) & 0xFF;
    uint8_t g = (base_color >> 8) & 0xFF;
    uint8_t b = base_color & 0xFF;
    pixels.setPixelColor(led_idx, pixels.Color(r * breath, g * breath, b * breath));
}

void effect_flashing(int led_idx, uint32_t color1, uint32_t color2, int period) {
    uint32_t color = (millis() % period < period / 2) ? color1 : color2;
    pixels.setPixelColor(led_idx, color);
}

void effect_scanner_all(uint32_t color, int speed_ms) {
    if (millis() - last_anim_tick > speed_ms) {
        scanner_pos += scanner_dir;
        if (scanner_pos >= NEOPIXEL_COUNT - 1 || scanner_pos <= 0) {
            scanner_dir *= -1;
        }
        last_anim_tick = millis();
    }
    pixels.clear();
    pixels.setPixelColor(scanner_pos, color);
}

void effect_rainbow_all() {
    for(int i=0; i<NEOPIXEL_COUNT; i++) {
        uint16_t hue = (((i * 65536 / NEOPIXEL_COUNT) + (millis()/10)) % 65536);
        pixels.setPixelColor(i, pixels.gamma32(pixels.ColorHSV(hue, 255, 255)));
    }
}

// --- Individual LED Handlers ---

// LED 0: System Heartbeat / Global Status
void update_led_system(const Robot& robot) {
    if (robot.batteryIsCritical || robot.isStuckConfirmed) {
        effect_flashing(0, COLOR_RED, COLOR_OFF, 200);
    } else if (robot.currentState == CLIFF_DETECTED) {
        effect_flashing(0, COLOR_ORANGE, COLOR_OFF, 400);
    } else {
        // Heartbeat green
        effect_breathing(0, COLOR_GREEN, 3000.0);
    }
}

// LED 1: Navigation Mode
void update_led_mode(const Robot& robot) {
    switch (robot.currentNavMode) {
        case MANUAL_CONTROL:
            pixels.setPixelColor(1, COLOR_BLUE);
            break;
        case AUTONOMOUS_CONTROL:
            if (robot.currentState == SENTRY_MODE) {
                pixels.setPixelColor(1, COLOR_CYAN);
            } else {
                pixels.setPixelColor(1, COLOR_PURPLE);
            }
            break;
        default:
            pixels.setPixelColor(1, COLOR_OFF);
            break;
    }
}

// LED 2: Battery Status
void update_led_battery(const Robot& robot) {
    if (robot.batteryIsCritical) {
        effect_flashing(2, COLOR_RED, COLOR_OFF, 400);
    } else if (robot.batteryIsLow) {
        pixels.setPixelColor(2, COLOR_ORANGE);
    } else {
        pixels.setPixelColor(2, COLOR_GREEN);
    }
}

// LED 3: Activity / Movement / Sensors
void update_led_activity(const Robot& robot) {
    // Flash white if an obstacle is very close
    if (robot.obstacleDetectedByLaser || (robot.dusm > 0 && robot.dusm < 15)) {
        effect_flashing(3, COLOR_WHITE, COLOR_OFF, 200);
        return;
    }

    switch (robot.currentState) {
        case IDLE:
            pixels.setPixelColor(3, COLOR_OFF);
            break;
        case MOVING_FORWARD:
        case FOLLOW_HEADING:
        case MAINTAIN_HEADING:
        case SMART_AVOIDANCE:
            pixels.setPixelColor(3, COLOR_GREEN);
            break;
        case MOVING_BACKWARD:
        case BACKING_UP_OBSTACLE:
            pixels.setPixelColor(3, COLOR_RED);
            break;
        case TURNING_LEFT:
        case TURNING_RIGHT:
        case SMART_TURNING:
            pixels.setPixelColor(3, COLOR_YELLOW);
            break;
        case SCANNING_FOR_PATH:
        case WAITING_FOR_TURRET:
            pixels.setPixelColor(3, COLOR_BLUE);
            break;
        default:
            pixels.setPixelColor(3, COLOR_OFF);
            break;
    }
}

// --- Main Update Function ---

void led_fx_update(const Robot& robot) {
    static unsigned long last_update = 0;
    const int update_interval = 40; 

    if (millis() - last_update < update_interval) {
        return;
    }
    last_update = millis();

    // 1. Check for High-Priority Global Events (Full LED Combinations)
    
    // Priority 1: Emergency / Impact / Stuck
    if (robot.currentState == EMERGENCY_EVASION || robot.isStuckConfirmed) {
        uint32_t flash_color = (millis() % 200 < 100) ? COLOR_WHITE : COLOR_RED;
        for(int i=0; i<NEOPIXEL_COUNT; i++) pixels.setPixelColor(i, flash_color);
        pixels.show();
        return;
    }
    
    // Priority 2: Cliff detected
    if (robot.currentState == CLIFF_DETECTED) {
        uint32_t flash_color = (millis() % 400 < 200) ? COLOR_ORANGE : COLOR_OFF;
        for(int i=0; i<NEOPIXEL_COUNT; i++) pixels.setPixelColor(i, flash_color);
        pixels.show();
        return;
    }

    // Priority 3: Calibration
    if (robot.currentState == CALIBRATING_COMPASS) {
        effect_rainbow_all();
        pixels.show();
        return;
    }

    // Priority 4: Obstacle Avoidance Scan
    if (robot.currentState == OBSTACLE_AVOIDANCE) {
        effect_scanner_all(COLOR_RED, 80);
        pixels.show();
        return;
    }

    // 2. Standard Mode: Individual LED roles
    pixels.clear();
    update_led_system(robot);
    update_led_mode(robot);
    update_led_battery(robot);
    update_led_activity(robot);
    
    pixels.show();
}

// --- Startup LED Test ---
void led_fx_startup_test() {
    LOG_INFO("LED FX: Starting enhanced LED startup sequence...");
    
    // 1. Mode LED Test (Blue then Purple)
    pixels.clear();
    pixels.setPixelColor(1, COLOR_BLUE); pixels.show(); delay(300);
    pixels.setPixelColor(1, COLOR_PURPLE); pixels.show(); delay(300);
    
    // 2. Battery LED Test (Red -> Orange -> Green)
    pixels.clear();
    pixels.setPixelColor(2, COLOR_RED); pixels.show(); delay(300);
    pixels.setPixelColor(2, COLOR_ORANGE); pixels.show(); delay(300);
    pixels.setPixelColor(2, COLOR_GREEN); pixels.show(); delay(300);

    // 3. Activity LED Test (White pulse)
    for(int i=0; i<3; i++) {
        pixels.setPixelColor(3, COLOR_WHITE); pixels.show(); delay(100);
        pixels.setPixelColor(3, COLOR_OFF); pixels.show(); delay(100);
    }

    // 4. Final Rainbow Wipe
    unsigned long start = millis();
    while (millis() - start < 1000) {
        effect_rainbow_all();
        pixels.show();
        delay(20);
    }
    
    led_fx_off();
    LOG_INFO("LED FX: Startup sequence complete");
}
