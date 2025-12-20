#ifndef LED_FX_H
#define LED_FX_H

#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "state.h"

// Declare the NeoPixel object
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Function Prototypes
void led_fx_init();
void led_fx_update(const Robot& robot);
void led_fx_set_all(uint8_t r, uint8_t g, uint8_t b);
void led_fx_off();

// --- IMPLEMENTATIONS ---

inline void led_fx_init() {
    pixels.begin(); // INITIALIZE NeoPixel strip object
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.show();
}

inline void led_fx_set_all(uint8_t r, uint8_t g, uint8_t b) {
    for(int i=0; i<NEOPIXEL_COUNT; i++) {
        pixels.setPixelColor(i, pixels.Color(r, g, b));
    }
    pixels.show();
}

inline void led_fx_off() {
    pixels.clear();
    pixels.show();
}

// Main function to update LEDs based on robot state
inline void led_fx_update(const Robot& robot) {
    static unsigned long last_update = 0;
    const int update_interval = 50; // ms

    if (millis() - last_update < update_interval) {
        return; // Update LEDs at a fixed interval
    }
    last_update = millis();

    // Priority 1: Critical Battery
    if (robot.batteryIsCritical) {
        // Fast blinking red
        uint32_t color = (millis() % 500 < 250) ? pixels.Color(255, 0, 0) : pixels.Color(0, 0, 0);
        pixels.fill(color);
        pixels.show();
        return;
    }
    
    // Priority 2: Cliff Detected
    if (robot.currentState == CLIFF_DETECTED) {
        // Blinking Orange
        uint32_t color = (millis() % 1000 < 500) ? pixels.Color(255, 165, 0) : pixels.Color(0, 0, 0);
        pixels.fill(color);
        pixels.show();
        return;
    }

    // Priority 3: Obstacle Avoidance / Sentry Mode
    if (robot.currentState == OBSTACLE_AVOIDANCE) {
        // "Cylon" eye effect
        static int eye_pos = 0;
        static int eye_dir = 1;
        pixels.clear();
        pixels.setPixelColor(eye_pos, pixels.Color(255, 0, 0));
        pixels.show();
        eye_pos += eye_dir;
        if (eye_pos == NEOPIXEL_COUNT -1 || eye_pos == 0) {
            eye_dir *= -1; // reverse direction
        }
        return;
    }

    if (robot.currentState == SENTRY_MODE) {
        if(robot.sentryState == SENTRY_TRACKING) {
            uint32_t color = (millis() % 200 < 100) ? pixels.Color(255, 0, 0) : pixels.Color(255, 255, 0);
            pixels.fill(color);
        } else {
             // "Cylon" eye effect in blue
            static int eye_pos = 0;
            static int eye_dir = 1;
            pixels.clear();
            pixels.setPixelColor(eye_pos, pixels.Color(0, 0, 255));
            pixels.show();
            eye_pos += eye_dir;
            if (eye_pos >= NEOPIXEL_COUNT - 1) eye_dir = -1;
            if (eye_pos <= 0) eye_dir = 1;
        }
        pixels.show();
        return;
    }
    
    // Default states
    switch (robot.currentState) {
        case IDLE: {
            // Breathing blue
            float breath = (sin(millis() / 2000.0 * 2 * PI) + 1) / 2;
            pixels.fill(pixels.Color(0, 0, 128 * breath));
            break;
        }
        case MOVING_FORWARD:
        case MANUAL_FORWARD:
        case FOLLOW_HEADING:
        case SMART_AVOIDANCE:
            pixels.fill(pixels.Color(0, 255, 0)); // Green
            break;
        case MOVING_BACKWARD:
        case MANUAL_BACKWARD:
            pixels.fill(pixels.Color(255, 100, 0)); // Orange-ish
            break;
        case TURNING_LEFT:
        case MANUAL_TURNING_LEFT:
            pixels.clear();
            pixels.setPixelColor(0, pixels.Color(255, 255, 0)); // Left pixel yellow
            break;
        case TURNING_RIGHT:
        case MANUAL_TURNING_RIGHT:
            pixels.clear();
            pixels.setPixelColor(NEOPIXEL_COUNT - 1, pixels.Color(255, 255, 0)); // Right pixel yellow
            break;
        case CALIBRATING_COMPASS:
            // Rainbow cycle
            pixels.rainbow(millis() / 10);
            break;
        default:
             // Low battery indicator (slow pulsing yellow) if idle
            if (robot.batteryIsLow) {
                uint32_t color = (millis() % 2000 < 1000) ? pixels.Color(150, 150, 0) : pixels.Color(30, 30, 0);
                pixels.fill(color);
            } else {
                // Default to off if state is not handled
                pixels.clear();
            }
            break;
    }
    pixels.show();
}


#endif // LED_FX_H
