#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "state.h"
#include "config.h"
#include "DFRobot_RGBLCD1602.h"
#include "sd_utils.h"
#include <string.h>

// Declare extern if lcd object is defined in NoNo.ino
extern DFRobot_RGBLCD1602 lcd;

// Configuration interne pour l'affichage
#define LCD_COLS 16
#define LCD_ROWS 2
// Capacité totale (ex: 4 lignes virtuelles de 16 chars)
#define MAX_LCD_TEXT_LENGTH 64 
// Taille d'une page écran (16x2 = 32 chars)
#define LCD_PAGE_CAPACITY (LCD_COLS * LCD_ROWS) 

/**
 * Fonction helper privée : Affiche une "page" spécifique du texte
 * page 0 = char 0 à 31
 * page 1 = char 32 à 63
 */
inline void _renderPage(const char* text, int pageIndex) {
    char lineBuffer[LCD_COLS + 1];
    int startOffset = pageIndex * LCD_PAGE_CAPACITY;
    int textLen = strlen(text);

    // --- LIGNE 1 ---
    memset(lineBuffer, 0, sizeof(lineBuffer));
    for (int i = 0; i < LCD_COLS; i++) {
        int charIdx = startOffset + i;
        // Si on est dans le texte, on copie, sinon on met un espace
        if (charIdx < textLen) {
            lineBuffer[i] = text[charIdx];
        } else {
            lineBuffer[i] = ' ';
        }
    }
    lcd.setCursor(0, 0);
    lcd.print(lineBuffer);

    // --- LIGNE 2 ---
    memset(lineBuffer, 0, sizeof(lineBuffer));
    for (int i = 0; i < LCD_COLS; i++) {
        int charIdx = startOffset + LCD_COLS + i;
        if (charIdx < textLen) {
            lineBuffer[i] = text[charIdx];
        } else {
            lineBuffer[i] = ' ';
        }
    }
    lcd.setCursor(0, 1);
    lcd.print(lineBuffer);
}

/**
 * Définit le texte à afficher.
 * Gère la PRIORITÉ : Si le texte change, on reset tout (timer, page) pour l'afficher immédiatement.
 */
inline void setLcdText(Robot& robot, const char* text) {
    // 1. Vérification de changement
    // Si le texte est identique, on ne fait rien (on laisse handleLcdScrolling gérer le défilement)
    if (strncmp(text, robot.lcdText, MAX_LCD_TEXT_LENGTH) == 0) return;

    // 2. Le texte a changé (Message Prioritaire ou Nouvel État)
    // On copie le nouveau texte
    strncpy(robot.lcdText, text, MAX_LCD_TEXT_LENGTH);
    robot.lcdText[MAX_LCD_TEXT_LENGTH - 1] = '\0'; // Sécurité null-terminated

    // 3. RESET pour affichage immédiat
    robot.currentPage = 0;             // Revenir au début
    robot.lastLcdUpdateTime = millis(); // Reset du timer

    // 4. Affichage immédiat de la première page
    // lcd.clear() n'est pas nécessaire car _renderPage écrase tout avec des espaces si besoin,
    // ce qui évite le clignotement.
    _renderPage(robot.lcdText, 0);
}

/**
 * Gère le défilement automatique si le texte est long.
 * À appeler dans la boucle principale.
 */
inline void handleLcdScrolling(Robot& robot) {
    size_t textLen = strlen(robot.lcdText);

    // 1. Si le texte tient sur un seul écran (<= 32 chars), on ne fait rien.
    if (textLen <= LCD_PAGE_CAPACITY) {
        return;
    }

    // 2. Calcul du nombre de pages nécessaires
    // Ex: 64 chars / 32 = 2 pages. 33 chars / 32 = 2 pages (arrondi sup).
    int totalPages = (textLen + LCD_PAGE_CAPACITY - 1) / LCD_PAGE_CAPACITY;

    // 3. Vérification du délai
    if (millis() - robot.lastLcdUpdateTime >= SCROLL_DELAY_MS) {
        robot.lastLcdUpdateTime = millis();
        
        // Passage à la page suivante (boucle)
        robot.currentPage++;
        if (robot.currentPage >= totalPages) {
            robot.currentPage = 0;
        }

        // Affichage de la nouvelle page
        _renderPage(robot.lcdText, robot.currentPage);
    }
}

inline void displayRandomJoke(Robot& robot) {
    char jokeBuffer[MAX_LCD_TEXT_LENGTH + 1];
    // Assurez-vous que getRandomJokeFromSD peut lire jusqu'à 64 chars
    getRandomJokeFromSD(robot, "jokes.txt", jokeBuffer, sizeof(jokeBuffer));
    setLcdText(robot, jokeBuffer);
    robot.lastJokeDisplayTime = millis();
}

inline void displayJokesIfIdle(Robot& robot) {
    // Don't show jokes if a custom message is active
    if (millis() - robot.customMessageSetTime < CUSTOM_MESSAGE_DURATION_MS) {
        return;
    }

    unsigned long currentTime = millis();
    // On n'affiche une blague que si l'état IDLE n'a pas changé depuis longtemps
    // ET qu'on a fini de lire le texte précédent (optionnel, ici basé sur le temps)
    if (currentTime - robot.lastLcdUpdateTime >= LCD_IDLE_TIMEOUT_MS) {
        if (currentTime - robot.lastJokeDisplayTime >= LCD_JOKE_INTERVAL_MS || robot.lastJokeDisplayTime == 0) {
            displayRandomJoke(robot);
        }
    }
}

inline void updateLcdDisplay(Robot& robot) {
    // If a custom message is active, just handle its scrolling and don't overwrite it
    if (millis() - robot.customMessageSetTime < CUSTOM_MESSAGE_DURATION_MS) {
        handleLcdScrolling(robot);
        return;
    }

    // Buffer augmenté à MAX_LCD_TEXT_LENGTH (64)
    char displayBuffer[MAX_LCD_TEXT_LENGTH + 1] = {0};

    // --- PRIORITY MESSAGES ---
    if (robot.batteryIsCritical) {
        snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", "BATTERIE VIDE", "ARRET IMMEDIAT");
        setLcdText(robot, displayBuffer);
        return; // Override all other display states
    }
    if (robot.batteryIsLow) {
        // For low battery, we can show a warning but still provide some state info.
        // Let's take line 2 for the warning.
        char line1_state[LCD_COLS + 1] = "Etat inconnu";
        switch(robot.currentState) {
            case IDLE: snprintf(line1_state, sizeof(line1_state), "IDLE"); break;
            case MOVING_FORWARD: snprintf(line1_state, sizeof(line1_state), "AVANT (LENT)"); break;
            case OBSTACLE_AVOIDANCE: snprintf(line1_state, sizeof(line1_state), "EVITEMENT (LENT)"); break;
            default: snprintf(line1_state, sizeof(line1_state), "EN ACTION (LENT)"); break;
        }
        snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1_state, "BATTERIE FAIBLE");
        setLcdText(robot, displayBuffer);
        return; // Override other display logic
    }
    
    // Buffers temporaires pour formater les lignes (16 chars)
    // On utilise snprintf pour formater proprement
    char line1[LCD_COLS + 1] = {0};
    char line2[LCD_COLS + 1] = {0};

    switch (robot.currentState) {
        case IDLE:
            // Exemple standard (court, pas de scroll)
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", "IDLE", "Waiting...");
            break;

        case MOVING_FORWARD:
        case MANUAL_FORWARD:
            snprintf(line1, sizeof(line1), "FORWARD");
            snprintf(line2, sizeof(line2), "CAP:%-11d", robot.cap);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;

        case MOVING_BACKWARD:
        case MANUAL_BACKWARD:
            snprintf(line1, sizeof(line1), "BACKWARD");
            snprintf(line2, sizeof(line2), "CAP:%-11d", robot.cap);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;

        case TURNING_LEFT:
        case MANUAL_TURNING_LEFT:
            snprintf(line1, sizeof(line1), "TURN LEFT");
            snprintf(line2, sizeof(line2), "CAP:%-11d", robot.cap);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;

        case TURNING_RIGHT:
        case MANUAL_TURNING_RIGHT:
            snprintf(line1, sizeof(line1), "TURN RIGHT");
            snprintf(line2, sizeof(line2), "CAP:%-11d", robot.cap);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;

        case OBSTACLE_AVOIDANCE:
        case SMART_AVOIDANCE:
            // Information critique
            snprintf(line1, sizeof(line1), "AVOIDING");
            snprintf(line2, sizeof(line2), "US:%-4d L:%-4d", robot.dusm, robot.distanceLaser);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;

        case CALIBRATING_COMPASS:
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", "CALIBRATING", "COMPASS...");
            break;

        case SENTRY_MODE:
            switch (robot.sentryState) {
                case SENTRY_IDLE:
                    snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", "SENTRY MODE", "Awaiting target");
                    break;
                case SENTRY_SCAN_START:
                case SENTRY_SCAN_STEP:
                    snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", "SENTRY MODE", "Scanning...");
                    break;
                case SENTRY_TRACKING:
                    snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", "SENTRY MODE", "Tracking target");
                    break;
                case SENTRY_ALARM:
                    snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", "SENTRY ALARM", "INTRUDER!!!");
                    break;
            }
            break;

        case FOLLOW_HEADING:
            snprintf(line1, sizeof(line1), "FOLLOW HDG");
            snprintf(line2, sizeof(line2), "TRG:%-4d CUR:%-4d", (int)robot.capCibleRotation, robot.cap);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;
            
        case MAINTAIN_HEADING:
            snprintf(line1, sizeof(line1), "MAINTAIN HDG");
            snprintf(line2, sizeof(line2), "TRG:%-4d CUR:%-4d", (int)robot.capCibleRotation, robot.cap);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;

        case SCANNING:
            snprintf(line1, sizeof(line1), "SCANNING");
            snprintf(line2, sizeof(line2), "ANG:%-4d DIST:%-4d", robot.currentScanAngleH, robot.distanceLaser);
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;

        default:
            snprintf(line1, sizeof(line1), "STATE: %-8d", robot.currentState);
            snprintf(line2, sizeof(line2), "UNKNOWN");
            snprintf(displayBuffer, sizeof(displayBuffer), "%-16s%-16s", line1, line2);
            break;
    }
    
    // Appelle la fonction qui gère la logique de mise à jour / reset
    setLcdText(robot, displayBuffer);
}

#endif // DISPLAY_H