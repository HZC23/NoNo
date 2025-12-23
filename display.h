#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "state.h"
#include "config.h"
#include "DFRobot_RGBLCD1602.h"
#include "sd_utils.h"
#include <string.h>

// Declare extern if lcd object is defined in NoNo.ino
extern DFRobot_RGBLCD1602 *lcd;

/**
 * @brief Applies word wrap to a string.
 */
inline void _applyWordWrap(const char* in, char* out, size_t bufferSize, int lineLength) {
    memset(out, 0, bufferSize);
    
    const char* source = in;
    int outIdx = 0;

    while (strlen(source) > 0 && outIdx < bufferSize - 2) {
        // If the remaining text fits on one line, just copy it and we're done.
        if ((int)strlen(source) <= lineLength) {
            strcat(out, source);
            break;
        }

        // Find the last space within the line length limit.
        int breakPoint = -1;
        for (int i = 0; i <= lineLength; i++) {
            if (source[i] == ' ') {
                breakPoint = i;
            }
            if (source[i] == '\0') {
                breakPoint = -1; 
                break;
            }
        }
        
        // If no space was found (a very long word), we must break the word.
        if (breakPoint == -1) {
            breakPoint = lineLength;
        }

        // Copy the part of the string up to the breakpoint.
        strncat(out, source, breakPoint);
        outIdx += breakPoint;
        
        // Add a newline.
        out[outIdx++] = '\n';
        out[outIdx] = '\0'; // Ensure null termination for strcat

        // Advance the source pointer. Skip the space at the breakpoint if it was a space.
        source += breakPoint + (source[breakPoint] == ' ' ? 1 : 0);
    }
}


/**
 * @brief Définit le texte cible pour le LCD.
 * @param isContinuation Si false (par défaut), l'écran est vidé et on recommence à (0,0).
 */
inline void setLcdText(Robot& robot, const char* text, bool isContinuation = false) {
    // Guard to prevent flickering by not re-rendering the same idle text
    if (robot.lcdAnimationState == Robot::LcdAnimationState::ANIM_IDLE && strncmp(text, robot.lcdText, MAX_LCD_TEXT_LENGTH) == 0) {
        return;
    }
    
    // 1. Si c'est un nouveau message (pas une suite)
    if (!isContinuation) {
        // Force l'arrêt de l'animation en cours
        robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_IDLE;
        
        // Nettoyage complet des tampons pour éviter les "fantômes" de texte
        memset(robot.lcdText, 0, MAX_LCD_TEXT_LENGTH + 1);
        memset(robot.lcdFormattedText, 0, sizeof(robot.lcdFormattedText));

        // Réinitialisation des positions de l'animation
        robot.lcdAnimationIndex = 0;
        robot.lcdCursorX = 0;
        robot.lcdCursorY = 0;

        // Nettoyage physique de l'écran
        lcd->clear();
        lcd->setCursor(0, 0);
    }

    // 2. Vérification si le texte est identique (pour éviter de relancer inutilement)
    if (isContinuation && strncmp(text, robot.lcdText, MAX_LCD_TEXT_LENGTH) == 0) {
        return;
    }
    
    // Protection contre le spam pendant les blagues
    if (millis() - robot.lastJokeDisplayTime < (LCD_JOKE_INTERVAL_MS - 100)) {
        return;
    }

    // 3. Copie du nouveau texte (ou ajout si c'est une suite)
    if (!isContinuation) {
        strncpy(robot.lcdText, text, MAX_LCD_TEXT_LENGTH);
    } else {
        // Optionnel : logique pour concaténer si vous gérez des messages longs en plusieurs fois
        strncat(robot.lcdText, text, MAX_LCD_TEXT_LENGTH - strlen(robot.lcdText) - 1);
    }
    robot.lcdText[MAX_LCD_TEXT_LENGTH] = '\0';

    // 4. Application du Word Wrap
    _applyWordWrap(robot.lcdText, robot.lcdFormattedText, sizeof(robot.lcdFormattedText), LCD_LINE_LENGTH);

    // 5. Lancement de l'animation
    robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_TYPEWRITER;
    robot.lcdAnimationNextCharTime = millis();
}

/**
 * @brief Moteur d'animation pour l'écran LCD. Doit être appelé dans la boucle principale.
 */
inline void handleLcdAnimations(Robot& robot) {
    unsigned long currentTime = millis();

    if (robot.lcdAnimationState == Robot::LcdAnimationState::ANIM_IDLE) {
        return;
    }

    // --- DEBUG OPTIONNEL ---
    // if (DEBUG_MODE) { ... } (Retiré pour la clarté, à remettre si besoin)

    switch(robot.lcdAnimationState) {
        // ---------------------------------------------------------
        // ÉTAT 1 : TYPEWRITER (Effet machine à écrire)
        // ---------------------------------------------------------
        case Robot::LcdAnimationState::ANIM_TYPEWRITER: {
            if (currentTime < robot.lcdAnimationNextCharTime) {
                return;
            }

            char nextChar = robot.lcdFormattedText[robot.lcdAnimationIndex];

            // 1. Fin du texte
            if (nextChar == '\0') {
                robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_IDLE;
                robot.lastLcdUpdateTime = currentTime;
                return;
            }

            // 2. Gestion du saut de ligne
            if (nextChar == '\n') {
                robot.lcdCursorY++;
                robot.lcdCursorX = 0;

                // Si on dépasse le nombre de lignes de l'écran (ex: on passe de la ligne 1 à 2 sur un écran 2 lignes)
                if (robot.lcdCursorY >= LCD_ROWS) {
                    // Vérifier s'il reste du texte après ce saut de ligne
                    if (robot.lcdFormattedText[robot.lcdAnimationIndex + 1] != '\0') {
                        // On pause pour laisser l'utilisateur lire avant de scroller
                        robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_SCROLL_PAUSE;
                        robot.lastLcdUpdateTime = currentTime; 
                    } else {
                        // C'était juste un \n à la toute fin du texte
                        robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_IDLE;
                        robot.lastLcdUpdateTime = currentTime;
                    }
                    return; // On sort pour attendre la pause ou finir
                }
                
                // Sinon, on déplace juste le curseur matériel
                lcd->setCursor(robot.lcdCursorX, robot.lcdCursorY);

            } else {
                // 3. Impression du caractère normal
                lcd->print(nextChar);
                robot.lcdCursorX++;
                
                // Correction ici : On utilise une valeur en dur ou une constante existante
                if (robot.lcdCursorX >= 16) { // Remplace 16 par LCD_LINE_LENGTH si disponible
                     robot.lcdCursorX = 0;
                     robot.lcdCursorY++; 
                     lcd->setCursor(robot.lcdCursorX, robot.lcdCursorY);
                }
            }

            // Préparation caractère suivant
            robot.lcdAnimationIndex++;
            robot.lcdAnimationNextCharTime = currentTime + LCD_TYPEWRITER_DELAY_MS;
            break;
        }

        // ---------------------------------------------------------
        // ÉTAT 2 : SCROLL PAUSE (Le scroll intelligent)
        // ---------------------------------------------------------
        case Robot::LcdAnimationState::ANIM_SCROLL_PAUSE: {
            if (currentTime - robot.lastLcdUpdateTime >= SCROLL_DELAY_MS) {
                
                // --- LOGIQUE DE SCROLL AMÉLIORÉE ---
                
                lcd->clear(); // On efface tout pour redessiner proprement

                // Objectif : Retrouver le début des (LCD_ROWS - 1) dernières lignes
                // On scanne en arrière depuis le caractère actuel (le '\n' qui a déclenché le scroll)
                
                int linesToKeep = LCD_ROWS - 1; // Sur un écran 2 lignes, on garde 1 ligne (la du bas devient celle du haut)
                int scanIndex = robot.lcdAnimationIndex - 1; // On regarde avant le \n actuel
                int foundNewlines = 0;
                int startPrintIndex = 0; // Par défaut début du texte si on ne trouve pas assez de lignes

                // Backtracking pour trouver le bon point de départ
                while (scanIndex >= 0) {
                    if (robot.lcdFormattedText[scanIndex] == '\n') {
                        foundNewlines++;
                        if (foundNewlines == linesToKeep) {
                            startPrintIndex = scanIndex + 1; // Le texte commence juste après ce \n
                            break;
                        }
                    }
                    scanIndex--;
                }

                // Réimpression instantanée des lignes conservées (décalées vers le haut)
                lcd->setCursor(0, 0);
                int tempCursorY = 0;
                
                // On réimprime du point trouvé jusqu'au caractère actuel
                for (int i = startPrintIndex; i < robot.lcdAnimationIndex; i++) {
                    char c = robot.lcdFormattedText[i];
                    if (c == '\n') {
                        tempCursorY++;
                        lcd->setCursor(0, tempCursorY);
                    } else {
                        lcd->print(c);
                    }
                }

                // Mise à jour de l'état du robot pour continuer sur la dernière ligne
                robot.lcdCursorX = 0;
                robot.lcdCursorY = LCD_ROWS - 1; // On se place sur la dernière ligne physique
                lcd->setCursor(robot.lcdCursorX, robot.lcdCursorY);

                // Reprise de l'animation machine à écrire
                robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_TYPEWRITER;
                
                // IMPORTANT : On saute le '\n' qui a causé le scroll, on passe au char suivant
                robot.lcdAnimationIndex++; 
                robot.lcdAnimationNextCharTime = currentTime + LCD_TYPEWRITER_DELAY_MS;
            }
            break;
        }
    }
}


// --- Functions to set display content ---

inline void displayRandomJoke(Robot& robot) {
    char jokeBuffer[MAX_LCD_TEXT_LENGTH + 1];
    getRandomJokeFromSD(robot, "/jokes.txt", jokeBuffer, sizeof(jokeBuffer));
    
    // Set the joke text, which will trigger the animation
    strncpy(robot.lcdText, jokeBuffer, MAX_LCD_TEXT_LENGTH);
    robot.lcdText[MAX_LCD_TEXT_LENGTH] = '\0';
    _applyWordWrap(robot.lcdText, robot.lcdFormattedText, sizeof(robot.lcdFormattedText), LCD_LINE_LENGTH);
    
    robot.lcdAnimationState = Robot::LcdAnimationState::ANIM_TYPEWRITER;
    robot.lcdAnimationIndex = 0;
    robot.lcdAnimationNextCharTime = millis();
    robot.lcdCursorX = 0;
    robot.lcdCursorY = 0;
    lcd->clear();
    lcd->setCursor(0, 0);

    robot.lastJokeDisplayTime = millis();
}

inline void displayJokesIfIdle(Robot& robot) {
    if (robot.lcdAnimationState != Robot::LcdAnimationState::ANIM_IDLE || (millis() - robot.customMessageSetTime < CUSTOM_MESSAGE_DURATION_MS)) {
        return;
    }

    unsigned long currentTime = millis();
    if (currentTime - robot.lastLcdUpdateTime >= LCD_IDLE_TIMEOUT_MS) {
        if (currentTime - robot.lastJokeDisplayTime >= LCD_JOKE_INTERVAL_MS || robot.lastJokeDisplayTime == 0) {
            displayRandomJoke(robot);
        }
    }
}


inline void updateLcdDisplay(Robot& robot) {
    // DO NOT update status text if any animation is running, a joke was just shown, or a custom message is active.
    if (robot.lcdAnimationState != Robot::LcdAnimationState::ANIM_IDLE || 
        (millis() - robot.lastJokeDisplayTime < (LCD_JOKE_INTERVAL_MS)) ||
        (millis() - robot.customMessageSetTime < CUSTOM_MESSAGE_DURATION_MS)) {
        return;
    }

    // --- Select text based on state ---
    char displayBuffer[MAX_LCD_TEXT_LENGTH + 1] = {0};
    
    switch (robot.currentState) {
        case IDLE:
            snprintf(displayBuffer, sizeof(displayBuffer), "Etat: IDLE. En attente de vos ordres.");
            break;
        case MOVING_FORWARD:
            snprintf(displayBuffer, sizeof(displayBuffer), "J'avance vers le cap %d", robot.cap);
            break;
        case MANUAL_FORWARD:
            snprintf(displayBuffer, sizeof(displayBuffer), "J'avance vers le cap %d", robot.cap);
            break;
        case MOVING_BACKWARD:
        case MANUAL_BACKWARD:
            snprintf(displayBuffer, sizeof(displayBuffer), "Je recule, attention derriere !");
            break;
        case TURNING_LEFT:
        case MANUAL_TURNING_LEFT:
            snprintf(displayBuffer, sizeof(displayBuffer), "Je tourne a gauche vers %d", robot.cap);
            break;
        case TURNING_RIGHT:
        case MANUAL_TURNING_RIGHT:
            snprintf(displayBuffer, sizeof(displayBuffer), "Je tourne a droite vers %d", robot.cap);
            break;
        case OBSTACLE_AVOIDANCE:
        case SMART_AVOIDANCE:
            snprintf(displayBuffer, sizeof(displayBuffer), "Obstacle detecte! J'analyse la situation.");
            break;
        // Add other cases as needed
        default:
            snprintf(displayBuffer, sizeof(displayBuffer), "Etat %d", robot.currentState);
            break;
    }
    
    setLcdText(robot, displayBuffer);
}

#endif // DISPLAY_H