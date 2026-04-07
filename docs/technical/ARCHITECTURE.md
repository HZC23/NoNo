# Robot NoNo - Architecture de Code

## Vue d'ensemble

Ce document décrit l'architecture logicielle du robot NoNo et mappe chaque composant matériel (défini dans [hardware.md](hardware.md)) vers les modules de code responsables.

### Principes architecturaux

- **Séparation des préoccupations** : Abstraction matérielle, logique métier, interfaces utilisateur
- **Non-bloquant** : Aucun `delay()`, utilise `millis()` pour timing
- **State Machine** : Comportement du robot piloté par états
- **Centralisé** : `config.h` = source unique des constantes

---

## Cartographie Composants Matériels → Code

| Composant | Catégorie | Rôle | Fichiers Responsables |
|-----------|-----------|------|----------------------|
| **Microcontrôleur ESP32-S3** | Hardware | Exécute le firmware | [NoNo.ino](../../NoNo.ino) (orchestration), [config.h](../../config.h) (pins) |
| **Driver Moteurs MX1508** | Moteurs | Contrôle vitesse/direction moteurs | [hardware.h/cpp](../../hardware.h) - `motorA`, `motorB` (FS_MX1508 library) |
| **Moteurs DC (x2)** | Moteurs | Propulsion différentielle | [hardware.cpp](../../hardware.cpp) - `updateMotorControl()` |
| **LSM303 (IMU)** | Capteurs | Accelérométrie + magnétomètre | [compass.cpp](../../compass.cpp) - `getCalibratedHeading()` |
| **HC-SR04 (Ultrasons)** | Capteurs | Détection obstacles moyen terme | [sensor_task.cpp](../../sensor_task.cpp) - ISR-based non-blocking |
| **VL53L1X (Laser ToF)** | Capteurs | Détection obstacles précis court terme | [hardware.h/cpp](../../hardware.h) - `vl53` object |
| **PIR Motion Sensor** | Capteurs | Mode sentinelle (non utilisé actuellement) | [config.h](../../config.h) - `SENTRY_MODE` state |
| **Mesure Batterie (ADC)** | Capteurs | Tension batterie | [battery_utils.cpp](../../battery_utils.cpp) - `readBatteryVoltage()` |
| **Bumper (Microswitch)** | Capteurs | Détection collisions | [hardware.cpp](../../hardware.cpp) - ISR `onBumperPress()` |
| **Moteurs Servo (x3)** | Actionneurs | Pan/Tilt tourelle + direction optionnelle | [hardware.h/cpp](../../hardware.h) - `Tourelle` class, `Servodirection` |
| **LEDs NeoPixel (x4)** | Actionneurs | Retour visuel état | [led_fx.cpp](../../led_fx.cpp) - `led_fx_update()` |
| **Phare LED** | Actionneurs | Éclairage avant | [hardware.cpp](../../hardware.cpp) - `headlightOn()`, `headlightOff()` |
| **Écran LCD I2C** | Interface | Affichage télémétrie/menu | [comms.cpp](../../comms.cpp) - `updateLcdDisplay()` |
| **Carte SD (MicroSD)** | Stockage | Configuration, blagues, logs | [sd_utils.cpp](../../sd_utils.cpp) |
| **Manette Xbox (Bluetooth)** | Interface | Contrôle manuel | [comms.h/cpp](../../comms.h) - `XboxControllerBluepad` |

---

## Architecture Logicielle en Couches

```
┌──────────────────────────────────────────────────────────────┐
│ Interface Layer (Utilisateur)                                │
│ - Serial Commands (protocol KEY:VALUE)                       │
│ - Xbox Controller Input (Bluepad32)                          │
│ - LCD Display Output                                         │
│ - JSON Telemetry (ArduinoJson)                              │
│         (comms.h/cpp)                                        │
└──────────────────────────────────────────────────────────────┘
                           ▲
                           │
┌──────────────────────────────────────────────────────────────┐
│ Logic & State Machine Layer                                  │
│ - Main State Machine (IDLE, MOVING_FORWARD, etc.)           │
│ - Obstacle Avoidance Sub-State Machine                       │
│ - Motor Control Logic (PWM + Steering)                       │
│ - Compass-based Navigation (heading maintenance)            │
│         (fonctions_motrices.cpp, robot.h struct)            │
└──────────────────────────────────────────────────────────────┘
                           ▲
                           │
┌──────────────────────────────────────────────────────────────┐
│ Hardware Abstraction Layer                                   │
│ - Motor Control (FS_MX1508)                                  │
│ - Servo Control (Tourelle class, direction servo)           │
│ - Sensor Updates (compass, ultrasonic, laser, battery)      │
│ - LED Effects (NeoPixel)                                     │
│ - Headlight Control                                          │
│         (hardware.h/cpp, compass.cpp, sensor_task.cpp,      │
│          led_fx.cpp, battery_utils.cpp)                      │
└──────────────────────────────────────────────────────────────┘
                           ▲
                           │
┌──────────────────────────────────────────────────────────────┐
│ Configuration Layer (Single Source of Truth)                │
│ - All Constants (pins, speeds, angles, thresholds)          │
│ - Enums (RobotState, ObstacleAvoidanceState)               │
│ - Battery Type Selection (LIPO vs NiMH)                     │
│         (config.h)                                           │
└──────────────────────────────────────────────────────────────┘
                           ▲
                           │
                 Physical Hardware
                (ESP32-S3, Motors, Sensors)
```

---

## Flux de Données Principal

### 1. Boucle Principale (loop cycle ~1ms)
```
loop() {
  → checkSerial()              [Interface] Commandes utilisateur
  → updateBatteryStatus()      [HAL] Lecture tension → alertes
  → sensor_update_task()       [HAL] Ultrasons/Laser non-bloquant
  → getCalibratedHeading()     [HAL] LSM303 + calcul cap
  → updateMotorControl()       [Logic] State machine → PWM moteurs
  → updateLcdDisplay()         [Interface] Affichage état
  → sendTelemetry()            [Interface] JSON output
  → updateTurret()             [HAL] Servo pan/tilt
  → xboxController.process()   [Interface] Manette Xbox
}
```

### 2. State Machine (fonctions_motrices.cpp)

Le cœur logique qui décide actions moteur/servo basé sur `robot.currentState` :

- **IDLE** → Aucun mouvement, affichage blagues
- **MOVING_FORWARD** → Propulsion avant + steering vers cap cible
- **OBSTACLE_AVOIDANCE** → Sub-state machine (scan → évaluation → manoeuvre)
- **MANUAL_COMMAND_MODE** → Commandes série direct (vitesse, virage)
- **EMERGENCY_EVASION** → Bumper déclenché, recul + pivot

---

## Modules Clés Expliqués

### **NoNo.ino** - Orchestration Principale
- **Rôle** : Initialisation matérielle, task scheduler principal
- **Clés** : `setup()` hardware init, `loop()` cooperative scheduling
- **Dépendances** : Tous les modules (coordonne l'exécution)

### **robot.h/cpp** - Conteneur d'État Central
- **Rôle** : Structure `Robot` = state container global
- **Contient** : Positions, vitesses, états moteurs, lectures capteurs, servo angles
- **Clés** : Accès thread-safe via `robotMutex` (FreeRTOS semaphore)

### **config.h** - Configuration Centralisée
- **Rôle** : Source unique de vérité pour 200+ constantes
- **Sections** : Pins, vitesses, angles, seuils, timings, états
- **Exemple** :
  ```c
  #define VITESSE_MOYENNE 200      // Speed constant
  #define NEUTRE_TOURELLE 90       // Servo neutral angle
  #define OBSTACLE_THRESHOLD_CM 20 // Avoidance trigger
  ```

### **hardware.h/cpp** - Abstraction Matérielle
- **Rôle** : Encapsule moteurs, servos, capteurs I2C
- **Objets globaux** : `motorA`, `motorB`, `tourelle`, `compass`, `vl53`, `lcd`
- **Fonctions clés** :
  - `Arret()` - Arrête tous les moteurs
  - `headlightOn()`, `headlightOff()` - Contrôle phare
  - `updateBatteryStatus()` - Gère seuils batterie
  - `scanDistances()` - Turret scanning

### **fonctions_motrices.cpp** - État Machine Logic
- **Rôle** : Cœur du comportement robot
- **Fonctions clés** :
  - `updateMotorControl(Robot&)` - Exécute logique état courant
  - `changeState(Robot&, RobotState, ObstacleAvoidanceState)` - Transition état
  - `updateSteering()` - Calcul PWM différentiel pour suivre cap
  - `scanDistances()` - Turret scan for obstacle avoidance

### **sensor_task.cpp** - Capteurs Non-Bloquants
- **Rôle** : Lectures capteurs sans jamais bloquer la boucle principale
- **Implémentation** : ISR pour ultrasons, polling `millis()`-based
- **Capteurs** : HC-SR04 (ultrasons), VL53L1X (laser)

### **compass.cpp** - Navigation par Magnétomètre
- **Rôle** : LSM303 heading + calibration
- **Fonctions clés** :
  - `getCalibratedHeading()` - Retourne cap 0-360°
  - `isCompassStuck()` - Détecte immobilité

### **comms.cpp** - Interfaces Utilisateur
- **Rôle** : Serial, LCD, Xbox, JSON telemetry
- **Fonctions clés** :
  - `processCommand()` - Parse commandes série
  - `updateLcdDisplay()` - Refresh LCD chaque loop
  - `sendTelemetry()` - JSON output ~10Hz
  - Xbox gamepad input handling

### **led_fx.cpp** - Effets Lumineux
- **Rôle** : NeoPixel LED feedback basé sur état robot
- **Mappages** : État → Couleur/Animation
- **Exemple** : OBSTACLE_AVOIDANCE → Scanner rouge

### **battery_utils.cpp** - Gestion Batterie
- **Rôle** : Lecture tension + calcul pourcentage
- **Fonctions** :
  - `readBatteryVoltage()` - Moving average filtrée
  - `readBatteryPercentage()` - Conversion Lipo/NiMH

### **support.cpp** - Legacy (Deprecated)
- **Ancien rôle** : Batterie + phare
- **Actuel** : Vide, code migré vers `battery_utils.cpp` et `hardware.cpp`

---

## Flux d'Exemple : Obstacle Avoidance

1. **Détection** → `sensor_update_task()` lit ultrasons, `robot.dusm` < seuil
2. **Trigger State** → `updateMotorControl()` voit obstacle, change état → `OBSTACLE_AVOIDANCE`
3. **Sub-State Init** → `AVOID_INIT` arrête moteurs, prépare scan
4. **Turret Scan** → `updateTurret()` pivote, `scanDistances()` teste 3 angles
5. **Évaluation** → `findWidestPath()` choisit meilleure direction
6. **Manœuvre** → Recul, pivot, avance vers gap le plus large
7. **Return** → Quand chemin clair, retour à `MOVING_FORWARD`, puis `FOLLOW_HEADING`

---

## Points d'Intégration pour Nouveaux Composants

Voir **[NEW_COMPONENT_CHECKLIST.md](NEW_COMPONENT_CHECKLIST.md)** pour procédure étape-par-étape ajouter un nouveau capteur ou actionneur.

---

## Dépendances Clés

| Librairie | Utilisation | Version |
|-----------|------------|---------|
| **FS_MX1508** | Contrôle moteurs DC | 1.1.0 |
| **LSM303** | IMU (accélérométrie + magnétomètre) | 3.0.1 |
| **VL53L1X** | Laser ToF | 1.3.1 |
| **ESP32Servo** | Contrôle servomoteurs | 3.0.9 |
| **DFRobot_RGBLCD1602** | Écran LCD I2C | 2.0.1 |
| **Adafruit_NeoPixel** | LEDs RGB | 1.15.2 |
| **ArduinoJson** | Parsing/génération JSON | 7.4.2 |
| **SdFat - Adafruit Fork** | Carte SD | 2.3.102 |
| **Bluepad32** (custom) | Xbox controller Bluetooth | custom@main / (specify commit hash or tag for reproducibility) |

---

## Considérations Temps Réel

### Budget Timing
- **Loop Target** : ~1ms par itération (1000 iterations/sec)
- **Non-bloquant** : Toutes les opérations doivent compléter en <1ms subset
- **Interrupts** : ISR pour bumper et ultrasons

### Utilisation Mémoire
- **RAM** : 320KB disponible ESP32-S3
- **PSRAM** : Non utilisé
- **Flash** : 8MB (actuellement utilisé < 50%)

### Thread Safety
- **robotMutex** : Sémaphore FreeRTOS garde accès `robot` struct
- **Lecture capteurs** : Safe car main loop est seul producteur
- **Xbox controller** : Protocole Bluetooth nonbloquant (Bluepad32)

---

## Fichiers de Documentation Associés

| Document | Contenu |
|----------|---------|
| [hardware.md](hardware.md) | Composants matériels, pinout, schémas |
| [app_guide.md](../getting_started/app_guide.md) | Guide de connexion et protocole série |
| [commands.md](../control/commands.md) | Référence des commandes série |
| [CHANGELOG.md](../maintenance/CHANGELOG.md) | Historique modifications |
| [NEW_COMPONENT_CHECKLIST.md](../maintenance/NEW_COMPONENT_CHECKLIST.md) | Procédure intégration nouveaux composants |
| [DEVELOPER_GUIDE.md](../getting_started/DEVELOPER_GUIDE.md) | Quick start pour développeurs |
| [controller_guide.md](../control/controller_guide.md) | Guide pour manette Xbox |
| [calibration.md](../maintenance/calibration.md) | Calibration du magnétomètre |
| [leds.md](leds.md) | Système de LEDs NeoPixel |
| [FREERTOS_OPTIMIZATION_GUIDE.md](FREERTOS_OPTIMIZATION_GUIDE.md) | Guide d'optimisation FreeRTOS |

---

## Améliorations Futures

- [ ] Extraire classe `BatteryManager` pour meilleure encapsulation
- [ ] Refactor `Tourelle` class en interface générique `Servo`
- [ ] Test framework pour state machine
- [ ] CI/CD pipeline (PlatformIO + GitHub Actions)
- [ ] Documentation API doxygen

---

**Dernière mise à jour** : 5 Avril 2026  
**Version Code** : Post-cleanup Phase 1
