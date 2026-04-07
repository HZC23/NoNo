# Developer Quick Start - Robot NoNo

**Objectif** : Comprendre le flow complet du code en 5 minutes.

---

## 1. Le Flux Principal (1 min)

### La boucle infinie (~1ms par itération)
```cpp
// src/NoNo.ino - void loop()
loop() {
  checkSerial()              // Reçoit commandes
  updateBatteryStatus()      // Lit batterie
  sensor_update_task()       // Lit capteurs (ultrasons/laser)
  getCalibratedHeading()     // Lit cap au magnétomètre
  updateMotorControl()       // ⭐ DÉCISION - Qu'est-ce que je fais?
  updateLcdDisplay()         // Refresh LCD
  sendTelemetry()            // JSON output
  updateTurret()             // Bouge tourelle
  xboxController.process()   // Manette Xbox
}
```

**À retenir** : La boucle lit des capteurs, prend une décision via state machine, puis commande les moteurs.

---

## 2. La State Machine - Cœur Logique (2 min)

### Où? [src/fonctions_motrices.cpp](../../fonctions_motrices.cpp) - fonction `updateMotorControl()`

### Comment?
```cpp
// Robot a un état courant (défini dans robot.h)
robot.currentState;  // ex: IDLE, MOVING_FORWARD, OBSTACLE_AVOIDANCE

// Basé sur cet état, on exécute une logique = switch/case
switch (robot.currentState) {
  case IDLE:
    // Ne rien faire
    Arret();  // Arrête moteurs
    break;

  case MOVING_FORWARD:
    // Avancer + suivre cap magnétique
    float targetHeading = robot.capCibleRotation;
    int headingError = calculateHeadingError(robot.cap, targetHeading);
    applySteeringCorrection(headingError);
    setPwm(motorA, motorB, VITESSE_MOYENNE);
    break;

  case OBSTACLE_AVOIDANCE:
    // Sub-state machine pour scan + manoeuvre
    handleObstacleAvoidanceSubStates(robot);
    break;

  case EMERGENCY_EVASION:
    // Bumper déclenché → recule + pivote
    Arret();
    backupAndPivot(robot);
    break;
    
  // ... etc 25+ autres états
}
```

### État → Actions
- **IDLE** : Stop
- **MOVING_FORWARD** : Moteurs + stéring vers cap
- **OBSTACLE_AVOIDANCE** : Turret scan → meilleur chemin → manoeuvre
- **EMERGENCY_EVASION** : Recul + pivot rapide

---

## 3. Fichiers Clés à Connaître (1 min)

| Fichier | Rôle | À savoir |
|---------|------|----------|
| [src/NoNo.ino](../../NoNo.ino) | Orchestration | `setup()` init, `loop()` scheduler |
| [src/robot.h](../../robot.h) | État global | Struct `Robot` = tout ce qu'on a besoin de savoir |
| [src/config.h](../../config.h) | **Source unique des constantes** | 200+ pins/speeds/angles/seuils |
| [src/fonctions_motrices.cpp](../../fonctions_motrices.cpp) | **State machine core** | `updateMotorControl()` = cerveau |
| [src/hardware.h](../../hardware.h), [src/hardware.cpp](../../hardware.cpp) | Moteurs/capteurs | Abstraction matérielle |
| [src/comms.cpp](../../comms.cpp) | Serial/LCD/Xbox | Interfaces utilisateur |
| [src/compass.cpp](../../compass.cpp) | Magnétomètre | `getCalibratedHeading()` |
| [src/sensor_task.cpp](../../sensor_task.cpp) | Ultrasons/Laser | Non-bloquant ISR |
| [src/comms.cpp](../../comms.cpp) | Serial/LCD/Xbox | Interfaces utilisateur |

---

## 4. Flux d'Exemple : Obstacle Droit Devant (1 min)

```
t=0ms   : Robot en MOVING_FORWARD
          sensor_update_task() lit HC-SR04 → 15cm devant
          
t=1ms   : updateMotorControl() voit obstacle
          robot.dusm = 15 < SEUIL (20cm)
          changeState(robot, OBSTACLE_AVOIDANCE, AVOID_INIT)
          
t=10ms  : État = OBSTACLE_AVOIDANCE / Sub = AVOID_INIT
          Arrête moteurs, prépare turret
          Sub-state → AVOID_QUICK_SCAN_LEFT
          
t=50ms  : Turret pivote gauche, re-lit distance → 50cm
          Bravo! Chemin libre
          Sub-state → AVOID_PERFORM_QUICK_MANEUVER 
          
t=100ms : Recule 300ms, pivote à gauche 45°
          
t=400ms : Avance → path clair, retour MOVING_FORWARD
```

**Le robot s'est adapté** sans intervention humaine!

---

## 5. Concepts Clés (1 min)

### A) Non-bloquant = Pas de delay()
```cpp
// ❌ MAUVAIS - Bloque tout pour 100ms
delay(100);

// ✅ BON - Vérifie temps, continue si pas prêt
static unsigned long lastRead = 0;
if (millis() - lastRead < 100) {
    return;  // Pas encore le moment
}
lastRead = millis();
// ... lecture capteur ...
```

### B) State Machine = Flexibilité
Au lieu de 1000 if/else imbriqués → 25 états linéaires. Ajouter nouveau comportement = nouveau case.

### C) Centralisé dans config.h
```cpp
#define VITESSE_MOYENNE 200      // Changez ici
#define TOUR_ANGLE 90            // Pas hardcoded partout
// 10x plus facile à tuner!
```

### D) Thread-safe avec Mutex
```cpp
// robot struct est protégée:
if(xSemaphoreTake(robotMutex, MUTEX_WAIT_TICKS) == pdTRUE) {
    // Lecture/écriture sûre robot.xxx
    xSemaphoreGive(robotMutex);
}
```

---

## 6. Pour Déboguer : Étapes de Base

### J'ajoute une nouvelle lumière LED
1. **Ajouter un nouvel état** : Ajouter `SCANNING_NEW_MODE` à `enum RobotState` in [config.h](../../config.h)
2. **Logique d'état** : Ajouter case dans [fonctions_motrices.cpp](../../fonctions_motrices.cpp) `updateMotorControl()`
3. **Effet LED** : Ajouter case dans [led_fx.cpp](../../led_fx.cpp) `led_fx_update()`
4. **Test** : Déclencher l'état via commande série ou automatiquement
5. **Compiler** : `pio run` depuis le terminal

### Mon capteur ne marche pas
1. **Check pins** : Vérifier [config.h](../../config.h) pin assignment vs datasheet
2. **Check init** : Exception lors `hardware_init()`? Check Serial logs
3. **Check read** : Non-bloquant? data timeout? Voir [sensor_task.cpp](../../sensor_task.cpp)
4. **Check robot struct** : Donnée apparaît dans JSON telemetry?
5. **Check logic** : État change basé sur nouvelle donnée?

### Mes moteurs vont dans la mauvaise direction
- Check direction pins dans [hardware.cpp](../../hardware.cpp) - peut inverser logique

### Vitesse trop lente/rapide
- Edit `VITESSE_MOYENNE` etc dans [config.h](../../config.h)

---

## 7. Où Chercher?

| Question | Fichier |
|----------|---------|
| Comment fonctionne l'obstacle avoidance? | [fonctions_motrices.cpp](../../fonctions_motrices.cpp) - état `OBSTACLE_AVOIDANCE` case |
| Comment fonctionne le cap magnétique? | [compass.cpp](../../compass.cpp) - `getCalibratedHeading()` |
| Comment la manette Xbox marche? | [comms.cpp](../../comms.cpp) - `processControllers()` + Bluepad32 lib |
| Où sont les constantes? | [config.h](../../config.h) - all 200+ defines |
| Qu'est-ce que chaque pin fait? | [config.h](../../config.h) (defines) + [hardware.md](hardware.md) (tableau) |

---

## 8. Architecture Visuelle (30 sec)

```
                    👤 UTILISATEUR
                         ↓
             ┌────────────┴────────────┐
             ↓                         ↓
        Serial Commands          Xbox Controller
             ↓                         ↓
         Interface Layer (comms.cpp) ←─┘
             ↓
        Robot.targetSpeed = 100
        Robot.targetHeading = 45°
             ↓
    Logic Layer (fonctions_motrices.cpp)
    State Machine: What should robot do?
             ↓
    updateMotorControl() → switch(state)
         case MOVING_FORWARD:
           - Calculate heading error
           - Apply steering correction
           - Set PWM → motorA, motorB
             ↓
    Hardware Layer (hardware.cpp)
         motorA.drive(pwm_value)
         motorB.drive(pwm_value)
             ↓
            🤖 MOTORS RUN!
```

---

## 9. Récapitulatif - 5 Points Clés

1. **Boucle = Read sensors → State machine decision → Command motors**
2. **State machine = 25+ états, switch/case, flexible**
3. **Config.h = Central source of truth**
4. **Non-bloquant = Pas de delay(), use millis()**
5. **Mutex = Sécurité d'accès robot struct**

---

## 10. Raccourcis Utiles

### Build & Upload
```bash
cd <your-project-root>              # Navigate to project directory
pio run                              # Build firmware
pio run --target upload              # Upload to device

# On Windows with full path:
cd %USERPROFILE%\Documents\PlatformIO\Projects\Nono
%USERPROFILE%\.platformio\penv\Scripts\pio.exe run

# On Linux/Mac with full path:
cd ~/.platformio/projects/Nono
~/.platformio/penv/bin/pio run
```

### Monitorer Serial (VS Code)
- PlatformIO: Monitor (button en bas)
- Ou Terminal: `pio device monitor --baud 115200`

### Debugging Tips
```cpp
// Log macro
LOG_INFO("Heading: %.1f°, Speed: %d", robot.cap, robot.targetSpeed);
LOG_DEBUG("Obstacle at %dcm", robot.dusm);
LOG_ERROR("Sensor timeout!");

// Check state
LOG_INFO("Current state: %s", stateToString(robot.currentState));

// Hardware tests
headlightOn();  // LED should light
robot.targetSpeed = 100;  // Check motors
```

---

## Questions?

Référez-vous à :
- [ARCHITECTURE.md](ARCHITECTURE.md) — Vue d'ensemble complète + tous modules
- [NEW_COMPONENT_CHECKLIST.md](NEW_COMPONENT_CHECKLIST.md) — Ajouter nouveau capteur/actionneur
- [hardware.md](hardware.md) — Pinout, schémas
- [commands.md](commands.md) — Protocole série

**Bon coding!** 🚀
