# New Component Integration Checklist

Processus étape-par-étape pour intégrer un nouveau capteur ou actionneur au robot NoNo.

> **Exemple concret** : Intégration d'un capteur de distance infini (10km+) appelé "LiDAR360"

---

## Phase 1 : Planification & Recherche

- [ ] **Identifier le capteur/actionneur** (modèle exact, datasheet)
- [ ] **Déterminer interface de communication** (I2C, SPI, Digital, Analog, Serial)
- [ ] **Vérifier compatibilité ESP32-S3** (tension, pins suffisantes)
- [ ] **Trouver/écrire driver** (librairie Arduino ou custom)
- [ ] **Documenter spécifications** (résolution, range, timing, consommation)

**Checklist pour exemple LiDAR360** :
- Modèle : SICK LMS511S
- Interface : Ethernet Serial RS-422
- Pins ESP32-S3 : UART2 (TX=17, RX=18) — **CONFLIT** avec motor A pins! Need alternative pins

---

## Phase 2 : Pinout & Configuration

### Step 1 : Réserver les Pins
- [ ] **Consulter [hardware.md](hardware.md)** pinout table
- [ ] **Vérifier pins disponibles** dans [config.h](../config.h)
- [ ] **Assigner pins** (noter conflits potentiels)

**Pour LiDAR360** :
```c
// config.h - Ajouter
#define LIDAR_SERIAL_PORT   1         // UART1 (non utilisé)
#define LIDAR_RX_PIN        10        // Free pin
#define LIDAR_TX_PIN        9         // Free pin
#define LIDAR_BAUD_RATE     500000    // 500kbps
```

### Step 2 : Ajouter Constantes
- [ ] **Puissance/consommation** si applicable
- [ ] **Seuils/calibration** (ex: distance minimale détectable)
- [ ] **Timing** (ex: fréquence de scan, timeout)
- [ ] **Calibration** (offsets, facteurs d'échelle)

**Pour LiDAR360** :
```c
// config.h
#define LIDAR_MIN_RANGE_M            0.5f      // 50cm minimum
#define LIDAR_MAX_RANGE_M            10.0f     // 10m maximum
#define LIDAR_SCAN_INTERVAL_MS       50        // 20Hz scan rate
#define LIDAR_TIMEOUT_MS             200       // Data timeout
#define LIDAR_DATA_BUFFER_SIZE       361       // 360 degrees + 1
```

---

## Phase 3 : Hardware Abstraction Layer (hardware.h/cpp)

### Step 1 : Déclarer l'objet global
- [ ] **Créer objet peripheral** dans [hardware.h](../hardware.h)
- [ ] **Déclarer extern** si librairie fournie la classe

**Pour LiDAR360** :
```cpp
// hardware.h
#include <SICKROS2Driver.h>  // Librairie hypothétique

extern SICKROS2Driver lidar360;
```

### Step 2 : Initialiser le hardware
- [ ] **Créer fonction init** : `void lidar360_init()`
- [ ] **Appeler depuis setup()** dans [NoNo.ino](../NoNo.ino)
- [ ] **Gérer erreurs d'init** (fallback gracieux)

**Pour LiDAR360** :
```cpp
// hardware.cpp
SICKROS2Driver lidar360;

void lidar360_init() {
    Serial1.begin(LIDAR_BAUD_RATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    if (!lidar360.begin(&Serial1)) {
        LOG_ERROR("LiDAR360 failed to initialize!");
        robot.lidar360_ready = false;
        return;
    }
    lidar360.setAngleRes(0.5f);  // 0.5 degree resolution
    robot.lidar360_ready = true;
    LOG_INFO("LiDAR360 initialized successfully");
}
```

### Step 3 : Créer fonction de lecture
- [ ] **Non-bloquante** : Utiliser `millis()` ou state machine
- [ ] **Gestion d'erreurs** : Timeouts, validations CRC
- [ ] **Stocker résultat** dans `robot` struct (Step 4)

**Pour LiDAR360** :
```cpp
// hardware.cpp
void lidar360_update(Robot& robot) {
    static unsigned long last_scan = 0;
    
    // Non-blocking read
    if (millis() - last_scan < LIDAR_SCAN_INTERVAL_MS) {
        return;  // Not yet time for next scan
    }
    
    if (!robot.lidar360_ready) return;
    
    if (lidar360.dataAvailable()) {
        // Read up to 361 distance measurements (0-360 degrees)
        for (int i = 0; i < LIDAR_DATA_BUFFER_SIZE; i++) {
            robot.lidar360_distances[i] = lidar360.getDistance(i);
        }
        robot.lidar360_timestamp = millis();
        last_scan = millis();
    } else if (millis() - robot.lidar360_timestamp > LIDAR_TIMEOUT_MS) {
        LOG_WARN("LiDAR360 timeout - no data received");
        robot.lidar360_ready = false;
    }
}
```

### Step 4 : Ajouter prototype à hardware.h
- [ ] **Déclarer fonction publique** si appelée de l'extérieur

**Pour LiDAR360** :
```cpp
// hardware.h
void lidar360_init();
void lidar360_update(Robot& robot);
```

---

## Phase 4 : Robot State Extension (robot.h/cpp)

### Step 1 : Ajouter champs au struct Robot
- [ ] **Lecture capteur** (ex: `int lidar360_distances[361]`)
- [ ] **État** (ex: `bool lidar360_ready`)
- [ ] **Timestamp** (pour timeouts)

**Pour LiDAR360** :
```cpp
// robot.h - Dans struct Robot
struct Robot {
    // ... existing fields ...
    
    // LiDAR360 Support
    bool lidar360_ready = false;
    int lidar360_distances[LIDAR_DATA_BUFFER_SIZE];  // Distance per angle
    unsigned long lidar360_timestamp = 0;
    int lidar360_closest_angle = 0;      // Angle to closest object
    int lidar360_closest_distance_cm = 0; // Distance to closest object
};
```

### Step 2 : Initialiser les nouvelles valeurs
- [ ] **Dans initializeRobotDefaults()** de [NoNo.ino](../NoNo.ino)

**Pour LiDAR360** :
```cpp
// NoNo.ino - fonction initializeRobotDefaults()
void initializeRobotDefaults(Robot& robot) {
    // ... existing init code ...
    
    robot.lidar360_ready = false;
    memset(robot.lidar360_distances, 0, sizeof(robot.lidar360_distances));
    robot.lidar360_timestamp = 0;
    robot.lidar360_closest_angle = -1;
    robot.lidar360_closest_distance_cm = 999999;
}
```

---

## Phase 5 : State Machine Logic (fonctions_motrices.cpp)

### Step 1 : Utiliser le capteur dans la logique
- [ ] **Lire les données** du capteur via `robot` struct
- [ ] **Prendre décisions** basées sur capteur
- [ ] **Trigger actions** (changement d'état, alerte)

**Pour LiDAR360** : Exemple - nouvel état **SCANNING_ROOM**
```cpp
// fonctions_motrices.cpp
case SCANNING_ROOM: {
    // 360-degree room scan using LiDAR
    
    // Find closest object
    int closest_dist = 999999;
    int closest_angle = -1;
    
    for (int angle = 0; angle <= 360; angle++) {
        if (robot.lidar360_distances[angle] < closest_dist && 
            robot.lidar360_distances[angle] > 0) {
            closest_dist = robot.lidar360_distances[angle];
            closest_angle = angle;
        }
    }
    
    robot.lidar360_closest_angle = closest_angle;
    robot.lidar360_closest_distance_cm = closest_dist / 10;  // Convert mm to cm
    
    // Create room map visualization
    LOG_INFO("LiDAR: Closest object at %d° = %dcm", 
             closest_angle, robot.lidar360_closest_distance_cm);
    
    // Transition to next state after scan
    changeState(robot, IDLE, AVOID_IDLE);
    break;
}
```

### Step 2 : Intégrer appel de mise à jour
- [ ] **Appeler fonction update** dans loop principal ou updateMotorControl()

**Pour LiDAR360** :
```cpp
// NoNo.ino - dans loop()
if (robot.currentState == SCANNING_ROOM) {
    lidar360_update(robot);
}
```

---

## Phase 6 : Interface Layer (comms.cpp)

### Step 1 : Ajouter telemetry output
- [ ] **Ajouter champs JSON** pour nouveau capteur si applicable
- [ ] **Formatter données** pour lisibilité

**Pour LiDAR360** :
```cpp
// comms.cpp - fonction sendTelemetry()
if (robot.lidar360_ready) {
    JsonArray lidar_array = doc.createNestedArray("lidar360");
    // Sample every 10 degrees to reduce JSON size
    for (int angle = 0; angle < 360; angle += 10) {
        lidar_array.add(robot.lidar360_distances[angle]);
    }
    doc["lidar_closest_angle"] = robot.lidar360_closest_angle;
    doc["lidar_closest_dist_cm"] = robot.lidar360_closest_distance_cm;
}
```

### Step 2 : Ajouter commandes série si applicable
- [ ] **Ajouter cases** dans `processCommand()` pour teste/contrôle
- [ ] **Documenter commandes** dans [commands.md](commands.md)

**Pour LiDAR360** :
```cpp
// comms.cpp - processCommand()
else if (strncmp(cmd, "LIDAR_SCAN", 10) == 0) {
    changeState(robot, SCANNING_ROOM, AVOID_IDLE);
    LOG_INFO("LiDAR scan initiated");
}
```

---

## Phase 7 : LED Feedback (led_fx.cpp) [Optionnel]

### Step 1 : Ajouter état visuel si pertinent
- [ ] **Mapper nouveau state → couleur/effet LED**
- [ ] **Indiquer état capteur** (actif, erreur, idle)

**Pour LiDAR360** :
```cpp
// led_fx.cpp - fonction led_fx_update()
case SCANNING_ROOM:
    // Rotating cyan effect during scan
    effect_scanner(COLOR_CYAN, 50);
    break;
```

---

## Phase 8 : Testing & Validation

### Step 1 : Tests unitaires basiques
- [ ] **Test init** : Capteur initialise sans crash
- [ ] **Test read** : Données valides reçues
- [ ] **Test timeout** : Graceful handle si pas de données
- [ ] **Test state transition** : Nouveau state fonctionne

**Pour LiDAR360** :
```cpp
void test_lidar360() {
    LOG_INFO("Testing LiDAR360...");
    
    lidar360_init();
    delay(100);
    
    for (int i = 0; i < 10; i++) {
        lidar360_update(robot);
        delay(100);
        if (robot.lidar360_closest_distance_cm < 999999) {
            LOG_INFO("✓ LiDAR data received: %dcm @ %d°",
                     robot.lidar360_closest_distance_cm,
                     robot.lidar360_closest_angle);
            break;
        }
    }
    
    if (!robot.lidar360_ready) {
        LOG_ERROR("✗ LiDAR not responding");
        return;
    }
}
```

### Step 2 : Integration test
- [ ] **Compiler sans erreurs**
- [ ] **Test en comportement autonome**
- [ ] **Vérifier pas d'interference** avec autres capteurs
- [ ] **Profile RAM/CPU usage**

---

## Phase 9 : Documentation

### Step 1 : Mettre à jour docs
- [ ] **Ajouter composant** à [hardware.md](hardware.md) tableau
- [ ] **Documenter pins** dans pinout
- [ ] **Ajouter constantes** à [config.h](../config.h) section comments
- [ ] **Ajouter ligne** dans ARCHITECTURE.md component mapping

**Pour LiDAR360** :
```markdown
# hardware.md - Update table

| LiDAR360 | SICK LMS511S | Serial RS-422 | 360° room mapping |
```

### Step 2 : Documenter commandes (si applicable)
- [ ] **Ajouter commandes** à [commands.md](commands.md)
- [ ] **Exemples usage**
- [ ] **Format de telemetry**

---

## Checklist Finale

- [ ] ✅ Toutes les phases complétées
- [ ] ✅ Code compile sans warnings
- [ ] ✅ Capteur teste & valide
- [ ] ✅ Documentation mise à jour
- [ ] ✅ Git commit avec message clair
- [ ] ✅ Code review (optional mais recommandé)

---

## Troubleshooting Commun

### Conflit de pins
**Problème** : Pin déjà utilisé par autre composant
**Solution** : Consulter [hardware.md](hardware.md), trouver pin libre ou dessouder/reconfigurer

### I2C/SPI address collision
**Problème** : Même adresse que autre périphérique I2C
**Solution** : Vérifier datasheet, voir si adresse configurable, ou utiliser autre bus

### Timeout capteur après init
**Problème** : Capteur ne répond pas après initialization
**Solution** : Vérifier voltages, connections, baudrate, drivers installés

### RAM overflow
**Problème** : Compilation fails, ROM trop grande
**Solution** : Réduire buffer `/361 distances` → seulement angles intéressants, ou compresser format

---

## Template Code Snippet

Voici un template generic pour nouveau capteur :

```cpp
// === config.h ===
#define MY_SENSOR_PIN          XX
#define MY_SENSOR_INTERVAL_MS  50
#define MY_SENSOR_TIMEOUT_MS   200

// === hardware.h ===
void my_sensor_init();
void my_sensor_update(Robot& robot);

// === hardware.cpp ===
void my_sensor_init() {
    LOG_INFO("Initializing MY_SENSOR...");
    // Init code
    robot.my_sensor_ready = true;
}

void my_sensor_update(Robot& robot) {
    static unsigned long last_update = 0;
    if (millis() - last_update < MY_SENSOR_INTERVAL_MS) return;
    
    if (!robot.my_sensor_ready) return;
    
    // Update sensor data
    robot.my_sensor_value = read_new_value();
    last_update = millis();
}

// === robot.h ===
// In struct Robot:
bool my_sensor_ready = false;
int my_sensor_value = 0;

// === NoNo.ino ===
// In setup():
my_sensor_init();

// In loop():
my_sensor_update(robot);
```

---

**Pour questions** : Consultez [ARCHITECTURE.md](ARCHITECTURE.md) ou demandez code review auprès de mainteneurs.
