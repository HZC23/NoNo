# Matériel du Robot NoNo

Ce document liste les composants électroniques et leur câblage pour la carte de développement Freenove ESP32-S3.

## Composants Principaux

| Composant | Modèle/Type | Rôle |
| :--- | :--- | :--- |
| **Microcontrôleur** | Freenove ESP32-S3-WROOM | Cerveau du robot, exécute le firmware. |
| **Driver Moteurs** | MX1508 | Contrôle la vitesse et la direction des deux moteurs de propulsion. |
| **Communication** | ESP32-S3 Built-in Bluetooth | Communication sans fil avec une manette de jeu compatible Bluepad32. |

## Capteurs

| Capteur | Modèle | Interface | Rôle |
| :--- | :--- | :--- | :--- |
| **Centrale inertielle** | LSM303 | I2C | Fournit les données de l'accéléromètre et du magnétomètre pour le calcul du cap. |
| **Distance (Ultrasons)** | HC-SR04 | Digital | Détection d'obstacles à moyenne portée. |
| **Distance (Laser)** | VL53L1X | I2C | Mesure de distance précise à courte portée (Time-of-Flight). |
| **Détection de mouvement**| PIR Sensor | Digital | Détecte les mouvements pour le mode sentinelle. |
| **Tension Batterie** | Pont diviseur | Analog | Mesure la tension de la batterie. |
| **Contacteur (Bumper)**| Bouton poussoir | Digital | Détecte les collisions frontales. |


## Actionneurs et Interface

| Actionneur/Interface | Modèle/Type | Rôle |
| :--- | :--- | :--- |
| **Moteurs (x2)** | Moteur DC standard | Propulsion du robot (direction différentielle). |
| **Servomoteurs (x3)** | SG90 ou équivalent | Contrôlent les mouvements Pan/Tilt de la tourelle et la direction (Ackermann, optionnel). |
| **LEDs RGB** | Adafruit NeoPixel (x4) | Bande de LEDs pour le retour visuel. |
| **Phare** | LED haute luminosité | Éclairage avant. |
| **Buzzer** | Buzzer passif | Émission de sons et de musique. |
| **Écran LCD** | LCD I2C 16x2 | I2C | Affiche l'état, la télémétrie et les menus. |
| **Lecteur de carte** | MicroSD Card Reader | SPI | Stockage de fichiers (configuration, blagues, etc.). |

---

## Tableau de Câblage (Pinout ESP32-S3)

Ce tableau est basé sur les constantes définies dans `src/config.h`.

| Pin ESP32-S3 | Composant | Description |
| :--- | :--- | :--- |
| **10** | Bus I2C | SDA (Serial Data) |
| **9** | Bus I2C | SCL (Serial Clock) |
| **17** | Driver Moteur A | AIN1 |
| **18** | Driver Moteur A | AIN2 |
| **8** | Driver Moteur B | BIN1 |
| **16** | Driver Moteur B | BIN2 |
| **1** | Servo Direction | (Ackermann, optionnel) |
| **4** | Servo Tourelle Pan | Horizontal (PINTOURELLE_H) |
| **2** | Servo Tourelle Tilt | Vertical (PINTOURELLE_V) |
| **6** | Capteur Ultrasons | TRIGGER |
| **7** | Capteur Ultrasons | ECHO |
| **5** | Mesure Batterie | VBAT (Entrée analogique) |
| **14** | Capteur PIR | Signal de sortie |
| **21** | Bouton Interruption | INTERUPTPIN (Bumper) |
| **48** | LEDs NeoPixel | NEOPIXEL_PIN |
| **42** | Buzzer | BUZZER_PIN |
| **45** | Phare | PIN_PHARE |
| **41** | Carte SD | CS (Chip Select) |
| **39** | Carte SD | SCK (Serial Clock) |
| **40** | Carte SD | MISO (Master In Slave Out) |
| **38** | Carte SD | MOSI (Master Out Slave In) |
