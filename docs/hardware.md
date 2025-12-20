# Matériel du Robot NoNo

Ce document liste les composants électroniques et leur câblage pour la carte de développement ESP32-S3.

## Composants Principaux

| Composant | Modèle/Type | Rôle |
| :--- | :--- | :--- |
| **Microcontrôleur** | Freenove ESP32-S3-WROOM | Cerveau du robot, exécute le firmware. |
| **Driver Moteurs** | MX1508 | Contrôle la vitesse et la direction des deux moteurs de propulsion. |
| **Communication** | ESP32-S3 Built-in Bluetooth | Communication sans fil avec l'application et la manette. |

## Capteurs

| Capteur | Modèle | Interface | Rôle |
| :--- | :--- | :--- | :--- |
| **Centrale inertielle** | LSM303 | I2C | Fournit les données de l'accéléromètre et du magnétomètre pour le calcul du cap. |
| **Distance (Ultrasons)** | HC-SR04 | Digital | Détection d'obstacles à moyenne portée. |
| **Distance (Laser)** | VL53L1X | I2C | Mesure de distance précise à courte portée (Time-of-Flight). |
| **Détection de mouvement**| PIR Sensor | Digital | Détecte les mouvements pour le mode sentinelle. |
| **Tension Batterie** | Pont diviseur | Analog | Mesure la tension de la batterie. |

## Actionneurs et Interface

| Actionneur/Interface | Modèle/Type | Rôle |
| :--- | :--- | :--- |
| **Moteurs (x2)** | Moteur DC standard | Propulsion du robot (direction différentielle). |
| **Servomoteurs (x3)** | SG90 ou équivalent | Contrôlent les mouvements Pan/Tilt de la tourelle et la direction (Ackermann, optionnel). |
| **LEDs RGB** | Adafruit NeoPixel | Bande de LEDs pour le retour visuel. |
| **Phare** | LED haute luminosité | Éclairage avant. |
| **Buzzer** | Buzzer passif | Émission de sons et de musique. |
| **Écran LCD** | LCD I2C 16x2 | I2C | Affiche l'état, la télémétrie et les menus. |
| **Lecteur de carte** | MicroSD Card Reader | SPI | Stockage de fichiers (blagues, musique, logs). |
| **Bouton d'arrêt** | Bouton poussoir | Digital | Arrêt d'urgence ou autre interruption. |

---

## Tableau de Câblage (Pinout ESP32-S3)

| Pin ESP32-S3 | Composant | Description |
| :--- | :--- | :--- |
| **3** | Bus I2C | SDA (Serial Data) |
| **9** | Bus I2C | SCL (Serial Clock) |
| **17** | Driver Moteur A | AIN1 |
| **18** | Driver Moteur A | AIN2 |
| **8** | Driver Moteur B | BIN1 |
| **16** | Driver Moteur B | BIN2 |
| **1** | Servo Direction | (Ackermann, optionnel) |
| **2** | Servo Tourelle Pan | Horizontal (PINTOURELLE_H) |
| **4** | Servo Tourelle Tilt | Vertical (PINTOURELLE_V) |
| **38** | Capteur Ultrasons | TRIGGER |
| **39** | Capteur Ultrasons | ECHO |
| **5** | Mesure Batterie | VBAT (Entrée analogique) |
| **40** | Capteur PIR | Signal de sortie |
| **21** | Bouton Interruption | INTERUPTPIN |
| **48** | LEDs NeoPixel | NEOPIXEL_PIN |
| **42** | Buzzer | BUZZER_PIN |
| **45** | Phare | PIN_PHARE |
| **10** | Carte SD | CS (Chip Select) |
| **12** | Carte SD | SCK (Serial Clock) |
| **13** | Carte SD | MISO (Master In Slave Out) |
| **11** | Carte SD | MOSI (Master Out Slave In) |