# Matériel du Robot NoNo

Ce document liste les composants électroniques et leur câblage sur la carte Arduino Mega.

## Composants Principaux

| Composant | Modèle | Rôle |
| :--- | :--- | :--- |
| **Microcontrôleur** | Arduino Mega 2560 | Cerveau du robot, exécute le firmware. |
| **Driver Moteurs** | MX1508 | Contrôle la vitesse et la direction des deux moteurs de propulsion. |
| **Communication** | DFRobot Bluno | Module BLE pour la communication sans fil avec l'application. |

## Capteurs

| Capteur | Modèle | Interface | Rôle |
| :--- | :--- | :--- | :--- |
| **Centrale inertielle** | LSM303 | I2C | Fournit les données de l'accéléromètre et du magnétomètre pour le calcul du cap. |
| **Distance (Ultrasons)** | HC-SR04 | Digital | Détection d'obstacles à moyenne portée. |
| **Distance (Laser)** | VL53L1X | I2C | Mesure de distance précise à courte portée (Time-of-Flight). |
| **Détection de mouvement**| PIR Sensor | Digital | Détecte les mouvements dans l'environnement. |

## Actionneurs

| Actionneur | Modèle | Rôle |
| :--- | :--- | :--- |
| **Moteurs (x2)** | Moteur DC standard | Propulsion du robot (direction différentielle). |
| **Servomoteurs (x2)** | SG90 ou équivalent | Contrôlent les mouvements Pan/Tilt de la tourelle. |
| **Phares** | LED haute luminosité | Éclairage avant. |

## Interface Utilisateur

| Composant | Modèle | Interface | Rôle |
| :--- | :--- | :--- | :--- |
| **Écran LCD** | LCD I2C 16x2 | I2C | Affiche l'état, la télémétrie et les menus. |
| **Bouton d'arrêt** | Bouton poussoir | Digital | Arrêt d'urgence matériel. |

---

## Tableau de Câblage (Pinout)

| Pin Arduino Mega | Composant | Broche du composant |
| :--- | :--- | :--- |
| **GND** | *Multiple* | GND |
| **5V** | *Multiple* | VCC |
| **SDA (20)** | LSM303, VL53L1X, LCD | SDA |
| **SCL (21)** | LSM303, VL53L1X, LCD | SCL |
| **2** | Driver Moteur MX1508 | IN1 (Moteur Gauche) |
| **3** | Driver Moteur MX1508 | IN2 (Moteur Gauche) |
| **4** | Driver Moteur MX1508 | IN3 (Moteur Droit) |
| **5** | Driver Moteur MX1508 | IN4 (Moteur Droit) |
| **8** | Servo Tourelle Horizontal | Signal |
| **9** | Servo Tourelle Vertical | Signal |
| **36** | Capteur Ultrasons | Echo |
| **37** | Capteur Ultrasons | Trig |
| **38** | Phares (via transistor) | Base |
| **39** | Bouton d'arrêt | Signal |
| **40** | Capteur PIR | Out |

> **Note sur les LEDs de statut :** Le design original incluait des LEDs de statut sur les pins 22 (Rouge) et 24 (Jaune), mais elles sont actuellement désactivées dans `config.h`.