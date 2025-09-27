# NoNo - Firmware de Robot Autonome

![Langage](https://img.shields.io/badge/langage-C++%20(Arduino)-blue.svg)
![Plateforme](https://img.shields.io/badge/plateforme-Arduino%20Mega-00979D.svg)
![Statut](https://img.shields.io/badge/statut-En%20développement-yellow.svg)

Ce dépôt contient le firmware complet pour **NoNo**, un robot mobile différentiel, autonome et contrôlable à distance, basé sur une carte Arduino Mega. Le projet met en œuvre une architecture logicielle non-bloquante et une machine à états pour gérer des comportements complexes comme la navigation au compas et l'évitement d'obstacles.

Le code de l'application Android pour contrôler le robot est disponible ici : [**Nono Controller sur GitHub**](https://github.com/HZC23/Nono_Controller)

## ✨ Fonctionnalités Principales

*   **Architecture Non-Bloquante** : Le firmware est basé sur une machine à états et des fonctions asynchrones (`millis()` au lieu de `delay()`) pour une réactivité maximale.
*   **Navigation Intelligente** :
    *   **Mode Autonome** : Exploration avec évitement d'obstacles intelligent grâce à un scan de l'environnement.
    *   **Suivi de Cap (GOTO)** : Le robot s'oriente et maintient un cap magnétique précis en utilisant un régulateur PID.
    *   **Contrôle Manuel** : Commandes de mouvement directes (avant, arrière, rotation).
*   **Fusion de Capteurs** : Utilise un capteur à ultrasons pour la détection lointaine et un capteur laser ToF (Time-of-Flight) pour des mesures de distance précises à courte portée.
*   **Tourelle Motorisée** : Une tourelle Pan/Tilt (2 axes) est utilisée pour scanner l'environnement et trouver le chemin le plus dégagé.
*   **Communication Robuste** : Contrôle et télémétrie via un port série (USB ou Bluetooth Low Energy) avec un protocole de commandes textuelles et des retours d'état au format JSON.
*   **Calibration en Direct** : Procédure de calibration du magnétomètre intégrée pour compenser les interférences magnétiques locales.
*   **Interface Embarquée** : Un écran LCD affiche l'état actuel du robot, la télémétrie et les messages.

## 🛠️ Matériel (Hardware)

| Composant | Modèle | Rôle |
| :--- | :--- | :--- |
| **Microcontrôleur** | Arduino Mega 2560 | Cerveau du robot. |
| **Driver Moteurs** | MX1508 | Contrôle des deux moteurs de propulsion. |
| **Communication** | DFRobot Bluno (BLE) | Communication sans fil avec l'application. |
| **Centrale inertielle** | LSM303 | Accéléromètre et magnétomètre pour le cap. |
| **Capteur Distance (US)**| HC-SR04 | Détection d'obstacles à moyenne portée. |
| **Capteur Distance (Laser)**| VL53L1X | Mesure de distance précise (ToF). |
| **Servomoteurs (x2)** | SG90 | Mouvements Pan/Tilt de la tourelle. |
| **Écran LCD** | LCD I2C 16x2 | Affichage de l'état et des données. |

## 🧠 Architecture Logicielle

Le firmware est conçu de manière modulaire pour une meilleure lisibilité et maintenance.

*   **`NoNo.ino`** : Point d'entrée principal, gère la `setup()` et la `loop()`.
*   **`config.h`** : Fichier de configuration central pour les broches, les constantes et les seuils.
*   **`fonctions_motrices.h`** : Cœur du robot, contient la **machine à états principale** (`updateMotorControl`) qui gère le comportement du robot (avancer, tourner, éviter, etc.).
*   **`terminal.h`** : Interprète les commandes reçues via le port série (`CMD:ACTION:VALUE`).
*   **`telemetry.h`** : Construit et envoie les paquets de données JSON contenant l'état du robot.
*   **`compass.h`** : Encapsule la logique du magnétomètre (lecture, calibration, filtrage).
*   **`sensor_task.h`** : Gère la lecture non-bloquante des capteurs.
*   **`tourelle.h`** : Classe de contrôle pour la tourelle.

## 📡 Protocole de Communication

La communication s'effectue via une liaison série (USB ou BLE) à **115200 bauds**.

### Commandes (App → Robot)
Les commandes sont envoyées au format `CMD:ACTION:VALUE\n`.
*   `CMD:MOVE:FWD` / `BWD` / `LEFT` / `RIGHT` / `STOP` : Mouvement manuel.
*   `CMD:SPEED:<0-255>` : Règle la vitesse.
*   `CMD:GOTO:<0-359>` : Active le mode de suivi de cap.
*   `CMD:MODE:AVOID` : Active le mode d'exploration autonome.
*   `CMD:CALIBRATE:COMPASS` : Lance la calibration de la boussole.
*   `CMD:SCAN:START` : Lance un scan de l'environnement avec la tourelle.
*   ... et bien d'autres.

### Télémétrie (Robot → App)
Le robot envoie périodiquement un objet JSON sur une seule ligne, terminé par `\n`.
```json
{"state":"FOLLOW_HEADING","heading":92,"distance":45,"distanceLaser":120,"battery":87,"speedTarget":150}
```

## ⚙️ Installation et Utilisation

### Prérequis
*   [Arduino IDE](https://www.arduino.cc/en/software) ou [Arduino CLI](https://arduino.github.io/arduino-cli/latest/installation/).
*   Les bibliothèques Arduino nécessaires (listées dans les `#include` de `NoNo.ino`), notamment :
    *   `Wire`, `VL53L1X`, `ArduinoJson`, `DFRobot_RGBLCD1602`.

### Téléversement
1.  Configurez les broches et les options dans `config.h` pour correspondre à votre matériel.
2.  Ouvrez `NoNo.ino` dans l'Arduino IDE.
3.  Sélectionnez le type de carte "Arduino Mega" et le port COM approprié.
4.  Téléversez le code.

### Calibration Initiale
Pour une navigation précise, la calibration du compas est **essentielle**.
1.  Connectez-vous au robot via le moniteur série (115200 bauds).
2.  Envoyez la commande `CMD:CALIBRATE:COMPASS\n`.
3.  Pendant 15 secondes, faites tourner lentement le robot sur tous ses axes (avant, arrière, côtés) pour qu'il mesure le champ magnétique environnant.
4.  Les valeurs de calibration sont automatiquement sauvegardées en EEPROM.

## 🤝 Contribution
Les contributions sont les bienvenues, que ce soit pour optimiser le code, ajouter de nouvelles fonctionnalités ou améliorer la documentation. N'hésitez pas à ouvrir une Pull Request !