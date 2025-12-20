# NoNo - Firmware de Robot Autonome

![Langage](https://img.shields.io/badge/langage-C++%20(Arduino%20Framework)-blue.svg)
![Plateforme](https://img.shields.io/badge/plateforme-ESP32--S3-orange.svg)
![Statut](https://img.shields.io/badge/statut-En%20développement-yellow.svg)

Ce dépôt contient le firmware complet pour **NoNo**, un robot mobile différentiel, autonome et contrôlable à distance, basé sur une carte ESP32-S3. Le projet met en œuvre une architecture logicielle non-bloquante, une machine à états et le framework Bluepad32 pour un contrôle via manette de jeu.

Le code de l'application Android pour contrôler le robot est disponible ici : [**Nono Controller sur GitHub**](https://github.com/HZC23/Nono_Controller)

## ✨ Fonctionnalités Principales

*   **Plateforme Moderne** : Utilise un microcontrôleur ESP32-S3 puissant avec Bluetooth et Wi-Fi intégrés.
*   **Architecture Non-Bloquante** : Le firmware est basé sur une machine à états et des fonctions asynchrones (`millis()`) pour une réactivité maximale.
*   **Contrôle via Manette de Jeu** : Intégration transparente avec les manettes de jeu (ex: Xbox) via la bibliothèque Bluepad32 pour un contrôle manuel intuitif.
*   **Navigation Intelligente** :
    *   **Mode Autonome** : Exploration avec évitement d'obstacles intelligent grâce à un scan de l'environnement par tourelle.
    *   **Suivi de Cap (GOTO)** : Le robot peut s'orienter et maintenir un cap magnétique précis.
*   **Fusion de Capteurs** : Utilise un capteur à ultrasons et un capteur laser ToF (Time-of-Flight) pour la détection d'obstacles.
*   **Tourelle Motorisée** : Une tourelle Pan/Tilt (2 axes) est utilisée pour scanner l'environnement.
*   **Télémétrie JSON** : Le robot envoie son état et les données des capteurs via le port série au format JSON pour le débogage ou l'intégration avec des applications externes.
*   **Interface Embarquée** : Un écran LCD affiche l'état actuel du robot, et une bande de LEDs NeoPixel donne un retour visuel coloré.

## 🛠️ Matériel (Hardware)

Le robot est basé sur une carte de développement **Freenove ESP32-S3-WROOM**.

| Composant | Rôle |
| :--- | :--- |
| **Driver Moteurs** | MX1508 |
| **Centrale inertielle** | LSM303 (Accéléromètre + Magnétomètre) |
| **Capteurs Distance** | HC-SR04 (Ultrasons) & VL53L1X (Laser ToF) |
| **Servomoteurs** | Pan/Tilt de la tourelle. |
| **Interface**| Écran LCD I2C, LEDs NeoPixel, Buzzer, Lecteur de carte SD. |

Pour le détail complet du câblage, consultez le document : [**`docs/hardware.md`**](./docs/hardware.md).

## 🧠 Architecture Logicielle

Le firmware est conçu de manière modulaire pour une meilleure lisibilité et maintenance.

*   **`NoNo.ino`** : Point d'entrée principal (`setup()` et `loop()`).
*   **`config.h`** : Fichier de configuration central pour les broches, constantes et seuils.
*   **`state.h`** : Définit la structure `Robot` qui contient l'état global.
*   **`fonctions_motrices.h`** : Cœur du robot, contient la machine à états principale (`updateMotorControl`).
*   **`xbox_controller_bluepad.h`** : Gère l'initialisation et la lecture des commandes de la manette.
*   **`telemetry.h`** : Construit et envoie les paquets de données JSON.
*   **`led_fx.h`** : Gère les effets visuels des LEDs NeoPixel.

Pour une description détaillée de l'architecture, consultez le document : [**`docs/software.md`**](./docs/software.md).

## 📡 Protocole de Communication

La communication et le contrôle se font principalement via deux méthodes.

### 1. Contrôle par Manette de Jeu
Le contrôle principal est assuré par une manette de type Xbox. Les actions (mouvement, tourelle, changement de mode) sont mappées aux joysticks et aux boutons.

### 2. Télémétrie
Le robot envoie périodiquement un objet JSON sur le port série USB à **115200 bauds**.
```json
{"state":"IDLE","heading":92,"distance":45,"distanceLaser":12,"battery":87,"speedTarget":150}
```

Pour les détails du protocole et le mappage de la manette, consultez le document : [**`docs/communication.md`**](./docs/communication.md).

## ⚙️ Installation et Utilisation

### Prérequis
*   [Visual Studio Code](https://code.visualstudio.com/)
*   L'extension [PlatformIO IDE](https://platformio.org/platformio-ide).

### Compilation et Téléversement
1.  Ouvrez ce projet dans Visual Studio Code.
2.  L'extension PlatformIO devrait vous proposer d'installer les bibliothèques et plateformes nécessaires.
3.  Utilisez les commandes de PlatformIO pour compiler (`Build`) et téléverser (`Upload`) le firmware sur la carte ESP32-S3.

## 🤝 Contribution
Les contributions sont les bienvenues. N'hésitez pas à ouvrir une Pull Request pour proposer des améliorations.