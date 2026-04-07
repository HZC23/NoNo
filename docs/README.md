# Documentation du Robot NoNo

Bienvenue dans la documentation technique du firmware NoNo (ESP32-S3). Ce dossier contient toutes les informations nécessaires pour comprendre, configurer, contrôler et étendre les capacités du robot.

## 🚀 Guides de démarrage
- **[DEVELOPER_GUIDE.md](./getting_started/DEVELOPER_GUIDE.md) :** Comprendre le flux du code et la machine à états en 5 minutes.
- **[App Guide](./getting_started/app_guide.md) :** Guide de connexion série et protocole de communication (Anglais).

## 🏗️ Architecture et Technique
- **[Architecture](./technical/ARCHITECTURE.md) :** Vue d'ensemble de l'architecture logicielle, mapping composants/code et flux de données.
- **[Hardware](./technical/hardware.md) :** Liste des composants électroniques, branchements et pinout de l'ESP32-S3.
- **[Système de LEDs](./technical/leds.md) :** Détails des effets visuels NeoPixel selon l'état du robot.
- **[FreeRTOS Optimization](./technical/FREERTOS_OPTIMIZATION_GUIDE.md) :** Guide sur l'architecture multi-tâches et l'utilisation des mutex.

## 🎮 Contrôle et Communication
- **[Commandes Série](./control/commands.md) :** Référence complète des commandes `KEY:VALUE` (Français).
- **[Controller Guide](./control/controller_guide.md) :** Guide de configuration et d'utilisation d'une manette Xbox sans fil.

## 🛠️ Maintenance et Évolution
- **[Calibration](./maintenance/calibration.md) :** Procédure détaillée pour calibrer le magnétomètre (boussole).
- **[New Component Checklist](./maintenance/NEW_COMPONENT_CHECKLIST.md) :** Guide étape par étape pour ajouter un nouveau capteur ou actionneur.
- **[Changelog](./maintenance/CHANGELOG.md) :** Historique des modifications notables du firmware.

---

## Fonctionnalités Principales

- **Architecture non-bloquante :** Basée sur une machine à états réactive (aucun `delay()`).
- **Modes de navigation :** Contrôle manuel, suivi de cap (GOTO) et évitement d'obstacles autonome.
- **Capteurs :** Magnétomètre (LSM303), Ultrasons (HC-SR04), Laser ToF (VL53L1X), Batterie, Bumper.
- **Actionneurs :** Moteurs DC (propulsion différentielle), Tourelle Pan/Tilt (2 servos).
- **Interfaces :** Écran LCD I2C, Télémétrie JSON via USB, Bluetooth Bluepad32.
