# NoNo Robot Project

Ce dépôt contient le code source pour le firmware Arduino du robot "NoNo", ainsi que la documentation associée.

NoNo est un robot mobile différentiel, contrôlé par une carte Arduino Mega, conçu pour la navigation autonome et le contrôle à distance.

## Navigation

- **[Software](./software.md):** Une description détaillée de l'architecture logicielle, de la machine à états et des principales bibliothèques.
- **[Hardware](./hardware.md):** La liste des composants électroniques et leur câblage.
- **[Communication](./communication.md):** Le protocole de communication BLE et série.
- **[Commands](./commands.md):** La liste exhaustive des commandes série pour contrôler le robot.
- **[Calibration](./calibration.md):** Des instructions détaillées pour le calibrage du magnétomètre.

## Fonctionnalités Principales

- **Architecture non bloquante :** Le firmware est basé sur une machine à états pour un comportement réactif.
- **Modes de navigation multiples :** Contrôle manuel, suivi de cap (GOTO), et virages relatifs.
- **Capteurs :**
    - Magnétomètre/Accéléromètre (LSM303) pour l'orientation.
    - Capteur de distance à ultrasons pour l'évitement d'obstacles.
    - Capteur de distance laser (VL53L1X) pour des mesures précises.
- **Actionneurs :**
    - Deux moteurs DC avec pilote MX1508 pour la propulsion.
    - Tourelle avec deux servomoteurs (pan/tilt).
- **Interface utilisateur :**
    - Écran LCD pour afficher l'état et les données des capteurs.
    - Communication série via USB ou Bluetooth pour le contrôle et la télémétrie.

## Développement

### Prérequis

- [Arduino CLI](https://arduino.github.io/arduino-cli/latest/installation/)
- Les bibliothèques Arduino nécessaires (listées dans `NoNo.ino`)

### Compilation

Pour compiler le firmware, exécutez la commande suivante à la racine du projet :

```bash
arduino-cli compile --fqbn arduino:avr:mega
```

### Téléversement

1.  Identifiez le port de votre carte Arduino :

    ```bash
    arduino-cli board list
    ```

2.  Téléversez le code sur la carte (en remplaçant `<PORT>` par le port identifié) :

    ```bash
    arduino-cli upload -p <PORT> --fqbn arduino:avr:mega NoNo.ino
    ```
