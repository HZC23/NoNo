# Architecture Logicielle

Ce document décrit l'architecture du firmware embarqué du robot NoNo.

## Philosophie de Conception

Le firmware est conçu autour de deux principes fondamentaux :

1.  **Approche non bloquante :** Toutes les opérations, en particulier les lectures de capteurs et les délais, sont implémentées de manière asynchrone (sans utiliser `delay()`). Cela garantit que le robot reste réactif aux commandes et aux changements de l'environnement à tout moment.
2.  **Machine à États (State Machine) :** Le comportement principal du robot est géré par une machine à états. L'état actuel (par exemple, `MOVING_FORWARD`, `AVOIDING_OBSTACLE`) dicte les actions à entreprendre, ce qui rend le code modulaire et facile à maintenir.

---

## Structure des Fichiers

Le code est organisé en plusieurs fichiers d'en-tête (`.h`) pour séparer les différentes fonctionnalités.

| Fichier | Description |
| :--- | :--- |
| `NoNo.ino` | Le fichier principal. Contient les fonctions `setup()` et `loop()`. Il initialise le matériel et orchestre les tâches principales. |
| `config.h` | Fichier de configuration central. Contient les constantes, les broches (pins), les flags d'activation du matériel et les définitions de types (comme l'énumération `RobotState`). |
| `state.h` | Gère la machine à états globale du robot. Contient la variable d'état et les fonctions pour la modifier. |
| `fonctions_motrices.h` | Implémente la logique de contrôle des moteurs et la machine à états principale (`updateMotorControl`). |
| `terminal.h` | Gère la réception et l'interprétation des commandes provenant du port série (USB ou Bluetooth). |
| `telemetry.h` | Gère la construction et l'envoi des messages de télémétrie JSON vers le port série. |
| `sensor_task.h` | Contient les tâches non bloquantes pour la lecture des capteurs (ultrasons, etc.). |
| `compass.h` | Encapsule toute la logique liée au magnétomètre LSM303 : lecture, filtrage et calibration. |
| `display.h` | Gère l'affichage des informations sur l'écran LCD I2C. |
| `tourelle.h` | Contrôle les servomoteurs de la tourelle (pan/tilt) et les séquences de scan. |
| `support.h` | Fonctions utilitaires diverses. |

---

## La Machine à États (`RobotState`)

La machine à états, définie dans `config.h` et implémentée dans `fonctions_motrices.h`, est le cœur du logiciel.

| État | Description |
| :--- | :--- |
| `IDLE` | Le robot est à l'arrêt et attend des instructions. |
| `MOVING_FORWARD` / `MOVING_BACKWARD` | Déplacement en ligne droite (avant/arrière). |
| `TURNING_LEFT` / `TURNING_RIGHT` | Pivot sur place vers la gauche ou la droite. |
| `FOLLOW_HEADING` | Le robot utilise le compas pour s'orienter et avancer vers un cap cible. |
| `MAINTAIN_HEADING` | Le robot est à l'arrêt mais ajuste activement son orientation pour maintenir un cap. |
| `OBSTACLE_AVOIDANCE` | Un obstacle a été détecté, le robot s'est arrêté en attente d'une nouvelle action. |
| `AVOID_MANEUVER` | Le robot exécute une manœuvre spécifique (ex: reculer et tourner) pour éviter un obstacle. |
| `SCANNING_ENVIRONMENT` | Le robot utilise sa tourelle pour scanner les environs et trouver la voie la plus dégagée. |
| `SMART_TURNING` | Le robot pivote vers la direction jugée la plus sûre après un scan. |
| `CALIBRATING_COMPASS` | Le robot est en mode de calibration du magnétomètre. |

---

## Tâches de la Boucle Principale (`loop()`)

La fonction `loop()` dans `NoNo.ino` est très simple et agit comme un planificateur de tâches (scheduler) coopératif. Elle appelle séquentiellement les fonctions suivantes à chaque itération :

1.  **`Terminal()` :** Vérifie si de nouvelles données sont arrivées sur le port série, les lit et interprète les commandes.
2.  **`sensor_update_task()` :** Met à jour les lectures des capteurs (comme les ultrasons) si leur intervalle de lecture est écoulé.
3.  **`updateMotorControl()` :** Exécute la logique de l'état actuel de la machine à états (par exemple, ajuste la vitesse des moteurs si dans l'état `FOLLOW_HEADING`).
4.  **`updateDisplay()` :** Rafraîchit les informations affichées sur l'écran LCD.
5.  **`sendTelemetry()` :** Envoie le paquet de télémétrie JSON si l'intervalle d'envoi est écoulé.