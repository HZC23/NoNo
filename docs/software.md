# Architecture Logicielle

Ce document décrit l'architecture du firmware embarqué du robot NoNo.

## Philosophie de Conception

Le firmware est conçu autour de deux principes fondamentaux :

1.  **Approche non bloquante :** Toutes les opérations, en particulier les lectures de capteurs et les délais, sont implémentées de manière asynchrone (sans utiliser `delay()`). Cela garantit que le robot reste réactif aux commandes et aux changements de l'environnement à tout moment.
2.  **Machine à États (State Machine) :** Le comportement principal du robot est géré par une machine à états. L'état actuel (par exemple, `MOVING_FORWARD`, `OBSTACLE_AVOIDANCE`) dicte les actions à entreprendre, ce qui rend le code modulaire et facile à maintenir.

---

## Structure des Fichiers

Le code est organisé en plusieurs fichiers d'en-tête (`.h`) pour séparer les différentes fonctionnalités.

| Fichier | Description |
| :--- | :--- |
| `NoNo.ino` | Le fichier principal. Contient les fonctions `setup()` et `loop()`. Il initialise le matériel et orchestre les tâches principales. |
| `config.h` | Fichier de configuration central. Contient les constantes, les broches (pins), les flags d'activation du matériel et les définitions de types (comme l'énumération `RobotState`). |
| `state.h` | Définit la structure `Robot` qui contient toutes les variables d'état globales. |
| `fonctions_motrices.h` | Implémente la logique de contrôle des moteurs et la machine à états principale (`updateMotorControl`). |
| `terminal.h` | Gère la réception et l'interprétation des commandes série. |
| `telemetry.h` | Gère la construction et l'envoi des messages de télémétrie JSON vers le port série. |
| `sensor_task.h` | Contient les tâches non bloquantes pour la lecture du capteur à ultrasons. |
| `compass.h` | Encapsule toute la logique liée au magnétomètre LSM303 : lecture, filtrage et calibration. |
| `display.h` | Gère l'affichage des informations sur l'écran LCD I2C. |
| `xbox_controller_bluepad.h` | Gère l'initialisation et la lecture des commandes d'une manette Xbox via Bluepad32. |
| `tourelle.h` | Classe de contrôle pour les servomoteurs de la tourelle (pan/tilt). |
| `support.h` | Fonctions utilitaires diverses (ex: lecture de la batterie). |
| `led_fx.h` | Gère les effets visuels de la bande de LEDs NeoPixel en fonction de l'état du robot. |
| `globals.h` / `globals.cpp` | Centralise la définition et la déclaration des objets et variables globaux. |

---

## La Machine à États (`RobotState`)

La machine à états, définie dans `config.h` et implémentée dans `fonctions_motrices.h`, est le cœur du logiciel.

| État | Description |
| :--- | :--- |
| `IDLE` | Le robot est à l'arrêt et attend des instructions. |
| `MOVING_FORWARD` / `MOVING_BACKWARD` | Déplacement en ligne droite. |
| `TURNING_LEFT` / `TURNING_RIGHT` | Pivot sur place vers la gauche ou la droite. |
| `FOLLOW_HEADING` | Le robot utilise le compas pour s'orienter et avancer vers un cap cible. |
| `EMERGENCY_EVASION` | Une collision (bumper) a été détectée. Le robot exécute une manœuvre de recul et de pivot. |
| `STUCK` | Le robot a détecté un blocage physique (IMU) et tente une manœuvre de dégagement. |
| `OBSTACLE_AVOIDANCE` | **État complexe non-bloquant.** Un obstacle a été détecté. Le robot exécute une séquence de scan pour trouver le chemin le plus dégagé, puis pivote dans cette direction. |
| `CALIBRATING_COMPASS` | Le robot est en mode de calibration du magnétomètre. |
| `SENTRY_MODE` | Mode sentinelle. Le robot surveille un mouvement et déclenche une alerte visuelle. |
| `CLIFF_DETECTED` | Le robot a détecté un vide et exécute une manœuvre d'arrêt et de recul. |


---

## Tâches de la Boucle Principale (`loop()`)

La fonction `loop()` dans `NoNo.ino` agit comme un planificateur de tâches (scheduler) coopératif. Elle appelle séquentiellement les fonctions suivantes à chaque itération :

1.  **`checkSerial()` (via `terminal.h`) :** Vérifie si de nouvelles données sont arrivées sur le port série, les lit et interprète les commandes.
2.  **`updateBatteryStatus()` :** Lit la tension de la batterie et met à jour l'état de la batterie.
3.  **`sensor_update_task()` :** Met à jour les lectures des capteurs (ultrasons, laser, etc.).
4.  **`updateMotorControl()` :** Exécute la logique de l'état actuel de la machine à états (par exemple, ajuste la vitesse des moteurs si dans l'état `FOLLOW_HEADING`). Cette étape est temporairement sautée si une commande manuelle `M:v,t` vient d'être reçue.
5.  **`updateLcdDisplay()` & `displayJokesIfIdle()`**: Met à jour l'écran LCD avec l'état actuel ou des blagues si le robot est inactif.
6.  **`sendTelemetry()` :** Envoie le paquet de télémétrie JSON si l'intervalle d'envoi est écoulé.

---

## Améliorations Récentes

### 1. Protocole de Commande

L'ancien protocole de commande à plusieurs niveaux (ex: `CMD:MOVE=...`) a été **refactorisé** en un format `CLÉ:VALEUR` plus simple et plus court (ex: `M:100,50`). Ce changement a été implémenté dans `terminal.cpp` et est documenté dans `app_guide.md`.

### 2. Correction de Bugs Critiques

Un effort important a été fait pour corriger plusieurs bugs qui empêchaient le robot de fonctionner correctement :
- **Erreurs de Linker :** Correction des erreurs de "multiple definition" en centralisant les variables globales.
- **Stabilité de Mouvement :** Correction de bugs qui causaient des arrêts intempestifs (détection d'impact trop sensible) et l'incapacité de bouger dans les modes autonomes (mauvaise variable de vitesse utilisée).
- **Précision des Capteurs :** Correction de la formule de lecture de la tension de la batterie.

### 3. Manœuvre d'Évitement d'Obstacle Non-Bloquante

L'ancienne logique d'évitement d'obstacles utilisait des `delay()` pour les mouvements de la tourelle, ce qui rendait le robot non réactif. Cette logique a été entièrement réécrite en une machine à états imbriquée non-bloquante, basée sur `millis()`, pour une réactivité maximale.

### 4. Qualité du Code et Refactoring

- **Centralisation des Constantes :** Des "nombres magiques" ont été extraits et remplacés par des constantes nommées dans `config.h`.
- **Suppression du Code Obsolète :** D'anciennes fonctions et logiques redondantes ont été nettoyées.