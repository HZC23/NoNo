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
| `terminal.h` | Gère la réception et l'interprétation des commandes provenant du port série (USB ou Bluetooth). |
| `telemetry.h` | Gère la construction et l'envoi des messages de télémétrie JSON vers le port série. |
| `sensor_task.h` | Contient les tâches non bloquantes pour la lecture du capteur à ultrasons. |
| `compass.h` | Encapsule toute la logique liée au magnétomètre LSM303 : lecture, filtrage et calibration. |
| `display.h` | Gère l'affichage des informations sur l'écran LCD I2C. |
| `tourelle.h` | Classe de contrôle pour les servomoteurs de la tourelle (pan/tilt). |
| `support.h` | Fonctions utilitaires diverses (ex: lecture de la batterie). |
| `balises.h` | Fonctions de contrôle des LEDs de statut. |
| `fonctions.h`| Fonctions diverses, partiellement dépréciées. |

---

## La Machine à États (`RobotState`)

La machine à états, définie dans `config.h` et implémentée dans `fonctions_motrices.h`, est le cœur du logiciel.

| État | Description |
| :--- | :--- |
| `IDLE` | Le robot est à l'arrêt et attend des instructions. |
| `MOVING_FORWARD` / `MOVING_BACKWARD` | Déplacement en ligne droite (avant/arrière), inclut les états manuels. |
| `TURNING_LEFT` / `TURNING_RIGHT` | Pivot sur place vers la gauche ou la droite, inclut les états manuels. |
| `FOLLOW_HEADING` | Le robot utilise le compas pour s'orienter et avancer vers un cap cible. |
| `MAINTAIN_HEADING` | Le robot est à l'arrêt mais ajuste activement son orientation pour maintenir un cap. |
| `OBSTACLE_AVOIDANCE` | **État complexe non-bloquant.** Un obstacle a été détecté. Le robot exécute une séquence de scan (rapide et complet) avec sa tourelle pour trouver le chemin le plus dégagé, puis pivote dans cette direction avant de reprendre sa route. S'il est coincé, il recule. |
| `SCANNING` | Le robot effectue un scan complet de l'environnement avec sa tourelle et envoie les données. |
| `CALIBRATING_COMPASS` | Le robot est en mode de calibration du magnétomètre. |
| `SENTRY_MODE` / `SENTRY_ALARM` | Mode sentinelle. Le robot surveille un mouvement (capteur PIR) et déclenche une alarme visuelle. |


---

## Tâches de la Boucle Principale (`loop()`)

La fonction `loop()` dans `NoNo.ino` agit comme un planificateur de tâches (scheduler) coopératif. Elle appelle séquentiellement les fonctions suivantes à chaque itération :

1.  **`Terminal()` :** Vérifie si de nouvelles données sont arrivées sur le port série, les lit et interprète les commandes.
2.  **`sensor_update_task()` :** Met à jour la lecture du capteur à ultrasons.
3.  **Lecture des capteurs intégrés :** Lit le capteur de distance laser (VL53L1X) et le compas.
4.  **`updateMotorControl()` :** Exécute la logique de l'état actuel de la machine à états (par exemple, ajuste la vitesse des moteurs si dans l'état `FOLLOW_HEADING`).
5.  **`sendTelemetry()` :** Envoie le paquet de télémétrie JSON si l'intervalle d'envoi est écoulé.

L'affichage LCD est mis à jour directement par les fonctions concernées via `setLcdText()` lorsque l'information change, plutôt qu'à chaque boucle.

---

## Améliorations Récentes

### 1. Manœuvre d'Évitement d'Obstacle Non-Bloquante

L'ancienne logique d'évitement d'obstacles utilisait des `delay()` pour les mouvements de la tourelle, ce qui rendait le robot non réactif pendant plusieurs secondes. Cette logique a été entièrement réécrite.

- **Machine à états imbriquée :** L'état `OBSTACLE_AVOIDANCE` contient désormais sa propre machine à états (gérée par `ObstacleAvoidanceState`) pour gérer les différentes étapes de la manœuvre (scan rapide, scan complet, recul, rotation).
- **Transitions basées sur le temps :** Les `delay()` ont été remplacés par des comparaisons avec `millis()` pour les mouvements de la tourelle, ce qui permet à la boucle principale de continuer à s'exécuter sans interruption.
- **Logique de décision améliorée :** Le robot effectue d'abord un scan rapide à gauche et à droite pour les obstacles latéraux. S'il n'y a rien, il procède à un scan complet de 180 degrés pour trouver le chemin le plus long et s'y engage. S'il est complètement bloqué, il recule.

### 2. Qualité du Code et Refactoring

Un effort important a été fait pour améliorer la qualité, la lisibilité et la maintenabilité du code.

- **Centralisation des Constantes :** Des dizaines de "nombres magiques" (valeurs hardcodées) ont été extraits des fichiers de logique (`fonctions_motrices.h`, `compass.h`, etc.) et remplacés par des constantes nommées dans `config.h`. Cela inclut les timeouts, les paramètres de capteurs, les dimensions du LCD, les seuils, etc.
- **Suppression du Code Dupliqué :** La fonction `calculateHeading()` dans `compass.h` a été surchargée pour accepter différentes entrées, éliminant ainsi le code de calcul de cap redondant.
- **Amélioration de la Lisibilité :** Des logiques complexes comme la détermination des points cardinaux dans `compass.h` ont été simplifiées.
- **Suppression du Code Obsolète :** L'état `AVOID_MANEUVER` (devenu redondant) et une fonction wrapper `setLcdText` superflue ont été supprimés.
- **Consistance :** L'ensemble du code a été revu pour utiliser les nouvelles constantes et les fonctions refactorisées, garantissant un comportement plus cohérent et prévisible.
