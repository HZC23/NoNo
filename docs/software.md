# Logiciel

Cette page décrit l'architecture logicielle du robot NoNo.

## Machine à états

Le comportement du robot est géré par une machine à états. La machine à états est implémentée dans la fonction `updateMotorControl()` dans `fonctions_motrices.h`. Les états possibles sont définis dans l'énumération `RobotState` dans `config.h`:

*   `IDLE`: Le robot attend une commande.
*   `MOVING_FORWARD`: Le robot avance.
*   `MOVING_BACKWARD`: Le robot recule.
*   `TURNING_LEFT`: Le robot tourne à gauche.
*   `TURNING_RIGHT`: Le robot tourne à droite.
*   `MANUAL_FORWARD`: Le robot avance en mode manuel.
*   `MANUAL_BACKWARD`: Le robot recule en mode manuel.
*   `MANUAL_TURNING_LEFT`: Le robot tourne à gauche en mode manuel.
*   `MANUAL_TURNING_RIGHT`: Le robot tourne à droite en mode manuel.
*   `OBSTACLE_AVOIDANCE`: Le robot est en mode d'évitement d'obstacles.
*   `FOLLOW_HEADING`: Le robot navigue vers un cap spécifique.
*   `MAINTAIN_HEADING`: Le robot est arrêté mais maintient son cap.
*   `PIR_DETECT`: Le robot est en mode de détection PIR.
*   `AVOID_MANEUVER`: Le robot effectue une manœuvre d'évitement d'obstacles.
*   `SCANNING_ENVIRONMENT`: Le robot analyse l'environnement à la recherche d'obstacles.
*   `SMART_TURNING`: Le robot se tourne dans la direction avec le plus d'espace libre.
*   `CALIBRATING_COMPASS`: Le robot est en mode de calibration du compas.

## Tâches

Le logiciel est divisé en plusieurs tâches qui s'exécutent dans la boucle principale:

*   **`Terminal()`:** Cette tâche lit le port série et exécute les commandes reçues.
*   **`sensor_update_task()`:** Cette tâche lit les capteurs (ultrasons, etc.) de manière non bloquante.
*   **`getCalibratedHeading()`:** Le cap du compas est lu directement via cette fonction à chaque boucle.
*   **`updateMotorControl()`:** Cette tâche implémente la machine à états et contrôle les moteurs.
