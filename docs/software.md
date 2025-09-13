# Logiciel

Cette page décrit l'architecture logicielle du robot NoNo.

## Machine à états

Le comportement du robot est géré par une machine à états. La machine à états est implémentée dans la fonction `updateMotorControl()` dans `fonctions_motrices.h`. Les états possibles sont définis dans l'énumération `RobotState` dans `fonctions.h`:

*   `IDLE`: Le robot attend une commande.
*   `OBSTACLE_AVOIDANCE`: Le robot est en mode d'évitement d'obstacles.
*   `FOLLOW_HEADING`: Le robot navigue vers un cap spécifique.
*   `MAINTAIN_HEADING`: Le robot est arrêté mais maintient son cap.
*   `MOVING_FORWARD`: Le robot avance.
*   `MOVING_BACKWARD`: Le robot recule.
*   `MANUAL_MOVING_FORWARD`: Le robot avance d'une distance spécifique en mode manuel.
*   `MANUAL_MOVING_BACKWARD`: Le robot recule d'une distance spécifique en mode manuel.
*   `TURNING_LEFT`: Le robot tourne à gauche.
*   `TURNING_RIGHT`: Le robot tourne à droite.
*   `PIR_DETECT`: Le robot est en mode de détection PIR.
*   `MANUAL_TURNING_LEFT`: Le robot tourne à gauche en mode manuel.
*   `MANUAL_TURNING_RIGHT`: Le robot tourne à droite en mode manuel.
*   `AVOID_MANEUVER`: Le robot effectue une manœuvre d'évitement d'obstacles.
*   `SCANNING_ENVIRONMENT`: Le robot analyse l'environnement à la recherche d'obstacles.
*   `SMART_TURNING`: Le robot se tourne dans la direction avec le plus d'espace libre.

## Tâches

Le logiciel est divisé en plusieurs tâches qui s'exécutent dans la boucle principale:

*   **`Terminal()`:** Cette tâche lit le port série et exécute les commandes reçues.
*   **`sensor_update_task()`:** Cette tâche lit les capteurs (ultrasons et boussole) de manière non bloquante.
*   **`updateMotorControl()`:** Cette tâche implémente la machine à états et contrôle les moteurs.
*   **`Mcap()`:** Cette tâche calcule le cap moyen de la boussole.