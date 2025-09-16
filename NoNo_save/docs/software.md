# Logiciel

Cette page décrit l'architecture logicielle du robot NoNo.

## Machine à états

Le comportement du robot est géré par une machine à états. Les états possibles sont définis dans l'énumération `RobotState` dans `fonctions.h`:

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
*   `AVOID_MANEUVER`: Le robot effectue une manœuvre d'évitement d'obstacles.
*   `SCANNING_ENVIRONMENT`: Le robot analyse l'environnement à la recherche d'obstacles.
*   `SMART_TURNING`: Le robot se tourne dans la direction avec le plus d'espace libre.

## Tâches

Le logiciel est structuré pour gérer différentes tâches, notamment la mise à jour des capteurs, le contrôle des moteurs et la gestion de l'interface utilisateur. Les tâches sont conçues pour être non-bloquantes afin d'assurer une réactivité optimale du robot.

## Initialisation

Lors du démarrage, le système initialise les composants matériels, configure les capteurs et prépare les moteurs pour le contrôle. Les états initiaux sont définis, et le robot est prêt à recevoir des commandes.

## Gestion des capteurs

Les capteurs sont gérés par des tâches dédiées qui effectuent des mises à jour non-bloquantes. Cela permet au robot de réagir rapidement aux changements dans son environnement.

## Conclusion

L'architecture logicielle du robot NoNo est conçue pour être modulaire et extensible, facilitant l'ajout de nouvelles fonctionnalités et l'amélioration des performances.