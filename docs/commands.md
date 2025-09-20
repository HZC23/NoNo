# Commandes

Cette page décrit les commandes qui peuvent être envoyées au robot NoNo via le port série.

## Format des Messages

Les commandes envoyées depuis l'application vers le robot doivent respecter un format `TYPE:ACTION:VALEUR`, terminé par un caractère de nouvelle ligne (`\n`).

**Format :** `CMD:ACTION:VALEUR`

| Type | Action | Valeur | Description |
| :--- | :--- | :--- | :--- |
| `CMD` | `MOVE` | `FWD` | Fait avancer le robot à vitesse moyenne. |
| `CMD` | `MOVE` | `BWD` | Fait reculer le robot à vitesse lente. |
| `CMD` | `MOVE` | `LEFT` | Fait pivoter le robot sur lui-même vers la gauche. |
| `CMD` | `MOVE` | `RIGHT` | Fait pivoter le robot sur lui-même vers la droite. |
| `CMD` | `MOVE` | `STOP` | Arrête tous les mouvements. |
| `CMD` | `SPEED`| `<0-255>` | Règle la vitesse cible des moteurs. |
| `CMD` | `GOTO` | `<0-359>` | Fait naviguer le robot en suivant un cap magnétique. |
| `CMD` | `TURN` | `<angle>` | Fait pivoter le robot d'un angle relatif (en degrés). |
| `CMD` | `LIGHT`| `ON` / `OFF` | Allume ou éteint le phare avant. |
| `CMD` | `CALIBRATE`| `COMPASS` | Démarre la procédure de calibration du magnétomètre. |
| `CMD` | `TURRET_H`| `<0-180>` | Règle l'angle horizontal de la tourelle. |
| `CMD` | `TURRET_V`| `<0-180>` | Règle l'angle vertical de la tourelle. |
| `CMD` | `SCAN` | `H` | Démarre un scan horizontal de la tourelle. |
| `CMD` | `SCAN` | `V` | Démarre un scan vertical de la tourelle. |
