# Commandes Série

Cette page documente l'ensemble des commandes qui peuvent être envoyées au robot NoNo via une liaison série (USB ou Bluetooth).

## Format des Commandes

Toutes les commandes doivent suivre un format texte simple, se terminant par un caractère de nouvelle ligne (`\n`).

**Format :** `CMD:<ACTION>:<VALEUR>\n`

---

## Commandes de Mouvement

Ces commandes contrôlent les déplacements de base du robot.

| Commande | Action | Valeur | Description | Exemple |
| :--- | :--- | :--- | :--- | :--- |
| `CMD` | `MOVE` | `FWD` | Fait avancer le robot. | `CMD:MOVE:FWD\n` |
| `CMD` | `MOVE` | `BWD` | Fait reculer le robot. | `CMD:MOVE:BWD\n` |
| `CMD` | `MOVE` | `LEFT` | Fait pivoter le robot sur lui-même vers la gauche. | `CMD:MOVE:LEFT\n` |
| `CMD` | `MOVE` | `RIGHT` | Fait pivoter le robot sur lui-même vers la droite. | `CMD:MOVE:RIGHT\n` |
| `CMD` | `MOVE` | `STOP` | Arrête immédiatement tous les mouvements des moteurs et passe en état `IDLE`. | `CMD:MOVE:STOP\n` |

> **Note sur le mode "Suivi de Cap" (`FOLLOW_HEADING` / `MAINTAIN_HEADING`):**
> Lorsque ce mode est actif, les commandes de mouvement ont un comportement spécial :
> - **`MOVE:FWD`**: Alterne entre la reprise (`FOLLOW_HEADING`) et la pause (`MAINTAIN_HEADING`) du mouvement.
> - **`MOVE:BWD`**: Met le mouvement en pause (`MAINTAIN_HEADING`).
> - **`MOVE:LEFT` / `MOVE:RIGHT`**: Ajuste le cap cible de +/- 5 degrés sans arrêter le robot.

## Commandes de Navigation

Ces commandes sont utilisées pour des déplacements plus contrôlés.

| Commande | Action | Valeur | Description | Exemple |
| :--- | :--- | :--- | :--- | :--- |
| `CMD` | `SPEED`| `<0-255>` | Règle la vitesse cible des moteurs. `0` est l'arrêt, `255` est la vitesse maximale. | `CMD:SPEED:150\n` |
| `CMD` | `GOTO` | `<0-359>` | Active le mode "Suivi de Cap". Le robot s'oriente et avance pour suivre le cap magnétique spécifié. | `CMD:GOTO:90\n` |
| `CMD` | `TURN` | `<angle>` | Fait pivoter le robot d'un angle relatif en degrés. | `CMD:TURN:-45\n` |

## Modes Autonomes

| Commande | Action | Valeur | Description | Exemple |
| :--- | :--- | :--- | :--- | :--- |
| `CMD` | `MODE` | `AVOID` | Active le mode d'évitement d'obstacles intelligent. Le robot avance et utilise sa tourelle pour trouver un chemin si un obstacle est détecté. | `CMD:MODE:AVOID\n` |
| `CMD` | `MODE` | `SENTRY` | Active le mode sentinelle. Le robot reste immobile et déclenche une alarme visuelle s'il détecte un mouvement (capteur PIR). | `CMD:MODE:SENTRY\n` |
| `CMD` | `MODE` | `MANUAL` | Désactive les modes autonomes et retourne au contrôle manuel de base. | `CMD:MODE:MANUAL\n` |


## Commandes Utilitaires

| Commande | Action | Valeur | Description | Exemple |
| :--- | :--- | :--- | :--- | :--- |
| `CMD` | `LIGHT`| `ON` / `OFF` | Allume ou éteint le phare avant du robot. | `CMD:LIGHT:ON\n` |
| `CMD` | `CALIBRATE`| `COMPASS` | Démarre la procédure de calibration du magnétomètre. | `CMD:CALIBRATE:COMPASS\n` |

## Commandes de la Tourelle

Ces commandes contrôlent la tourelle motorisée.

| Commande | Action | Valeur | Description | Exemple |
| :--- | :--- | :--- | :--- | :--- |
| `CMD` | `TURRET`| `CENTER` | Centre la tourelle à sa position par défaut (90° horizontal, 90° vertical). | `CMD:TURRET:CENTER\n` |
| `CMD` | `SCAN` | `START` | Démarre un scan panoramique horizontal avec la tourelle pour mesurer les distances. | `CMD:SCAN:START\n` |
