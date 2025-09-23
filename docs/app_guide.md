# Guide de l'Application et Protocole de Communication v2

Ce document sert de guide complet pour interagir avec le robot NoNo via une interface série (comme le moniteur série de l'Arduino IDE) ou l'application Android dédiée.

## 1. Protocole de Communication Série

La communication est basée sur des commandes textuelles et des messages de télémétrie au format JSON.

*   **Baud Rate :** `115200`
*   **Terminaison de Ligne :** Chaque commande envoyée au robot doit se terminer par un caractère de nouvelle ligne (`\n`).

### Structure des Commandes

Le format est `TYPE:ACTION:VALUE` (ex: `CMD:MOVE:FWD`). Les commandes sont insensibles à la casse.

## 2. Télémétrie du Robot

Le robot envoie périodiquement son état via des messages JSON terminés par `\n`.

**Format JSON de la Télémétrie:**
```json
{
  "state": "<RobotState_Name>",
  "heading": <current_heading>,
  "distance": <ultrasonic_distance_cm>,
  "distanceLaser": <laser_distance_cm>,
  "battery": <battery_percentage>,
  "speedTarget": <target_speed_pwm>
}
```

**Description des Champs:**
*   `state`: `string` - État actuel de la machine à états (ex: `IDLE`, `SMART_AVOIDANCE`).
*   `heading`: `int` - Cap actuel de la boussole en degrés (0-359).
*   `distance`: `int` - Distance mesurée par le capteur à ultrasons en cm.
*   `distanceLaser`: `int` - Distance mesurée par le capteur laser en cm.
*   `battery`: `int` - Pourcentage de batterie restant.
*   `speedTarget`: `int` - Vitesse cible actuelle des moteurs (0-255).

## 3. Guide de l'Application Android

L'application Android a été entièrement repensée pour offrir un contrôle complet et intuitif sur toutes les fonctionnalités du robot depuis un seul écran.

### Vue d'ensemble de l'Interface

L'application est organisée en cartes thématiques empilées verticalement.

#### 1. Barre d'État (en haut)
*   Affiche l'état de la connexion Bluetooth.
*   Permet de lancer le scan et de se connecter au robot.

#### 2. Carte "Télémétrie"
*   Affiche en temps réel les informations essentielles envoyées par le robot, correspondant aux champs JSON décrits ci-dessus.

#### 3. Carte "Contrôles Principaux"
*   **ARRÊT D'URGENCE :** Un grand bouton rouge qui envoie la commande `CMD:MOVE:STOP` pour arrêter immédiatement toute action.
*   **Interrupteur "Phares" :** Allume ou éteint les phares (`CMD:LIGHT:ON`/`OFF`).
*   **Curseur "Vitesse" :** Permet de régler la vitesse cible des moteurs (`CMD:SPEED:xxx`).

#### 4. Carte "Contrôles Manuels"
*   **D-Pad :** Des boutons directionnels (Haut, Bas, Gauche, Droite) pour un contrôle manuel direct (`CMD:MOVE:FWD`, `BWD`, `LEFT`, `RIGHT`). Le mouvement s'arrête dès que le bouton est relâché.
#### 5. Carte "Modes Autonomes"
*   **Bouton "Mode Exploration" :** Active le mode d'exploration et d'évitement d'obstacles intelligent (`CMD:MODE:AVOID`). C'est le mode principal pour laisser le robot se déplacer seul.
*   **Bouton "Aller au Cap" :** Ouvre une boîte de dialogue pour saisir un cap en degrés. Le robot se tournera puis avancera vers ce cap (`CMD:GOTO:xxx`).
*   **Bouton "Mode Sentinelle" :** Active le mode de surveillance. Le robot reste immobile et réagit aux mouvements détectés par son capteur PIR (`CMD:MODE:SENTRY`).

#### 6. Carte "Système & Calibration"
*   **Bouton "Calibrer Compas" :** Lance la procédure de calibration de la boussole (`CMD:CALIBRATE:COMPASS`).
*   **Bouton "Définir Offset Compas" :** Ouvre une boîte de dialogue pour entrer une valeur de correction manuelle pour la boussole (`CMD:COMPASS_OFFSET:xxx`).
*   **Zone de texte "Message LCD" :** Permet d'envoyer un message personnalisé qui s'affichera sur l'écran LCD du robot (`CMD:LCD:xxx`).

#### 7. Console
*   Affiche les messages bruts envoyés par le robot, utile pour le débogage.

## 4. Référence Complète des Commandes

*   `CMD:MOVE:FWD/BWD/LEFT/RIGHT/STOP` : Commandes de mouvement manuel. Place le robot en `MANUAL_CONTROL`.
*   `CMD:SPEED:<0-255>` : Définit la vitesse cible.
*   `CMD:GOTO:<0-359>` : Active le mode de suivi de cap. Place le robot en `AUTONOMOUS_CONTROL`.
*   `CMD:TURN:<angle>` : Fait tourner le robot vers un cap absolu (non utilisé dans l'app actuelle).
*   `CMD:LIGHT:ON/OFF` : Contrôle des phares.
*   `CMD:CALIBRATE:COMPASS` : Lance la calibration de la boussole.
*   `CMD:TURRET:CENTER` : Centre la tourelle.
*   `CMD:SCAN:START` : Lance un scan de l'environnement.
*   `CMD:COMPASS_OFFSET:<offset>` : Applique une correction manuelle au cap.
*   `CMD:MODE:AVOID` : Active le mode d'exploration autonome. Place le robot en `AUTONOMOUS_CONTROL`.
*   `CMD:MODE:SENTRY` : Active le mode sentinelle.
*   `CMD:MODE:MANUAL` : Force le robot en mode manuel et à l'état `IDLE`.
*   `CMD:LCD:<message>` : Affiche un message sur l'écran LCD.