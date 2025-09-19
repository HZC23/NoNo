# Protocole de Communication Arduino ↔ Application Android

Ce document décrit le fonctionnement de la communication entre le robot NoNo (code Arduino) et son application de contrôle (NonoApp sur Android).

## 1. Technologie Utilisée

La communication est basée sur le **Bluetooth Low Energy (BLE)**.

Le module BLE utilisé sur le robot (probablement une carte DFRobot Bluno ou équivalent) fonctionne comme un **pont série transparent (Transparent Serial Bridge)**. Cela signifie que le module relaie simplement tout ce qui est envoyé sur le port série de l'Arduino vers l'application, et vice-versa.

## 2. Format des Messages

### Commandes (App → Arduino)

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

**Exemple de code Arduino pour la réception :**
```cpp
// Dans terminal.h
String command = Serial.readStringUntil('\n');
command.trim();
// ... parsing de la commande "TYPE:ACTION:VALEUR" ...
if (type.equalsIgnoreCase("CMD")) {
    if (action.equalsIgnoreCase("MOVE")) {
        if (value.equalsIgnoreCase("FWD")) {
            // ...
        }
    }
}
```

### Télémétrie (Arduino → App)

La télémétrie est envoyée par le robot à intervalle régulier sous la forme d'un objet JSON, terminé par un caractère de nouvelle ligne (`\n`).

**Format :** Objet JSON

| Clé | Type de Donnée | Description |
| :--- | :--- | :--- |
| `state` | `String` | État actuel de la machine à états du robot (ex: "IDLE", "MOVING_FORWARD"). |
| `heading` | `Number` | Cap magnétique actuel du robot (en degrés). |
| `distance`| `Number` | Distance mesurée par le capteur à ultrasons (en cm). |
| `battery` | `Number` | Pourcentage de batterie estimé. |
| `speedTarget`| `Number` | Vitesse cible actuelle des moteurs (0-255). |
| `speedCurrent`| `Number` | Vitesse réelle moyenne des moteurs (0-255). |

**Exemple de message JSON envoyé par le robot :**
```json
{"state":"FOLLOW_HEADING","heading":92,"distance":45,"battery":87,"speedTarget":150,"speedCurrent":148}
```

**Exemple de code Arduino pour l'envoi :**
```cpp
// Dans telemetry.h
StaticJsonDocument<200> doc;
doc["state"] = "FOLLOW_HEADING";
doc["heading"] = robot.cap;
// ... peupler le reste du document ...
serializeJson(doc, Serial);
Serial.println();
```