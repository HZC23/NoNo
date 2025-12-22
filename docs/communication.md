# Protocole de Communication

Ce document décrit les protocoles de communication pour le contrôle et la télémétrie du robot NoNo, basés sur la version firmware utilisant un ESP32-S3.

## 1. Couche Physique : Bluetooth & Série

Le robot utilise deux canaux de communication principaux :

1.  **Bluetooth Low Energy (BLE) :** Le module Bluetooth intégré de l'ESP32-S3 est utilisé pour la communication sans fil. Il est géré par la bibliothèque [Bluepad32](http://gitlab.com/ricardoquesada/bluepad32), qui permet une connexion directe avec des manettes de jeu compatibles (ex: manette Xbox).
2.  **Série (USB) :** Le port USB-C de la carte ESP32-S3 est utilisé pour la programmation, le débogage et l'envoi de données de télémétrie.

---

## 2. Méthodes de Contrôle

### 2.1. Contrôle par Manette Xbox (Principal)

Le contrôle manuel principal du robot s'effectue via une manette Xbox connectée en Bluetooth. Le robot doit être dans l'état `APP_CONTROL` pour que les commandes de la manette soient actives.

#### Logique de Connexion
- La bibliothèque Bluepad32 gère automatiquement la connexion et la déconnexion des manettes.
- Le simple fait de bouger un joystick de la manette connectée fait passer le robot en mode `APP_CONTROL`.
- Si les joysticks sont relâchés, le robot repasse en mode `IDLE`.

#### Mappage des Commandes

| Contrôle | Action sur le Robot |
| :--- | :--- |
| **Joystick Gauche** | Contrôle les mouvements (marche avant/arrière et rotation). |
| **Joystick Droit** | Contrôle l'orientation de la tourelle (Pan & Tilt). |
| **Bouton A** | Allume ou éteint le phare. |
| **Bouton B** | Active ou désactive le mode d'évitement d'obstacles (`OBSTACLE_AVOIDANCE`). |
| **Bouton X** | Active ou désactive le mode sentinelle (`SENTRY_MODE`). |
| **Bouton Y** | Démarre un scan 3D de l'environnement. |
| **Gâchette Haute Droite (RB)**| Augmente la vitesse de déplacement. |
| **Gâchette Haute Gauche (LB)** | Diminue la vitesse de déplacement. |
| **Bouton "View" (Select)** | Centre la tourelle. |
| **Bouton "Menu" (Start)** | Force le robot à l'état `IDLE` et désactive le contrôle par la manette. |

### 2.2. Commandes par Application (Protocole JSON - Inactif)

Le fichier `terminal.h` contient une nouvelle implémentation (actuellement désactivée) pour un protocole de commande basé sur JSON via une liaison série (potentiellement Bluetooth Serial). Ce système est destiné à remplacer l'ancien format `CMD:*:*`.

**Format général :** `KEY:VALUE\n` ou `KEY1:VALUE1;KEY2:VALUE2\n`

| Clé | Exemple de Valeur | Description |
| :--- | :--- | :--- |
| `V` / `D` | `V:100;D:-50` | Contrôle direct des moteurs. `V` pour la vitesse (-100 à 100), `D` pour la direction (-100 à 100). |
| `HL` | `ON` / `OFF` | Contrôle du phare. |
| `M` | `AVOID`, `SENTRY`, `IDLE` | Changement de mode. |
| `S` | `200` | Réglage de la vitesse cible. |

> **Note :** Ce protocole n'est pas appelé dans la boucle principale (`loop`) et n'est donc pas fonctionnel en l'état.

---

## 3. Télémétrie (Robot → App/PC)

Le robot envoie périodiquement des données de télémétrie via la liaison série USB pour informer une application ou un développeur de son état.

**Format :** Objet JSON, sur une seule ligne, terminé par `\n`.

| Clé | Type | Description |
| :--- | :--- | :--- |
| `state` | `String` | L'état actuel de la machine à états (ex: "IDLE", "MOVING_FORWARD"). |
| `heading` | `Number` | Le cap magnétique actuel du robot, en degrés (0-359). |
| `distance`| `Number` | La distance mesurée par le capteur à ultrasons, en centimètres. |
| `distanceLaser`| `Number` | La distance mesurée par le capteur laser ToF, en centimètres. |
| `battery` | `Number` | Le niveau de la batterie, en pourcentage (0-100). |
| `speedTarget`| `Number` | La consigne de vitesse actuelle des moteurs (0-255). |

**Exemple de message JSON :**
```json
{"state":"FOLLOW_HEADING","heading":92,"distance":45,"distanceLaser":12,"battery":87,"speedTarget":150}
```
Ce message est généré par la fonction `sendTelemetry()` dans `telemetry.h`.

