# Protocole de Communication

Ce document décrit le protocole de communication entre le robot NoNo (Arduino) and the Nono Controller Android application.

## 1. Couche Physique : Bluetooth Low Energy (BLE)

La communication est établie via Bluetooth Low Energy. Le module BLE sur le robot (un DFRobot Bluno) est configuré pour fonctionner comme un pont série transparent.

- **Découverte :** L'application scanne les appareils BLE qui exposent le service spécifique de Bluno.
- **Connexion :** L'application se connecte au service GATT du robot pour accéder aux caractéristiques de communication.
- **Communication :**
    - L'application **écrit** les commandes sur une caractéristique spécifique.
    - L'application **s'abonne** aux notifications d'une autre caractéristique pour recevoir la télémétrie.

### UUIDs BLE (pour DFRobot Bluno)

- **Service UUID :** `0000dfb0-0000-1000-8000-00805f9b34fb`
- **Caractéristique (Commandes / Écriture) :** `0000dfb2-0000-1000-8000-00805f9b34fb`
- **Caractéristique (Télémétrie / Notifications) :** `0000dfb1-0000-1000-8000-00805f9b34fb`

> **Note :** Il n'est pas nécessaire d'appairer le robot dans les paramètres Bluetooth du système d'exploitation. La connexion est gérée directement par l'application.

---

## 2. Protocole de Données

Les données échangées sont des chaînes de caractères ASCII, terminées par un caractère de nouvelle ligne (`\n`).

### 2.1. Commandes (App → Robot)

Les commandes sont envoyées de l'application vers le robot pour le contrôler. Elles sont structurées comme suit : `CMD:<ACTION>:<VALEUR>\n`.

Pour une liste détaillée de toutes les commandes disponibles et de leurs paramètres, veuillez consulter le document [**Commandes.md**](./commands.md).

**Exemple de traitement (pseudo-code Arduino) :**
```cpp
// Fichier terminal.h

void handleSerialCommand() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    // Le parsing de la commande "CMD:ACTION:VALEUR" est effectué ici.
    // Exemple: "CMD:MOVE:FWD"
    // type   = "CMD"
    // action = "MOVE"
    // value  = "FWD"
    executeCommand(type, action, value);
  }
}
```

### 2.2. Télémétrie (Robot → App)

La télémétrie est envoyée périodiquement par le robot à l'application pour l'informer de son état. Les données sont formatées en tant qu'objet JSON sur une seule ligne, terminé par `\n`.

**Format :** Objet JSON

| Clé | Type | Description |
| :--- | :--- | :--- |
| `state` | `String` | L'état actuel de la machine à états principale du robot (ex: "IDLE", "MOVING_FORWARD", "OBSTACLE_AVOIDANCE"). |
| `heading` | `Number` | Le cap magnétique actuel du robot, en degrés (0-359). |
| `distance`| `Number` | La distance mesurée par le capteur à ultrasons frontal, en centimètres. Utile pour la détection d'obstacles proches. |
| `distanceLaser`| `Number` | La distance mesurée par le capteur laser ToF (Time-of-Flight), en millimètres. Plus précis que les ultrasons. |
| `battery` | `Number` | Une estimation du niveau de la batterie, en pourcentage (0-100). |
| `speedTarget`| `Number` | La consigne de vitesse actuelle des moteurs (0-255). |
| `speedLeft`| `Number` | La vitesse actuelle du moteur gauche (0-255). |
| `speedRight`| `Number` | La vitesse actuelle du moteur droit (0-255). |


**Exemple de message JSON :**
```json
{"state":"FOLLOW_HEADING","heading":92,"distance":45,"distanceLaser":120,"battery":87,"speedTarget":150,"speedLeft":148,"speedRight":152}
```

**Exemple de génération (pseudo-code Arduino) :**
```cpp
// Fichier telemetry.h

void sendTelemetry() {
  // La librairie ArduinoJson est utilisée pour créer l'objet
  StaticJsonDocument<256> doc;

  doc["state"] = getCurrentStateAsString();
  doc["heading"] = compass.getHeading();
  doc["distance"] = ultrasonic.getDistance();
  // ... etc. 

  // Sérialisation du JSON sur le port série
  serializeJson(doc, Serial);
  Serial.println(); // Ne pas oublier le terminateur de ligne !
}
```
