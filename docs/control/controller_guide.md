# Guide de Contrôle - Manette Xbox

Ce document décrit comment contrôler le robot Nono à l'aide d'une manette sans fil Xbox.

## Initial pairing and prerequisites

Before using the Xbox controller with the robot, you must first pair it with the robot's Bluetooth module:

### Requirements
- **Bluetooth Module**: The robot must have a Bluetooth adapter (BLE or Classic). Check your configuration for which type your robot supports.
- **Supported Controller Models**: Standard Xbox controllers (Xbox One, Xbox Series X/S) or compatible third-party controllers.
- **Safe Environment**: Ensure the robot is stationary and the surrounding area is clear before pairing and testing.

### Step-by-step pairing steps
1. **Put the robot's Bluetooth into discovery mode**:
   - Via serial monitor, check logs for Bluetooth initialization (look for "Bluetooth initialized" or similar messages).
   - The robot's Bluetooth is typically in discoverable mode at startup if `DEFAULT_COMM_MODE` is set to `COMM_MODE_XBOX` in config.

2. **Put the Xbox controller into pairing mode**:
   - Press and hold the pairing button (small button on the back of the controller, near the batteries) until the LED on the controller begins to flash/pulse.

3. **Initiate pairing on the robot**:
   - The Bluepad32 library will automatically scan and attempt to pair with nearby Bluetooth devices.
   - Monitor the serial output for pairing confirmation messages. You may see messages like "Controller connected" or similar.

4. **Confirm pairing**:
   - Some controllers may require a PIN confirmation (typically `0000`).
   - Once pairing is confirmed, the controller should remain paired across reboots.

### Troubleshooting pairing issues
- **Controller not discovered**: 
  - Restart both the robot and the controller.
  - Check that the Bluetooth adapter is working (look for initialization logs via serial monitor).
  - Verify the adapter firmware is up to date.
- **Pairing fails or times out**:
  - Reset the controller by holding the power button for 10 seconds.
  - Re-enable Bluetooth discovery on the robot by restarting it or re-running the `sm:xbox` command via serial.
  - Check the serial monitor logs for Bluetooth errors (e.g., "BLE: Connection failed", "Pairing timeout").
- **Retry pairing**:
  - If the controller is not listed in the robot's available devices, repeat the pairing steps above.
  - Clear any stored pairing on the controller (consult the controller's manual) and try again.

### Where to run these steps
- **Before issuing `sm:xbox` command**, ensure the controller is already in pairing mode or previously paired and powered on.
- **After issuing `sm:xbox` command**, restart the robot so it boots into Xbox controller mode and automatically connects.
- If you need to return to serial control, use the `sm:serial` command via the serial monitor and restart.

Expected output during successful pairing:
```
Bluetooth initialized
Scanning for controllers...
Xbox controller detected
Controller connected - Ready for commands
```

---

## Activation du Mode Manette

Pour que le robot réponde à une manette, il doit être dans le mode de communication approprié.

1.  Connectez-vous au robot via le moniteur série.
2.  Envoyez la commande `sm:xbox`.
3.  Redémarrez le robot.

Au redémarrage, le robot tentera de se connecter à une manette Xbox configurée. Pour revenir au contrôle par port série, utilisez la commande `sm:serial`.

## Assignation des Commandes

### Contrôles Principaux

| Action | Contrôle | Description |
| :--- | :--- | :--- |
| **Mouvement (Av/Arr/Direction)** | Stick Gauche | Contrôle les déplacements du robot, de type "Ackermann" (comme une voiture). |
| **Rotation sur place** | Gâchettes LT / RT | Permet de faire pivoter le robot sur lui-même sans avancer. |
| **Contrôle de la Tourelle** | Stick Droit | Oriente la tête du robot (Panoramique et Inclinaison). |
| **Augmenter la Vitesse** | Bumper Droit (RB) | Augmente la vitesse de déplacement maximale. |
| **Diminuer la Vitesse** | Bumper Gauche (LB) | Diminue la vitesse de déplacement maximale. |

### Boutons de Fonction (Façade)

| Bouton | Action | Description |
| :--- | :--- | :--- |
| **Y** | Activer/Désactiver le mode manuel | Bascule entre le contrôle manuel (manette) et le mode autonome. En mode autonome, le robot suit des waypoints prédéfinis, exécute des schémas de patrouille, répond aux capteurs (évitement d'obstacles, détection de motion via PIR), et exécute des tâches programmées. Le contrôle manuel reprend le contrôle complet via la manette. |
| **A** | Allumer/Éteindre les phares | Contrôle les phares avant. |
| **B** | Activer/Désactiver l'évitement | Active ou désactive le mode d'évitement d'obstacles intelligent. |
| **X** | Activer/Désactiver le mode sentinelle | Active ou désactive le mode sentinelle avec surveillance PIR. En mode sentinelle, le robot surveille l'environnement via le capteur PIR (détecteur de mouvement infrarouge) pour détecter la motion ou la présence de personnes. À la détection, le robot déclenche une alerte (notification LED, affichage LCD, ou émission sonore selon la configuration), enregistre ou transmet des données (si caméra présente), et commence un suivi/enregistrement. Le robot peut entrer en mode veille à faible puissance entre les détections. Mode de veille actif : LED clignotante rouge. Détection active : LED fixe rouge + alerte LCD. |

### Croix Directionnelle (D-Pad)

| Bouton | Action | Description |
| :--- | :--- | :--- |
| **Haut** | Calibrer le Compas | Lance la routine de calibration du compas. Le robot tournera sur lui-même. |
| **Bas** | Activer le Stockage USB | **Appuyer et maintenir Bas pendant 3 secondes pour activer**. Met le robot en pause et le fait apparaître comme un disque USB sur le PC. **AVERTISSEMENT** : L'activation du mode Stockage USB redémarre le système et interrompt toute opération en cours. **Pour quitter ce mode** : Accédez au disque USB du robot sur votre PC, éjectez-le proprement, puis redémarrez le robot. Un redémarrage est nécessaire pour reprendre le fonctionnement normal du robot. |
| **Droite**| Afficher une blague | Affiche une blague aléatoire (tirée de `jokes.txt`) sur l'écran LCD. **Format du fichier** : `jokes.txt` doit être situé à la racine de la carte SD du robot (`/jokes.txt`). Format accepté : une blague par ligne (UTF-8, sans balises). En cas de fichier manquant, vide ou malformé, un message par défaut s'affiche ("No jokes available" ou similaire). Les erreurs de parsing sont enregistrées dans les logs série. |
| **Gauche**| (Aucune action) | - |

### Boutons de Menu

| Bouton | Action | Description |
| :--- | :--- | :--- |
| **Bouton View** (gauche) | Arrêt d'Urgence | Arrête immédiatement tous les moteurs et active le mode d'urgence. **Procédure de reprise** : 1) Appuyez à nouveau sur le Bouton View pour confirmer la levée de l'arrêt d'urgence. 2) Les moteurs sont re-activés. 3) L'affichage LCD affiche "Emergency Cleared" et les LEDs passent au vert. **Récupération d'état** : Le robot tente de reprendre sa dernière pose/tâche en cours, mais un redémarrage peut être nécessaire pour réinitialiser complètement les systèmes (capteurs, calculs d'obstacles). **Indicateurs visuels/audio** : Pendant l'arrêt d'urgence, les LEDs clignotent en rouge et un son d'alerte (bip court) est émis. **Réinitialisation manuelle** : Si le robot ne répond pas, arrêtez l'alimentation et redémarrez. |
| **Bouton Xbox** (Home) | Changer l'affichage LCD | Bascule entre l'affichage simple et l'affichage détaillé des capteurs. |

---
*Dernière mise à jour : 29/12/2025*
