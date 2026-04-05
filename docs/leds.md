# Système de LEDs (NeoPixel)

Le robot Nono est équipé de 4 LEDs NeoPixel (WS2812B) pour fournir un retour visuel sur son état de fonctionnement, le niveau de batterie et les alertes de sécurité.

## Configuration Matérielle

- **Broche (PIN) :** 48 (Souvent la LED intégrée sur l'ESP32-S3)
- **Nombre de LEDs :** 4
  - **LED 0 :** LED "Système" (interne ou de diagnostic).
  - **LED 1, 2, 3 :** LEDs "Visibles" (externes, montées sur le châssis).

## États et Effets Visuels

Le système de LEDs suit une hiérarchie de priorité. Les états critiques surchargent les états normaux.

### Priorités Hautes (Alertes)

| État | Priorité | Effet Visuel | Description |
| :--- | :--- | :--- | :--- |
| **Batterie Critique** | 1 | Rouge clignotant (toutes) | Tension batterie en dessous du seuil critique. |
| **Falaise Détectée** | 2 | Orange clignotant (visibles) | Le capteur de sol a détecté un vide. |
| **Collision (Bumper)** | 3 | Rouge/Blanc clignotant rapide | Le bumper a été activé. |

### États de Fonctionnement (Standard)

| État | Effet Visuel | Description |
| :--- | :--- | :--- |
| **IDLE (Attente)** | Bleu "Respiration" | Le robot est immobile et attend une commande. |
| **Mouvement Avant** | Vert Fixe | Modes manuel ou autonome vers l'avant. |
| **Mouvement Arrière** | Orange Fixe | Le robot recule. |
| **Virage Gauche** | Jaune (LED 3) | Indicateur de direction gauche. |
| **Virage Droite** | Jaune (LED 1) | Indicateur de direction droite. |
| **Évitement d'obstacle** | "Cylon" Rouge | Balayage de gauche à droite en rouge. |
| **Mode Sentinelle (Scan)** | "Cylon" Bleu | Balayage de gauche à droite en bleu. |
| **Mode Sentinelle (Alerte)**| Rouge/Jaune clignotant | Cible détectée ou alarme active. |
| **Calibration Boussole** | Arc-en-ciel tournant | Procédure de calibration du magnétomètre. |
| **Batterie Faible** | Jaune pulsant | Indicateur de batterie faible (non critique). |

## Améliorations du Système

### 1. Gestion de la LED Système (LED 0)
La LED 0 est réservée aux diagnostics système :
- **Clignotement Vert :** Système OK / Boucle principale active.
- **Bleu Fixe :** Connexion Bluetooth active.
- **Violet :** Accès Carte SD / Logging.
- **Rouge :** Erreur Système (I2C, Capteurs, etc.).

### 2. Animations et Transitions
Le système utilise des fonctions d'interpolation pour des transitions fluides entre les couleurs, évitant les changements brusques.

### 3. Abstraction des Effets
Les effets sont isolés dans des fonctions dédiées pour faciliter l'ajout de nouveaux comportements sans complexifier la boucle principale `led_fx_update`.

## Utilisation dans le Code

Le fichier `src/led_fx.cpp` contient toute la logique. L'initialisation se fait via `led_fx_init()` et la mise à jour périodique via `led_fx_update(const Robot& robot)`.

```cpp
// Exemple d'appel dans la boucle principale
led_fx_update(robot);
```
