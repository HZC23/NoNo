# Référence des Commandes pour l'Application

Ce document détaille toutes les commandes texte qui peuvent être envoyées au robot Nono via le service BLE de l'application.

## Format Général

La plupart des commandes suivent une structure `PREFIXE:VALEUR`. Certaines commandes plus complexes utilisent un format `PREFIXE:SOUS_PREFIXE=VALEUR`. Toutes les commandes doivent se terminer par un caractère de nouvelle ligne (`\n`). Les commandes ne sont pas sensibles à la casse (ex: `HL:ON` et `hl:on` sont identiques).

---

## Préfixe `CMD` - Commandes Générales

Ce sont les commandes principales pour le contrôle et la configuration du robot.

| Commande | Paramètres | Description |
| :--- | :--- | :--- |
| `CMD:MOVE` | `velocity,turn` | Contrôle le mouvement direct du robot. `velocity` et `turn` sont des valeurs de -255 à 255. |
| `CMD:GOTO` | `heading` | Fait pivoter le robot pour faire face au cap (en degrés) spécifié. |
| `CMD:SPEED`| `speed` | Définit la vitesse cible (0-255) pour les modes de déplacement autonomes. |
| `CMD:COMPASS_OFFSET` | `offset` | Applique une correction manuelle (en degrés) à la boussole pour un réglage fin. |
| `CMD:SET_MODE` | `XBOX` ou `APP` | Change le mode de contrôle principal et redémarre le robot. Utile pour basculer vers le mode manette depuis l'application. |

---

## Préfixe `MODE` - Changement de Comportement

Change l'état ou le mode de fonctionnement actuel du robot.

| Commande | Description |
| :--- | :--- |
| `MODE:IDLE` | Arrête toute action en cours et met le robot en attente. |
| `MODE:AVOID` | Active le mode autonome d'esquive d'obstacles. |
| `MODE:SENTRY` | Active le mode sentinelle, où le robot surveille les mouvements. |
| `MODE:SCAN3D`| Lance un scan 3D de l'environnement devant lui. |
| `MODE:CALIBRATE`| Démarre la procédure de calibrage de la boussole. |
| `MODE:TOGGLE_AVOID` | Bascule (active/désactive) le mode `AVOID`. |
| `MODE:TOGGLE_SENTRY`| Bascule (active/désactive) le mode `SENTRY`. |

---

## Préfixe `HL` - Contrôle des Phares

| Commande | Description |
| :--- | :--- |
| `HL:ON` | Allume les phares. |
| `HL:OFF` | Éteint les phares. |
| `HL:TOGGLE`| Bascule l'état actuel des phares. |

---