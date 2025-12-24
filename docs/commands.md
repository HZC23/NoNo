# Référence des Commandes Série

Ce document détaille toutes les commandes texte qui peuvent être envoyées au robot Nono via une connexion série (USB).

## Format Général

Toutes les commandes suivent une structure `CLÉ:VALEUR` et doivent se terminer par un caractère de nouvelle ligne (`\n`). Les clés de commande ne sont pas sensibles à la casse (ex: `L:ON` et `l:on` sont identiques).

---

## Référence des Commandes

| Clé | Paramètres | Description | Exemple |
| :--- | :--- | :--- | :--- |
| `M` | `vitesse,virage` | **Move**: Contrôle directement les moteurs. `vitesse` est la vitesse avant/arrière (-255 à 255), `virage` est la composante de rotation (-255 à 255). Non-persistant. | `M:150,50` |
| `G` | `cap` | **Goto**: Fait pivoter le robot vers le `cap` absolu spécifié (0-359 degrés) et active le mode `FOLLOW_HEADING`. | `G:90` |
| `S` | `vitesse` | **Speed**: Définit la vitesse cible manuelle (`vitesseCible`) de 0 à 255. | `S:200` |
| `CO`| `offset` | **Compass Offset**: Applique une correction permanente (en degrés) au cap de la boussole. | `CO:-5.5` |
| `SM`| `XBOX` ou `SERIAL` | **Set Mode**: Change le mode de contrôle principal et redémarre le robot. | `SM:XBOX` |
| `E` | `mode` | **État**: Change l'état comportemental du robot. Voir les modes ci-dessous. | `E:AVOID` |
| `L` | `ON`, `OFF`, `TOGGLE` | **Light**: Contrôle les phares. | `L:ON` |

---
## Modes disponibles pour la commande `E:`

-   `IDLE`: Arrête toutes les actions.
-   `AVOID`: Active le mode autonome `OBSTACLE_AVOIDANCE`.
-   `SENTRY`: Active le mode `SENTRY_MODE`.
-   `CALIBRATE`: Démarre la procédure de calibrage de la boussole (15 sec).
-   `TOGGLE_AVOID`: Bascule (active/désactive) le mode `OBSTACLE_AVOIDANCE`.
-   `TOGGLE_SENTRY`: Bascule (active/désactive) le mode `SENTRY_MODE`.
