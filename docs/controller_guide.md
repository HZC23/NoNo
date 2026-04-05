# Guide de Contrôle - Manette Xbox

Ce document décrit comment contrôler le robot Nono à l'aide d'une manette sans fil Xbox.

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
| **Y** | Activer/Désactiver le mode manuel | Passe du mode autonome au mode de contrôle manuel. |
| **A** | Allumer/Éteindre les phares | Contrôle les phares avant. |
| **B** | Activer/Désactiver l'évitement | Active ou désactive le mode d'évitement d'obstacles intelligent. |
| **X** | Activer/Désactiver le mode sentinelle | Active ou désactive le mode sentinelle (surveillance avec le capteur PIR). |

### Croix Directionnelle (D-Pad)

| Bouton | Action | Description |
| :--- | :--- | :--- |
| **Haut** | Calibrer le Compas | Lance la routine de calibration du compas. Le robot tournera sur lui-même. |
| **Bas** | Activer le Stockage USB | Met le robot en pause et le fait apparaître comme un disque USB sur le PC. **Nécessite un redémarrage pour quitter ce mode.** |
| **Droite**| Afficher une blague | Affiche une blague aléatoire (tirée de `jokes.txt`) sur l'écran LCD. |
| **Gauche**| (Aucune action) | - |

### Boutons de Menu

| Bouton | Action | Description |
| :--- | :--- | :--- |
| **Bouton View** (gauche) | Arrêt d'Urgence | Coupe immédiatement les moteurs. |
| **Bouton Xbox** (Home) | Changer l'affichage LCD | Bascule entre l'affichage simple et l'affichage détaillé des capteurs. |

---
*Dernière mise à jour : 29/12/2025*
