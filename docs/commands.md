# Commandes

Cette page décrit les commandes qui peuvent être envoyées au robot NoNo via le port série.

**NOTES IMPORTANTES :**
- LEDs et tourelle sont temporairement desactivees (hardware non installe)
- Les commandes LED et scan ne fonctionneront pas jusqu'a l'installation du hardware

## Commandes de mouvement

*   `stop`: Arrête le robot et le met dans l'état `IDLE`.
*   `U`: Fait avancer le robot.
*   `D`: Fait reculer le robot.
*   `L`: Fait tourner le robot à gauche.
*   `R`: Fait tourner le robot à droite.    
*   `virageprecis <angle>`: Effectue un virage précis.
*   `virage <angle>`: Effectue un virage standard.

## Commandes de mode

*   `manual`: Passe en mode de navigation manuel.
*   `auto`: Passe en mode de navigation automatique (contrôle de cap).
*   `obstacle`: Passe en mode d'évitement d'obstacles.
*   `detect`: Passe en mode de détection PIR.
*   `mode`: Affiche le mode de navigation actuel.

## Commandes de configuration

*   `vitesse <0-255>`: Définit la vitesse cible.
*   `servo <0-180>`: Définit l'angle du servo.
*   `distance <5-100>`: Définit la distance de déplacement manuelle.

## Commandes de capteur

*   `dusm`: Affiche la distance mesurée par le capteur à ultrasons.
*   `Vbat`: Affiche la tension de la batterie.

## Commandes de la boussole

*   `cap <0-359>`: Définit le cap cible.
*   `capactuel`: Affiche le cap actuel.
*   `calibrer`: Démarre le processus de calibrage de la boussole.
*   `debugcompas`: Affiche les informations de débogage de la boussole.
*   `compasinfo`: Affiche des informations sur la boussole.
*   `resetcalib`: Réinitialise le calibrage de la boussole.

## Commandes des phares

*   `on`: Allume le phare.
*   `off`: Éteint le phare.

## MODES DE NAVIGATION

Le robot a deux modes de navigation :

### 1. MODE MANUEL (MANUAL_CONTROL) :
*   U : Avance de distance précise (par défaut 20 cm, réglable)
*   D : Recule de distance précise (par défaut 20 cm, réglable)
*   L : Tourne à gauche CONTINU (pas de gestion de cap)
*   R : Tourne à droite CONTINU (pas de gestion de cap)
*   stop : Arrête immédiatement tous les mouvements
*   distance[X] : Régler la distance pour U/D (ex: distance30 pour 30 cm)

### 2. MODE AUTO (HEADING_CONTROL) :
*   U : Avance en suivant le cap défini
*   D : Arrêt avec maintien du cap (le robot reste orienté vers le cap)
*   L : Ajuste le cap de -15° (fonctionne même à l'arrêt)
*   R : Ajuste le cap de +15° (fonctionne même à l'arrêt)

## NOTES

- Les commandes sont sensibles à la casse
- Les commandes avec paramètres utilisent la syntaxe : commande[valeur]
- Exemple : "cap90" pour régler le cap à 90 degrés
- Exemple : "vitesse75" pour régler la vitesse à 75%
- Exemple : "servo120" pour positionner le servo à 120 degrés
- Exemple : "virage45" pour tourner de 45° vers la droite
- Exemple : "virage-30" pour tourner de 30° vers la gauche