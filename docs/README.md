# Robot NoNo

Ceci est un contrôleur de robot basé sur Arduino pour un robot nommé "NoNo".

## Fonctionnalités

*   **Contrôle basé sur une machine à états:** Le comportement du robot est géré par une machine à états, ce qui rend le code plus organisé et plus facile à comprendre.
*   **Code non bloquant:** Le code est écrit pour être non bloquant, ce qui signifie que le robot est toujours réactif aux commandes.
*   **Plusieurs modes de navigation:** Le robot dispose de deux modes de navigation: manuel et automatique (contrôle de cap).
*   **Navigation par boussole:** Le robot peut utiliser une boussole pour naviguer et maintenir un cap spécifique.
*   **Évitement d'obstacles:** Le robot peut détecter et éviter les obstacles à l'aide d'un capteur à ultrasons.
*   **Capteur PIR:** Le robot peut détecter les mouvements à l'aide d'un capteur PIR.
*   **Écran LCD:** Le robot peut afficher des informations sur un écran LCD.
*   **Interface de commande série:** Le robot peut être contrôlé via le port série.

## Matériel

*   Arduino Mega
*   Boussole LSM303
*   Capteur à ultrasons
*   Capteur PIR
*   Écran LCD 16x2
*   Pilote de moteur MX1508
*   Servomoteur

## Compilation

Pour compiler le code, vous aurez besoin de `arduino-cli`. Vous pouvez trouver les instructions d'installation [ici](https://arduino.github.io/arduino-cli/latest/installation/).

Une fois `arduino-cli` installé, vous pouvez compiler le code à l'aide de la commande suivante:

```
ar-cli compile --fqbn arduino:avr:mega
```

## Téléversement

Pour téléverser le code sur l'Arduino, vous devrez trouver le port sur lequel l'Arduino est connecté. Vous pouvez le faire à l'aide de la commande suivante:

```
ar-cli board list
```

Une fois que vous avez le port, vous pouvez téléverser le code à l'aide de la commande suivante:

```
ar-cli upload -p <PORT> --fqbn arduino:avr:mega
```

Remplacez `<PORT>` par le port sur lequel l'Arduino est connecté.

## Commandes

Le robot peut être contrôlé via le port série. Voici une liste des commandes disponibles:

*   `stop`: Arrête le robot.
*   `manual`: Passe en mode de navigation manuel.
*   `auto`: Passe en mode de navigation automatique (contrôle de cap).
*   `off`: Éteint le phare.
*   `on`: Allume le phare.
*   `dusm`: Affiche la distance mesurée par le capteur à ultrasons.
*   `obstacle`: Passe en mode d'évitement d'obstacles.
*   `scan`: Analyse l'environnement à la recherche d'obstacles.
*   `detect`: Passe en mode de détection PIR.
*   `cap <0-359>`: Définit le cap cible.
*   `vitesse <0-255>`: Définit la vitesse cible.
*   `servo <0-180>`: Définit l'angle du servo.
*   `Vbat`: Affiche la tension de la batterie.
*   `capactuel`: Affiche le cap actuel.
*   `calibrer`: Démarre le processus de calibrage de la boussole.
*   `debugcompas`: Affiche les informations de débogage de la boussole.
*   `compasinfo`: Affiche des informations sur la boussole.
*   `resetcalib`: Réinitialise le calibrage de la boussole.
*   `distance <5-100>`: Définit la distance de déplacement manuelle.
*   `mode`: Affiche le mode de navigation actuel.
*   `virageprecis <angle>`: Effectue un virage précis.
*   `virage <angle>`: Effectue un virage standard.
*   `U`: Fait avancer le robot.
*   `D`: Fait reculer le robot.
*   `L`: Fait tourner le robot à gauche.
*   `R`: Fait tourner le robot à droite.

## Modes de navigation

Le robot dispose de deux modes de navigation:

*   **Manuel:** Dans ce mode, le robot est contrôlé directement par l'utilisateur. Les commandes `U`, `D`, `L` et `R` sont utilisées pour déplacer le robot.
*   **Automatique (contrôle de cap):** Dans ce mode, le robot utilise la boussole pour maintenir un cap spécifique. La commande `U` est utilisée pour faire avancer le robot, et les commandes `L` et `R` sont utilisées pour modifier le cap cible.

## Calibrage

Le calibrage de la boussole est crucial pour une navigation précise. Le robot utilise une méthode de calibrage à 360 degrés pour compenser les interférences magnétiques.

Pour calibrer la boussole:

1.  Envoyez la commande `calibrer` via le port série.
2.  **Faites pivoter le robot LENTEMENT sur tous ses axes (X, Y, Z)** pendant environ 15 secondes, en suivant les instructions affichées sur le moniteur série et l'écran LCD.

Pour des instructions détaillées sur le processus de calibrage, y compris les conseils pour un calibrage réussi et la réinitialisation, veuillez consulter le document dédié : [Calibrage de la Boussole](docs/calibration.md).