# Guide de Calibration du Magnétomètre

Ce guide explique pourquoi et comment calibrer le magnétomètre (boussole) du robot NoNo.

## Pourquoi la Calibration est-elle Cruciale ?

Le magnétomètre mesure le champ magnétique terrestre pour déterminer le cap. Cependant, il est très sensible aux **interférences magnétiques** générées par :
- Les moteurs du robot.
- La batterie.
- Les structures métalliques dans l'environnement (sols en béton armé, etc.).

Ces interférences créent des distorsions ("hard-iron" et "soft-iron") qui rendent les lectures brutes du capteur inexactes. La calibration est le processus qui mesure ces distorsions et calcule des offsets pour les compenser, garantissant ainsi une navigation précise.

**Sans une bonne calibration, les modes de navigation comme `G:heading` (GOTO) ne fonctionneront pas correctement.**

---

## Processus de Calibration

La calibration est une procédure simple qui prend moins d'une minute.

### Étape 1 : Préparation

- Placez le robot dans la zone où il sera le plus souvent utilisé.
- Éloignez-le de gros objets métalliques ou de sources magnétiques puissantes (enceintes, aimants).

### Étape 2 : Lancement de la Commande

Envoyez la commande de calibration via le terminal série, suivie d'une nouvelle ligne (`\n`) :

`E:CALIBRATE`

Le robot entrera en mode calibration, et l'écran LCD affichera "Calibrating...".

### Étape 3 : La Danse de la Calibration (15 secondes)

C'est l'étape la plus importante. Pendant 15 secondes, vous devez **faire tourner lentement le robot sur tous ses axes**.

Imaginez que le robot est au centre d'un globe. Vous devez lui faire "peindre" toute la surface intérieure de ce globe.

- **Penchez-le** en avant, en arrière.
- **Roulez-le** sur ses côtés gauche et droit.
- **Faites-le pivoter** à plat.

```
      /--->\
     |      |
     \--<--/
  (Pivoter sur place)

      ^
     / \
    /   \
   <----->
(Penchez d'avant en arrière)

   ------
  /      \
 |        |
  \      /
   ------
(Roulez sur les côtés)
```

Le but est d'exposer le capteur à la plus grande variété d'orientations possible par rapport au champ magnétique terrestre.

### Étape 4 : Fin et Sauvegarde

Après 15 secondes, le robot arrêtera automatiquement la procédure.
- Les nouvelles valeurs de calibration (offsets) seront calculées.
- Elles seront **automatiquement sauvegardées dans la mémoire EEPROM** du robot.
- L'écran LCD affichera "Calibration OK".

Les données de calibration sont maintenant persistantes et seront chargées à chaque démarrage.
