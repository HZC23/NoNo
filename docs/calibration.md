# Calibrage de la Boussole (360 degrés)

Cette page décrit le processus de calibrage avancé de la boussole, qui permet d'obtenir des lectures de cap plus précises en compensant les interférences magnétiques.

Pour calibrer la boussole, vous devez suivre ces étapes:

1.  **Préparation:** Placez le robot sur une surface plane, à l'écart de tout objet métallique ou source de champ magnétique puissant (haut-parleurs, moteurs, etc.).
2.  **Lancement du calibrage:** Envoyez la commande `calibrer` via le moniteur série de l'IDE Arduino ou tout autre terminal série connecté au robot.
3.  **Processus de Rotation:** Le robot démarrera le processus de calibrage, qui durera environ **15 secondes**. Pendant ce temps, vous devez **faire pivoter le robot LENTEMENT sur tous ses axes (X, Y, Z)**. Assurez-vous de couvrir toutes les orientations possibles pour que le capteur puisse enregistrer la plage complète des valeurs magnétiques. Le moniteur série affichera les valeurs min/max collectées en temps réel.
4.  **Fin du calibrage:** Une fois les 15 secondes écoulées, le robot affichera les résultats finaux sur le moniteur série et l'écran LCD. Les données de calibrage (valeurs min/max pour chaque axe) seront automatiquement enregistrées dans l'EEPROM.

## Données de Calibrage

Les données de calibrage sont stockées dans l'EEPROM et sont chargées automatiquement à chaque démarrage du robot. Ces données incluent:

*   **Valeurs Min/Max du Magnétomètre:** Les valeurs minimales et maximales enregistrées pour les axes X, Y et Z du magnétomètre pendant le processus de rotation. Ces valeurs sont utilisées pour corriger les distorsions magnétiques (hard-iron et soft-iron).
*   **Statut de Calibrage:** Un indicateur booléen confirmant si le calibrage a été effectué avec succès.

## Réinitialisation du Calibrage

Pour réinitialiser le calibrage et effacer les données de calibrage de l'EEPROM, envoyez la commande `resetcalib` via le port série. Après cette commande, le robot utilisera les lectures brutes du compas jusqu'à ce qu'un nouveau calibrage soit effectué.

## Conseils pour un Calibrage Réussi

*   **Rotation Lente et Complète:** Assurez-vous de faire pivoter le robot lentement et de couvrir toutes les orientations (haut, bas, côtés, etc.) pour obtenir des données précises.
*   **Environnement Magnétique Stable:** Évitez de calibrer à proximité de sources d'interférences magnétiques.
*   **Vérification:** Après le calibrage, vous pouvez utiliser la commande `capactuel` ou `debugcompas` pour vérifier la précision des lectures du cap.
