# Configuration du Hardware - Robot Nono

## 🚨 Statut du Hardware

### LEDs (Désactivées)
- **Statut**: Désactivé dans `config.h` (`ENABLE_LEDS false`).
- **LED Rouge** (Pin 22)
- **LED Jaune** (Pin 24)
- **Fonctions affectées** : `balise_jaune()`, `balise_rouge()`

### Tourelle (Activée)
- **Statut**: Activé dans `config.h` (`ENABLE_TOWER true`).
- **Servos Tourelle** (Horizontal: Pin 8, Vertical: Pin 9)
- **Fonctions affectées** : `scan`, `SCANNING_ENVIRONMENT`, `SMART_TURNING`

### Servo Direction (Désactivé)
- **Statut**: Le code pour le servo de direction (Ackermann) a été désactivé au profit de la direction différentielle.
- Le code est commenté dans `NoNo.ino` et `fonctions_motrices.h`.

## 🔧 Activation du Hardware

Pour activer les LEDs, modifiez le flag dans `config.h` :

```cpp
// Hardware availability flags - set to true to enable installed hardware
#define ENABLE_LEDS true       // Set to true when LEDs are installed
```

## 📋 Hardware Actuellement Actif

### ✅ Composants Fonctionnels
- **Moteurs** (Pins 2,3,4,5) - Contrôle MX1508. La direction est maintenant gérée de manière différentielle.
- **Capteur Ultrasonique** (Pins 36,37) - Détection d'obstacles
- **Compas LSM303** (I2C) - Navigation par cap
- **LCD RGB** (I2C 0x60) - Affichage
- **Phares** (Pin 38) - Éclairage
- **Capteur PIR** (Pin 40) - Détection de mouvement
- **Bouton d'arrêt** (Pin 39) - Arrêt d'urgence

### 🎮 Commandes Disponibles
- **Mouvement** : U, D, L, R, stop
- **Modes** : manual, auto, obstacle, detect
- **Navigation** : cap[angle], virage[angle]
- **Contrôle** : vitesse[valeur]
- **Phares** : on, off
- **Capteurs** : dusm, Vbat
- **Compas** : capactuel, calibrer, debugcompas, compasinfo

## 🔄 Réactivation des Composants

### LEDs
1. Connecter les LEDs aux pins 22 et 24
2. Modifier `#define ENABLE_LEDS true` dans `config.h`
3. Recompiler et téléverser

## ⚠️ Notes Importantes

- Le robot fonctionne parfaitement sans les LEDs.
- Les modes d'évitement d'obstacles utilisent le capteur ultrasonique fixe
- La navigation par compas reste entièrement fonctionnelle

## 🐛 Dépannage

Si vous rencontrez des erreurs après activation :
1. Vérifiez les connexions hardware
2. Vérifiez que les pins sont corrects
3. Vérifiez l'alimentation des composants
4. Utilisez `debugcompas` pour vérifier le compas
5. Utilisez `dusm` pour tester le capteur ultrasonique
