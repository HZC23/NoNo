# Configuration du Hardware - Robot Nono

## 🚨 Hardware Temporairement Désactivé

Les composants suivants sont **temporairement désactivés** car ils ne sont pas encore installés :
<!-- NOTE: Cette section peut être obsolète. Les flags dans config.h sont à `true`. -->
### LEDs (Désactivées)
- **LED Rouge** (Pin 22)
- **LED Jaune** (Pin 24)
- **Fonctions affectées** : `balise_jaune()`, `balise_rouge()`

### Tourelle (Désactivée)
- **Servos Tourelle** (Horizontal: Pin 8, Vertical: Pin 9)
- **Fonctions affectées** : `scan`, `SCANNING_ENVIRONMENT`, `SMART_TURNING`

## 🔧 Activation du Hardware

Pour activer ces composants une fois installés, modifiez les flags dans `config.h` :

```cpp
// Hardware availability flags - set to true to enable installed hardware
#define ENABLE_LEDS false       // Set to true when LEDs are installed
#define ENABLE_TOWER true         // Set to true when turret is installed
```

## 📋 Hardware Actuellement Actif

### ✅ Composants Fonctionnels
- **Moteurs** (Pins 2,3,4,5) - Contrôle MX1508
- **Servo Direction** (Pin 10) - Contrôle de direction
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
- **Contrôle** : vitesse[valeur], servo[angle]
- **Phares** : on, off
- **Capteurs** : dusm, Vbat
- **Compas** : capactuel, calibrer, debugcompas, compasinfo

## 🔄 Réactivation des Composants

### LEDs
1. Connecter les LEDs aux pins A1 et A2
2. Modifier `#define ENABLE_LEDS 1` dans `fonctions.h`
3. Recompiler et téléverser

### Tourelle
1. Installer le servo sur le pin 12
2. Modifier `#define ENABLE_TOWER 1` dans `fonctions.h`
3. Recompiler et téléverser

## ⚠️ Notes Importantes

- Le robot fonctionne parfaitement sans les LEDs et la tourelle
- Les modes d'évitement d'obstacles utilisent le capteur ultrasonique fixe
- La navigation par compas reste entièrement fonctionnelle
- Aucune modification du code n'est nécessaire pour les autres fonctionnalités

## 🐛 Dépannage

Si vous rencontrez des erreurs après activation :
1. Vérifiez les connexions hardware
2. Vérifiez que les pins sont corrects
3. Vérifiez l'alimentation des composants
4. Utilisez `debugcompas` pour vérifier le compas
5. Utilisez `dusm` pour tester le capteur ultrasonique
