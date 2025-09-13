# Guide d'utilisation des modes de navigation de Nono

## 🎮 Système de navigation à deux modes

Nono dispose maintenant d'un système de navigation sophistiqué avec deux modes complémentaires :

### **🕹️ Mode Manuel (par défaut)**
Contrôle direct des moteurs - pilotage traditionnel

### **🤖 Mode Cap (Auto)**
Navigation autonome par cap - pilotage intelligent

---

## 🔧 Commandes de changement de mode

### **Changement de mode**
- `manual` - Passe en mode manuel (contrôle direct)
- `auto` - Passe en mode cap (navigation autonome)
- `mode` - Affiche le mode actuel

---

## 🕹️ Mode Manuel (MANUAL_CONTROL)

### **Comportement**
- Contrôle direct des moteurs
- Réactivité immédiate
- Pilotage traditionnel

### **Commandes**
- `U` - **Avance** en ligne droite
- `D` - **Recule** en ligne droite  
- `L` - **Virage à gauche** de 90° (utilise le compas)
- `R` - **Virage à droite** de 90° (utilise le compas)

### **Exemple d'utilisation**
```
manual    // Passe en mode manuel
U         // Avance
L         // Virage à gauche de 90°
U         // Avance
R         // Virage à droite de 90°
D         // Recule
stop      // Arrêt complet
```

---

## 🤖 Mode Cap (HEADING_CONTROL)

### **Comportement**
- Navigation autonome par cap
- Maintien automatique de la direction
- Corrections automatiques de trajectoire

### **Principe**
1. Le robot lit son cap actuel (ex: 270°)
2. Il définit ce cap comme **cap cible** (`Ncap = 270°`)
3. Il maintient automatiquement cette direction
4. Les touches `L` et `R` **modifient le cap cible** (pas le robot directement)

### **Commandes**
- `U` - **Avance** en suivant le cap actuel
- `D` - **Arrêt avec maintien du cap** (le robot reste orienté vers le cap cible)
- `L` - **Modifie le cap** de -15° (gauche) - fonctionne même à l'arrêt
- `R` - **Modifie le cap** de +15° (droite) - fonctionne même à l'arrêt

### **Exemple d'utilisation**
```
auto      // Passe en mode cap (cap actuel = 270°)
U         // Avance en maintenant le cap 270°
D         // Arrêt avec maintien du cap 270° (robot reste orienté)
R         // Nouveau cap = 285° (robot tourne automatiquement sur place)
R         // Nouveau cap = 300° (robot tourne encore sur place)
L         // Nouveau cap = 285° (robot revient)
L         // Nouveau cap = 270° (robot revient)
+U         // Reprend l'avance vers 270°
```

---

## 🔄 Comparaison des modes

| Aspect | Mode Manuel | Mode Cap |
|--------|-------------|----------|
| **Contrôle** | Direct des moteurs | Par cap cible |
| **Réactivité** | Immédiate | Avec corrections |
| **Précision** | Dépend du pilote | Automatique |
| **Virages** | 90° fixes | Ajustements de 15° |
| **Maintien** | Manuel | Automatique |
| **Usage** | Pilotage sportif | Navigation précise |

---

## 🎯 Cas d'usage recommandés

### **Mode Manuel** - Idéal pour :
- Pilotage réactif et sportif
- Manœuvres d'urgence
- Apprentissage du pilotage
- Situations imprévisibles

### **Mode Cap** - Idéal pour :
- Navigation de longue distance
- Maintien d'une direction précise
- Éviter la dérive
- Missions autonomes

---

## 🔧 Commandes communes

### **Navigation**
- `stop` - Arrêt complet (tous modes)
- `cap180` - Navigation vers le cap 180°
- `virage45` - Virage de 45° à droite
- `virage-30` - Virage de 30° à gauche

### **Débogage compas**
- `capactuel` - Affiche le cap actuel
- `calibrer` - Calibration du compas
- `debugcompas` - Informations détaillées
- `compasinfo` - Direction cardinale

### **Autres**
- `vitesse150` - Régler la vitesse à 150
- `servo90` - Positionner le servo à 90°
- `on/off` - Phares
- `dusm` - Distance ultrasonique
- `Vbat` - Niveau batterie

---

## 💡 Conseils d'utilisation

### **Pour débuter**
1. Commencez en **mode manuel** pour vous familiariser
2. Testez les virages `L` et `R` pour comprendre la précision
3. Passez en **mode cap** pour la navigation autonome

### **Mode Cap**
1. **Calibrez le compas** avant utilisation (`calibrer`)
2. Vérifiez le cap actuel (`capactuel`)
3. Utilisez `L` et `R` par petites touches (15°)
4. Le robot corrige automatiquement sa trajectoire

### **Changement de mode**
- Le robot **s'arrête automatiquement** lors du changement
- Le mode **cap** initialise le cap cible sur le cap actuel
- Utilisez `mode` pour vérifier le mode actuel

---

## 🚨 Dépannage

### **Problème : Robot ne répond pas**
- Vérifiez le mode actuel (`mode`)
- Essayez `stop` puis recommencez
- Vérifiez la calibration du compas

### **Problème : Virages imprécis**
- Recalibrez le compas (`calibrer`)
- Vérifiez l'environnement magnétique
- Utilisez le mode manuel pour les virages critiques

### **Problème : Dérive en mode cap**
- Recalibrez le compas
- Vérifiez `Kp_HEADING` dans le code
- Ajustez la tolérance si nécessaire

---

## 🎉 Avantages du système

1. **Flexibilité** : Deux modes pour tous les besoins
2. **Précision** : Navigation par cap pour les missions longues
3. **Réactivité** : Mode manuel pour les situations d'urgence
4. **Simplicité** : Même interface (U, D, L, R) pour les deux modes
5. **Robustesse** : Architecture non-bloquante

Le robot Nono est maintenant un véritable système de navigation professionnel ! 🚀

# Guide d'utilisation du compas LSM303 pour Nono

## 🧭 Nouvelles commandes de débogage et calibration

### Commandes de base
- `capactuel` - Affiche le cap actuel calibré
- `calibrer` - Lance la calibration du compas
- `debugcompas` - Affiche toutes les informations de débogage du compas
- `compasinfo` - Affiche les informations du compas avec direction cardinale
- `resetcalib` - Réinitialise la calibration du compas

### Commandes de navigation
- `L` - Virage à gauche de 90° (utilise le compas)
- `R` - Virage à droite de 90° (utilise le compas)
- `virage45` - Virage de 45° à droite
- `virage-30` - Virage de 30° à gauche
- `cap180` - Navigation vers le cap 180°

## 🔧 Procédure de calibration

### Étape 1 : Préparation
1. Placez Nono sur une surface plane et stable
2. Éloignez-le des objets métalliques (métal, aimants, etc.)
3. Assurez-vous qu'il n'y a pas d'interférences magnétiques

### Étape 2 : Calibration
1. Envoyez la commande `calibrer`
2. Placez le robot face au Nord magnétique (utilisez une boussole)
3. Appuyez sur une touche pour commencer
4. **Ne bougez PAS** le robot pendant 10 secondes
5. Le système collecte 50 échantillons et calcule l'offset

### Étape 3 : Vérification
1. Utilisez `debugcompas` pour vérifier la qualité du signal
2. Utilisez `compasinfo` pour voir la direction cardinale
3. Si la variation est > 10°, recalibrez dans un autre environnement

## 📊 Interprétation des résultats

### Signal magnétique
- **< 100** : Signal faible (vérifiez l'environnement)
- **100-1000** : Signal normal ✅
- **> 1000** : Signal très fort (interférence possible)

### Qualité de calibration
- **Variation < 10°** : Calibration réussie ✅
- **Variation > 10°** : Recalibrez dans un autre environnement

### Directions cardinales
- **NORD** : 337.5° - 22.5°
- **NE** : 22.5° - 67.5°
- **EST** : 67.5° - 112.5°
- **SE** : 112.5° - 157.5°
- **SUD** : 157.5° - 202.5°
- **SO** : 202.5° - 247.5°
- **OUEST** : 247.5° - 292.5°
- **NO** : 292.5° - 337.5°

## 🚨 Dépannage

### Problème : Cap instable
- Vérifiez l'environnement magnétique
- Éloignez les objets métalliques
- Recalibrez le compas

### Problème : Signal faible
- Vérifiez les connexions du LSM303
- Assurez-vous que l'alimentation est stable
- Testez dans un autre environnement

### Problème : Virages imprécis
- Recalibrez le compas
- Vérifiez la tolérance (`TOLERANCE_VIRAGE`)
- Ajustez `Kp_HEADING` si nécessaire

## 🔍 Informations de débogage

### Valeurs brutes du magnétomètre
- **X, Y, Z** : Composantes du champ magnétique
- **Magnitude** : Intensité totale du champ magnétique

### Valeurs de l'accéléromètre
- **X, Y, Z** : Accélération sur chaque axe
- **Pitch, Roll** : Inclinaison du robot

### Cap
- **Brut** : Valeur non calibrée du compas
- **Calibré** : Valeur après application de l'offset

## ⚙️ Paramètres ajustables

Dans `fonctions.h` :
```cpp
const float TOLERANCE_VIRAGE = 5.0; // Tolérance pour les virages
const int CALIBRATION_SAMPLES = 50; // Nombre d'échantillons
const float Kp_HEADING = 1.5; // Gain du contrôleur proportionnel
```

## 🎯 Conseils d'utilisation

1. **Calibrez régulièrement** le compas, surtout après déplacement
2. **Évitez les environnements magnétiques** (métal, aimants, etc.)
3. **Vérifiez la qualité** du signal avant navigation
4. **Utilisez les commandes de débogage** pour diagnostiquer les problèmes
5. **Testez les virages** avec des angles simples (90°, 180°) d'abord
