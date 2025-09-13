# Rapport de Profilage du Firmware "NoNo"

Ce document détaille les mesures de performance, les optimisations appliquées et leur impact sur le firmware du robot "NoNo".

## 1. Analyse Initiale (Baseline)

### Méthodologie

Le profilage initial est réalisé en instrumentant le code directement dans `NoNo.ino`. La fonction `micros()` est utilisée pour mesurer le temps d'exécution des fonctions principales dans la `loop()`.

Les mesures sont effectuées avant toute modification. Le robot est statique.

### Résultats de la Baseline

*   **`loop()` principale :**
    *   `Terminal()`: Temps de traitement des commandes série.
    *   `Dusm()`: **Lecture bloquante** du capteur de distance à ultrasons.
    *   `updateMotorControl()`: Exécution de la machine à états et des commandes moteur.

| Fonction               | Temps d'exécution moyen (µs) | Observations                               |
| ---------------------- | ------------------------------ | ------------------------------------------ |
| `Dusm()`               | À mesurer                      | Suspecté d'être le principal point de blocage. |
| `Terminal()`             | À mesurer                      | Faible, sauf en cas de réception de données. |
| `updateMotorControl()` | À mesurer                      | Contient des lectures I2C (`compass.read()`). |
| **Total `loop()`**     | **À mesurer**                  |                                            |

---

## 2. Optimisation - Tâche de Capteur Non-Bloquante

### Modifications

1.  **Activation de `sensor_task.h`**:
    *   Remplacement de l'appel bloquant `Dusm()` par `sensor_update_task()` dans la `loop()`.
    *   Configuration d'une interruption (`echo_isr`) pour la lecture non-bloquante du capteur à ultrasons.
2.  **Centralisation des lectures I2C**:
    *   Déplacement de `compass.read()` dans `sensor_update_task()` pour assurer une lecture à fréquence fixe (200 Hz).
    *   Suppression des appels `compass.read()` redondants dans la machine à états (`fonctions_motrices.h`).

### Résultats Post-Optimisation

| Fonction               | Temps d'exécution moyen (µs) | Gain (%) | Observations                                                                 |
| ---------------------- | ------------------------------ | -------- | ---------------------------------------------------------------------------- |
| `sensor_update_task()` | À mesurer                      | N/A      | Temps très court, car ne fait que déclencher/traiter des opérations non-bloquantes. |
| `Terminal()`             | À mesurer                      | -        | Inchangé.                                                                    |
| `updateMotorControl()` | À mesurer                      | À mesurer | Devrait être plus rapide sans les appels `compass.read()`.                   |
| **Total `loop()`**     | **À mesurer**                  | **À mesurer** | **Gain de latence majeur attendu.**                                          |

