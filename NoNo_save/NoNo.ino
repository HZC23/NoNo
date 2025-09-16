//2024 12 07 10:00
#include <Arduino.h>
#include "fonctions.h"
#include "sensor_task.h"
#include "tourelle.h"

// --- Global Variables ---
RobotState currentState;
unsigned long lastActionTime;
bool actionStarted;
NavigationMode currentNavMode;
bool hasReculed;
bool hasTurned;
MX1508 motorA(AIN1, AIN2);
MX1508 motorB(BIN1, BIN2);
UltraDistSensor sonic;
Servo Servodirection;
Tourelle tourelle(PINTOURELLE_H, PINTOURELLE_V);
LSM303 compass;
DFRobot_RGBLCD1602 lcd(0x60 ,16,2);
String lcdText;
int vitesseCible;
int vitesseCourante;
float capCibleRotation;
int manualDistance;
int manualStartDistance;
bool manualMovementActive;
float compassOffset;

bool compassCalibrated;
bool compassInitialized;
bool compassInverted;
int dusm;
byte neutredirection;
byte neutretourelle;
byte dmarge, darret;
byte vmini;
int Ncap, cap;

// Variables for 360-degree calibration
LSM303::vector<int16_t> magMin = {32767, 32767, 32767};
LSM303::vector<int16_t> magMax = {-32768, -32768, -32768};

// --- Profiling Variables ---
unsigned long loopStartTime, loopEndTime;
unsigned long terminalTime, sensorTaskTime, motorControlTime;
unsigned long lastReportTime = 0;
const unsigned long reportInterval = 2000; // Report every 2 seconds

void setLcdText(const String& text) {
    lcd.clear();
    lcd.print(text);
    lcdText = text;
}

void setup() {            //SETUP
  Serial.begin(115200); // Initialize serial communication for Bluno BLE module
  lcd.init();
  Wire.begin();
  if (DEBUG_MODE) Serial.println("1 setup ");
  
  // Initialize global variables
  currentState = IDLE;
  currentNavMode = MANUAL_CONTROL;
  neutredirection = NEUTRE_DIRECTION;
  neutretourelle = NEUTRE_TOURELLE;
  dmarge = DMARGE;
  darret = DARRET;
  vmini = VMINI;
  cap = INITIAL_CAP;
  Ncap = INITIAL_NCAP;

  //balise_rouge();
  
  // Initialisation de la tâche de capteur non-bloquante
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO), echo_isr, CHANGE);
  if (DEBUG_MODE) Serial.println("2. Tâche de capteur ultrasonique initialisée (non-bloquante)");
  
  setLcdText("Je suis NONO present");
  
  Batterie();
  PhareAllume();

  pinMode(PIR, INPUT);
  
#if ENABLE_LEDS
  // Configure LED pins
  pinMode(LED_ROUGE, OUTPUT);
  pinMode(LED_JAUNE, OUTPUT);
  digitalWrite(LED_ROUGE, LOW);
  digitalWrite(LED_JAUNE, LOW);
  if (DEBUG_MODE) Serial.println("LEDs configurees");
#else
  if (DEBUG_MODE) Serial.println("LEDs desactivees (hardware non installe)");
#endif

  if (DEBUG_MODE) Serial.print("4 compass : ");
  
  // Initialisation du compas avec gestion d'erreur
  if (DEBUG_MODE) {
    Serial.println("Initialisation du compas...");
    Serial.println("Tentative d'initialisation LSM303...");
  }
  
  // Essayer d'initialiser le compas
  bool initResult = compass.init();
  if (DEBUG_MODE) {
    Serial.print("Resultat init(): ");
    Serial.println(initResult ? "SUCCES" : "ECHEC");
  }
  
  if (!initResult) {
    if (DEBUG_MODE) {
      Serial.println("ERREUR: Impossible d'initialiser le compas!");
    }
    
    setLcdText("ERREUR COMPAS");
    
    // Continuer sans compas pour debug
    if (DEBUG_MODE) Serial.println("Continuation sans compas pour debug...");
  } else {
    if (DEBUG_MODE) Serial.println("Compas initialise avec succes!");
    compassInitialized = true; // Mark compass as successfully initialized
    
    // Charger la calibration depuis l'EEPROM
    loadCompassCalibration();
  }
  
  // Only proceed with compass operations if initialization succeeded
  if (initResult) {
    if (DEBUG_MODE) Serial.println("Activation du compas...");
    compass.enableDefault();
    if (DEBUG_MODE) Serial.println("enableDefault() execute");
    
    if (DEBUG_MODE) Serial.println("Test initial du compas...");
    compass.read();
    
    // Vérifier si les valeurs du compas sont valides
    if (compass.m.x == 0 && compass.m.y == 0 && compass.m.z == 0) {
      if (DEBUG_MODE) Serial.println("ERREUR: Valeurs compas nulles!");
      setLcdText("COMPAS ERREUR");
    } else {
      float initialHeading = calculateHeading(compass);
      if (DEBUG_MODE) {
        Serial.print("Cap initial: ");
        Serial.println(initialHeading);
      }
      
      // Afficher les informations du compas
      if (DEBUG_MODE) Serial.println("Affichage des infos compas...");
      displayCompassInfo();
    }
  } else {
    if (DEBUG_MODE) Serial.println("Compas non initialise - fonctionnalites limitees");
    setLcdText("Compas non init");
  }
  
  if (DEBUG_MODE) Serial.println("5 moteurs"); 
  pinMode(VBAT, INPUT);
  pinMode(INTERUPTPIN, INPUT_PULLUP);

  Servodirection.attach(PINDIRECTION,70,105);
#if ENABLE_TOWER
  tourelle.attach();
  tourelle.write(SCAN_CENTER_ANGLE, 90); // Positionner la tourelle au centre
  if (DEBUG_MODE) Serial.println("6 DIRECTION + TOURELLE");
#else
  if (DEBUG_MODE) Serial.println("6 DIRECTION (tourelle desactivee)");
#endif
  Servodirection.write(neutredirection);
  
  PhareAllume();

  if (DEBUG_MODE) Serial.println("Fin setup");
}

void loop() {
  // Toujours écouter les commandes
  Terminal(); 

  // Tâche de mise à jour des capteurs (non-bloquante)
  sensor_update_task();

  // Main state machine
  switch (currentState) {
    case CALIBRATING_COMPASS:
      calibrateCompass(); // Call continuously until it finishes
      break;

    // Add other states here, e.g., AUTONOMOUS, MANUAL_CONTROL
    // case AUTONOMOUS:
    //   run_autonomous_logic();
    //   break;

    default: // For IDLE and other operational states
      updateMotorControl(); // Centralized motor control with acceleration ramp
      Mcap(10);             // Non-blocking compass reading (10 samples)
      sendPeriodicData();
      break;
  }
}