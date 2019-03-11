//Bibliothèques concernant l'enregistrement des données sur la carte SD
#include <SD.h>
#include <SPI.h>
#include <DS3231.h> //Librairie à installer

//Bibliothèques concernant le fonctionnement du capteur BME280
#include "SparkFunBME280.h"
//Library allows either I2C or SPI, so include both.
#include "Wire.h"

//Bibliothèque concernant la sonde DS18B20
#include <OneWire.h>

//Bibliothèques concernant la mise en veille de l'Arduino
#include <avr/sleep.h>
#include <avr/power.h>
#include <DS3231_Simple.h>
DS3231_Simple Clock;

BME280 capteur;

/**************************************
 * Affectation des pins
 */
#define CS_PIN 10                 //Pin pour la communication avec la carte SD
const byte BROCHE_ONEWIRE = 7;    //Pin pour la communication avec la sonde DS18B20
int pin2 = 2;                     //Pin destiné à receuillir les interruptions extérieures

/**************************************
 * Création des instances
 */
File monFichier;
DS3231  monRTC(SDA, SCL);

/**************************************
 * Initialisation
 */


//Préparation de la communication avec la sonde DS18B20
enum DS18B20_RCODES {
  READ_OK,  // Lecture ok
  NO_SENSOR_FOUND,  // Pas de capteur
  INVALID_ADDRESS,  // Adresse reçue invalide
  INVALID_SENSOR  // Capteur invalide (pas un DS18B20)
};


/* Création de l'objet OneWire pour manipuler le bus 1-Wire */
OneWire ds(BROCHE_ONEWIRE);
 
byte getTemperature(float *temperature, byte reset_search) {
  byte data[9], addr[8];
  // data[] : Données lues depuis le scratchpad
  // addr[] : Adresse du module 1-Wire détecté
  
  /* Reset le bus 1-Wire ci nécessaire (requis pour la lecture du premier capteur) */
  if (reset_search) {
    ds.reset_search();
  }
 
  /* Recherche le prochain capteur 1-Wire disponible */
  if (!ds.search(addr)) {
    // Pas de capteur
    return NO_SENSOR_FOUND;
  }
  
  /* Vérifie que l'adresse a été correctement reçue */
  if (OneWire::crc8(addr, 7) != addr[7]) {
    return INVALID_ADDRESS;
  }
 
  /* Vérifie qu'il s'agit bien d'un DS18B20 */
  if (addr[0] != 0x28) {
    return INVALID_SENSOR;
  }
 
  /* Reset le bus 1-Wire et sélectionne le capteur */
  ds.reset();
  ds.select(addr);
  
  /* Lance une prise de mesure de température et attend la fin de la mesure */
  ds.write(0x44, 1);
  delay(800);
  
  /* Reset le bus 1-Wire, sélectionne le capteur et envoie une demande de lecture du scratchpad */
  ds.reset();
  ds.select(addr);
  ds.write(0xBE);
 
 /* Lecture du scratchpad */
  for (byte i = 0; i < 8; i++) {
    data[i] = ds.read();
  }
   
  /* Calcul de la température en degré Celsius */
  *temperature = (int16_t) ((data[1] << 8) | data[0]) * 0.0625; 
  return READ_OK;
}

void pin2Interrupt(void)
{
  /* This will bring us back from sleep. */
  //toggle = 1;
  /* We detach the interrupt to stop it from 
   * continuously firing while the interrupt pin
   * is low.
   */
  //detachInterrupt(0);
  delay(100);
}

void enterSleep(void)
{
  
  /* Setup pin2 as an interrupt and attach handler. */
  attachInterrupt(0, pin2Interrupt, FALLING);
  Clock.disableAlarms();
  DateTime MyTimestamp = Clock.read();
  MyTimestamp = Clock.read();
  MyTimestamp.Second = 10;
  Clock.setAlarm(MyTimestamp, DS3231_Simple::ALARM_MATCH_MINUTE_HOUR);
  //Clock.setAlarm(DS3231_Simple::ALARM_EVERY_DAY); 
  Serial.println("bonne nuit!");
  delay(100);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  
  /* The program will continue from here. */
  /* First thing to do is disable sleep. */
  sleep_disable(); 
}

void setup()
{
  Serial.begin(9600);
  monRTC.begin();

  while (!Serial) {
    // Attente de l'ouverture du port série pour Arduino LEONARDO
  }
  //configuration du capteur
  capteur.settings.commInterface = I2C_MODE; 
  capteur.settings.I2CAddress = 0x76;
  capteur.settings.runMode = 3; 
  capteur.settings.tStandby = 0;
  capteur.settings.filter = 0;
  capteur.settings.tempOverSample = 1 ;
  capteur.settings.pressOverSample = 1;
  capteur.settings.humidOverSample = 1;
 
  Serial.println("Starting BME280... ");
  delay(10);  // attente de la mise en route du capteur. 2 ms minimum
  // chargement de la configuration du capteur
  capteur.begin();

  Serial.print("Initialisation de la carte SD en cours...");
  if (SD.begin())
  {
    Serial.println(" Terminee.");
  } else
  {
    Serial.println(" Echec.");
    return;
  }  
  monFichier = SD.open("donnees.csv", FILE_WRITE);
  if (monFichier) 
  {   
    monFichier.println("Heure,Temperature A,Pression,Humidite, Temperature W, Turbidite");
    Serial.println("");
    Serial.println("   Heure  |  Temperature A  |  Pression  |  Humidite  |  Temperature W  | Turbidity ");
    monFichier.close();    
  } 
}
/**************************************
 * Boucle Infinie
 */
void loop()
{  
  //On lit la température avec la sonde DS18B20
  float temperature; 
  /* Lit la température ambiante à ~1Hz */
  if (getTemperature(&temperature, true) != READ_OK) {
    Serial.println(F("Erreur de lecture du capteur"));
    return;
  }
  
  // On mesure la température, la pression et l'humidité avec le BME280
  float Temperature = capteur.readTempC();
  float pression = capteur.readFloatPressure();
  float humidite = capteur.readFloatHumidity();  

  //On mesure la turbidité
  int sensorValue = analogRead(A0);             //Lecture de l'information
  float voltage = sensorValue * (5.0/1024.0);   //Conversion de l'information
  
  // On demande l'heure exacte au module Real Time Clock.
  String temps = String(monRTC.getTimeStr());

  // On met en forme la donnée au formar csv, c'est-à dire chaque paramètre séparé par une virgule.
  String donnee = temps + "," + String(Temperature) + "," + String(pression) + "," + String(humidite)+","+String(temperature)+","+String(voltage);

  // On enregistre la donnée 
  monFichier = SD.open("donnees.csv", FILE_WRITE); //Maximum 8 caractères avant le .csv
  if (monFichier) 
  {   
    monFichier.println(donnee);
    Serial.println(" " + temps + " |    " + String(Temperature)+ "    |   " + String(pression)+ "    |   " + String(humidite)+ "    |   "  + String(temperature)+ "    |   "  + String(voltage));
    monFichier.close();    
  } 
  else 
  {
    Serial.println("Impossible d'ouvrir le fichier");
  } 
  enterSleep(); 
  delay(1000);
}
