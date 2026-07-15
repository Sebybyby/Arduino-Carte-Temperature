/*
 * ESP32C3_TempBLE.ino
 * -------------------
 * Firmware pour ESP32-C3 FH4 : acquisition de température (sonde PT1000)
 * et transmission en temps réel via Bluetooth Low Energy (BLE).
 *
 * Fonctionnement (machine à états simplifiée, non bloquante) :
 *   VEILLE     : LED allumée fixe. Un appui sur le bouton active le Bluetooth.
 *   PUBLICITE  : LED clignote lentement. La carte est visible en BLE pendant
 *                30 s. Sans connexion, retour en VEILLE.
 *   CONNECTE   : LED clignote rapidement. La température est mesurée et
 *                notifiée toutes les secondes au client connecté (IHM PyQt).
 *                Un appui sur le bouton, ou une déconnexion du client,
 *                ramène la carte en VEILLE.
 *
 * La température est transmise sous forme d'un float 32 bits little-endian
 * (4 octets), ce qui conserve les décimales contrairement à un entier.
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_phy_init.h"

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
#define NOM_BLE             "ESP32-TEMP"
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

constexpr int PIN_BOUTON  = 5;   // Bouton poussoir (appuyé = LOW, pull-up interne)
constexpr int PIN_LED     = 10;  // LED d'état
constexpr int PIN_CAPTEUR = 3;   // Entrée analogique de la sonde PT1000

constexpr unsigned long DUREE_PUBLICITE_MS   = 120000; // Visibilité BLE : 2 min
constexpr unsigned long PERIODE_MESURE_MS    = 1000;  // Une mesure par seconde
constexpr unsigned long CLIGNOTEMENT_LENT_MS = 800;   // Période LED en publicité
constexpr unsigned long CLIGNOTEMENT_RAPIDE_MS = 150; // Période LED connecté
constexpr unsigned long ANTI_REBOND_MS       = 250;   // Anti-rebond du bouton

// Pont diviseur de la sonde : R = V * 500 / (3.3 - V), T = (R - 1000) / 3.9
constexpr float R_SERIE_OHMS   = 500.0f;
constexpr float TENSION_ALIM_V = 3.3f;

// ---------------------------------------------------------------------------
// États
// ---------------------------------------------------------------------------
enum Etat { VEILLE, PUBLICITE, CONNECTE };

Etat etat = VEILLE;
volatile bool clientConnecte = false;

BLEServer*         pServeur        = nullptr;
BLECharacteristic* pCaracteristique = nullptr;

unsigned long debutPublicite   = 0;
unsigned long derniereMesure   = 0;
unsigned long dernierClignote  = 0;
unsigned long dernierAppui     = 0;
bool          etatLed          = false;

// ---------------------------------------------------------------------------
// Callbacks BLE : mise à jour du drapeau de connexion
// ---------------------------------------------------------------------------
class CallbacksServeur : public BLEServerCallbacks {
  void onConnect(BLEServer*) override    { clientConnecte = true; }
  void onDisconnect(BLEServer*) override { clientConnecte = false; }
};

// ---------------------------------------------------------------------------
// Fonctions utilitaires
// ---------------------------------------------------------------------------

// Détecte un appui sur le bouton (front descendant, avec anti-rebond)
bool boutonAppuye() {
  if (digitalRead(PIN_BOUTON) == LOW && millis() - dernierAppui > ANTI_REBOND_MS) {
    dernierAppui = millis();
    return true;
  }
  return false;
}

// Fait clignoter la LED sans bloquer le programme
void clignoterLed(unsigned long periodeMs) {
  if (millis() - dernierClignote >= periodeMs) {
    dernierClignote = millis();
    etatLed = !etatLed;
    digitalWrite(PIN_LED, etatLed);
  }
}

// Lit la sonde et convertit la tension en température (°C)
float lireTemperature() {
  float tension = analogReadMilliVolts(PIN_CAPTEUR) / 1000.0f;
  float resistance = tension * R_SERIE_OHMS / (TENSION_ALIM_V - tension);
  float temperature = (resistance - 1000.0f) / 3.9f;

  Serial.printf("Tension : %.3f V | Température : %.2f °C\n", tension, temperature);
  return temperature;
}

// Envoie la température au client BLE (float 32 bits, little-endian)
void notifierTemperature(float temperature) {
  pCaracteristique->setValue((uint8_t*)&temperature, sizeof(temperature));
  pCaracteristique->notify();
}

// Changement d'état centralisé (gère la LED et la publicité BLE)
void changerEtat(Etat nouvelEtat) {
  etat = nouvelEtat;
  switch (etat) {
    case VEILLE:
      BLEDevice::stopAdvertising();
      if (clientConnecte) {
        // Déconnecter proprement le client encore connecté (ex. sortie par
        // appui bouton), sinon l'IHM resterait connectée sans recevoir de données
        pServeur->disconnect(pServeur->getConnId());
      }
      digitalWrite(PIN_LED, HIGH);
      Serial.println("[VEILLE] Appuyez sur le bouton pour activer le Bluetooth.");
      break;
    case PUBLICITE:
      debutPublicite = millis();
      BLEDevice::startAdvertising();
      Serial.printf("[PUBLICITE] Bluetooth actif, en attente d'une connexion (%lu s)...\n",
                    DUREE_PUBLICITE_MS / 1000);
      break;
    case CONNECTE:
      Serial.println("[CONNECTE] Client connecté, envoi de la température...");
      break;
  }
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_BOUTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  analogReadResolution(12);

  // Force une calibration RF complète à chaque démarrage. Des données de
  // calibration corrompues en flash rendent la connexion BLE impossible sur
  // cette carte (publicité visible mais liaison qui tombe aussitôt,
  // erreur « BT_HCI: CC evt: op=0x2022, status=0x2 »).
  esp_phy_erase_cal_data_in_nvs();

  // Initialisation du serveur BLE
  BLEDevice::init(NOM_BLE);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P3);      // Puissance max
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P3);

  pServeur = BLEDevice::createServer();
  pServeur->setCallbacks(new CallbacksServeur());

  BLEService* pService = pServeur->createService(SERVICE_UUID);
  pCaracteristique = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCaracteristique->addDescriptor(new BLE2902()); // Nécessaire pour les notifications
  pService->start();

  // Configuration de la publicité (faite une seule fois)
  BLEAdvertising* pPub = BLEDevice::getAdvertising();
  pPub->addServiceUUID(SERVICE_UUID);
  pPub->setScanResponse(false);
  pPub->setMinInterval(80);   // 50 ms
  pPub->setMaxInterval(160);  // 100 ms

  Serial.printf("Adresse BLE de la carte : %s\n",
                BLEDevice::getAddress().toString().c_str());
  changerEtat(VEILLE);
}

// ---------------------------------------------------------------------------
// Boucle principale (non bloquante)
// ---------------------------------------------------------------------------
void loop() {
  switch (etat) {

    case VEILLE:
      // Rattrape une connexion qui aboutit juste après le retour en veille
      // (négociation commencée en fin de fenêtre de publicité)
      if (clientConnecte) {
        changerEtat(CONNECTE);
      } else if (boutonAppuye()) {
        changerEtat(PUBLICITE);
      }
      break;

    case PUBLICITE:
      clignoterLed(CLIGNOTEMENT_LENT_MS);
      if (clientConnecte) {
        changerEtat(CONNECTE);
      } else if (millis() - debutPublicite >= DUREE_PUBLICITE_MS) {
        Serial.println("Aucune connexion, retour en veille.");
        changerEtat(VEILLE);
      }
      break;

    case CONNECTE:
      clignoterLed(CLIGNOTEMENT_RAPIDE_MS);

      // Mesure et envoi périodiques
      if (millis() - derniereMesure >= PERIODE_MESURE_MS) {
        derniereMesure = millis();
        notifierTemperature(lireTemperature());
      }

      // Retour en veille : appui bouton ou déconnexion du client
      if (boutonAppuye() || !clientConnecte) {
        Serial.println("Fin de session (bouton ou déconnexion).");
        changerEtat(VEILLE);
      }
      break;
  }

  delay(10); // Petite pause pour laisser respirer le processeur
}
