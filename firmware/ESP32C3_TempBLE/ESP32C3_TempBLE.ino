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

// Pont diviseur de la sonde : 3,3 V -> R série 499 ohms -> GPIO 3 -> PT1000 -> GND
// R = V * R_SERIE / (V_alim - V), puis T = (R - 1000) / 3.9
constexpr float R_SERIE_OHMS   = 499.0f;  // Valeur réelle sur le PCB (E96)
constexpr float TENSION_ALIM_V = 3.129f;  // Tension réelle du rail, calée sur référence.
                                          // Réglage : T affichée trop HAUTE -> augmenter,
                                          // trop BASSE -> diminuer. 10 mV = ~3 °C.

// Étalonnage fin résiduel : R_corrigée = R_mesurée × GAIN + OFFSET
// À caler en dernier avec un thermomètre de référence (1 °C = 3,9 ohms d'offset).
constexpr float ETALONNAGE_GAIN   = 1.0f;
constexpr float ETALONNAGE_OFFSET = 0.0f;  // en ohms
constexpr int   NB_LECTURES_ADC   = 16;    // moyennage anti-bruit (moyenne tronquée)

// Puissance d'émission BLE. Ce PCB ne tient pas la connexion à +3 dBm
// (l'appel de courant de l'ampli RF fait décrocher la liaison : publicité
// visible mais connexions qui tombent en BLE_HCI_CONN_FAILED_TO_BE_ESTABLISHED).
// À -12 dBm la connexion est fiable, au prix d'une portée de quelques mètres.
// À réévaluer après révision matérielle (découplage 3,3 V / régulateur).
constexpr esp_power_level_t PUISSANCE_TX = ESP_PWR_LVL_N12;

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

// Conversion résistance -> température selon IEC 60751 (Callendar-Van Dusen)
// pour élément platine 1000 ohms à 0 °C, 3850 ppm/K (famille TE NB-PTCO).
// R = R0·(1 + A·t + B·t²), inversé en t. Valide de -50 à +250 °C.
constexpr float PT_R0 = 1000.0f;
constexpr float PT_A  = 3.9083e-3f;
constexpr float PT_B  = -5.775e-7f;

float resistanceVersTemperature(float r) {
  float discriminant = PT_A * PT_A - 4.0f * PT_B * (1.0f - r / PT_R0);
  return (-PT_A + sqrtf(discriminant)) / (2.0f * PT_B);
}

// Lit la sonde et convertit la tension en température (°C)
// Moyenne tronquée : 16 lectures, on écarte les 4 plus basses et les 4 plus
// hautes (rejette les valeurs polluées par les rafales d'émission BLE),
// puis on moyenne les 8 restantes.
float lireTemperature() {
  // Lectures étalées sur ~200 ms : une rafale d'émission BLE (~10 ms) ne
  // pollue ainsi qu'une minorité d'échantillons, que la moyenne tronquée
  // rejette. Des lectures trop rapprochées tomberaient toutes dedans.
  uint32_t lectures[NB_LECTURES_ADC];
  for (int i = 0; i < NB_LECTURES_ADC; i++) {
    lectures[i] = analogReadMilliVolts(PIN_CAPTEUR);
    delay(12);
  }

  // Tri par insertion (16 valeurs : simple et suffisant)
  for (int i = 1; i < NB_LECTURES_ADC; i++) {
    uint32_t v = lectures[i];
    int j = i - 1;
    while (j >= 0 && lectures[j] > v) { lectures[j + 1] = lectures[j]; j--; }
    lectures[j + 1] = v;
  }

  // Moyenne des 8 valeurs centrales
  uint32_t somme_mV = 0;
  for (int i = 4; i < NB_LECTURES_ADC - 4; i++) somme_mV += lectures[i];
  float tension = (somme_mV / 8.0f) / 1000.0f;

  float resistance = tension * R_SERIE_OHMS / (TENSION_ALIM_V - tension);
  resistance = resistance * ETALONNAGE_GAIN + ETALONNAGE_OFFSET;
  float temperature = resistanceVersTemperature(resistance);

  Serial.printf("Tension : %.3f V | Resistance : %.1f ohms | Temperature : %.2f C\n",
                tension, resistance, temperature);
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
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, PUISSANCE_TX);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, PUISSANCE_TX);

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
