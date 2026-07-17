/*
 * ESP32C3_TempBLE_Ratiometrique.ino
 * ---------------------------------
 * Firmware ESP32-C3 : acquisition PT1000 + transmission BLE, avec mesure
 * RATIOMÉTRIQUE : un second canal ADC surveille en continu la tension réelle
 * du rail 3,3 V (via un pont diviseur par 2). Les variations du rail
 * s'annulent alors dans le calcul — plus de constante TENSION_ALIM_V à
 * régler au millivolt, plus de dérive d'une session à l'autre.
 *
 * MONTAGE REQUIS (en plus du pont capteur existant) :
 *   3,3 V -> 10 kOhm (1 %) -> [PIN_REF_RAIL] -> 10 kOhm (1 %) -> GND
 *
 * Machine à états identique à la version standard :
 *   VEILLE (LED fixe) -> bouton -> PUBLICITE (LED lente, 2 min)
 *   -> connexion -> CONNECTE (LED rapide, 1 mesure/s) -> bouton ou
 *   déconnexion -> VEILLE.
 *
 * Température transmise en float 32 bits little-endian (4 octets).
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

constexpr int PIN_BOUTON   = 5;   // Bouton poussoir (appuyé = LOW, pull-up interne)
constexpr int PIN_LED      = 10;  // LED d'état
constexpr int PIN_CAPTEUR  = 3;   // Entrée analogique de la sonde PT1000

// >>> À ADAPTER : GPIO ADC libre relié au point milieu du pont 10k/10k <<<
// Sur ESP32-C3, l'ADC n'existe que sur les GPIO 0, 1, 2, 3 (pris) et 4.
constexpr int PIN_REF_RAIL = 4;

// Rapport du pont de référence (2.0 pour deux résistances identiques).
// Étalonnage fin : ajuster de quelques % selon la tolérance réelle des 10 k.
constexpr float RATIO_PONT_REF = 2.0f;

constexpr unsigned long DUREE_PUBLICITE_MS     = 120000; // Visibilité BLE : 2 min
constexpr unsigned long PERIODE_MESURE_MS      = 1000;   // Une mesure par seconde
constexpr unsigned long CLIGNOTEMENT_LENT_MS   = 800;    // Période LED en publicité
constexpr unsigned long CLIGNOTEMENT_RAPIDE_MS = 150;    // Période LED connecté
constexpr unsigned long ANTI_REBOND_MS         = 250;    // Anti-rebond du bouton

// Pont capteur : V_rail -> R série 499 ohms -> GPIO 3 -> PT1000 -> GND
constexpr float R_SERIE_OHMS = 499.0f;  // Valeur réelle sur le PCB (E96)

// Étalonnage fin résiduel : R_corrigée = R_mesurée × GAIN + OFFSET
// À caler en dernier avec un thermomètre de référence (1 °C = 3,9 ohms).
constexpr float ETALONNAGE_GAIN   = 1.0f;
constexpr float ETALONNAGE_OFFSET = 0.0f;  // en ohms
constexpr int   NB_LECTURES_ADC   = 16;    // moyenne tronquée anti-bruit

// Puissance d'émission BLE réduite : ce PCB ne tient pas la connexion à
// +3 dBm (chute du rail pendant les rafales RF). À réévaluer après ajout
// du condensateur de découplage / révision du régulateur.
constexpr esp_power_level_t PUISSANCE_TX = ESP_PWR_LVL_N12;

// ---------------------------------------------------------------------------
// États
// ---------------------------------------------------------------------------
enum Etat { VEILLE, PUBLICITE, CONNECTE };

Etat etat = VEILLE;
volatile bool clientConnecte = false;

BLEServer*         pServeur         = nullptr;
BLECharacteristic* pCaracteristique = nullptr;

unsigned long debutPublicite  = 0;
unsigned long derniereMesure  = 0;
unsigned long dernierClignote = 0;
unsigned long dernierAppui    = 0;
bool          etatLed         = false;

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

// Moyenne tronquée d'un canal ADC : n lectures espacées de pasMs,
// on écarte le quart le plus bas et le quart le plus haut (rejette les
// échantillons pollués par les rafales d'émission BLE), moyenne du reste.
// Retourne des millivolts.
float lireCanalTronque(int pin, int n, unsigned long pasMs) {
  uint32_t lectures[NB_LECTURES_ADC];
  for (int i = 0; i < n; i++) {
    lectures[i] = analogReadMilliVolts(pin);
    delay(pasMs);
  }
  // Tri par insertion
  for (int i = 1; i < n; i++) {
    uint32_t v = lectures[i];
    int j = i - 1;
    while (j >= 0 && lectures[j] > v) { lectures[j + 1] = lectures[j]; j--; }
    lectures[j + 1] = v;
  }
  // Moyenne de la moitié centrale
  int debut = n / 4, fin = n - n / 4;
  uint32_t somme = 0;
  for (int i = debut; i < fin; i++) somme += lectures[i];
  return somme / (float)(fin - debut);
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

// Lit la sonde et convertit en température (°C), en mesurant AUSSI le rail :
// ses variations s'annulent dans le rapport tension capteur / tension rail.
float lireTemperature() {
  // Canal capteur : 16 lectures étalées sur ~200 ms
  float tension = lireCanalTronque(PIN_CAPTEUR, NB_LECTURES_ADC, 12) / 1000.0f;
  // Canal référence rail : 8 lectures rapprochées suffisent
  float tensionRail = lireCanalTronque(PIN_REF_RAIL, 8, 2) / 1000.0f * RATIO_PONT_REF;

  float resistance = tension * R_SERIE_OHMS / (tensionRail - tension);
  resistance = resistance * ETALONNAGE_GAIN + ETALONNAGE_OFFSET;
  float temperature = resistanceVersTemperature(resistance);

  Serial.printf("V: %.3f V | Vrail: %.3f V | R: %.1f ohms | T: %.2f C\n",
                tension, tensionRail, resistance, temperature);
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
        // Déconnecter proprement le client encore connecté, sinon l'IHM
        // resterait connectée sans recevoir de données
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

  // Calibration RF complète à chaque démarrage (données de calibration en
  // flash corrompues = connexion BLE impossible sur cette carte).
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
