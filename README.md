# Arduino Carte Température — ESP32-C3 + BLE + IHM PyQt

Acquisition de température en temps réel : une carte **ESP32-C3 FH4** mesure la
température (sonde PT1000) et la transmet en **Bluetooth Low Energy (BLE)** à
une **IHM PyQt** qui l'affiche sous forme de courbe temps réel.

```
┌──────────────────┐   BLE (notifications)   ┌──────────────────────┐
│  ESP32-C3 FH4    │ ──────────────────────► │  IHM PyQt (PC)       │
│  sonde PT1000    │   float 32 bits, 1 Hz   │  courbe + stats +CSV │
└──────────────────┘                         └──────────────────────┘
```

## Contenu du dépôt

| Dossier | Description |
|---|---|
| `firmware/ESP32C3_TempBLE/` | Sketch Arduino à flasher sur l'ESP32-C3 |
| `ihm/` | Application PyQt d'acquisition et son `requirements.txt` |

---

## 1. Firmware ESP32-C3

### Fonctionnement

Le firmware est une machine à états **non bloquante** (pas de `delay()` longs,
tout est cadencé avec `millis()`), ce qui simplifie et fiabilise le code
d'origine :

| État | LED | Description |
|---|---|---|
| **VEILLE** | Allumée fixe | Un appui sur le bouton active le Bluetooth |
| **PUBLICITE** | Clignotement lent | Carte visible en BLE pendant 30 s, sinon retour en veille |
| **CONNECTE** | Clignotement rapide | Envoi de la température toutes les secondes ; appui bouton ou déconnexion → retour en veille |

La température est envoyée sous forme d'un **float 32 bits little-endian**
(4 octets), ce qui conserve les décimales (l'ancien code tronquait en entier).

### Câblage

| Broche ESP32-C3 | Élément |
|---|---|
| GPIO 5 | Bouton poussoir vers GND (pull-up interne activé, appuyé = LOW) |
| GPIO 10 | LED d'état (avec résistance série) |
| GPIO 3 | Point milieu du pont diviseur : 3,3 V → R 500 Ω → **GPIO 3** → PT1000 → GND |

Conversion utilisée : `R = V × 500 / (3,3 − V)` puis `T = (R − 1000) / 3,9` (°C).

### Compilation / flash

1. Dans l'IDE Arduino, installer le support **esp32** (Boards Manager, paquet Espressif).
2. Sélectionner la carte **ESP32C3 Dev Module**.
3. Ouvrir `firmware/ESP32C3_TempBLE/ESP32C3_TempBLE.ino`, compiler et téléverser.
4. Ouvrir le moniteur série à **115200 bauds** pour suivre les états et les mesures.

---

## 2. IHM PyQt

### Installation

```bash
cd ihm
python -m venv .venv
source .venv/bin/activate        # Windows : .venv\Scripts\activate
pip install -r requirements.txt
```

> Nécessite un PC avec Bluetooth (BLE). Sous Linux, BlueZ doit être installé.

### Utilisation

1. Lancer l'application :
   ```bash
   python temperature_monitor.py
   ```
2. **Appuyer sur le bouton de la carte** pour activer le Bluetooth (LED clignote lentement).
3. Cliquer sur **« Se connecter »** dans l'IHM : elle recherche la carte
   `ESP32-TEMP`, se connecte et l'acquisition démarre (LED clignote rapidement).

Fonctionnalités :
- température courante affichée en grand,
- statistiques min / max / moyenne,
- courbe temps réel (zoom/déplacement à la souris),
- bouton **Effacer** pour remettre l'acquisition à zéro,
- bouton **Exporter CSV** pour sauvegarder les mesures.

## Dépannage

**« A fatal error occurred: This chip is ESP32-C3, not ESP32 » au téléversement**
Mauvaise carte sélectionnée dans l'IDE : choisir **ESP32C3 Dev Module** (et non
« ESP32 Dev Module »), avec **USB CDC On Boot : Enabled**.

**La carte est visible en BLE mais aucune connexion n'aboutit** (bloqué sur
« Connecting » depuis le PC comme depuis un téléphone, alors que la publicité
est reçue avec un bon signal)
Cause vécue sur ce projet : données résiduelles corrompues en flash
(calibration RF / NVS) laissées par d'anciens programmes. Correctif :
**Outils → Erase All Flash Before Sketch Upload → Enabled**, re-téléverser,
puis remettre l'option sur Disabled.

**L'IHM affiche ~529 °C constant**
Entrée analogique en circuit ouvert : la PT1000 est absente ou débranchée
(GPIO 3 tiré à 3,3 V, ADC saturé). Vérifier le pont diviseur ; pour tester
sans sonde, placer une résistance de 1 kΩ entre GPIO 3 et GND → l'IHM doit
afficher ≈ 0 °C.

**Échec du téléversement (« Failed to connect »)**
Forcer le mode bootloader : maintenir **BOOT** (GPIO 9), appuyer brièvement
sur **RST**, relâcher BOOT, relancer le téléversement.

### Paramètres BLE (communs firmware / IHM)

| Paramètre | Valeur |
|---|---|
| Nom BLE | `ESP32-TEMP` |
| Service UUID | `4fafc201-1fb5-459e-8fcc-c5c9c331914b` |
| Caractéristique UUID | `beb5483e-36e1-4688-b7f5-ea07361b26a8` |
| Format des données | float 32 bits little-endian (°C) |
| Cadence | 1 mesure / seconde |
