#!/usr/bin/env python3
"""
temperature_monitor.py
----------------------
IHM PyQt5 d'acquisition de température en temps réel via Bluetooth Low Energy.

Se connecte à la carte ESP32-C3 (firmware ESP32C3_TempBLE.ino), s'abonne aux
notifications BLE et affiche la température :
  - valeur courante en grand,
  - statistiques (min / max / moyenne),
  - courbe temps réel,
  - export CSV des mesures.

Dépendances : PyQt5, pyqtgraph, bleak  (voir requirements.txt)

Usage :
    python temperature_monitor.py
"""

import asyncio
import csv
import struct
import sys
import time

from bleak import BleakClient, BleakScanner
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

# --- Paramètres BLE (doivent correspondre au firmware) -----------------------
NOM_CARTE = "ESP32-TEMP"
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
DUREE_SCAN_S = 15.0


# =============================================================================
# Thread BLE : fait tourner bleak (asyncio) sans bloquer l'interface Qt
# =============================================================================
class BleWorker(QtCore.QThread):
    """Scan, connexion et réception des notifications BLE dans un thread dédié."""

    temperature_recue = QtCore.pyqtSignal(float)
    statut_change = QtCore.pyqtSignal(str)
    connexion_changee = QtCore.pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._loop = None
        self._tache = None
        self._stop_event = None

    # --- Exécuté dans le thread ---------------------------------------------
    def run(self):
        try:
            asyncio.run(self._session_ble())
        except asyncio.CancelledError:  # arrêt demandé par l'utilisateur
            self.statut_change.emit("Arrêté.")
        except Exception as exc:  # erreur BLE inattendue
            self.statut_change.emit(f"Erreur : {exc}")
        finally:
            self.connexion_changee.emit(False)

    async def _session_ble(self):
        self._loop = asyncio.get_running_loop()
        self._tache = asyncio.current_task()
        self._stop_event = asyncio.Event()

        # 1) Recherche de la carte
        self.statut_change.emit(f"Recherche de « {NOM_CARTE} »… "
                                "(appuyez sur le bouton de la carte)")
        appareil = await BleakScanner.find_device_by_name(
            NOM_CARTE, timeout=DUREE_SCAN_S)
        if appareil is None:
            self.statut_change.emit(
                "Carte introuvable. Vérifiez que le Bluetooth de la carte "
                "est activé (bouton) puis réessayez.")
            return

        # 2) Connexion et abonnement aux notifications
        self.statut_change.emit(f"Connexion à {appareil.address}…")

        def _sur_deconnexion(_client):
            self._loop.call_soon_threadsafe(self._stop_event.set)

        async with BleakClient(appareil,
                               disconnected_callback=_sur_deconnexion) as client:
            await client.start_notify(CHARACTERISTIC_UUID, self._sur_notification)
            self.connexion_changee.emit(True)
            self.statut_change.emit(f"Connecté à {NOM_CARTE} — acquisition en cours")

            # 3) Attente : arrêt demandé par l'utilisateur ou déconnexion carte
            await self._stop_event.wait()

        self.statut_change.emit("Déconnecté.")

    def _sur_notification(self, _sender, donnees: bytearray):
        """Décodage du float 32 bits little-endian envoyé par la carte."""
        if len(donnees) >= 4:
            temperature = struct.unpack("<f", donnees[:4])[0]
            self.temperature_recue.emit(temperature)

    # --- Appelé depuis le thread Qt ------------------------------------------
    def arreter(self):
        """Interrompt la session BLE, y compris un scan ou une connexion en cours."""
        if self._loop is not None and self._tache is not None:
            try:
                # Annuler la tâche interrompt aussi find_device_by_name(),
                # qu'un simple évènement d'arrêt ne débloquerait pas
                self._loop.call_soon_threadsafe(self._tache.cancel)
            except RuntimeError:
                pass  # la boucle asyncio est déjà terminée


# =============================================================================
# Fenêtre principale
# =============================================================================
class FenetrePrincipale(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Acquisition de température — ESP32-C3 BLE")
        self.resize(900, 600)

        self._worker = None
        self._t0 = None
        self._temps = []        # secondes depuis le début de l'acquisition
        self._temperatures = []  # °C

        self._construire_interface()

    # --- Construction de l'interface ----------------------------------------
    def _construire_interface(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        colonne = QtWidgets.QVBoxLayout(central)

        # Bandeau supérieur : boutons + température courante
        bandeau = QtWidgets.QHBoxLayout()

        self.btn_connexion = QtWidgets.QPushButton("Se connecter")
        self.btn_connexion.setMinimumHeight(40)
        self.btn_connexion.clicked.connect(self._basculer_connexion)
        bandeau.addWidget(self.btn_connexion)

        self.btn_effacer = QtWidgets.QPushButton("Effacer")
        self.btn_effacer.setMinimumHeight(40)
        self.btn_effacer.clicked.connect(self._effacer_donnees)
        bandeau.addWidget(self.btn_effacer)

        self.btn_export = QtWidgets.QPushButton("Exporter CSV")
        self.btn_export.setMinimumHeight(40)
        self.btn_export.clicked.connect(self._exporter_csv)
        bandeau.addWidget(self.btn_export)

        bandeau.addStretch()

        self.lbl_temperature = QtWidgets.QLabel("-- °C")
        self.lbl_temperature.setStyleSheet(
            "font-size: 42px; font-weight: bold; color: #2980b9;")
        bandeau.addWidget(self.lbl_temperature)

        colonne.addLayout(bandeau)

        # Statistiques
        self.lbl_stats = QtWidgets.QLabel("Min : --   Max : --   Moyenne : --")
        self.lbl_stats.setStyleSheet("font-size: 14px; color: #555;")
        colonne.addWidget(self.lbl_stats)

        # Courbe temps réel
        pg.setConfigOptions(antialias=True)
        self.graphe = pg.PlotWidget()
        self.graphe.setBackground("w")
        self.graphe.showGrid(x=True, y=True, alpha=0.3)
        self.graphe.setLabel("left", "Température", units="°C")
        self.graphe.setLabel("bottom", "Temps", units="s")
        self._courbe = self.graphe.plot(
            pen=pg.mkPen(color="#e74c3c", width=2),
            symbol="o", symbolSize=4, symbolBrush="#e74c3c")
        colonne.addWidget(self.graphe, stretch=1)

        # Barre de statut
        self.statusBar().showMessage("Prêt. Cliquez sur « Se connecter ».")

    # --- Gestion de la connexion BLE -----------------------------------------
    def _basculer_connexion(self):
        if self._worker is None:
            self._demarrer_acquisition()
        else:
            self._worker.arreter()
            self.btn_connexion.setEnabled(False)  # le thread va se terminer seul

    def _demarrer_acquisition(self):
        self._worker = BleWorker(self)
        self._worker.temperature_recue.connect(self._sur_temperature)
        self._worker.statut_change.connect(self.statusBar().showMessage)
        self._worker.connexion_changee.connect(self._sur_connexion)
        self._worker.finished.connect(self._sur_fin_worker)
        self._worker.start()
        self.btn_connexion.setText("Recherche…")
        self.btn_connexion.setEnabled(False)

    def _sur_connexion(self, connecte: bool):
        if connecte:
            self.btn_connexion.setText("Se déconnecter")
            self.btn_connexion.setEnabled(True)
            if self._t0 is None:
                self._t0 = time.monotonic()

    def _sur_fin_worker(self):
        self._worker = None
        self.btn_connexion.setText("Se connecter")
        self.btn_connexion.setEnabled(True)

    # --- Réception et affichage des mesures ----------------------------------
    def _sur_temperature(self, temperature: float):
        t = time.monotonic() - self._t0 if self._t0 is not None else 0.0
        self._temps.append(t)
        self._temperatures.append(temperature)

        self.lbl_temperature.setText(f"{temperature:.2f} °C")
        self.lbl_stats.setText(
            f"Min : {min(self._temperatures):.2f} °C   "
            f"Max : {max(self._temperatures):.2f} °C   "
            f"Moyenne : {sum(self._temperatures) / len(self._temperatures):.2f} °C   "
            f"({len(self._temperatures)} mesures)")
        self._courbe.setData(self._temps, self._temperatures)

    def _effacer_donnees(self):
        self._temps.clear()
        self._temperatures.clear()
        self._t0 = time.monotonic() if self._worker is not None else None
        self._courbe.setData([], [])
        self.lbl_temperature.setText("-- °C")
        self.lbl_stats.setText("Min : --   Max : --   Moyenne : --")

    # --- Export CSV -----------------------------------------------------------
    def _exporter_csv(self):
        if not self._temperatures:
            QtWidgets.QMessageBox.information(
                self, "Export CSV", "Aucune mesure à exporter.")
            return
        chemin, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Exporter les mesures", "temperatures.csv",
            "Fichiers CSV (*.csv)")
        if not chemin:
            return
        with open(chemin, "w", newline="", encoding="utf-8") as fichier:
            writer = csv.writer(fichier, delimiter=";")
            writer.writerow(["Temps (s)", "Température (°C)"])
            for t, temp in zip(self._temps, self._temperatures):
                writer.writerow([f"{t:.1f}", f"{temp:.2f}"])
        self.statusBar().showMessage(f"Mesures exportées vers {chemin}")

    # --- Fermeture propre ------------------------------------------------------
    def closeEvent(self, event):
        if self._worker is not None:
            self._worker.arreter()
            if not self._worker.wait(5000):
                # Dernier recours : ne jamais détruire un QThread encore actif
                self._worker.terminate()
                self._worker.wait(1000)
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    fenetre = FenetrePrincipale()
    fenetre.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
