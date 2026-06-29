"""
Survey - Validation 2024/2025 (rattrapage) — VERSION SIMPLIFIEE
================================================================

Même cahier des charges, mais le code est réduit au minimum :
  - Un SEUL thread (RobotWorker) qui, à chaque pas, lit l'obstacle PUIS bouge.
  - Plus de QTimer, plus de verrou (lock) : un seul endroit touche au robot.
  - La couleur de "middle_cmd" est envoyée à l'interface par un simple signal Qt.
"""

import sys
import os

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton,
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal

try:
    from martypy import Marty
except ImportError:
    Marty = None
    print("[ATTENTION] martypy non installé : interface seule.")


SEUIL_MM = 100   # en-dessous de cette distance = obstacle


# ===========================================================================
#  UN SEUL THREAD : il lit l'obstacle ET exécute la trajectoire
# ===========================================================================
class RobotWorker(QThread):
    # Signal envoyé à l'interface : True = obstacle, False = libre.
    obstacle = pyqtSignal(bool)

    def __init__(self, marty, commands):
        super().__init__()
        self.marty = marty
        self.commands = commands
        self._running = True

    def run(self):
        # On parcourt chaque commande, pas par pas.
        for code, steps in self.commands:
            for _ in range(steps):
                if not self._running:
                    return
                # 1) lire l'obstacle et prévenir l'interface (couleur)
                try:
                    distance = self.marty.get_distance_sensor()
                    self.obstacle.emit(distance < SEUIL_MM)
                except Exception as e:
                    print(f"[ERREUR] capteur : {e}")
                # 2) faire UN pas selon la commande
                try:
                    if code == "FW":
                        self.marty.walk(num_steps=1)
                    elif code == "BW":
                        self.marty.walk(num_steps=1, step_length=-25)
                    elif code == "LT":
                        self.marty.sidestep("left", steps=1)
                    elif code == "RT":
                        self.marty.sidestep("right", steps=1)
                except Exception as e:
                    print(f"[ERREUR] mouvement : {e}")

    def stop(self):
        self._running = False
        self.wait()


# ===========================================================================
#  FENÊTRE
# ===========================================================================
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.marty = None
        self.connected = False
        self.commands = []
        self.worker = None

        # --- Interface ---
        self.setWindowTitle("Survey - Pilotage robot")
        self.resize(420, 260)
        layout = QVBoxLayout(self)

        top = QHBoxLayout()
        self.state_label = QLabel("Déconnecté")
        self.ip_field = QLineEdit()
        self.ip_field.setPlaceholderText("Adresse IP du robot")
        self.connect_button = QPushButton("Connexion")
        self.connect_button.clicked.connect(self.toggle_connection)
        top.addWidget(self.state_label)
        top.addWidget(self.ip_field)
        top.addWidget(self.connect_button)

        self.middle_cmd = QLabel("middle_cmd")
        self.middle_cmd.setObjectName("middle_cmd")
        self.middle_cmd.setAlignment(Qt.AlignCenter)
        self.middle_cmd.setMinimumHeight(150)

        layout.addLayout(top)
        layout.addWidget(self.middle_cmd, stretch=1)

        # --- Chargement du fichier au démarrage ---
        self.load_trajectory("survey.traj")

    # -- Chargement .traj (erreur console si absent, sans quitter) --
    def load_trajectory(self, path):
        if not os.path.exists(path):
            print(f"[ERREUR] '{path}' introuvable.")
            return
        with open(path, encoding="utf-8") as f:
            for line in f:
                p = line.replace(":", " ").split()
                if p:
                    steps = int(p[1]) if len(p) > 1 else 1
                    self.commands.append((p[0].upper(), steps))
        print(f"[INFO] {len(self.commands)} commande(s) chargée(s).")

    # -- Bouton : connecte ou déconnecte selon l'état --
    def toggle_connection(self):
        if self.connected:
            self.disconnect_robot()
        else:
            self.connect_robot()

    def connect_robot(self):
        ip = self.ip_field.text().strip()
        if not ip:
            print("[ERREUR] Aucune IP saisie.")
            return
        try:
            self.marty = Marty("wifi", ip)
        except Exception as e:
            print(f"[ERREUR] Connexion impossible : {e}")
            return

        self.connected = True
        self.state_label.setText("Connecté")
        self.connect_button.setText("Déconnexion")

        # On lance le thread unique : il gère obstacle + trajectoire.
        self.worker = RobotWorker(self.marty, self.commands)
        self.worker.obstacle.connect(self.show_obstacle)   # met à jour la couleur
        self.worker.start()

    def disconnect_robot(self):
        if self.worker:
            self.worker.stop()
            self.worker = None
        if self.marty:
            try:
                self.marty.close()
            except Exception:
                pass
            self.marty = None
        self.connected = False
        self.state_label.setText("Déconnecté")
        self.connect_button.setText("Connexion")
        self.middle_cmd.setStyleSheet("")
        self.middle_cmd.setText("middle_cmd")

    # -- Slot appelé par le signal du thread : vert / rouge --
    def show_obstacle(self, obstacle):
        if obstacle:
            self.middle_cmd.setStyleSheet("background-color: red; color: white;")
            self.middle_cmd.setText("OBSTACLE")
        else:
            self.middle_cmd.setStyleSheet("background-color: green; color: white;")
            self.middle_cmd.setText("LIBRE")

    def closeEvent(self, event):
        if self.connected:
            self.disconnect_robot()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
