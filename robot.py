"""
Survey - Validation 2024/2025 (rattrapage)
===========================================

Application PyQt5 de pilotage d'un robot Marty V2.

Fonctionnalités demandées par le sujet :
  1. Interface : une fenêtre, une barre du haut (état + champ IP + bouton
     connexion/déconnexion) et au centre un label "middle_cmd".
  2. Au démarrage : chargement du fichier "survey.traj" (erreur console si absent,
     sans crasher).
  3. Bouton connexion : tentative de connexion au robot via l'IP saisie
     (erreur console si échec, sans crasher).
  4. Une fois connecté : le label "middle_cmd" devient VERT si aucun obstacle,
     ROUGE si obstacle détecté.
  5. Le robot exécute les commandes du fichier .traj.
  6. POINT CLÉ : pendant l'exécution des commandes, "middle_cmd" doit continuer
     à fonctionner (détection d'obstacle qui ne se fige pas).

Le point 6 impose de NE PAS exécuter la trajectoire sur le thread graphique,
sinon l'interface (et donc la détection d'obstacle) gèlerait. On exécute donc
la trajectoire dans un QThread séparé, et la détection d'obstacle est faite par
un QTimer qui tourne sur le thread principal (resté libre).
"""

import sys
import os
import threading

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton,
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal

# On protège l'import de martypy : ainsi l'interface peut quand même se lancer
# pour être vérifiée même si la librairie n'est pas installée sur la machine.
try:
    from martypy import Marty
except ImportError:
    Marty = None
    print("[ATTENTION] martypy n'est pas installé : la connexion réelle "
          "au robot sera impossible (interface lançable malgré tout).")


# ===========================================================================
#  THREAD D'EXÉCUTION DE LA TRAJECTOIRE
# ===========================================================================
class TrajectoryWorker(QThread):
    """
    Exécute la liste de commandes dans un thread séparé du thread graphique.

    Pourquoi un thread ?
      Les déplacements Marty (marty.walk, marty.sidestep) sont BLOQUANTS :
      l'appel ne rend la main qu'une fois le mouvement terminé (~1,5 s par pas).
      Si on les lançait sur le thread principal, la fenêtre se figerait et le
      QTimer de détection d'obstacle ne pourrait plus se déclencher.
      En déléguant à un QThread, le thread principal reste libre : l'interface
      répond et "middle_cmd" continue de se mettre à jour.
    """

    # Signal Qt pour renvoyer des messages au thread principal de façon
    # thread-safe (on n'écrit JAMAIS dans un widget depuis un autre thread).
    log = pyqtSignal(str)

    def __init__(self, marty, commands, lock):
        super().__init__()
        self.marty = marty          # l'objet Marty déjà connecté
        self.commands = commands    # liste de tuples (code, nb_pas)
        self.lock = lock            # verrou partagé avec le QTimer d'obstacle
        self._running = True        # drapeau pour pouvoir interrompre proprement

    def run(self):
        """Boucle principale du thread : exécutée quand on appelle .start()."""
        for code, steps in self.commands:
            if not self._running:           # arrêt demandé (déconnexion)
                break
            self.log.emit(f"➤ Exécution : {code} {steps} pas")

            # On découpe le déplacement de X pas en X déplacements d'1 pas.
            # Avantage : le verrou est relâché après CHAQUE pas, ce qui laisse
            # le QTimer lire le capteur d'obstacle entre deux pas. La détection
            # reste donc fluide pendant toute l'exécution (exigence du sujet).
            for _ in range(steps):
                if not self._running:
                    break
                with self.lock:             # accès exclusif au robot
                    try:
                        if code == "FW":                 # avancer
                            self.marty.walk(num_steps=1)
                        elif code == "BW":               # reculer
                            # un pas en arrière = longueur de pas négative
                            self.marty.walk(num_steps=1, step_length=-25)
                        elif code == "LT":               # pas chassé à gauche
                            self.marty.sidestep("left", steps=1)
                        elif code == "RT":               # pas chassé à droite
                            self.marty.sidestep("right", steps=1)
                        else:
                            self.log.emit(f"Commande inconnue ignorée : {code}")
                            break
                    except Exception as e:
                        self.log.emit(f"Erreur pendant le mouvement : {e}")

        self.log.emit("Trajectoire terminée.")

    def stop(self):
        """Demande l'arrêt et attend que le thread se termine proprement."""
        self._running = False
        self.wait()


# ===========================================================================
#  FENÊTRE PRINCIPALE
# ===========================================================================
class MainWindow(QWidget):

    # Seuil de distance (en mm) en dessous duquel on considère qu'il y a
    # un obstacle. À calibrer selon le capteur ; valeur de départ raisonnable.
    OBSTACLE_THRESHOLD_MM = 100

    def __init__(self):
        super().__init__()

        # --- État interne du programme ---
        self.marty = None                    # objet robot (None tant que déconnecté)
        self.connected = False               # état de connexion
        self.commands = []                   # commandes chargées depuis le .traj
        self.marty_lock = threading.Lock()   # verrou : sérialise les accès robot
        self.worker = None                   # thread d'exécution de trajectoire

        # --- Construction de l'interface ---
        self._build_ui()

        # --- Chargement du fichier de trajectoire au démarrage (point 2) ---
        self._load_trajectory("survey.traj")

        # --- Timer de détection d'obstacle ---
        # Il se déclenche toutes les 200 ms SUR LE THREAD PRINCIPAL. Comme la
        # trajectoire tourne dans un autre thread, le thread principal reste
        # disponible et ce timer s'exécute en continu : c'est lui qui maintient
        # "middle_cmd" à jour, y compris pendant l'exécution (point 6).
        self.obstacle_timer = QTimer(self)
        self.obstacle_timer.setInterval(200)
        self.obstacle_timer.timeout.connect(self._update_obstacle)

    # -----------------------------------------------------------------------
    #  Construction de l'interface graphique (point 1)
    # -----------------------------------------------------------------------
    def _build_ui(self):
        self.setWindowTitle("Survey - Pilotage robot Marty")
        self.resize(420, 260)

        # Disposition verticale globale : barre du haut + zone centrale.
        main_layout = QVBoxLayout(self)

        # --- Barre du haut : 3 éléments alignés horizontalement ---
        top_bar = QHBoxLayout()

        # 1) libellé d'état (connecté / déconnecté)
        self.state_label = QLabel("Déconnecté")

        # 2) champ de texte pour saisir l'IP du robot
        self.ip_field = QLineEdit()
        self.ip_field.setPlaceholderText("Adresse IP du robot")

        # 3) bouton qui sert à se connecter OU se déconnecter selon l'état
        self.connect_button = QPushButton("Connexion")
        self.connect_button.clicked.connect(self._toggle_connection)

        top_bar.addWidget(self.state_label)
        top_bar.addWidget(self.ip_field)
        top_bar.addWidget(self.connect_button)

        # --- Zone centrale : l'unique label "middle_cmd" ---
        self.middle_cmd = QLabel("middle_cmd")
        self.middle_cmd.setObjectName("middle_cmd")   # nommé comme demandé
        self.middle_cmd.setAlignment(Qt.AlignCenter)
        self.middle_cmd.setMinimumHeight(150)

        # Assemblage final.
        main_layout.addLayout(top_bar)
        main_layout.addWidget(self.middle_cmd, stretch=1)  # prend la place restante

    # -----------------------------------------------------------------------
    #  Chargement du fichier .traj (point 2)
    # -----------------------------------------------------------------------
    def _load_trajectory(self, path):
        # Si le fichier n'existe pas : erreur en console, MAIS on ne quitte pas.
        if not os.path.exists(path):
            print(f"[ERREUR] Fichier de trajectoire '{path}' introuvable.")
            return

        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):   # ignore vides/commentaires
                    continue
                # On accepte "FW 5" comme "FW:5" : on remplace ':' par un espace
                # puis on découpe sur les espaces.
                parts = line.replace(":", " ").split()
                code = parts[0].upper()
                # nombre de pas : 1 par défaut s'il n'est pas précisé / invalide
                try:
                    steps = int(parts[1]) if len(parts) > 1 else 1
                except ValueError:
                    steps = 1
                self.commands.append((code, steps))

        print(f"[INFO] {len(self.commands)} commande(s) chargée(s) "
              f"depuis '{path}'.")

    # -----------------------------------------------------------------------
    #  Bouton : aiguillage connexion / déconnexion (point 3)
    # -----------------------------------------------------------------------
    def _toggle_connection(self):
        if not self.connected:
            self._connect()
        else:
            self._disconnect()

    def _connect(self):
        ip = self.ip_field.text().strip()

        # Le sujet précise : on tente la connexion SI une IP est saisie.
        if not ip:
            print("[ERREUR] Aucune adresse IP saisie.")
            return

        if Marty is None:
            print("[ERREUR] martypy indisponible : connexion impossible.")
            return

        # Tentative de connexion. En cas d'échec : erreur console, pas de crash.
        try:
            self.marty = Marty("wifi", ip)
        except Exception as e:
            print(f"[ERREUR] Connexion au robot impossible : {e}")
            self.marty = None
            return

        # Connexion réussie : on met à jour l'état et le bouton (point 3).
        self.connected = True
        self.state_label.setText("Connecté")
        self.connect_button.setText("Déconnexion")
        print(f"[INFO] Connecté au robot {ip}.")

        # On démarre la détection d'obstacle (point 4)...
        self.obstacle_timer.start()
        # ...puis l'exécution de la trajectoire (point 5).
        self._run_trajectory()

    def _disconnect(self):
        # Arrêt propre : on stoppe le timer, le thread, puis on ferme le robot.
        self.obstacle_timer.stop()

        if self.worker is not None:
            self.worker.stop()
            self.worker = None

        if self.marty is not None:
            try:
                self.marty.close()
            except Exception as e:
                print(f"[ATTENTION] Fermeture robot : {e}")
            self.marty = None

        # Remise à zéro de l'interface.
        self.connected = False
        self.state_label.setText("Déconnecté")
        self.connect_button.setText("Connexion")
        self.middle_cmd.setStyleSheet("")          # plus de couleur
        self.middle_cmd.setText("middle_cmd")
        print("[INFO] Déconnecté.")

    # -----------------------------------------------------------------------
    #  Détection d'obstacle : appelée par le QTimer toutes les 200 ms (point 4)
    # -----------------------------------------------------------------------
    def _update_obstacle(self):
        if not self.connected or self.marty is None:
            return

        # Lecture du capteur. On prend le verrou pour ne pas accéder au robot
        # en même temps que le thread de trajectoire (une seule connexion réseau).
        try:
            with self.marty_lock:
                distance = self.marty.get_distance_sensor()
        except Exception as e:
            # Si le capteur n'est pas disponible, on signale sans crasher.
            print(f"[ERREUR] Lecture du capteur d'obstacle : {e}")
            return

        # Vert si pas d'obstacle, rouge si obstacle détecté.
        if distance is not None and distance < self.OBSTACLE_THRESHOLD_MM:
            self.middle_cmd.setStyleSheet(
                "background-color: red; color: white; font-weight: bold;")
            self.middle_cmd.setText("OBSTACLE")
        else:
            self.middle_cmd.setStyleSheet(
                "background-color: green; color: white; font-weight: bold;")
            self.middle_cmd.setText("LIBRE")

    # -----------------------------------------------------------------------
    #  Lancement du thread de trajectoire (point 5)
    # -----------------------------------------------------------------------
    def _run_trajectory(self):
        if not self.commands:
            print("[INFO] Aucune commande à exécuter.")
            return

        self.worker = TrajectoryWorker(self.marty, self.commands, self.marty_lock)
        # Les messages du thread sont affichés en console sur le thread principal.
        self.worker.log.connect(lambda msg: print(msg))
        self.worker.start()

    # -----------------------------------------------------------------------
    #  Fermeture de la fenêtre : on libère tout proprement
    # -----------------------------------------------------------------------
    def closeEvent(self, event):
        if self.connected:
            self._disconnect()
        event.accept()


# ===========================================================================
#  POINT D'ENTRÉE DU PROGRAMME
# ===========================================================================
def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
