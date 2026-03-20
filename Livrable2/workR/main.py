#!/usr/bin/env python3
import MBsysPy
import os
import numpy as np
import matplotlib.pyplot as plt

# =========================================================
# CHOIX DE LA SIMULATION
# =========================================================
simulation = "MRU"  # "MRU", "acceleration", "freinage", "dos_d_ane"

print(f"Starting Mazda MX-5 MBS project! {simulation}")
work_dir = os.path.dirname(os.path.abspath(__file__))
mbs_file = os.path.normpath(os.path.join(work_dir, "..", "dataR", "Livrable2.mbs"))

mbs_data = MBsysPy.MbsData(mbs_file)

# =========================================================
# INITIALISATION DU USER MODEL
# =========================================================
um = {}
um['simulation']      = simulation
um['FrontTire']       = {'R': 0.295, 'K': 200000.0}
um['RearTire']        = {'R': 0.295, 'K': 200000.0}
um['FrontSuspension'] = {'K': 25000.0, 'C': 2000.0, 'C_bar': 1200.0}
um['RearSuspension']  = {'K': 22000.0, 'C': 2000.0, 'C_bar':  800.0}
mbs_data.user_model = um

# =========================================================
# CONDITIONS INITIALES
# =========================================================
mbs_data.q[3] = 0.118  # roues légèrement en contact avec le sol

# =========================================================
# PARTITIONNEMENT 
# =========================================================
print("\n>> PARTITIONNEMENT...")
mbs_part = MBsysPy.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

# --- ON NE LANCE PLUS MbsEquil ICI CAR IL PLANTE ---

# =========================================================
# 1. SIMULATION FANTÔME (Tassement)
# =========================================================
print("\n>> Tassement de la voiture sur place (2 secondes)...")
mbs_dirdyn = MBsysPy.MbsDirdyn(mbs_data)
# save2file=0 : on ne sauvegarde pas, on laisse juste la voiture trouver son équilibre
mbs_dirdyn.set_options(dt0=1e-3, tf=2.0, save2file=0)
mbs_dirdyn.run()

# =========================================================
# 2. INJECTION DU MRU (Voiture stabilisée)
# =========================================================
print("\n>> Injection des vitesses (MRU)...")

vitesse_kmh = {"MRU": 36, "acceleration": 7, "freinage": 70, "dos_d_ane": 5}[simulation]
vitesse_ms = vitesse_kmh / 3.6  # Modification de la vitesse en mètres par seconde.

mbs_data.qd[1] = vitesse_ms  # On applique vitesse_ms sur le châssis grace à q[1].

# Vitesse angulaire (omega) = Vitesse (vitesse_ms) / Rayon (R)
omega = vitesse_ms / 0.295

# On applique omega aux 4 roues, à la bonne vitesse pour ne pas déraper.
mbs_data.qd[11] = omega  # Roue AV_G
mbs_data.qd[16] = omega  # Roue AV_D
mbs_data.qd[26] = omega  # Roue AR_G
mbs_data.qd[30] = omega  # Roue AR_D

# =========================================================
# 3. LA VRAIE DYNAMIQUE (Enregistrement sur 8s)
# =========================================================
print(f"\n>> LANCEMENT DU MRU ({vitesse_kmh} km/h)...")
# On relance de t=2.0 à t=8.0 avec la sauvegarde activée
mbs_dirdyn.set_options(dt0=1e-3, tf=8.0, save2file=1)
mbs_dirdyn.run()

print("\nSimulation terminée avec succès !")

# =========================================================
# AFFICHAGE ET SAUVEGARDE DES GRAPHES DE SUSPENSION
# =========================================================
print("\n>> Génération des graphes de suspension...")
results_dir = os.path.normpath(os.path.join(work_dir, "..", "resultsR"))
results_path = os.path.join(results_dir, "dirdyn_q.res")
results = np.loadtxt(results_path)

time = results[:, 0]
q_av_g = results[:, 7]
q_av_d = results[:, 12]
q_ar_g = results[:, 24]
q_ar_d = results[:, 28]

fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
plt.tight_layout(rect=[0, 0.03, 1, 0.95])

if simulation in ("MRU", "acceleration", "freinage"):
    # --- BLOC GRAPHE MRU / ACCELERATION / FREINAGE ---
    fig.suptitle('Angle des bras de suspension vs Temps - Mazda MX-5', fontsize=16)
    plot_save_path = os.path.join(results_dir, "suspension_drop_MRU.png")

if simulation == "dos_d_ane":
    # --- BLOC GRAPHE DOS D'ANE ---
    fig.suptitle(f'Réponse des Suspensions au Dos d\'âne ({vitesse_kmh} km/h)', fontsize=16)
    plot_save_path = os.path.join(results_dir, f"suspension_dos_d_ane_{vitesse_kmh}kmh.png")

axs[0, 0].plot(time, q_av_g, color='blue')
axs[0, 0].set_title('Avant Gauche (q7)')
axs[0, 0].grid(True)
axs[0, 0].set_ylabel('Angle (rad)')

axs[0, 1].plot(time, q_av_d, color='darkblue')
axs[0, 1].set_title('Avant Droit (q12)')
axs[0, 1].grid(True)

axs[1, 0].plot(time, q_ar_g, color='red')
axs[1, 0].set_title('Arrière Gauche (q24)')
axs[1, 0].grid(True)
axs[1, 0].set_ylabel('Angle (rad)')
axs[1, 0].set_xlabel('Temps (s)')

axs[1, 1].plot(time, q_ar_d, color='darkred')
axs[1, 1].set_title('Arrière Droit (q28)')
axs[1, 1].grid(True)
axs[1, 1].set_xlabel('Temps (s)')

plt.savefig(plot_save_path, dpi=300)
plt.show()
