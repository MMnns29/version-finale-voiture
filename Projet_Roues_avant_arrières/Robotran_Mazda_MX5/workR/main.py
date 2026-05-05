#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import MBsysPy as Robotran
import os
import numpy as np
import matplotlib.pyplot as plt

# =============================================================================
# 1. PARAMÈTRES DE LA SIMULATION
# =============================================================================
simulation = "evitement"  # Options: "MRU", "acceleration", "freinage", "dos_d_ane", "virage", "evitement"
vitesse_kmh = {"MRU": 36, "acceleration": 7, "freinage": 70, "dos_d_ane": 60, "virage": 50, "evitement": 60}[simulation]

print(f"--- Démarrage du projet Mazda MX-5 : Mode {simulation} ---")

# Chargement du projet
work_dir = os.path.dirname(os.path.abspath(__file__))
mbs_file = os.path.normpath(os.path.join(work_dir, "..", "dataR", "Robotran_Mazda_MX5_transmition_integrale.mbs")) 
mbs_data = Robotran.MbsData(mbs_file)

# =============================================================================
# 2. INITIALISATION DU USER MODEL
# =============================================================================
um = {}
um['simulation']      = simulation
um['FrontTire']       = {'R': 0.288, 'K': 180000.0}
um['RearTire']        = {'R': 0.288, 'K': 180000.0}
um['FrontSuspension'] = {'K': 27000.0, 'C': 2200.0, 'C_bar': 2500.0, 'Z0': 0.43}
um['RearSuspension']  = {'K': 27000.0, 'C': 1800.0, 'C_bar': 1800.0, 'Z0': 0.43}

mbs_data.user_model = um

# Configuration initiale (Hauteur pour garantir le contact pneu/sol)
mbs_data.q[3] = 0.2 

# =============================================================================
# 3. PARTITIONNEMENT
# =============================================================================
print("\n>> PARTITIONNEMENT...")
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=0)
mbs_part.run()



mbs_data.q[43] = mbs_data.qd[43] = mbs_data.qdd[43] = 0 # joint 43 quand on est en transmission intégrale (Joint de la barre de direction avant) sinon c'est le joint 39
mbs_data.q[48] = mbs_data.qd[48] = mbs_data.qdd[48] = 0 # joint 48 quand on est en transmission intégrale (Joint de la barre de direction arrière)




# =============================================================================
# 4. PHASE DE TASSEMENT (Remplace MbsEquil)
# =============================================================================
mbs_data.process = 2
mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
# On utilise un pas de temps fin (1e-3) pour stabiliser le modèle Bakker
print(">> Phase de tassement (2 secondes)...")
mbs_dirdyn.set_options(dt0=1e-2, tf=2.0, save2file=0) 
mbs_dirdyn.run()

# =============================================================================
# 5. INJECTION DES VITESSES ET SIMULATION DYNAMIQUE
# =============================================================================
print(f">> Injection de la vitesse : {vitesse_kmh} km/h")
mbs_data.process = 3
vitesse_ms = vitesse_kmh / 3.6
omega = vitesse_ms / 0.288 # Vitesse angulaire

# Application des vitesses
mbs_data.qd[1] = vitesse_ms  # Châssis (vitesse longitudinale X)

mbs_data.q[2]  = 0.0  # Y
mbs_data.q[4]  = 0.0  # Roulis (Nouveau !)
mbs_data.q[6]  = 0.0  # Lacet (Yaw)
mbs_data.qd[2] = 0.0  
mbs_data.qd[4] = 0.0  # Vitesse de Roulis (Nouveau !)
mbs_data.qd[6] = 0.0

mbs_data.qd[29] = omega      # Roue AV_G (indices à vérifier selon votre .mbs)
mbs_data.qd[35] = omega      # Roue AV_D
mbs_data.qd[18] = omega      # Roue AR_G
mbs_data.qd[12] = omega      # Roue AR_D
# Application des vitesses
mbs_data.qd[2] = 0.0         # Tuer le glissement latéral parasite (Y)
mbs_data.qd[6] = 0.0         # Tuer la rotation parasite (Yaw)

print(f">> Lancement de la simulation ({simulation})...")
mbs_dirdyn.set_options(dt0=1e-3, tf=6.0, save2file=1)

# =============================================================================
# 5. GESTION DU CRASH-TEST (Anti-arrêt de Python)
# =============================================================================
try:
    mbs_dirdyn.run()
except Exception as e:
    # Si la voiture se retourne, on atterrit ici au lieu de faire planter Python
    print("\n" + "="*50)
    print("CRASH PHYSIQUE DÉTECTÉ")
    print("La voiture a perdu le contrôle (tonneau ou géométrie cassée).")
    print("Génération des graphiques avec les données de la boîte noire...")
    print("="*50 + "\n")

# =============================================================================
# 6. RÉCUPÉRATION DES DONNÉES
# =============================================================================
print("\n>> Récupération des données...")
results_dir = os.path.normpath(os.path.join(work_dir, "..", "resultsR"))
results_path     = os.path.join(results_dir, "dirdyn_q.res")
results_path_qd  = os.path.join(results_dir, "dirdyn_qd.res")

# =============================================================================
# GRAPHIQUE 1 : RÉPONSE DES SUSPENSIONS
# =============================================================================
try:
    results = np.loadtxt(results_path)
    time = results[:, 0]

    id_av_g = mbs_data.joint_id["R1_Bras_sup_AV_G"]
    id_ar_g = mbs_data.joint_id["R1_Bras_inf_AR_G"]

    q_av_g = results[:, id_av_g] * (180 / np.pi)
    q_ar_g = results[:, id_ar_g] * (180 / np.pi)

    fig, axs = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    fig.suptitle(f'Réponse des Suspensions - {simulation}', fontsize=12, fontweight='bold')

    axs[0].plot(time, q_av_g, color='darkblue', linewidth=1.5, label='Suspension AV Gauche')
    axs[0].set_ylabel('Angle (°)')
    axs[0].grid(True, linestyle='--', alpha=0.7)
    axs[0].legend(loc='best')

    axs[1].plot(time, q_ar_g, color='red', linewidth=1.5, label='Suspension AR Gauche')
    axs[1].set_ylabel('Angle (°)')
    axs[1].set_xlabel('Temps (s)')
    axs[1].grid(True, linestyle='--', alpha=0.7)
    axs[1].legend(loc='best')

    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, f"suspension_{simulation}.pdf"), format="pdf", bbox_inches='tight')
    print(f">> Graphique suspensions sauvegardé.")

except Exception as e:
    print(f"Impossible de générer le graphique suspensions : {e}")

# =============================================================================
# GRAPHIQUE 2 : ANGLE DE DIRECTION (roue avant)
# =============================================================================
try:
    results_dir2 = np.loadtxt(results_path)
    time2 = results_dir2[:, 0]

    id_dir = mbs_data.joint_id["T2_barre_direction"]
    q_dir  = results_dir2[:, id_dir] * (180 / np.pi)

    fig_dir, ax_dir = plt.subplots(figsize=(8, 4))
    fig_dir.suptitle(f'Angle de direction (barre de direction) - {simulation}',
                     fontsize=12, fontweight='bold')

    ax_dir.plot(time2, q_dir, color='red', linewidth=1.5, label='Angle barre de direction')
    ax_dir.set_xlabel('Temps (s)')
    ax_dir.set_ylabel('Angle (°)')
    ax_dir.grid(True, linestyle='--', alpha=0.7)
    ax_dir.legend(loc='best')

    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, f"direction_{simulation}.pdf"), format="pdf", bbox_inches='tight')
    print(f">> Graphique direction sauvegardé.")

except Exception as e:
    print(f"Impossible de générer le graphique direction : {e}")

# =============================================================================
# GRAPHIQUE 3 : FORCE LONGITUDINALE DU PNEU
# =============================================================================
try:
    id_ext_ar_g = mbs_data.extforce_id.get("ExtForce_Roue_AR_G", 3)
    nom_output_force = f"dirdyn_F_Longi_Roue_{id_ext_ar_g}.res"
    results_path_force = os.path.join(results_dir, nom_output_force)

    if os.path.exists(results_path_force):
        results_force = np.loadtxt(results_path_force)
        time_force  = results_force[:, 0]
        force_valeur = results_force[:, 1]

        fig_force, ax_force = plt.subplots(figsize=(8, 4))
        fig_force.suptitle(f'Force longitudinale du pneu AR Gauche - {simulation}',
                           fontsize=12, fontweight='bold')

        ax_force.plot(time_force, force_valeur, color='purple', linewidth=2.0,
                      label='Force longitudinale Fx (N)')
        ax_force.set_xlabel('Temps (s)')
        ax_force.set_ylabel('Force (N)')
        ax_force.grid(True, linestyle='--', alpha=0.7)
        ax_force.legend(loc='best')

        plt.tight_layout()
        plt.savefig(os.path.join(results_dir, f"force_longitudinale_{simulation}.pdf"),
                    format="pdf", bbox_inches='tight')
        print(f">> Graphique force longitudinale sauvegardé.")
    else:
        print(f"⚠️ Fichier {nom_output_force} introuvable — vérifie set_output dans user_ExtForces.py")

except Exception as e:
    print(f"Impossible de générer le graphique force longitudinale : {e}")
    
# =============================================================================
# 7bis. GRAPHIQUE : FORCE LATÉRALE PNEU ARRIÈRE GAUCHE (Prévention Tête-à-queue)
# =============================================================================
print("\n>> Génération du graphique de la Force Latérale...")

# On utilise le même id_ext_ar_g que pour la force longitudinale
nom_output_force_lat = f"dirdyn_F_Lat_Roue_{id_ext_ar_g}.res"
results_path_force_lat = os.path.join(results_dir, nom_output_force_lat)

try:
    if os.path.exists(results_path_force_lat):
        results_force_lat = np.loadtxt(results_path_force_lat)
        time_lat = results_force_lat[:, 0]
        force_lat_valeur = results_force_lat[:, 1] # La valeur de la force Fy

        fig_lat, ax_lat = plt.subplots(figsize=(8, 4))
        # Titre propre et professionnel pour le rapport
        fig_lat.suptitle(f'Force Latérale (Adhérence en virage) - {simulation}', fontsize=12, fontweight='bold')

        # Tracé en noir comme demandé
        ax_lat.plot(time_lat, force_lat_valeur, color='black', linewidth=2.0, label='Force latérale arrière gauche (N)')
        
        # Axes nommés correctement avec unités
        ax_lat.set_ylabel('Force latérale $F_y$ (N)')
        ax_lat.set_xlabel('Temps (s)')
        
        # Grille discrète et légende pour un rendu "rapport scientifique"
        ax_lat.grid(True, linestyle='--', alpha=0.7)
        ax_lat.legend(loc='best')

        plt.tight_layout()
        plot_save_path_lat = os.path.join(results_dir, f"forces_lat_contact_{simulation}.pdf")
        plt.savefig(plot_save_path_lat, format="pdf", bbox_inches='tight')
        print(f">> Graphique de la force latérale sauvegardé : {plot_save_path_lat}")
    else:
        print(f"⚠️ Le fichier {nom_output_force_lat} n'a pas été trouvé.")

except Exception as e:
    print(f"Impossible de générer le graphique de la force latérale : {e}")

# =============================================================================
# GRAPHIQUE 4 : GLISSEMENT DES PNEUS (ABS)
# =============================================================================
try:
    results_qd = np.loadtxt(results_path_qd)
    time_qd = results_qd[:, 0]

    id_T1_chassis = mbs_data.joint_id["T1_chassis"]
    id_roue_ar_g  = mbs_data.joint_id["R2_Roue_AR_G"]

    v_vehicule         = results_qd[:, id_T1_chassis]
    omega_roue         = results_qd[:, id_roue_ar_g]
    R_wheel            = 0.288
    v_roue_tangentielle = np.abs(omega_roue * R_wheel)

    fig_abs, ax_abs = plt.subplots(figsize=(8, 5))
    fig_abs.suptitle(f'Glissement des pneus (ABS) - {simulation}', fontsize=12, fontweight='bold')

    ax_abs.plot(time_qd, v_vehicule, color='black', linewidth=2.0, label='Vitesse châssis (m/s)')
    ax_abs.plot(time_qd, v_roue_tangentielle, color='green', linewidth=1.5, label='Vitesse tangentielle roue AR G (m/s)')

    # Zone de glissement : roue plus lente que le châssis (freinage)
    ax_abs.fill_between(time_qd, v_vehicule, v_roue_tangentielle,
                        where=(v_vehicule > v_roue_tangentielle),
                        color='red', alpha=0.3, label='Zone de glissement')

    ax_abs.set_xlabel('Temps (s)')
    ax_abs.set_ylabel('Vitesse (m/s)')
    ax_abs.grid(True, linestyle='--', alpha=0.7)
    ax_abs.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, f"glissement_abs_{simulation}.pdf"),
                format="pdf", bbox_inches='tight')
    print(f">> Graphique glissement ABS sauvegardé.")

except Exception as e:
    print(f"Impossible de générer le graphique ABS : {e}")

plt.show()
print("\n--- Simulation terminée ---")