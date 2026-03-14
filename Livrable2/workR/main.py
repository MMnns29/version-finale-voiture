#!/usr/bin/env python3
import MBsysPy
import os

# 1. Chargement du modèle
print("Starting Mazda MX-5 MBS project!")
work_dir = os.path.dirname(os.path.abspath(__file__))
mbs_file = os.path.normpath(os.path.join(work_dir, "..", "dataR", "Livrable2.mbs"))

mbs_data = MBsysPy.MbsData(mbs_file)

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
# * COORDINATE PARTITIONING (Calcul des boucles)            *
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
print("\n>> PARTITIONNEMENT...")
mbs_part = MBsysPy.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
# * STATIC EQUILIBRIUM (ON LE DESACTIVE TEMPORAIREMENT)     *
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
print("\n>> SAUT DE L'EQUILIBRE (Debug mode)...")
# mbs_equil = MBsysPy.MbsEquil(mbs_data)
# mbs_equil.run()

# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
# * DIRECT DYNAMICS (La simulation de conduite)             *
# * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
print("\n>> LANCEMENT DE LA DYNAMIQUE (Dirdyn)...")
mbs_dirdyn = MBsysPy.MbsDirdyn(mbs_data)
mbs_dirdyn.set_options(dt0=1e-3, tf=2.0, save2file=1)
mbs_dirdyn.run()

print("\nSimulation terminée avec succès !")