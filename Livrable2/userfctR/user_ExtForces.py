# -*- coding: utf-8 -*-
import numpy as np

def user_ExtForces(PxF, RxF, VxF, OMxF, AxF, OMPxF, mbs_data, tsim, ixF):
    Fx = Fy = Fz = Mx = My = Mz = 0.0
    idpt = mbs_data.xfidpt[ixF]
    dxF = mbs_data.dpt[1:, idpt]

    # Identification des pneus par leur nom dans le .mbs
    try:
        pneu_ids = [mbs_data.extforce_id[n] for n in ['F_pneu_av_g', 'F_pneu_av_d', 'F_pneu_ar_g', 'F_pneu_ar_d']]
    except:
        pneu_ids = []

    if ixF in pneu_ids:
        # 1. Déterminer si c'est un pneu avant ou arrière pour les paramètres
        is_front = ixF in [mbs_data.extforce_id.get('F_pneu_av_g'), mbs_data.extforce_id.get('F_pneu_av_d')]
        
        # Accès correct au User Model via mbs_data
        if is_front:
            R = mbs_data.user_model['FrontTire']['R']
            K = mbs_data.user_model['FrontTire']['K']
        else:
            R = mbs_data.user_model['RearTire']['R']
            K = mbs_data.user_model['RearTire']['K']
            
        D = 50000.0 # Amortissement pour stabiliser la voiture

        # 2. Gestion du SOL & BOSSE
        x_wheel = PxF[1]
        z_sol = 0.0
        # Exemple : Bosse de 10cm entre x=10m et x=12m
        if 10.0 <= x_wheel <= 12.0:
            z_sol = -0.10 # Monter = Z diminue car gravité à +9.81

        # 3. Pénétration (Z vers le bas)
        z_pneu_bas = PxF[3] + R
        pen = z_pneu_bas - z_sol

        if pen > 0:
            # La force Fz doit être négative pour pousser vers le haut contre la gravité
            Fz = - (K * pen + D * VxF[3])
            
            # Sécurité : la route ne peut que pousser, pas aspirer
            if Fz > 0: 
                Fz = 0.0
        else:
            Fz = 0.0

    Swr = mbs_data.SWr[ixF]
    Swr[1:] = [Fx, Fy, Fz, Mx, My, Mz, dxF[0], dxF[1], dxF[2]]
    return Swr