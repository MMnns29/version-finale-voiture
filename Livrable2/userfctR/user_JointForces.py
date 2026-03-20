import MBsysPy

def user_JointForces(mbs_data, tsim):
    mbs_data.Qq[1:] = 0.0
    um = mbs_data.user_model

    # ----------------------------------------------------------------
    # 1. BARRE ANTI-ROULIS — effet différentiel gauche/droite
    # ----------------------------------------------------------------
    # Avant : R2 de Bras_sup_AV_G (q8) et R2 de Bras_sup_AV_D (q13)
    C_bar_av = um['FrontSuspension']['C_bar']
    delta_av = mbs_data.q[8] - mbs_data.q[13]
    mbs_data.Qq[8]  += -C_bar_av * delta_av
    mbs_data.Qq[13] +=  C_bar_av * delta_av

    # Arrière : R1 de Bras_sup_AR_G (q24) et R1 de Bras_sup_AR_D (q28)
    C_bar_ar = um['RearSuspension']['C_bar']
    delta_ar = mbs_data.q[24] - mbs_data.q[28]
    mbs_data.Qq[24] += -C_bar_ar * delta_ar
    mbs_data.Qq[28] +=  C_bar_ar * delta_ar

    # ----------------------------------------------------------------
    # 2. STABILISATION CRÉMAILLÈRE — T2 de Bras_Direction (q18)
    # Sans ça, les roues avant oscillent librement pendant la simu
    # ----------------------------------------------------------------
    K_rack = 50000.0  # [N/m]  raideur de rappel virtuelle vers centre
    C_rack =  1000.0  # [N.s/m] amortissement virtuel
    mbs_data.Qq[18] = -K_rack * mbs_data.q[18] - C_rack * mbs_data.qd[18]

    if um['simulation'] == "acceleration":
        # ----------------------------------------------------------------
        # 3. Accélération
        # ----------------------------------------------------------------
        
        # Il faut modifier la vitesse dans main.py pour mettre la vitesse initiale à 7km/h si on applique 200Nm par roue.
        # Dans le cas d'une vitesse inférieure, on observe le phénomène de patinage et la voiture n'avance pas.

        if tsim > 3:  # Attente de 2 [s] pour la chute de la voiture puis 1 [s] pour que la voiture soit parfaitement stable.
            
            couple_moteur = 200.0 #Nm On applique 200 [Nm] de couple sur chaque roue arrière
            
            # Application de la force sur les joints des roues.
            mbs_data.Qq[26] = couple_moteur # Roue AR_G
            mbs_data.Qq[30] = couple_moteur # Roue AR_D
        ##

    if um['simulation'] == "freinage":
        # ----------------------------------------------------------------
        # 4. FREINAGE D'URGENCE
        # ----------------------------------------------------------------
        
        # Il faut modifier la vitesse dans main.py pour mettre la vitesse initiale à 70km/h
        # (pour que ça soit comme nos tests et nos graphes, mais toute vitesse supérieure à 3 [m/s] pour que notre modèle fonctionne parfaitement)
        
        
        vitesse = mbs_data.qd[1]

        if tsim > 5.0:  # Attente de 2 [s] pour la chute de la voiture puis de 3 [s] pour que la voiture soit parfaitement stable.

            
            if vitesse > 3.0:
                pedale = 1.0  # Freinage total tant que la vitesse est supérieure à 3 [m/s] (vitesse> 3 [m/s]).
            
            elif 3.0 >= vitesse > 0.5:
                pedale = (vitesse - 0.5) / 2.5  # Relâchement progressif de la pédale entre 3 [m/s] et 0.5 [m/s] (3 [m/s] >= vitesse > 0.5 [m/s]).
            
            else:
                pedale = 0.0  ## En dessous de 0.5 [m/s], on relâche totalement les freins pour éviter le rebond de la suspension juste avant l'arrêt complet.
                
            # Application de la force sur les joints des roues.
            # 66% avant / 33% arrière, pour suivre le transfert de charge et éviter le blocage des roues arrière (tête-à-queue)
            mbs_data.Qq[11] = -800.0 * pedale # Roue AV_G
            mbs_data.Qq[16] = -800.0 * pedale # Roue AV_D
            mbs_data.Qq[26] = -400.0 * pedale # Roue AR_G
            mbs_data.Qq[30] = -400.0 * pedale # Roue AR_D

    return mbs_data.Qq
