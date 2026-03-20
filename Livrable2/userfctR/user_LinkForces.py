# -*- coding: utf-8 -*-
def user_LinkForces(Z, Zd, mbs_data, tsim, identity):
    um = mbs_data.user_model

    # identity 1 = Link_0 (AV_D), identity 2 = Link_1 (AV_G) → avant
    # identity 3 = Link_2 (AR_G), identity 4 = Link_3 (AR_D) → arrière
    if identity in [1, 2]:
        K = um['FrontSuspension']['K']   # 25000 N/m
        C = um['FrontSuspension']['C']   # 2000 N.s/m
        Z0 = 0.510784  # longueur naturelle [m] pour l'AVANT (on ne touche pas)
    else:
        K = um['RearSuspension']['K']    # 22000 N/m
        C = um['RearSuspension']['C']    # 2000 N.s/m
        #Z0 = 0.650000  # à activer quand MRU / Accélération / Freinage : ressort ARRIÈRE allongé artificiellement
        Z0 = 0.510784  # à activer quand DOS D'ANE : même longueur naturelle qu'à l'avant

    Flink = K * (Z - Z0) + C * Zd

    return Flink



