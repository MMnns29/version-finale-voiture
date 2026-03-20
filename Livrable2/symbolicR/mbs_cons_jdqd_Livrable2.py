#
#	MBsysTran - Release 8.1
#
#	Copyright 
#	Universite catholique de Louvain (UCLouvain) 
#	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
#	2, Place du Levant
#	1348 Louvain-la-Neuve 
#	Belgium 
#
#	http://www.robotran.be 
#
#	==> Generation Date: Mon Mar 16 14:28:07 2026
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Livrable2
#
#	==> Number of joints: 31
#
#	==> Function: F18 - Constraints Quadratic Velocity Terms (Jdqd)
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def cons_jdqd(Jdqd, s):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S27 = sin(q[27])
    C27 = cos(q[27])
    S24 = sin(q[24])
    C24 = cos(q[24])
    S25 = sin(q[25])
    C25 = cos(q[25])
    S31 = sin(q[31])
    C31 = cos(q[31])
    S28 = sin(q[28])
    C28 = cos(q[28])
    S29 = sin(q[29])
    C29 = cos(q[29])
    S23 = sin(q[23])
    C23 = cos(q[23])
    S7 = sin(q[7])
    C7 = cos(q[7])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
    S10 = sin(q[10])
    C10 = cos(q[10])
    S17 = sin(q[17])
    C17 = cos(q[17])
    S12 = sin(q[12])
    C12 = cos(q[12])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S14 = sin(q[14])
    C14 = cos(q[14])
    S15 = sin(q[15])
    C15 = cos(q[15])
    S19 = sin(q[19])
    C19 = cos(q[19])
    S20 = sin(q[20])
    C20 = cos(q[20])
    S21 = sin(q[21])
    C21 = cos(q[21])
    S22 = sin(q[22])
    C22 = cos(q[22])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    RLjdqd1_22 = s.dpt[2,37]*C27-s.dpt[3,37]*S27
    RLjdqd1_32 = s.dpt[2,37]*S27+s.dpt[3,37]*C27
    ORjdqd1_22 = -RLjdqd1_32*qd[27]
    ORjdqd1_32 = RLjdqd1_22*qd[27]
    Apqpjdqd1_22 = -ORjdqd1_32*qd[27]
    Apqpjdqd1_32 = ORjdqd1_22*qd[27]
    ROjdqd2_52 = C24*C25-S24*S25
    ROjdqd2_62 = C24*S25+S24*C25
    ROjdqd2_82 = -C24*S25-S24*C25
    ROjdqd2_92 = C24*C25-S24*S25
    RLjdqd2_22 = s.dpt[2,33]*C24
    RLjdqd2_32 = s.dpt[2,33]*S24
    OMjdqd2_12 = qd[24]+qd[25]
    ORjdqd2_22 = -RLjdqd2_32*qd[24]
    ORjdqd2_32 = RLjdqd2_22*qd[24]
    Apqpjdqd2_22 = -ORjdqd2_32*qd[24]
    Apqpjdqd2_32 = ORjdqd2_22*qd[24]
    RLjdqd2_23 = ROjdqd2_52*s.dpt[2,34]+ROjdqd2_82*s.dpt[3,34]
    RLjdqd2_33 = ROjdqd2_62*s.dpt[2,34]+ROjdqd2_92*s.dpt[3,34]
    ORjdqd2_23 = -OMjdqd2_12*RLjdqd2_33
    ORjdqd2_33 = OMjdqd2_12*RLjdqd2_23
    Apqpjdqd2_23 = Apqpjdqd2_22-OMjdqd2_12*ORjdqd2_33
    Apqpjdqd2_33 = Apqpjdqd2_32+OMjdqd2_12*ORjdqd2_23
    jdqd2 = Apqpjdqd1_22-Apqpjdqd2_23
    jdqd3 = Apqpjdqd1_32-Apqpjdqd2_33
    RLjdqd3_22 = s.dpt[2,43]*C31-s.dpt[3,43]*S31
    RLjdqd3_32 = s.dpt[2,43]*S31+s.dpt[3,43]*C31
    ORjdqd3_22 = -RLjdqd3_32*qd[31]
    ORjdqd3_32 = RLjdqd3_22*qd[31]
    Apqpjdqd3_22 = -ORjdqd3_32*qd[31]
    Apqpjdqd3_32 = ORjdqd3_22*qd[31]
    ROjdqd4_52 = C28*C29-S28*S29
    ROjdqd4_62 = C28*S29+S28*C29
    ROjdqd4_82 = -C28*S29-S28*C29
    ROjdqd4_92 = C28*C29-S28*S29
    RLjdqd4_22 = s.dpt[2,39]*C28
    RLjdqd4_32 = s.dpt[2,39]*S28
    OMjdqd4_12 = qd[28]+qd[29]
    ORjdqd4_22 = -RLjdqd4_32*qd[28]
    ORjdqd4_32 = RLjdqd4_22*qd[28]
    Apqpjdqd4_22 = -ORjdqd4_32*qd[28]
    Apqpjdqd4_32 = ORjdqd4_22*qd[28]
    RLjdqd4_23 = ROjdqd4_52*s.dpt[2,40]+ROjdqd4_82*s.dpt[3,40]
    RLjdqd4_33 = ROjdqd4_62*s.dpt[2,40]+ROjdqd4_92*s.dpt[3,40]
    ORjdqd4_23 = -OMjdqd4_12*RLjdqd4_33
    ORjdqd4_33 = OMjdqd4_12*RLjdqd4_23
    Apqpjdqd4_23 = Apqpjdqd4_22-OMjdqd4_12*ORjdqd4_33
    Apqpjdqd4_33 = Apqpjdqd4_32+OMjdqd4_12*ORjdqd4_23
    jdqd5 = Apqpjdqd3_22-Apqpjdqd4_23
    jdqd6 = Apqpjdqd3_32-Apqpjdqd4_33
    RLjdqd5_22 = s.dpt[2,31]*C23-s.dpt[3,31]*S23
    RLjdqd5_32 = s.dpt[2,31]*S23+s.dpt[3,31]*C23
    ORjdqd5_22 = -RLjdqd5_32*qd[23]
    ORjdqd5_32 = RLjdqd5_22*qd[23]
    Apqpjdqd5_22 = -ORjdqd5_32*qd[23]
    Apqpjdqd5_32 = ORjdqd5_22*qd[23]
    ROjdqd6_22 = S7*S8
    ROjdqd6_32 = -C7*S8
    ROjdqd6_82 = -S7*C8
    ROjdqd6_92 = C7*C8
    ROjdqd6_43 = S8*S9
    ROjdqd6_53 = ROjdqd6_82*S9+C7*C9
    ROjdqd6_63 = ROjdqd6_92*S9+S7*C9
    ROjdqd6_73 = S8*C9
    ROjdqd6_83 = ROjdqd6_82*C9-C7*S9
    ROjdqd6_93 = ROjdqd6_92*C9-S7*S9
    ROjdqd6_44 = ROjdqd6_43*C10-S10*C8
    ROjdqd6_54 = -ROjdqd6_22*S10+ROjdqd6_53*C10
    ROjdqd6_64 = -ROjdqd6_32*S10+ROjdqd6_63*C10
    RLjdqd6_22 = s.dpt[2,14]*C7
    RLjdqd6_32 = s.dpt[2,14]*S7
    OMjdqd6_22 = qd[8]*C7
    OMjdqd6_32 = qd[8]*S7
    ORjdqd6_22 = -RLjdqd6_32*qd[7]
    ORjdqd6_32 = RLjdqd6_22*qd[7]
    Ompqpjdqd6_22 = -qd[7]*qd[8]*S7
    Ompqpjdqd6_32 = qd[7]*qd[8]*C7
    Apqpjdqd6_22 = -ORjdqd6_32*qd[7]
    Apqpjdqd6_32 = ORjdqd6_22*qd[7]
    OMjdqd6_13 = qd[7]+qd[9]*C8
    OMjdqd6_23 = OMjdqd6_22+ROjdqd6_22*qd[9]
    OMjdqd6_33 = OMjdqd6_32+ROjdqd6_32*qd[9]
    Ompqpjdqd6_13 = qd[9]*(OMjdqd6_22*ROjdqd6_32-OMjdqd6_32*ROjdqd6_22)
    Ompqpjdqd6_23 = Ompqpjdqd6_22+qd[9]*(OMjdqd6_32*C8-ROjdqd6_32*qd[7])
    Ompqpjdqd6_33 = Ompqpjdqd6_32+qd[9]*(-OMjdqd6_22*C8+ROjdqd6_22*qd[7])
    OMjdqd6_14 = OMjdqd6_13+ROjdqd6_73*qd[10]
    OMjdqd6_24 = OMjdqd6_23+ROjdqd6_83*qd[10]
    OMjdqd6_34 = OMjdqd6_33+ROjdqd6_93*qd[10]
    Ompqpjdqd6_14 = Ompqpjdqd6_13+qd[10]*(OMjdqd6_23*ROjdqd6_93-OMjdqd6_33*ROjdqd6_83)
    Ompqpjdqd6_24 = Ompqpjdqd6_23+qd[10]*(-OMjdqd6_13*ROjdqd6_93+OMjdqd6_33*ROjdqd6_73)
    Ompqpjdqd6_34 = Ompqpjdqd6_33+qd[10]*(OMjdqd6_13*ROjdqd6_83-OMjdqd6_23*ROjdqd6_73)
    RLjdqd6_15 = ROjdqd6_44*s.dpt[2,17]+ROjdqd6_73*s.dpt[3,17]
    RLjdqd6_25 = ROjdqd6_54*s.dpt[2,17]+ROjdqd6_83*s.dpt[3,17]
    RLjdqd6_35 = ROjdqd6_64*s.dpt[2,17]+ROjdqd6_93*s.dpt[3,17]
    ORjdqd6_15 = OMjdqd6_24*RLjdqd6_35-OMjdqd6_34*RLjdqd6_25
    ORjdqd6_25 = -OMjdqd6_14*RLjdqd6_35+OMjdqd6_34*RLjdqd6_15
    ORjdqd6_35 = OMjdqd6_14*RLjdqd6_25-OMjdqd6_24*RLjdqd6_15
    Apqpjdqd6_15 = OMjdqd6_24*ORjdqd6_35-OMjdqd6_34*ORjdqd6_25+Ompqpjdqd6_24*RLjdqd6_35-Ompqpjdqd6_34*RLjdqd6_25
    Apqpjdqd6_25 = Apqpjdqd6_22-OMjdqd6_14*ORjdqd6_35+OMjdqd6_34*ORjdqd6_15-Ompqpjdqd6_14*RLjdqd6_35+Ompqpjdqd6_34* \
 	  RLjdqd6_15
    Apqpjdqd6_35 = Apqpjdqd6_32+OMjdqd6_14*ORjdqd6_25-OMjdqd6_24*ORjdqd6_15+Ompqpjdqd6_14*RLjdqd6_25-Ompqpjdqd6_24* \
 	  RLjdqd6_15
    jdqd8 = Apqpjdqd5_22-Apqpjdqd6_25
    jdqd9 = Apqpjdqd5_32-Apqpjdqd6_35
    RLjdqd7_22 = s.dpt[2,25]*C17-s.dpt[3,25]*S17
    RLjdqd7_32 = s.dpt[2,25]*S17+s.dpt[3,25]*C17
    ORjdqd7_22 = -RLjdqd7_32*qd[17]
    ORjdqd7_32 = RLjdqd7_22*qd[17]
    Apqpjdqd7_22 = -ORjdqd7_32*qd[17]
    Apqpjdqd7_32 = ORjdqd7_22*qd[17]
    ROjdqd8_22 = S12*S13
    ROjdqd8_32 = -C12*S13
    ROjdqd8_82 = -S12*C13
    ROjdqd8_92 = C12*C13
    ROjdqd8_43 = S13*S14
    ROjdqd8_53 = ROjdqd8_82*S14+C12*C14
    ROjdqd8_63 = ROjdqd8_92*S14+S12*C14
    ROjdqd8_73 = S13*C14
    ROjdqd8_83 = ROjdqd8_82*C14-C12*S14
    ROjdqd8_93 = ROjdqd8_92*C14-S12*S14
    ROjdqd8_44 = ROjdqd8_43*C15-C13*S15
    ROjdqd8_54 = -ROjdqd8_22*S15+ROjdqd8_53*C15
    ROjdqd8_64 = -ROjdqd8_32*S15+ROjdqd8_63*C15
    RLjdqd8_22 = s.dpt[2,20]*C12
    RLjdqd8_32 = s.dpt[2,20]*S12
    OMjdqd8_22 = qd[13]*C12
    OMjdqd8_32 = qd[13]*S12
    ORjdqd8_22 = -RLjdqd8_32*qd[12]
    ORjdqd8_32 = RLjdqd8_22*qd[12]
    Ompqpjdqd8_22 = -qd[12]*qd[13]*S12
    Ompqpjdqd8_32 = qd[12]*qd[13]*C12
    Apqpjdqd8_22 = -ORjdqd8_32*qd[12]
    Apqpjdqd8_32 = ORjdqd8_22*qd[12]
    OMjdqd8_13 = qd[12]+qd[14]*C13
    OMjdqd8_23 = OMjdqd8_22+ROjdqd8_22*qd[14]
    OMjdqd8_33 = OMjdqd8_32+ROjdqd8_32*qd[14]
    Ompqpjdqd8_13 = qd[14]*(OMjdqd8_22*ROjdqd8_32-OMjdqd8_32*ROjdqd8_22)
    Ompqpjdqd8_23 = Ompqpjdqd8_22+qd[14]*(OMjdqd8_32*C13-ROjdqd8_32*qd[12])
    Ompqpjdqd8_33 = Ompqpjdqd8_32+qd[14]*(-OMjdqd8_22*C13+ROjdqd8_22*qd[12])
    OMjdqd8_14 = OMjdqd8_13+ROjdqd8_73*qd[15]
    OMjdqd8_24 = OMjdqd8_23+ROjdqd8_83*qd[15]
    OMjdqd8_34 = OMjdqd8_33+ROjdqd8_93*qd[15]
    Ompqpjdqd8_14 = Ompqpjdqd8_13+qd[15]*(OMjdqd8_23*ROjdqd8_93-OMjdqd8_33*ROjdqd8_83)
    Ompqpjdqd8_24 = Ompqpjdqd8_23+qd[15]*(-OMjdqd8_13*ROjdqd8_93+OMjdqd8_33*ROjdqd8_73)
    Ompqpjdqd8_34 = Ompqpjdqd8_33+qd[15]*(OMjdqd8_13*ROjdqd8_83-OMjdqd8_23*ROjdqd8_73)
    RLjdqd8_15 = ROjdqd8_44*s.dpt[2,21]+ROjdqd8_73*s.dpt[3,21]
    RLjdqd8_25 = ROjdqd8_54*s.dpt[2,21]+ROjdqd8_83*s.dpt[3,21]
    RLjdqd8_35 = ROjdqd8_64*s.dpt[2,21]+ROjdqd8_93*s.dpt[3,21]
    ORjdqd8_15 = OMjdqd8_24*RLjdqd8_35-OMjdqd8_34*RLjdqd8_25
    ORjdqd8_25 = -OMjdqd8_14*RLjdqd8_35+OMjdqd8_34*RLjdqd8_15
    ORjdqd8_35 = OMjdqd8_14*RLjdqd8_25-OMjdqd8_24*RLjdqd8_15
    Apqpjdqd8_15 = OMjdqd8_24*ORjdqd8_35-OMjdqd8_34*ORjdqd8_25+Ompqpjdqd8_24*RLjdqd8_35-Ompqpjdqd8_34*RLjdqd8_25
    Apqpjdqd8_25 = Apqpjdqd8_22-OMjdqd8_14*ORjdqd8_35+OMjdqd8_34*ORjdqd8_15-Ompqpjdqd8_14*RLjdqd8_35+Ompqpjdqd8_34* \
 	  RLjdqd8_15
    Apqpjdqd8_35 = Apqpjdqd8_32+OMjdqd8_14*ORjdqd8_25-OMjdqd8_24*ORjdqd8_15+Ompqpjdqd8_14*RLjdqd8_25-Ompqpjdqd8_24* \
 	  RLjdqd8_15
    jdqd11 = Apqpjdqd7_22-Apqpjdqd8_25
    jdqd12 = Apqpjdqd7_32-Apqpjdqd8_35
    ROjdqd9_22 = S7*S8
    ROjdqd9_32 = -C7*S8
    ROjdqd9_82 = -S7*C8
    ROjdqd9_92 = C7*C8
    ROjdqd9_43 = S8*S9
    ROjdqd9_53 = ROjdqd9_82*S9+C7*C9
    ROjdqd9_63 = ROjdqd9_92*S9+S7*C9
    ROjdqd9_73 = S8*C9
    ROjdqd9_83 = ROjdqd9_82*C9-C7*S9
    ROjdqd9_93 = ROjdqd9_92*C9-S7*S9
    ROjdqd9_14 = ROjdqd9_43*S10+C10*C8
    ROjdqd9_24 = ROjdqd9_22*C10+ROjdqd9_53*S10
    ROjdqd9_34 = ROjdqd9_32*C10+ROjdqd9_63*S10
    ROjdqd9_44 = ROjdqd9_43*C10-S10*C8
    ROjdqd9_54 = -ROjdqd9_22*S10+ROjdqd9_53*C10
    ROjdqd9_64 = -ROjdqd9_32*S10+ROjdqd9_63*C10
    RLjdqd9_22 = s.dpt[2,14]*C7
    RLjdqd9_32 = s.dpt[2,14]*S7
    OMjdqd9_22 = qd[8]*C7
    OMjdqd9_32 = qd[8]*S7
    ORjdqd9_22 = -RLjdqd9_32*qd[7]
    ORjdqd9_32 = RLjdqd9_22*qd[7]
    Ompqpjdqd9_22 = -qd[7]*qd[8]*S7
    Ompqpjdqd9_32 = qd[7]*qd[8]*C7
    Apqpjdqd9_22 = -ORjdqd9_32*qd[7]
    Apqpjdqd9_32 = ORjdqd9_22*qd[7]
    OMjdqd9_13 = qd[7]+qd[9]*C8
    OMjdqd9_23 = OMjdqd9_22+ROjdqd9_22*qd[9]
    OMjdqd9_33 = OMjdqd9_32+ROjdqd9_32*qd[9]
    Ompqpjdqd9_13 = qd[9]*(OMjdqd9_22*ROjdqd9_32-OMjdqd9_32*ROjdqd9_22)
    Ompqpjdqd9_23 = Ompqpjdqd9_22+qd[9]*(OMjdqd9_32*C8-ROjdqd9_32*qd[7])
    Ompqpjdqd9_33 = Ompqpjdqd9_32+qd[9]*(-OMjdqd9_22*C8+ROjdqd9_22*qd[7])
    OMjdqd9_14 = OMjdqd9_13+ROjdqd9_73*qd[10]
    OMjdqd9_24 = OMjdqd9_23+ROjdqd9_83*qd[10]
    OMjdqd9_34 = OMjdqd9_33+ROjdqd9_93*qd[10]
    Ompqpjdqd9_14 = Ompqpjdqd9_13+qd[10]*(OMjdqd9_23*ROjdqd9_93-OMjdqd9_33*ROjdqd9_83)
    Ompqpjdqd9_24 = Ompqpjdqd9_23+qd[10]*(-OMjdqd9_13*ROjdqd9_93+OMjdqd9_33*ROjdqd9_73)
    Ompqpjdqd9_34 = Ompqpjdqd9_33+qd[10]*(OMjdqd9_13*ROjdqd9_83-OMjdqd9_23*ROjdqd9_73)
    RLjdqd9_15 = ROjdqd9_14*s.dpt[1,15]+ROjdqd9_44*s.dpt[2,15]+ROjdqd9_73*s.dpt[3,15]
    RLjdqd9_25 = ROjdqd9_24*s.dpt[1,15]+ROjdqd9_54*s.dpt[2,15]+ROjdqd9_83*s.dpt[3,15]
    RLjdqd9_35 = ROjdqd9_34*s.dpt[1,15]+ROjdqd9_64*s.dpt[2,15]+ROjdqd9_93*s.dpt[3,15]
    ORjdqd9_15 = OMjdqd9_24*RLjdqd9_35-OMjdqd9_34*RLjdqd9_25
    ORjdqd9_25 = -OMjdqd9_14*RLjdqd9_35+OMjdqd9_34*RLjdqd9_15
    ORjdqd9_35 = OMjdqd9_14*RLjdqd9_25-OMjdqd9_24*RLjdqd9_15
    Apqpjdqd9_15 = OMjdqd9_24*ORjdqd9_35-OMjdqd9_34*ORjdqd9_25+Ompqpjdqd9_24*RLjdqd9_35-Ompqpjdqd9_34*RLjdqd9_25
    Apqpjdqd9_25 = Apqpjdqd9_22-OMjdqd9_14*ORjdqd9_35+OMjdqd9_34*ORjdqd9_15-Ompqpjdqd9_14*RLjdqd9_35+Ompqpjdqd9_34* \
 	  RLjdqd9_15
    Apqpjdqd9_35 = Apqpjdqd9_32+OMjdqd9_14*ORjdqd9_25-OMjdqd9_24*ORjdqd9_15+Ompqpjdqd9_14*RLjdqd9_25-Ompqpjdqd9_24* \
 	  RLjdqd9_15
    ROjdqd10_53 = C19*C20
    ROjdqd10_63 = S19*C20
    OMjdqd10_23 = -qd[20]*S19
    OMjdqd10_33 = qd[20]*C19
    Ompqpjdqd10_23 = -qd[19]*qd[20]*C19
    Ompqpjdqd10_33 = -qd[19]*qd[20]*S19
    RLjdqd10_14 = -s.dpt[2,29]*S20
    RLjdqd10_24 = ROjdqd10_53*s.dpt[2,29]
    RLjdqd10_34 = ROjdqd10_63*s.dpt[2,29]
    ORjdqd10_14 = OMjdqd10_23*RLjdqd10_34-OMjdqd10_33*RLjdqd10_24
    ORjdqd10_24 = OMjdqd10_33*RLjdqd10_14-RLjdqd10_34*qd[19]
    ORjdqd10_34 = -OMjdqd10_23*RLjdqd10_14+RLjdqd10_24*qd[19]
    Apqpjdqd10_14 = OMjdqd10_23*ORjdqd10_34-OMjdqd10_33*ORjdqd10_24+Ompqpjdqd10_23*RLjdqd10_34-Ompqpjdqd10_33* \
 	  RLjdqd10_24
    Apqpjdqd10_24 = OMjdqd10_33*ORjdqd10_14-ORjdqd10_34*qd[19]+Ompqpjdqd10_33*RLjdqd10_14
    Apqpjdqd10_34 = -OMjdqd10_23*ORjdqd10_14+ORjdqd10_24*qd[19]-Ompqpjdqd10_23*RLjdqd10_14
    jdqd13 = -Apqpjdqd10_14+Apqpjdqd9_15
    jdqd14 = -Apqpjdqd10_24+Apqpjdqd9_25
    jdqd15 = -Apqpjdqd10_34+Apqpjdqd9_35
    ROjdqd11_53 = C21*C22
    ROjdqd11_63 = S21*C22
    OMjdqd11_23 = -qd[22]*S21
    OMjdqd11_33 = qd[22]*C21
    Ompqpjdqd11_23 = -qd[21]*qd[22]*C21
    Ompqpjdqd11_33 = -qd[21]*qd[22]*S21
    RLjdqd11_14 = -s.dpt[2,30]*S22
    RLjdqd11_24 = ROjdqd11_53*s.dpt[2,30]
    RLjdqd11_34 = ROjdqd11_63*s.dpt[2,30]
    ORjdqd11_14 = OMjdqd11_23*RLjdqd11_34-OMjdqd11_33*RLjdqd11_24
    ORjdqd11_24 = OMjdqd11_33*RLjdqd11_14-RLjdqd11_34*qd[21]
    ORjdqd11_34 = -OMjdqd11_23*RLjdqd11_14+RLjdqd11_24*qd[21]
    Apqpjdqd11_14 = OMjdqd11_23*ORjdqd11_34-OMjdqd11_33*ORjdqd11_24+Ompqpjdqd11_23*RLjdqd11_34-Ompqpjdqd11_33* \
 	  RLjdqd11_24
    Apqpjdqd11_24 = OMjdqd11_33*ORjdqd11_14-ORjdqd11_34*qd[21]+Ompqpjdqd11_33*RLjdqd11_14
    Apqpjdqd11_34 = -OMjdqd11_23*ORjdqd11_14+ORjdqd11_24*qd[21]-Ompqpjdqd11_23*RLjdqd11_14
    ROjdqd12_22 = S12*S13
    ROjdqd12_32 = -C12*S13
    ROjdqd12_82 = -S12*C13
    ROjdqd12_92 = C12*C13
    ROjdqd12_43 = S13*S14
    ROjdqd12_53 = ROjdqd12_82*S14+C12*C14
    ROjdqd12_63 = ROjdqd12_92*S14+S12*C14
    ROjdqd12_73 = S13*C14
    ROjdqd12_83 = ROjdqd12_82*C14-C12*S14
    ROjdqd12_93 = ROjdqd12_92*C14-S12*S14
    ROjdqd12_14 = ROjdqd12_43*S15+C13*C15
    ROjdqd12_24 = ROjdqd12_22*C15+ROjdqd12_53*S15
    ROjdqd12_34 = ROjdqd12_32*C15+ROjdqd12_63*S15
    ROjdqd12_44 = ROjdqd12_43*C15-C13*S15
    ROjdqd12_54 = -ROjdqd12_22*S15+ROjdqd12_53*C15
    ROjdqd12_64 = -ROjdqd12_32*S15+ROjdqd12_63*C15
    RLjdqd12_22 = s.dpt[2,20]*C12
    RLjdqd12_32 = s.dpt[2,20]*S12
    OMjdqd12_22 = qd[13]*C12
    OMjdqd12_32 = qd[13]*S12
    ORjdqd12_22 = -RLjdqd12_32*qd[12]
    ORjdqd12_32 = RLjdqd12_22*qd[12]
    Ompqpjdqd12_22 = -qd[12]*qd[13]*S12
    Ompqpjdqd12_32 = qd[12]*qd[13]*C12
    Apqpjdqd12_22 = -ORjdqd12_32*qd[12]
    Apqpjdqd12_32 = ORjdqd12_22*qd[12]
    OMjdqd12_13 = qd[12]+qd[14]*C13
    OMjdqd12_23 = OMjdqd12_22+ROjdqd12_22*qd[14]
    OMjdqd12_33 = OMjdqd12_32+ROjdqd12_32*qd[14]
    Ompqpjdqd12_13 = qd[14]*(OMjdqd12_22*ROjdqd12_32-OMjdqd12_32*ROjdqd12_22)
    Ompqpjdqd12_23 = Ompqpjdqd12_22+qd[14]*(OMjdqd12_32*C13-ROjdqd12_32*qd[12])
    Ompqpjdqd12_33 = Ompqpjdqd12_32+qd[14]*(-OMjdqd12_22*C13+ROjdqd12_22*qd[12])
    OMjdqd12_14 = OMjdqd12_13+ROjdqd12_73*qd[15]
    OMjdqd12_24 = OMjdqd12_23+ROjdqd12_83*qd[15]
    OMjdqd12_34 = OMjdqd12_33+ROjdqd12_93*qd[15]
    Ompqpjdqd12_14 = Ompqpjdqd12_13+qd[15]*(OMjdqd12_23*ROjdqd12_93-OMjdqd12_33*ROjdqd12_83)
    Ompqpjdqd12_24 = Ompqpjdqd12_23+qd[15]*(-OMjdqd12_13*ROjdqd12_93+OMjdqd12_33*ROjdqd12_73)
    Ompqpjdqd12_34 = Ompqpjdqd12_33+qd[15]*(OMjdqd12_13*ROjdqd12_83-OMjdqd12_23*ROjdqd12_73)
    RLjdqd12_15 = ROjdqd12_14*s.dpt[1,22]+ROjdqd12_44*s.dpt[2,22]+ROjdqd12_73*s.dpt[3,22]
    RLjdqd12_25 = ROjdqd12_24*s.dpt[1,22]+ROjdqd12_54*s.dpt[2,22]+ROjdqd12_83*s.dpt[3,22]
    RLjdqd12_35 = ROjdqd12_34*s.dpt[1,22]+ROjdqd12_64*s.dpt[2,22]+ROjdqd12_93*s.dpt[3,22]
    ORjdqd12_15 = OMjdqd12_24*RLjdqd12_35-OMjdqd12_34*RLjdqd12_25
    ORjdqd12_25 = -OMjdqd12_14*RLjdqd12_35+OMjdqd12_34*RLjdqd12_15
    ORjdqd12_35 = OMjdqd12_14*RLjdqd12_25-OMjdqd12_24*RLjdqd12_15
    Apqpjdqd12_15 = OMjdqd12_24*ORjdqd12_35-OMjdqd12_34*ORjdqd12_25+Ompqpjdqd12_24*RLjdqd12_35-Ompqpjdqd12_34* \
 	  RLjdqd12_25
    Apqpjdqd12_25 = Apqpjdqd12_22-OMjdqd12_14*ORjdqd12_35+OMjdqd12_34*ORjdqd12_15-Ompqpjdqd12_14*RLjdqd12_35+ \
 	  Ompqpjdqd12_34*RLjdqd12_15
    Apqpjdqd12_35 = Apqpjdqd12_32+OMjdqd12_14*ORjdqd12_25-OMjdqd12_24*ORjdqd12_15+Ompqpjdqd12_14*RLjdqd12_25- \
 	  Ompqpjdqd12_24*RLjdqd12_15
    jdqd16 = Apqpjdqd11_14-Apqpjdqd12_15
    jdqd17 = Apqpjdqd11_24-Apqpjdqd12_25
    jdqd18 = Apqpjdqd11_34-Apqpjdqd12_35
    Jdqd[1] = jdqd2
    Jdqd[2] = jdqd3
    Jdqd[3] = jdqd5
    Jdqd[4] = jdqd6
    Jdqd[5] = -Apqpjdqd6_15
    Jdqd[6] = jdqd8
    Jdqd[7] = jdqd9
    Jdqd[8] = -Apqpjdqd8_15
    Jdqd[9] = jdqd11
    Jdqd[10] = jdqd12
    Jdqd[11] = jdqd13
    Jdqd[12] = jdqd14
    Jdqd[13] = jdqd15
    Jdqd[14] = jdqd16
    Jdqd[15] = jdqd17
    Jdqd[16] = jdqd18

# Number of continuation lines = 1


