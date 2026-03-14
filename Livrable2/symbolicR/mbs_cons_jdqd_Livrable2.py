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
#	==> Generation Date: Sat Mar 14 21:52:06 2026
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Livrable2
#
#	==> Number of joints: 43
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

    S33 = sin(q[33])
    C33 = cos(q[33])
    S30 = sin(q[30])
    C30 = cos(q[30])
    S31 = sin(q[31])
    C31 = cos(q[31])
    S37 = sin(q[37])
    C37 = cos(q[37])
    S34 = sin(q[34])
    C34 = cos(q[34])
    S35 = sin(q[35])
    C35 = cos(q[35])
    S29 = sin(q[29])
    C29 = cos(q[29])
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
    S38 = sin(q[38])
    C38 = cos(q[38])
    S39 = sin(q[39])
    C39 = cos(q[39])
    S40 = sin(q[40])
    C40 = cos(q[40])
    S41 = sin(q[41])
    C41 = cos(q[41])
    S42 = sin(q[42])
    C42 = cos(q[42])
    S43 = sin(q[43])
    C43 = cos(q[43])
    S23 = sin(q[23])
    C23 = cos(q[23])
    S27 = sin(q[27])
    C27 = cos(q[27])
    S28 = sin(q[28])
    C28 = cos(q[28])
    S24 = sin(q[24])
    C24 = cos(q[24])
    S25 = sin(q[25])
    C25 = cos(q[25])
    S26 = sin(q[26])
    C26 = cos(q[26])
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

    RLjdqd1_22 = s.dpt[2,46]*C33-s.dpt[3,46]*S33
    RLjdqd1_32 = s.dpt[2,46]*S33+s.dpt[3,46]*C33
    ORjdqd1_22 = -RLjdqd1_32*qd[33]
    ORjdqd1_32 = RLjdqd1_22*qd[33]
    Apqpjdqd1_22 = -ORjdqd1_32*qd[33]
    Apqpjdqd1_32 = ORjdqd1_22*qd[33]
    ROjdqd2_52 = C30*C31-S30*S31
    ROjdqd2_62 = C30*S31+S30*C31
    ROjdqd2_82 = -C30*S31-S30*C31
    ROjdqd2_92 = C30*C31-S30*S31
    RLjdqd2_22 = s.dpt[2,41]*C30
    RLjdqd2_32 = s.dpt[2,41]*S30
    OMjdqd2_12 = qd[30]+qd[31]
    ORjdqd2_22 = -RLjdqd2_32*qd[30]
    ORjdqd2_32 = RLjdqd2_22*qd[30]
    Apqpjdqd2_22 = -ORjdqd2_32*qd[30]
    Apqpjdqd2_32 = ORjdqd2_22*qd[30]
    RLjdqd2_23 = ROjdqd2_52*s.dpt[2,43]+ROjdqd2_82*s.dpt[3,43]
    RLjdqd2_33 = ROjdqd2_62*s.dpt[2,43]+ROjdqd2_92*s.dpt[3,43]
    ORjdqd2_23 = -OMjdqd2_12*RLjdqd2_33
    ORjdqd2_33 = OMjdqd2_12*RLjdqd2_23
    Apqpjdqd2_23 = Apqpjdqd2_22-OMjdqd2_12*ORjdqd2_33
    Apqpjdqd2_33 = Apqpjdqd2_32+OMjdqd2_12*ORjdqd2_23
    jdqd2 = Apqpjdqd1_22-Apqpjdqd2_23
    jdqd3 = Apqpjdqd1_32-Apqpjdqd2_33
    RLjdqd3_22 = s.dpt[2,53]*C37-s.dpt[3,53]*S37
    RLjdqd3_32 = s.dpt[2,53]*S37+s.dpt[3,53]*C37
    ORjdqd3_22 = -RLjdqd3_32*qd[37]
    ORjdqd3_32 = RLjdqd3_22*qd[37]
    Apqpjdqd3_22 = -ORjdqd3_32*qd[37]
    Apqpjdqd3_32 = ORjdqd3_22*qd[37]
    ROjdqd4_52 = C34*C35-S34*S35
    ROjdqd4_62 = C34*S35+S34*C35
    ROjdqd4_82 = -C34*S35-S34*C35
    ROjdqd4_92 = C34*C35-S34*S35
    RLjdqd4_22 = s.dpt[2,48]*C34
    RLjdqd4_32 = s.dpt[2,48]*S34
    OMjdqd4_12 = qd[34]+qd[35]
    ORjdqd4_22 = -RLjdqd4_32*qd[34]
    ORjdqd4_32 = RLjdqd4_22*qd[34]
    Apqpjdqd4_22 = -ORjdqd4_32*qd[34]
    Apqpjdqd4_32 = ORjdqd4_22*qd[34]
    RLjdqd4_23 = ROjdqd4_52*s.dpt[2,50]+ROjdqd4_82*s.dpt[3,50]
    RLjdqd4_33 = ROjdqd4_62*s.dpt[2,50]+ROjdqd4_92*s.dpt[3,50]
    ORjdqd4_23 = -OMjdqd4_12*RLjdqd4_33
    ORjdqd4_33 = OMjdqd4_12*RLjdqd4_23
    Apqpjdqd4_23 = Apqpjdqd4_22-OMjdqd4_12*ORjdqd4_33
    Apqpjdqd4_33 = Apqpjdqd4_32+OMjdqd4_12*ORjdqd4_23
    jdqd5 = Apqpjdqd3_22-Apqpjdqd4_23
    jdqd6 = Apqpjdqd3_32-Apqpjdqd4_33
    RLjdqd5_22 = s.dpt[2,39]*C29-s.dpt[3,39]*S29
    RLjdqd5_32 = s.dpt[2,39]*S29+s.dpt[3,39]*C29
    ORjdqd5_22 = -RLjdqd5_32*qd[29]
    ORjdqd5_32 = RLjdqd5_22*qd[29]
    Apqpjdqd5_22 = -ORjdqd5_32*qd[29]
    Apqpjdqd5_32 = ORjdqd5_22*qd[29]
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
    RLjdqd6_22 = s.dpt[2,16]*C7
    RLjdqd6_32 = s.dpt[2,16]*S7
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
    RLjdqd6_15 = ROjdqd6_44*s.dpt[2,20]+ROjdqd6_73*s.dpt[3,20]
    RLjdqd6_25 = ROjdqd6_54*s.dpt[2,20]+ROjdqd6_83*s.dpt[3,20]
    RLjdqd6_35 = ROjdqd6_64*s.dpt[2,20]+ROjdqd6_93*s.dpt[3,20]
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
    RLjdqd7_22 = s.dpt[2,28]*C17-s.dpt[3,28]*S17
    RLjdqd7_32 = s.dpt[2,28]*S17+s.dpt[3,28]*C17
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
    RLjdqd8_22 = s.dpt[2,23]*C12
    RLjdqd8_32 = s.dpt[2,23]*S12
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
    RLjdqd8_15 = ROjdqd8_44*s.dpt[2,24]+ROjdqd8_73*s.dpt[3,24]
    RLjdqd8_25 = ROjdqd8_54*s.dpt[2,24]+ROjdqd8_83*s.dpt[3,24]
    RLjdqd8_35 = ROjdqd8_64*s.dpt[2,24]+ROjdqd8_93*s.dpt[3,24]
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
    RLjdqd9_22 = s.dpt[2,42]*C30
    RLjdqd9_32 = s.dpt[2,42]*S30
    ORjdqd9_22 = -RLjdqd9_32*qd[30]
    ORjdqd9_32 = RLjdqd9_22*qd[30]
    Apqpjdqd9_22 = -ORjdqd9_32*qd[30]
    Apqpjdqd9_32 = ORjdqd9_22*qd[30]
    ROjdqd10_42 = S38*S39
    ROjdqd10_62 = C38*S39
    ROjdqd10_72 = S38*C39
    ROjdqd10_92 = C38*C39
    ROjdqd10_73 = ROjdqd10_72*C40+C38*S40
    ROjdqd10_83 = -S39*C40
    ROjdqd10_93 = ROjdqd10_92*C40-S38*S40
    RLjdqd10_12 = s.dpt[1,55]*C38
    RLjdqd10_32 = -s.dpt[1,55]*S38
    OMjdqd10_12 = qd[39]*C38
    OMjdqd10_32 = -qd[39]*S38
    ORjdqd10_12 = RLjdqd10_32*qd[38]
    ORjdqd10_32 = -RLjdqd10_12*qd[38]
    Ompqpjdqd10_12 = -qd[38]*qd[39]*S38
    Ompqpjdqd10_32 = -qd[38]*qd[39]*C38
    Apqpjdqd10_12 = ORjdqd10_32*qd[38]
    Apqpjdqd10_32 = -ORjdqd10_12*qd[38]
    OMjdqd10_13 = OMjdqd10_12+ROjdqd10_42*qd[40]
    OMjdqd10_23 = qd[38]+qd[40]*C39
    OMjdqd10_33 = OMjdqd10_32+ROjdqd10_62*qd[40]
    Ompqpjdqd10_13 = Ompqpjdqd10_12+qd[40]*(-OMjdqd10_32*C39+ROjdqd10_62*qd[38])
    Ompqpjdqd10_23 = qd[40]*(-OMjdqd10_12*ROjdqd10_62+OMjdqd10_32*ROjdqd10_42)
    Ompqpjdqd10_33 = Ompqpjdqd10_32+qd[40]*(OMjdqd10_12*C39-ROjdqd10_42*qd[38])
    RLjdqd10_14 = ROjdqd10_73*s.dpt[3,57]
    RLjdqd10_24 = ROjdqd10_83*s.dpt[3,57]
    RLjdqd10_34 = ROjdqd10_93*s.dpt[3,57]
    ORjdqd10_14 = OMjdqd10_23*RLjdqd10_34-OMjdqd10_33*RLjdqd10_24
    ORjdqd10_24 = -OMjdqd10_13*RLjdqd10_34+OMjdqd10_33*RLjdqd10_14
    ORjdqd10_34 = OMjdqd10_13*RLjdqd10_24-OMjdqd10_23*RLjdqd10_14
    Apqpjdqd10_14 = Apqpjdqd10_12+OMjdqd10_23*ORjdqd10_34-OMjdqd10_33*ORjdqd10_24+Ompqpjdqd10_23*RLjdqd10_34- \
 	  Ompqpjdqd10_33*RLjdqd10_24
    Apqpjdqd10_24 = -OMjdqd10_13*ORjdqd10_34+OMjdqd10_33*ORjdqd10_14-Ompqpjdqd10_13*RLjdqd10_34+Ompqpjdqd10_33* \
 	  RLjdqd10_14
    Apqpjdqd10_34 = Apqpjdqd10_32+OMjdqd10_13*ORjdqd10_24-OMjdqd10_23*ORjdqd10_14+Ompqpjdqd10_13*RLjdqd10_24- \
 	  Ompqpjdqd10_23*RLjdqd10_14
    jdqd14 = -Apqpjdqd10_24+Apqpjdqd9_22
    jdqd15 = -Apqpjdqd10_34+Apqpjdqd9_32
    RLjdqd11_22 = s.dpt[2,49]*C34
    RLjdqd11_32 = s.dpt[2,49]*S34
    ORjdqd11_22 = -RLjdqd11_32*qd[34]
    ORjdqd11_32 = RLjdqd11_22*qd[34]
    Apqpjdqd11_22 = -ORjdqd11_32*qd[34]
    Apqpjdqd11_32 = ORjdqd11_22*qd[34]
    ROjdqd12_12 = C38*C41-S38*S41
    ROjdqd12_32 = -C38*S41-S38*C41
    ROjdqd12_72 = C38*S41+S38*C41
    ROjdqd12_92 = C38*C41-S38*S41
    ROjdqd12_43 = ROjdqd12_72*S42
    ROjdqd12_63 = ROjdqd12_92*S42
    ROjdqd12_73 = ROjdqd12_72*C42
    ROjdqd12_93 = ROjdqd12_92*C42
    ROjdqd12_74 = ROjdqd12_12*S43+ROjdqd12_73*C43
    ROjdqd12_84 = -S42*C43
    ROjdqd12_94 = ROjdqd12_32*S43+ROjdqd12_93*C43
    OMjdqd12_22 = qd[38]+qd[41]
    RLjdqd12_13 = ROjdqd12_12*s.dpt[1,58]
    RLjdqd12_33 = ROjdqd12_32*s.dpt[1,58]
    OMjdqd12_13 = ROjdqd12_12*qd[42]
    OMjdqd12_33 = ROjdqd12_32*qd[42]
    ORjdqd12_13 = OMjdqd12_22*RLjdqd12_33
    ORjdqd12_33 = -OMjdqd12_22*RLjdqd12_13
    Ompqpjdqd12_13 = OMjdqd12_22*ROjdqd12_32*qd[42]
    Ompqpjdqd12_33 = -OMjdqd12_22*ROjdqd12_12*qd[42]
    Apqpjdqd12_13 = OMjdqd12_22*ORjdqd12_33
    Apqpjdqd12_33 = -OMjdqd12_22*ORjdqd12_13
    OMjdqd12_14 = OMjdqd12_13+ROjdqd12_43*qd[43]
    OMjdqd12_24 = OMjdqd12_22+qd[43]*C42
    OMjdqd12_34 = OMjdqd12_33+ROjdqd12_63*qd[43]
    Ompqpjdqd12_14 = Ompqpjdqd12_13+qd[43]*(OMjdqd12_22*ROjdqd12_63-OMjdqd12_33*C42)
    Ompqpjdqd12_24 = qd[43]*(-OMjdqd12_13*ROjdqd12_63+OMjdqd12_33*ROjdqd12_43)
    Ompqpjdqd12_34 = Ompqpjdqd12_33+qd[43]*(OMjdqd12_13*C42-OMjdqd12_22*ROjdqd12_43)
    RLjdqd12_15 = ROjdqd12_74*s.dpt[3,59]
    RLjdqd12_25 = ROjdqd12_84*s.dpt[3,59]
    RLjdqd12_35 = ROjdqd12_94*s.dpt[3,59]
    ORjdqd12_15 = OMjdqd12_24*RLjdqd12_35-OMjdqd12_34*RLjdqd12_25
    ORjdqd12_25 = -OMjdqd12_14*RLjdqd12_35+OMjdqd12_34*RLjdqd12_15
    ORjdqd12_35 = OMjdqd12_14*RLjdqd12_25-OMjdqd12_24*RLjdqd12_15
    Apqpjdqd12_15 = Apqpjdqd12_13+OMjdqd12_24*ORjdqd12_35-OMjdqd12_34*ORjdqd12_25+Ompqpjdqd12_24*RLjdqd12_35- \
 	  Ompqpjdqd12_34*RLjdqd12_25
    Apqpjdqd12_25 = -OMjdqd12_14*ORjdqd12_35+OMjdqd12_34*ORjdqd12_15-Ompqpjdqd12_14*RLjdqd12_35+Ompqpjdqd12_34* \
 	  RLjdqd12_15
    Apqpjdqd12_35 = Apqpjdqd12_33+OMjdqd12_14*ORjdqd12_25-OMjdqd12_24*ORjdqd12_15+Ompqpjdqd12_14*RLjdqd12_25- \
 	  Ompqpjdqd12_24*RLjdqd12_15
    jdqd17 = Apqpjdqd11_22-Apqpjdqd12_25
    jdqd18 = Apqpjdqd11_32-Apqpjdqd12_35
    RLjdqd13_22 = s.dpt[2,17]*C7
    RLjdqd13_32 = s.dpt[2,17]*S7
    ORjdqd13_22 = -RLjdqd13_32*qd[7]
    ORjdqd13_32 = RLjdqd13_22*qd[7]
    Apqpjdqd13_22 = -ORjdqd13_32*qd[7]
    Apqpjdqd13_32 = ORjdqd13_22*qd[7]
    ROjdqd14_42 = S23*S27
    ROjdqd14_62 = C23*S27
    ROjdqd14_72 = S23*C27
    ROjdqd14_92 = C23*C27
    ROjdqd14_73 = ROjdqd14_72*C28+C23*S28
    ROjdqd14_83 = -S27*C28
    ROjdqd14_93 = ROjdqd14_92*C28-S23*S28
    RLjdqd14_12 = s.dpt[3,35]*S23
    RLjdqd14_32 = s.dpt[3,35]*C23
    OMjdqd14_12 = qd[27]*C23
    OMjdqd14_32 = -qd[27]*S23
    ORjdqd14_12 = RLjdqd14_32*qd[23]
    ORjdqd14_32 = -RLjdqd14_12*qd[23]
    Ompqpjdqd14_12 = -qd[23]*qd[27]*S23
    Ompqpjdqd14_32 = -qd[23]*qd[27]*C23
    Apqpjdqd14_12 = ORjdqd14_32*qd[23]
    Apqpjdqd14_32 = -ORjdqd14_12*qd[23]
    OMjdqd14_13 = OMjdqd14_12+ROjdqd14_42*qd[28]
    OMjdqd14_23 = qd[23]+qd[28]*C27
    OMjdqd14_33 = OMjdqd14_32+ROjdqd14_62*qd[28]
    Ompqpjdqd14_13 = Ompqpjdqd14_12+qd[28]*(-OMjdqd14_32*C27+ROjdqd14_62*qd[23])
    Ompqpjdqd14_23 = qd[28]*(-OMjdqd14_12*ROjdqd14_62+OMjdqd14_32*ROjdqd14_42)
    Ompqpjdqd14_33 = Ompqpjdqd14_32+qd[28]*(OMjdqd14_12*C27-ROjdqd14_42*qd[23])
    RLjdqd14_14 = ROjdqd14_73*s.dpt[3,38]
    RLjdqd14_24 = ROjdqd14_83*s.dpt[3,38]
    RLjdqd14_34 = ROjdqd14_93*s.dpt[3,38]
    ORjdqd14_14 = OMjdqd14_23*RLjdqd14_34-OMjdqd14_33*RLjdqd14_24
    ORjdqd14_24 = -OMjdqd14_13*RLjdqd14_34+OMjdqd14_33*RLjdqd14_14
    ORjdqd14_34 = OMjdqd14_13*RLjdqd14_24-OMjdqd14_23*RLjdqd14_14
    Apqpjdqd14_14 = Apqpjdqd14_12+OMjdqd14_23*ORjdqd14_34-OMjdqd14_33*ORjdqd14_24+Ompqpjdqd14_23*RLjdqd14_34- \
 	  Ompqpjdqd14_33*RLjdqd14_24
    Apqpjdqd14_24 = -OMjdqd14_13*ORjdqd14_34+OMjdqd14_33*ORjdqd14_14-Ompqpjdqd14_13*RLjdqd14_34+Ompqpjdqd14_33* \
 	  RLjdqd14_14
    Apqpjdqd14_34 = Apqpjdqd14_32+OMjdqd14_13*ORjdqd14_24-OMjdqd14_23*ORjdqd14_14+Ompqpjdqd14_13*RLjdqd14_24- \
 	  Ompqpjdqd14_23*RLjdqd14_14
    jdqd20 = Apqpjdqd13_22-Apqpjdqd14_24
    jdqd21 = Apqpjdqd13_32-Apqpjdqd14_34
    RLjdqd15_22 = s.dpt[2,22]*C12
    RLjdqd15_32 = s.dpt[2,22]*S12
    ORjdqd15_22 = -RLjdqd15_32*qd[12]
    ORjdqd15_32 = RLjdqd15_22*qd[12]
    Apqpjdqd15_22 = -ORjdqd15_32*qd[12]
    Apqpjdqd15_32 = ORjdqd15_22*qd[12]
    ROjdqd16_12 = C23*C24-S23*S24
    ROjdqd16_32 = -C23*S24-S23*C24
    ROjdqd16_72 = C23*S24+S23*C24
    ROjdqd16_92 = C23*C24-S23*S24
    ROjdqd16_43 = ROjdqd16_72*S25
    ROjdqd16_63 = ROjdqd16_92*S25
    ROjdqd16_73 = ROjdqd16_72*C25
    ROjdqd16_93 = ROjdqd16_92*C25
    ROjdqd16_14 = ROjdqd16_12*C26-ROjdqd16_73*S26
    ROjdqd16_24 = S25*S26
    ROjdqd16_34 = ROjdqd16_32*C26-ROjdqd16_93*S26
    ROjdqd16_74 = ROjdqd16_12*S26+ROjdqd16_73*C26
    ROjdqd16_84 = -S25*C26
    ROjdqd16_94 = ROjdqd16_32*S26+ROjdqd16_93*C26
    RLjdqd16_12 = s.dpt[1,34]*C23+s.dpt[3,34]*S23
    RLjdqd16_32 = -s.dpt[1,34]*S23+s.dpt[3,34]*C23
    OMjdqd16_22 = qd[23]+qd[24]
    ORjdqd16_12 = RLjdqd16_32*qd[23]
    ORjdqd16_32 = -RLjdqd16_12*qd[23]
    Apqpjdqd16_12 = ORjdqd16_32*qd[23]
    Apqpjdqd16_32 = -ORjdqd16_12*qd[23]
    OMjdqd16_13 = ROjdqd16_12*qd[25]
    OMjdqd16_33 = ROjdqd16_32*qd[25]
    Ompqpjdqd16_13 = OMjdqd16_22*ROjdqd16_32*qd[25]
    Ompqpjdqd16_33 = -OMjdqd16_22*ROjdqd16_12*qd[25]
    OMjdqd16_14 = OMjdqd16_13+ROjdqd16_43*qd[26]
    OMjdqd16_24 = OMjdqd16_22+qd[26]*C25
    OMjdqd16_34 = OMjdqd16_33+ROjdqd16_63*qd[26]
    Ompqpjdqd16_14 = Ompqpjdqd16_13+qd[26]*(OMjdqd16_22*ROjdqd16_63-OMjdqd16_33*C25)
    Ompqpjdqd16_24 = qd[26]*(-OMjdqd16_13*ROjdqd16_63+OMjdqd16_33*ROjdqd16_43)
    Ompqpjdqd16_34 = Ompqpjdqd16_33+qd[26]*(OMjdqd16_13*C25-OMjdqd16_22*ROjdqd16_43)
    RLjdqd16_15 = ROjdqd16_14*s.dpt[1,37]+ROjdqd16_74*s.dpt[3,37]
    RLjdqd16_25 = ROjdqd16_24*s.dpt[1,37]+ROjdqd16_84*s.dpt[3,37]
    RLjdqd16_35 = ROjdqd16_34*s.dpt[1,37]+ROjdqd16_94*s.dpt[3,37]
    ORjdqd16_15 = OMjdqd16_24*RLjdqd16_35-OMjdqd16_34*RLjdqd16_25
    ORjdqd16_25 = -OMjdqd16_14*RLjdqd16_35+OMjdqd16_34*RLjdqd16_15
    ORjdqd16_35 = OMjdqd16_14*RLjdqd16_25-OMjdqd16_24*RLjdqd16_15
    Apqpjdqd16_15 = Apqpjdqd16_12+OMjdqd16_24*ORjdqd16_35-OMjdqd16_34*ORjdqd16_25+Ompqpjdqd16_24*RLjdqd16_35- \
 	  Ompqpjdqd16_34*RLjdqd16_25
    Apqpjdqd16_25 = -OMjdqd16_14*ORjdqd16_35+OMjdqd16_34*ORjdqd16_15-Ompqpjdqd16_14*RLjdqd16_35+Ompqpjdqd16_34* \
 	  RLjdqd16_15
    Apqpjdqd16_35 = Apqpjdqd16_32+OMjdqd16_14*ORjdqd16_25-OMjdqd16_24*ORjdqd16_15+Ompqpjdqd16_14*RLjdqd16_25- \
 	  Ompqpjdqd16_24*RLjdqd16_15
    jdqd23 = Apqpjdqd15_22-Apqpjdqd16_25
    jdqd24 = Apqpjdqd15_32-Apqpjdqd16_35
    ROjdqd17_22 = S7*S8
    ROjdqd17_32 = -C7*S8
    ROjdqd17_82 = -S7*C8
    ROjdqd17_92 = C7*C8
    ROjdqd17_43 = S8*S9
    ROjdqd17_53 = ROjdqd17_82*S9+C7*C9
    ROjdqd17_63 = ROjdqd17_92*S9+S7*C9
    ROjdqd17_73 = S8*C9
    ROjdqd17_83 = ROjdqd17_82*C9-C7*S9
    ROjdqd17_93 = ROjdqd17_92*C9-S7*S9
    ROjdqd17_14 = ROjdqd17_43*S10+C10*C8
    ROjdqd17_24 = ROjdqd17_22*C10+ROjdqd17_53*S10
    ROjdqd17_34 = ROjdqd17_32*C10+ROjdqd17_63*S10
    ROjdqd17_44 = ROjdqd17_43*C10-S10*C8
    ROjdqd17_54 = -ROjdqd17_22*S10+ROjdqd17_53*C10
    ROjdqd17_64 = -ROjdqd17_32*S10+ROjdqd17_63*C10
    RLjdqd17_22 = s.dpt[2,16]*C7
    RLjdqd17_32 = s.dpt[2,16]*S7
    OMjdqd17_22 = qd[8]*C7
    OMjdqd17_32 = qd[8]*S7
    ORjdqd17_22 = -RLjdqd17_32*qd[7]
    ORjdqd17_32 = RLjdqd17_22*qd[7]
    Ompqpjdqd17_22 = -qd[7]*qd[8]*S7
    Ompqpjdqd17_32 = qd[7]*qd[8]*C7
    Apqpjdqd17_22 = -ORjdqd17_32*qd[7]
    Apqpjdqd17_32 = ORjdqd17_22*qd[7]
    OMjdqd17_13 = qd[7]+qd[9]*C8
    OMjdqd17_23 = OMjdqd17_22+ROjdqd17_22*qd[9]
    OMjdqd17_33 = OMjdqd17_32+ROjdqd17_32*qd[9]
    Ompqpjdqd17_13 = qd[9]*(OMjdqd17_22*ROjdqd17_32-OMjdqd17_32*ROjdqd17_22)
    Ompqpjdqd17_23 = Ompqpjdqd17_22+qd[9]*(OMjdqd17_32*C8-ROjdqd17_32*qd[7])
    Ompqpjdqd17_33 = Ompqpjdqd17_32+qd[9]*(-OMjdqd17_22*C8+ROjdqd17_22*qd[7])
    OMjdqd17_14 = OMjdqd17_13+ROjdqd17_73*qd[10]
    OMjdqd17_24 = OMjdqd17_23+ROjdqd17_83*qd[10]
    OMjdqd17_34 = OMjdqd17_33+ROjdqd17_93*qd[10]
    Ompqpjdqd17_14 = Ompqpjdqd17_13+qd[10]*(OMjdqd17_23*ROjdqd17_93-OMjdqd17_33*ROjdqd17_83)
    Ompqpjdqd17_24 = Ompqpjdqd17_23+qd[10]*(-OMjdqd17_13*ROjdqd17_93+OMjdqd17_33*ROjdqd17_73)
    Ompqpjdqd17_34 = Ompqpjdqd17_33+qd[10]*(OMjdqd17_13*ROjdqd17_83-OMjdqd17_23*ROjdqd17_73)
    RLjdqd17_15 = ROjdqd17_14*s.dpt[1,18]+ROjdqd17_44*s.dpt[2,18]+ROjdqd17_73*s.dpt[3,18]
    RLjdqd17_25 = ROjdqd17_24*s.dpt[1,18]+ROjdqd17_54*s.dpt[2,18]+ROjdqd17_83*s.dpt[3,18]
    RLjdqd17_35 = ROjdqd17_34*s.dpt[1,18]+ROjdqd17_64*s.dpt[2,18]+ROjdqd17_93*s.dpt[3,18]
    ORjdqd17_15 = OMjdqd17_24*RLjdqd17_35-OMjdqd17_34*RLjdqd17_25
    ORjdqd17_25 = -OMjdqd17_14*RLjdqd17_35+OMjdqd17_34*RLjdqd17_15
    ORjdqd17_35 = OMjdqd17_14*RLjdqd17_25-OMjdqd17_24*RLjdqd17_15
    Apqpjdqd17_15 = OMjdqd17_24*ORjdqd17_35-OMjdqd17_34*ORjdqd17_25+Ompqpjdqd17_24*RLjdqd17_35-Ompqpjdqd17_34* \
 	  RLjdqd17_25
    Apqpjdqd17_25 = Apqpjdqd17_22-OMjdqd17_14*ORjdqd17_35+OMjdqd17_34*ORjdqd17_15-Ompqpjdqd17_14*RLjdqd17_35+ \
 	  Ompqpjdqd17_34*RLjdqd17_15
    Apqpjdqd17_35 = Apqpjdqd17_32+OMjdqd17_14*ORjdqd17_25-OMjdqd17_24*ORjdqd17_15+Ompqpjdqd17_14*RLjdqd17_25- \
 	  Ompqpjdqd17_24*RLjdqd17_15
    ROjdqd18_53 = C19*C20
    ROjdqd18_63 = S19*C20
    OMjdqd18_23 = -qd[20]*S19
    OMjdqd18_33 = qd[20]*C19
    Ompqpjdqd18_23 = -qd[19]*qd[20]*C19
    Ompqpjdqd18_33 = -qd[19]*qd[20]*S19
    RLjdqd18_14 = -s.dpt[2,32]*S20
    RLjdqd18_24 = ROjdqd18_53*s.dpt[2,32]
    RLjdqd18_34 = ROjdqd18_63*s.dpt[2,32]
    ORjdqd18_14 = OMjdqd18_23*RLjdqd18_34-OMjdqd18_33*RLjdqd18_24
    ORjdqd18_24 = OMjdqd18_33*RLjdqd18_14-RLjdqd18_34*qd[19]
    ORjdqd18_34 = -OMjdqd18_23*RLjdqd18_14+RLjdqd18_24*qd[19]
    Apqpjdqd18_14 = OMjdqd18_23*ORjdqd18_34-OMjdqd18_33*ORjdqd18_24+Ompqpjdqd18_23*RLjdqd18_34-Ompqpjdqd18_33* \
 	  RLjdqd18_24
    Apqpjdqd18_24 = OMjdqd18_33*ORjdqd18_14-ORjdqd18_34*qd[19]+Ompqpjdqd18_33*RLjdqd18_14
    Apqpjdqd18_34 = -OMjdqd18_23*ORjdqd18_14+ORjdqd18_24*qd[19]-Ompqpjdqd18_23*RLjdqd18_14
    jdqd25 = Apqpjdqd17_15-Apqpjdqd18_14
    jdqd26 = Apqpjdqd17_25-Apqpjdqd18_24
    jdqd27 = Apqpjdqd17_35-Apqpjdqd18_34
    ROjdqd19_53 = C21*C22
    ROjdqd19_63 = S21*C22
    OMjdqd19_23 = -qd[22]*S21
    OMjdqd19_33 = qd[22]*C21
    Ompqpjdqd19_23 = -qd[21]*qd[22]*C21
    Ompqpjdqd19_33 = -qd[21]*qd[22]*S21
    RLjdqd19_14 = -s.dpt[2,33]*S22
    RLjdqd19_24 = ROjdqd19_53*s.dpt[2,33]
    RLjdqd19_34 = ROjdqd19_63*s.dpt[2,33]
    ORjdqd19_14 = OMjdqd19_23*RLjdqd19_34-OMjdqd19_33*RLjdqd19_24
    ORjdqd19_24 = OMjdqd19_33*RLjdqd19_14-RLjdqd19_34*qd[21]
    ORjdqd19_34 = -OMjdqd19_23*RLjdqd19_14+RLjdqd19_24*qd[21]
    Apqpjdqd19_14 = OMjdqd19_23*ORjdqd19_34-OMjdqd19_33*ORjdqd19_24+Ompqpjdqd19_23*RLjdqd19_34-Ompqpjdqd19_33* \
 	  RLjdqd19_24
    Apqpjdqd19_24 = OMjdqd19_33*ORjdqd19_14-ORjdqd19_34*qd[21]+Ompqpjdqd19_33*RLjdqd19_14
    Apqpjdqd19_34 = -OMjdqd19_23*ORjdqd19_14+ORjdqd19_24*qd[21]-Ompqpjdqd19_23*RLjdqd19_14
    ROjdqd20_22 = S12*S13
    ROjdqd20_32 = -C12*S13
    ROjdqd20_82 = -S12*C13
    ROjdqd20_92 = C12*C13
    ROjdqd20_43 = S13*S14
    ROjdqd20_53 = ROjdqd20_82*S14+C12*C14
    ROjdqd20_63 = ROjdqd20_92*S14+S12*C14
    ROjdqd20_73 = S13*C14
    ROjdqd20_83 = ROjdqd20_82*C14-C12*S14
    ROjdqd20_93 = ROjdqd20_92*C14-S12*S14
    ROjdqd20_14 = ROjdqd20_43*S15+C13*C15
    ROjdqd20_24 = ROjdqd20_22*C15+ROjdqd20_53*S15
    ROjdqd20_34 = ROjdqd20_32*C15+ROjdqd20_63*S15
    ROjdqd20_44 = ROjdqd20_43*C15-C13*S15
    ROjdqd20_54 = -ROjdqd20_22*S15+ROjdqd20_53*C15
    ROjdqd20_64 = -ROjdqd20_32*S15+ROjdqd20_63*C15
    RLjdqd20_22 = s.dpt[2,23]*C12
    RLjdqd20_32 = s.dpt[2,23]*S12
    OMjdqd20_22 = qd[13]*C12
    OMjdqd20_32 = qd[13]*S12
    ORjdqd20_22 = -RLjdqd20_32*qd[12]
    ORjdqd20_32 = RLjdqd20_22*qd[12]
    Ompqpjdqd20_22 = -qd[12]*qd[13]*S12
    Ompqpjdqd20_32 = qd[12]*qd[13]*C12
    Apqpjdqd20_22 = -ORjdqd20_32*qd[12]
    Apqpjdqd20_32 = ORjdqd20_22*qd[12]
    OMjdqd20_13 = qd[12]+qd[14]*C13
    OMjdqd20_23 = OMjdqd20_22+ROjdqd20_22*qd[14]
    OMjdqd20_33 = OMjdqd20_32+ROjdqd20_32*qd[14]
    Ompqpjdqd20_13 = qd[14]*(OMjdqd20_22*ROjdqd20_32-OMjdqd20_32*ROjdqd20_22)
    Ompqpjdqd20_23 = Ompqpjdqd20_22+qd[14]*(OMjdqd20_32*C13-ROjdqd20_32*qd[12])
    Ompqpjdqd20_33 = Ompqpjdqd20_32+qd[14]*(-OMjdqd20_22*C13+ROjdqd20_22*qd[12])
    OMjdqd20_14 = OMjdqd20_13+ROjdqd20_73*qd[15]
    OMjdqd20_24 = OMjdqd20_23+ROjdqd20_83*qd[15]
    OMjdqd20_34 = OMjdqd20_33+ROjdqd20_93*qd[15]
    Ompqpjdqd20_14 = Ompqpjdqd20_13+qd[15]*(OMjdqd20_23*ROjdqd20_93-OMjdqd20_33*ROjdqd20_83)
    Ompqpjdqd20_24 = Ompqpjdqd20_23+qd[15]*(-OMjdqd20_13*ROjdqd20_93+OMjdqd20_33*ROjdqd20_73)
    Ompqpjdqd20_34 = Ompqpjdqd20_33+qd[15]*(OMjdqd20_13*ROjdqd20_83-OMjdqd20_23*ROjdqd20_73)
    RLjdqd20_15 = ROjdqd20_14*s.dpt[1,25]+ROjdqd20_44*s.dpt[2,25]+ROjdqd20_73*s.dpt[3,25]
    RLjdqd20_25 = ROjdqd20_24*s.dpt[1,25]+ROjdqd20_54*s.dpt[2,25]+ROjdqd20_83*s.dpt[3,25]
    RLjdqd20_35 = ROjdqd20_34*s.dpt[1,25]+ROjdqd20_64*s.dpt[2,25]+ROjdqd20_93*s.dpt[3,25]
    ORjdqd20_15 = OMjdqd20_24*RLjdqd20_35-OMjdqd20_34*RLjdqd20_25
    ORjdqd20_25 = -OMjdqd20_14*RLjdqd20_35+OMjdqd20_34*RLjdqd20_15
    ORjdqd20_35 = OMjdqd20_14*RLjdqd20_25-OMjdqd20_24*RLjdqd20_15
    Apqpjdqd20_15 = OMjdqd20_24*ORjdqd20_35-OMjdqd20_34*ORjdqd20_25+Ompqpjdqd20_24*RLjdqd20_35-Ompqpjdqd20_34* \
 	  RLjdqd20_25
    Apqpjdqd20_25 = Apqpjdqd20_22-OMjdqd20_14*ORjdqd20_35+OMjdqd20_34*ORjdqd20_15-Ompqpjdqd20_14*RLjdqd20_35+ \
 	  Ompqpjdqd20_34*RLjdqd20_15
    Apqpjdqd20_35 = Apqpjdqd20_32+OMjdqd20_14*ORjdqd20_25-OMjdqd20_24*ORjdqd20_15+Ompqpjdqd20_14*RLjdqd20_25- \
 	  Ompqpjdqd20_24*RLjdqd20_15
    jdqd28 = Apqpjdqd19_14-Apqpjdqd20_15
    jdqd29 = Apqpjdqd19_24-Apqpjdqd20_25
    jdqd30 = Apqpjdqd19_34-Apqpjdqd20_35
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
    Jdqd[11] = -Apqpjdqd10_14
    Jdqd[12] = jdqd14
    Jdqd[13] = jdqd15
    Jdqd[14] = -Apqpjdqd12_15
    Jdqd[15] = jdqd17
    Jdqd[16] = jdqd18
    Jdqd[17] = -Apqpjdqd14_14
    Jdqd[18] = jdqd20
    Jdqd[19] = jdqd21
    Jdqd[20] = -Apqpjdqd16_15
    Jdqd[21] = jdqd23
    Jdqd[22] = jdqd24
    Jdqd[23] = jdqd25
    Jdqd[24] = jdqd26
    Jdqd[25] = jdqd27
    Jdqd[26] = jdqd28
    Jdqd[27] = jdqd29
    Jdqd[28] = jdqd30

# Number of continuation lines = 1


