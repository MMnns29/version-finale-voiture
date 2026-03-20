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
#	==> Function: F7 - Link Forces (1D)
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos, sqrt

def link(frc, trq, Flink, Z, Zd, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S17 = sin(q[17])
    C17 = cos(q[17])
    S23 = sin(q[23])
    C23 = cos(q[23])
    S27 = sin(q[27])
    C27 = cos(q[27])
    S31 = sin(q[31])
    C31 = cos(q[31])
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    RLlnk2_22 = s.dpt[2,26]*C17
    RLlnk2_32 = s.dpt[2,26]*S17
    POlnk2_22 = RLlnk2_22+s.dpt[2,4]
    POlnk2_32 = RLlnk2_32+s.dpt[3,4]
    ORlnk2_22 = -qd[17]*RLlnk2_32
    ORlnk2_32 = qd[17]*RLlnk2_22
    Plnk21 = POlnk2_22-s.dpt[2,3]
    Plnk31 = POlnk2_32-s.dpt[3,3]
    PPlnk1 = Plnk21*Plnk21+Plnk31*Plnk31
    Z1 = sqrt(PPlnk1)
    e21 = Plnk21/Z1
    e31 = Plnk31/Z1
    Zd1 = ORlnk2_22*e21+ORlnk2_32*e31
    RLlnk4_22 = s.dpt[2,32]*C23
    RLlnk4_32 = s.dpt[2,32]*S23
    POlnk4_22 = RLlnk4_22+s.dpt[2,7]
    POlnk4_32 = RLlnk4_32+s.dpt[3,7]
    ORlnk4_22 = -qd[23]*RLlnk4_32
    ORlnk4_32 = qd[23]*RLlnk4_22
    Plnk22 = POlnk4_22-s.dpt[2,6]
    Plnk32 = POlnk4_32-s.dpt[3,6]
    PPlnk2 = Plnk22*Plnk22+Plnk32*Plnk32
    Z2 = sqrt(PPlnk2)
    e22 = Plnk22/Z2
    e32 = Plnk32/Z2
    Zd2 = ORlnk4_22*e22+ORlnk4_32*e32
    RLlnk6_22 = s.dpt[2,38]*C27
    RLlnk6_32 = s.dpt[2,38]*S27
    POlnk6_22 = RLlnk6_22+s.dpt[2,10]
    POlnk6_32 = RLlnk6_32+s.dpt[3,10]
    ORlnk6_22 = -qd[27]*RLlnk6_32
    ORlnk6_32 = qd[27]*RLlnk6_22
    Plnk13 = s.dpt[1,10]-s.dpt[1,9]
    Plnk23 = POlnk6_22-s.dpt[2,9]
    Plnk33 = POlnk6_32-s.dpt[3,9]
    PPlnk3 = Plnk13*Plnk13+Plnk23*Plnk23+Plnk33*Plnk33
    Z3 = sqrt(PPlnk3)
    e13 = Plnk13/Z3
    e23 = Plnk23/Z3
    e33 = Plnk33/Z3
    Zd3 = ORlnk6_22*e23+ORlnk6_32*e33
    RLlnk8_22 = s.dpt[2,44]*C31
    RLlnk8_32 = s.dpt[2,44]*S31
    POlnk8_22 = RLlnk8_22+s.dpt[2,13]
    POlnk8_32 = RLlnk8_32+s.dpt[3,13]
    ORlnk8_22 = -qd[31]*RLlnk8_32
    ORlnk8_32 = qd[31]*RLlnk8_22
    Plnk14 = -s.dpt[1,12]+s.dpt[1,13]
    Plnk24 = POlnk8_22-s.dpt[2,12]
    Plnk34 = POlnk8_32-s.dpt[3,12]
    PPlnk4 = Plnk14*Plnk14+Plnk24*Plnk24+Plnk34*Plnk34
    Z4 = sqrt(PPlnk4)
    e14 = Plnk14/Z4
    e24 = Plnk24/Z4
    e34 = Plnk34/Z4
    Zd4 = ORlnk8_22*e24+ORlnk8_32*e34

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
    Flink2 = s.user_LinkForces(Z2,Zd2,s,tsim,2)
    Flink3 = s.user_LinkForces(Z3,Zd3,s,tsim,3)
    Flink4 = s.user_LinkForces(Z4,Zd4,s,tsim,4)
 
# Link Dynamics: forces projection on body-fixed frames

    fPlnk21 = Flink1*e21
    fPlnk31 = Flink1*e31
    trqlnk6_1_1 = -fPlnk21*(s.dpt[3,3]-s.l[3,6])+fPlnk31*s.dpt[2,3]
    trqlnk6_1_2 = fPlnk31*s.l[1,6]
    trqlnk6_1_3 = -fPlnk21*s.l[1,6]
    fSlnk21 = Flink1*(e21*C17+e31*S17)
    fSlnk31 = Flink1*(-e21*S17+e31*C17)
    trqlnk17_1_1 = -fSlnk31*s.dpt[2,26]
    fPlnk22 = Flink2*e22
    fPlnk32 = Flink2*e32
    frclnk6_2_2 = fPlnk21+fPlnk22
    frclnk6_2_3 = fPlnk31+fPlnk32
    trqlnk6_2_1 = trqlnk6_1_1-fPlnk22*(s.dpt[3,6]-s.l[3,6])+fPlnk32*s.dpt[2,6]
    trqlnk6_2_2 = trqlnk6_1_2+fPlnk32*s.l[1,6]
    trqlnk6_2_3 = trqlnk6_1_3-fPlnk22*s.l[1,6]
    fSlnk22 = Flink2*(e22*C23+e32*S23)
    fSlnk32 = Flink2*(-e22*S23+e32*C23)
    trqlnk23_2_1 = -fSlnk32*s.dpt[2,32]
    fPlnk13 = Flink3*e13
    fPlnk23 = Flink3*e23
    fPlnk33 = Flink3*e33
    frclnk6_3_2 = fPlnk23+frclnk6_2_2
    frclnk6_3_3 = fPlnk33+frclnk6_2_3
    trqlnk6_3_1 = trqlnk6_2_1-fPlnk23*(s.dpt[3,9]-s.l[3,6])+fPlnk33*s.dpt[2,9]
    trqlnk6_3_2 = trqlnk6_2_2+fPlnk13*(s.dpt[3,9]-s.l[3,6])-fPlnk33*(s.dpt[1,9]-s.l[1,6])
    trqlnk6_3_3 = trqlnk6_2_3-fPlnk13*s.dpt[2,9]+fPlnk23*(s.dpt[1,9]-s.l[1,6])
    fSlnk13 = Flink3*e13
    fSlnk23 = Flink3*(e23*C27+e33*S27)
    fSlnk33 = Flink3*(-e23*S27+e33*C27)
    trqlnk27_3_1 = -fSlnk33*s.dpt[2,38]
    trqlnk27_3_3 = fSlnk13*s.dpt[2,38]
    fPlnk14 = Flink4*e14
    fPlnk24 = Flink4*e24
    fPlnk34 = Flink4*e34
    frclnk6_4_1 = fPlnk13+fPlnk14
    frclnk6_4_2 = fPlnk24+frclnk6_3_2
    frclnk6_4_3 = fPlnk34+frclnk6_3_3
    trqlnk6_4_1 = trqlnk6_3_1-fPlnk24*(s.dpt[3,12]-s.l[3,6])+fPlnk34*s.dpt[2,12]
    trqlnk6_4_2 = trqlnk6_3_2+fPlnk14*(s.dpt[3,12]-s.l[3,6])-fPlnk34*(s.dpt[1,12]-s.l[1,6])
    trqlnk6_4_3 = trqlnk6_3_3-fPlnk14*s.dpt[2,12]+fPlnk24*(s.dpt[1,12]-s.l[1,6])
    fSlnk14 = Flink4*e14
    fSlnk24 = Flink4*(e24*C31+e34*S31)
    fSlnk34 = Flink4*(-e24*S31+e34*C31)
    trqlnk31_4_1 = -fSlnk34*s.dpt[2,44]
    trqlnk31_4_3 = fSlnk14*s.dpt[2,44]
 
# Symbolic model output

    frc[1,6] = s.frc[1,6]+frclnk6_4_1
    frc[2,6] = s.frc[2,6]+frclnk6_4_2
    frc[3,6] = s.frc[3,6]+frclnk6_4_3
    trq[1,6] = s.trq[1,6]+trqlnk6_4_1
    trq[2,6] = s.trq[2,6]+trqlnk6_4_2
    trq[3,6] = s.trq[3,6]+trqlnk6_4_3
    frc[2,17] = s.frc[2,17]-fSlnk21
    frc[3,17] = s.frc[3,17]-fSlnk31
    trq[1,17] = s.trq[1,17]+trqlnk17_1_1
    frc[2,23] = s.frc[2,23]-fSlnk22
    frc[3,23] = s.frc[3,23]-fSlnk32
    trq[1,23] = s.trq[1,23]+trqlnk23_2_1
    frc[1,27] = s.frc[1,27]-fSlnk13
    frc[2,27] = s.frc[2,27]-fSlnk23
    frc[3,27] = s.frc[3,27]-fSlnk33
    trq[1,27] = s.trq[1,27]+trqlnk27_3_1
    trq[3,27] = s.trq[3,27]+trqlnk27_3_3
    frc[1,31] = s.frc[1,31]-fSlnk14
    frc[2,31] = s.frc[2,31]-fSlnk24
    frc[3,31] = s.frc[3,31]-fSlnk34
    trq[1,31] = s.trq[1,31]+trqlnk31_4_1
    trq[3,31] = s.trq[3,31]+trqlnk31_4_3
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1
    Z[2] = Z2
    Zd[2] = Zd2
    Flink[2] = Flink2
    Z[3] = Z3
    Zd[3] = Zd3
    Flink[3] = Flink3
    Z[4] = Z4
    Zd[4] = Zd4
    Flink[4] = Flink4

# Number of continuation lines = 0


