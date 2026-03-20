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
#	==> Function: F2 - Recursive Inverse Dynamics of tree-like MBS
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def invdyna(phi,s,tsim):
    Qq = phi  # compatibility with symbolic generation
    q = s.q
    qd = s.qd
    qdd = s.qdd
 
# Trigonometric functions

    S4 = sin(q[4])
    C4 = cos(q[4])
    S5 = sin(q[5])
    C5 = cos(q[5])
    S6 = sin(q[6])
    C6 = cos(q[6])
    S7 = sin(q[7])
    C7 = cos(q[7])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
    S10 = sin(q[10])
    C10 = cos(q[10])
    S11 = sin(q[11])
    C11 = cos(q[11])
    S12 = sin(q[12])
    C12 = cos(q[12])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S14 = sin(q[14])
    C14 = cos(q[14])
    S15 = sin(q[15])
    C15 = cos(q[15])
    S16 = sin(q[16])
    C16 = cos(q[16])
    S17 = sin(q[17])
    C17 = cos(q[17])
    S19 = sin(q[19])
    C19 = cos(q[19])
    S20 = sin(q[20])
    C20 = cos(q[20])
    S21 = sin(q[21])
    C21 = cos(q[21])
    S22 = sin(q[22])
    C22 = cos(q[22])
    S23 = sin(q[23])
    C23 = cos(q[23])
    S24 = sin(q[24])
    C24 = cos(q[24])
    S25 = sin(q[25])
    C25 = cos(q[25])
    S26 = sin(q[26])
    C26 = cos(q[26])
    S27 = sin(q[27])
    C27 = cos(q[27])
    S28 = sin(q[28])
    C28 = cos(q[28])
    S29 = sin(q[29])
    C29 = cos(q[29])
    S30 = sin(q[30])
    C30 = cos(q[30])
    S31 = sin(q[31])
    C31 = cos(q[31])
 
# Augmented Joint Position Vectors

 
# Forward Kinematics

    ALPHA33 = qdd[3]-s.g[3]
    ALPHA14 = qdd[1]*C4+qdd[2]*S4
    ALPHA24 = -qdd[1]*S4+qdd[2]*C4
    OM25 = qd[4]*S5
    OM35 = qd[4]*C5
    OMp25 = qd[4]*qd[5]*C5+qdd[4]*S5
    OMp35 = -qd[4]*qd[5]*S5+qdd[4]*C5
    ALPHA25 = ALPHA24*C5+ALPHA33*S5
    ALPHA35 = -ALPHA24*S5+ALPHA33*C5
    OM16 = qd[5]*C6-OM35*S6
    OM26 = qd[6]+OM25
    OM36 = qd[5]*S6+OM35*C6
    OMp16 = C6*(qdd[5]-qd[6]*OM35)-S6*(OMp35+qd[5]*qd[6])
    OMp26 = qdd[6]+OMp25
    OMp36 = C6*(OMp35+qd[5]*qd[6])+S6*(qdd[5]-qd[6]*OM35)
    BS16 = -OM26*OM26-OM36*OM36
    BS26 = OM16*OM26
    BS36 = OM16*OM36
    BS56 = -OM16*OM16-OM36*OM36
    BS66 = OM26*OM36
    BS96 = -OM16*OM16-OM26*OM26
    BETA26 = BS26-OMp36
    BETA36 = BS36+OMp26
    BETA46 = BS26+OMp36
    BETA66 = BS66-OMp16
    BETA76 = BS36-OMp26
    BETA86 = BS66+OMp16
    ALPHA16 = ALPHA14*C6-ALPHA35*S6
    ALPHA36 = ALPHA14*S6+ALPHA35*C6
    OM17 = qd[7]+OM16
    OM27 = OM26*C7+OM36*S7
    OM37 = -OM26*S7+OM36*C7
    OMp17 = qdd[7]+OMp16
    OMp27 = C7*(OMp26+qd[7]*OM36)+S7*(OMp36-qd[7]*OM26)
    OMp37 = C7*(OMp36-qd[7]*OM26)-S7*(OMp26+qd[7]*OM36)
    BS27 = OM17*OM27
    BS57 = -OM17*OM17-OM37*OM37
    BS67 = OM27*OM37
    BETA27 = BS27-OMp37
    BETA87 = BS67+OMp17
    ALPHA17 = ALPHA16+BETA26*s.dpt[2,1]+BETA36*s.dpt[3,1]
    ALPHA27 = C7*(ALPHA25+BETA66*s.dpt[3,1]+BS56*s.dpt[2,1])+S7*(ALPHA36+BETA86*s.dpt[2,1]+BS96*s.dpt[3,1])
    ALPHA37 = C7*(ALPHA36+BETA86*s.dpt[2,1]+BS96*s.dpt[3,1])-S7*(ALPHA25+BETA66*s.dpt[3,1]+BS56*s.dpt[2,1])
    OM18 = OM17*C8-OM37*S8
    OM28 = qd[8]+OM27
    OM38 = OM17*S8+OM37*C8
    OMp18 = C8*(OMp17-qd[8]*OM37)-S8*(OMp37+qd[8]*OM17)
    OMp28 = qdd[8]+OMp27
    OMp38 = C8*(OMp37+qd[8]*OM17)+S8*(OMp17-qd[8]*OM37)
    ALPHA18 = C8*(ALPHA17+BETA27*s.dpt[2,14])-S8*(ALPHA37+BETA87*s.dpt[2,14])
    ALPHA28 = ALPHA27+BS57*s.dpt[2,14]
    ALPHA38 = C8*(ALPHA37+BETA87*s.dpt[2,14])+S8*(ALPHA17+BETA27*s.dpt[2,14])
    OM19 = qd[9]+OM18
    OM29 = OM28*C9+OM38*S9
    OM39 = -OM28*S9+OM38*C9
    OMp19 = qdd[9]+OMp18
    OMp29 = C9*(OMp28+qd[9]*OM38)+S9*(OMp38-qd[9]*OM28)
    OMp39 = C9*(OMp38-qd[9]*OM28)-S9*(OMp28+qd[9]*OM38)
    ALPHA29 = ALPHA28*C9+ALPHA38*S9
    ALPHA39 = -ALPHA28*S9+ALPHA38*C9
    OM110 = OM19*C10+OM29*S10
    OM210 = -OM19*S10+OM29*C10
    OM310 = qd[10]+OM39
    OMp110 = C10*(OMp19+qd[10]*OM29)+S10*(OMp29-qd[10]*OM19)
    OMp210 = C10*(OMp29-qd[10]*OM19)-S10*(OMp19+qd[10]*OM29)
    OMp310 = qdd[10]+OMp39
    BS210 = OM110*OM210
    BS310 = OM110*OM310
    BS510 = -OM110*OM110-OM310*OM310
    BS610 = OM210*OM310
    BS910 = -OM110*OM110-OM210*OM210
    BETA210 = BS210-OMp310
    BETA310 = BS310+OMp210
    BETA610 = BS610-OMp110
    BETA810 = BS610+OMp110
    ALPHA110 = ALPHA18*C10+ALPHA29*S10
    ALPHA210 = -ALPHA18*S10+ALPHA29*C10
    OM111 = OM110*C11-OM310*S11
    OM211 = qd[11]+OM210
    OM311 = OM110*S11+OM310*C11
    OMp111 = C11*(OMp110-qd[11]*OM310)-S11*(OMp310+qd[11]*OM110)
    OMp211 = qdd[11]+OMp210
    OMp311 = C11*(OMp310+qd[11]*OM110)+S11*(OMp110-qd[11]*OM310)
    ALPHA111 = C11*(ALPHA110+BETA210*s.dpt[2,16]+BETA310*s.dpt[3,16])-S11*(ALPHA39+BETA810*s.dpt[2,16]+BS910* \
 	  s.dpt[3,16])
    ALPHA211 = ALPHA210+BETA610*s.dpt[3,16]+BS510*s.dpt[2,16]
    ALPHA311 = C11*(ALPHA39+BETA810*s.dpt[2,16]+BS910*s.dpt[3,16])+S11*(ALPHA110+BETA210*s.dpt[2,16]+BETA310* \
 	  s.dpt[3,16])
    OM112 = qd[12]+OM16
    OM212 = OM26*C12+OM36*S12
    OM312 = -OM26*S12+OM36*C12
    OMp112 = qdd[12]+OMp16
    OMp212 = C12*(OMp26+qd[12]*OM36)+S12*(OMp36-qd[12]*OM26)
    OMp312 = C12*(OMp36-qd[12]*OM26)-S12*(OMp26+qd[12]*OM36)
    BS212 = OM112*OM212
    BS512 = -OM112*OM112-OM312*OM312
    BS612 = OM212*OM312
    BETA212 = BS212-OMp312
    BETA812 = BS612+OMp112
    ALPHA112 = ALPHA16+BETA26*s.dpt[2,2]+BETA36*s.dpt[3,2]
    ALPHA212 = C12*(ALPHA25+BETA66*s.dpt[3,2]+BS56*s.dpt[2,2])+S12*(ALPHA36+BETA86*s.dpt[2,2]+BS96*s.dpt[3,2])
    ALPHA312 = C12*(ALPHA36+BETA86*s.dpt[2,2]+BS96*s.dpt[3,2])-S12*(ALPHA25+BETA66*s.dpt[3,2]+BS56*s.dpt[2,2])
    OM113 = OM112*C13-OM312*S13
    OM213 = qd[13]+OM212
    OM313 = OM112*S13+OM312*C13
    OMp113 = C13*(OMp112-qd[13]*OM312)-S13*(OMp312+qd[13]*OM112)
    OMp213 = qdd[13]+OMp212
    OMp313 = C13*(OMp312+qd[13]*OM112)+S13*(OMp112-qd[13]*OM312)
    ALPHA113 = C13*(ALPHA112+BETA212*s.dpt[2,20])-S13*(ALPHA312+BETA812*s.dpt[2,20])
    ALPHA213 = ALPHA212+BS512*s.dpt[2,20]
    ALPHA313 = C13*(ALPHA312+BETA812*s.dpt[2,20])+S13*(ALPHA112+BETA212*s.dpt[2,20])
    OM114 = qd[14]+OM113
    OM214 = OM213*C14+OM313*S14
    OM314 = -OM213*S14+OM313*C14
    OMp114 = qdd[14]+OMp113
    OMp214 = C14*(OMp213+qd[14]*OM313)+S14*(OMp313-qd[14]*OM213)
    OMp314 = C14*(OMp313-qd[14]*OM213)-S14*(OMp213+qd[14]*OM313)
    ALPHA214 = ALPHA213*C14+ALPHA313*S14
    ALPHA314 = -ALPHA213*S14+ALPHA313*C14
    OM115 = OM114*C15+OM214*S15
    OM215 = -OM114*S15+OM214*C15
    OM315 = qd[15]+OM314
    OMp115 = C15*(OMp114+qd[15]*OM214)+S15*(OMp214-qd[15]*OM114)
    OMp215 = C15*(OMp214-qd[15]*OM114)-S15*(OMp114+qd[15]*OM214)
    OMp315 = qdd[15]+OMp314
    BS215 = OM115*OM215
    BS315 = OM115*OM315
    BS515 = -OM115*OM115-OM315*OM315
    BS615 = OM215*OM315
    BS915 = -OM115*OM115-OM215*OM215
    BETA215 = BS215-OMp315
    BETA315 = BS315+OMp215
    BETA615 = BS615-OMp115
    BETA815 = BS615+OMp115
    ALPHA115 = ALPHA113*C15+ALPHA214*S15
    ALPHA215 = -ALPHA113*S15+ALPHA214*C15
    OM116 = OM115*C16-OM315*S16
    OM216 = qd[16]+OM215
    OM316 = OM115*S16+OM315*C16
    OMp116 = C16*(OMp115-qd[16]*OM315)-S16*(OMp315+qd[16]*OM115)
    OMp216 = qdd[16]+OMp215
    OMp316 = C16*(OMp315+qd[16]*OM115)+S16*(OMp115-qd[16]*OM315)
    ALPHA116 = C16*(ALPHA115+BETA215*s.dpt[2,23]+BETA315*s.dpt[3,23])-S16*(ALPHA314+BETA815*s.dpt[2,23]+BS915* \
 	  s.dpt[3,23])
    ALPHA216 = ALPHA215+BETA615*s.dpt[3,23]+BS515*s.dpt[2,23]
    ALPHA316 = C16*(ALPHA314+BETA815*s.dpt[2,23]+BS915*s.dpt[3,23])+S16*(ALPHA115+BETA215*s.dpt[2,23]+BETA315* \
 	  s.dpt[3,23])
    OM117 = qd[17]+OM16
    OM217 = OM26*C17+OM36*S17
    OM317 = -OM26*S17+OM36*C17
    OMp117 = qdd[17]+OMp16
    OMp217 = C17*(OMp26+qd[17]*OM36)+S17*(OMp36-qd[17]*OM26)
    OMp317 = C17*(OMp36-qd[17]*OM26)-S17*(OMp26+qd[17]*OM36)
    ALPHA117 = ALPHA16+BETA26*s.dpt[2,4]+BETA36*s.dpt[3,4]
    ALPHA217 = C17*(ALPHA25+BETA66*s.dpt[3,4]+BS56*s.dpt[2,4])+S17*(ALPHA36+BETA86*s.dpt[2,4]+BS96*s.dpt[3,4])
    ALPHA317 = C17*(ALPHA36+BETA86*s.dpt[2,4]+BS96*s.dpt[3,4])-S17*(ALPHA25+BETA66*s.dpt[3,4]+BS56*s.dpt[2,4])
    BS218 = OM16*OM26
    BS518 = -OM16*OM16-OM36*OM36
    BS618 = OM26*OM36
    BETA218 = BS218-OMp36
    BETA818 = BS618+OMp16
    ALPHA118 = ALPHA16+q[18]*BETA26-(2.0)*qd[18]*OM36+BETA36*s.dpt[3,5]+BS16*s.dpt[1,5]
    ALPHA218 = qdd[18]+ALPHA25+q[18]*BS56+BETA46*s.dpt[1,5]+BETA66*s.dpt[3,5]
    ALPHA318 = ALPHA36+q[18]*BETA86+(2.0)*qd[18]*OM16+BETA76*s.dpt[1,5]+BS96*s.dpt[3,5]
    OM119 = qd[19]+OM16
    OM219 = OM26*C19+OM36*S19
    OM319 = -OM26*S19+OM36*C19
    OMp119 = qdd[19]+OMp16
    OMp219 = C19*(OMp26+qd[19]*OM36)+S19*(OMp36-qd[19]*OM26)
    OMp319 = C19*(OMp36-qd[19]*OM26)-S19*(OMp26+qd[19]*OM36)
    ALPHA119 = ALPHA118+BETA218*s.dpt[2,27]
    ALPHA219 = C19*(ALPHA218+BS518*s.dpt[2,27])+S19*(ALPHA318+BETA818*s.dpt[2,27])
    ALPHA319 = C19*(ALPHA318+BETA818*s.dpt[2,27])-S19*(ALPHA218+BS518*s.dpt[2,27])
    OM120 = OM119*C20+OM219*S20
    OM220 = -OM119*S20+OM219*C20
    OM320 = qd[20]+OM319
    OMp120 = C20*(OMp119+qd[20]*OM219)+S20*(OMp219-qd[20]*OM119)
    OMp220 = C20*(OMp219-qd[20]*OM119)-S20*(OMp119+qd[20]*OM219)
    OMp320 = qdd[20]+OMp319
    ALPHA120 = ALPHA119*C20+ALPHA219*S20
    ALPHA220 = -ALPHA119*S20+ALPHA219*C20
    OM121 = qd[21]+OM16
    OM221 = OM26*C21+OM36*S21
    OM321 = -OM26*S21+OM36*C21
    OMp121 = qdd[21]+OMp16
    OMp221 = C21*(OMp26+qd[21]*OM36)+S21*(OMp36-qd[21]*OM26)
    OMp321 = C21*(OMp36-qd[21]*OM26)-S21*(OMp26+qd[21]*OM36)
    ALPHA121 = ALPHA118+BETA218*s.dpt[2,28]
    ALPHA221 = C21*(ALPHA218+BS518*s.dpt[2,28])+S21*(ALPHA318+BETA818*s.dpt[2,28])
    ALPHA321 = C21*(ALPHA318+BETA818*s.dpt[2,28])-S21*(ALPHA218+BS518*s.dpt[2,28])
    OM122 = OM121*C22+OM221*S22
    OM222 = -OM121*S22+OM221*C22
    OM322 = qd[22]+OM321
    OMp122 = C22*(OMp121+qd[22]*OM221)+S22*(OMp221-qd[22]*OM121)
    OMp222 = C22*(OMp221-qd[22]*OM121)-S22*(OMp121+qd[22]*OM221)
    OMp322 = qdd[22]+OMp321
    ALPHA122 = ALPHA121*C22+ALPHA221*S22
    ALPHA222 = -ALPHA121*S22+ALPHA221*C22
    OM123 = qd[23]+OM16
    OM223 = OM26*C23+OM36*S23
    OM323 = -OM26*S23+OM36*C23
    OMp123 = qdd[23]+OMp16
    OMp223 = C23*(OMp26+qd[23]*OM36)+S23*(OMp36-qd[23]*OM26)
    OMp323 = C23*(OMp36-qd[23]*OM26)-S23*(OMp26+qd[23]*OM36)
    ALPHA123 = ALPHA16+BETA26*s.dpt[2,7]+BETA36*s.dpt[3,7]
    ALPHA223 = C23*(ALPHA25+BETA66*s.dpt[3,7]+BS56*s.dpt[2,7])+S23*(ALPHA36+BETA86*s.dpt[2,7]+BS96*s.dpt[3,7])
    ALPHA323 = C23*(ALPHA36+BETA86*s.dpt[2,7]+BS96*s.dpt[3,7])-S23*(ALPHA25+BETA66*s.dpt[3,7]+BS56*s.dpt[2,7])
    OM124 = qd[24]+OM16
    OM224 = OM26*C24+OM36*S24
    OM324 = -OM26*S24+OM36*C24
    OMp124 = qdd[24]+OMp16
    OMp224 = C24*(OMp26+qd[24]*OM36)+S24*(OMp36-qd[24]*OM26)
    OMp324 = C24*(OMp36-qd[24]*OM26)-S24*(OMp26+qd[24]*OM36)
    BS224 = OM124*OM224
    BS524 = -OM124*OM124-OM324*OM324
    BS624 = OM224*OM324
    BETA224 = BS224-OMp324
    BETA824 = BS624+OMp124
    ALPHA124 = ALPHA16+BETA26*s.dpt[2,8]+BETA36*s.dpt[3,8]+BS16*s.dpt[1,8]
    ALPHA224 = C24*(ALPHA25+BETA46*s.dpt[1,8]+BETA66*s.dpt[3,8]+BS56*s.dpt[2,8])+S24*(ALPHA36+BETA76*s.dpt[1,8]+ \
 	  BETA86*s.dpt[2,8]+BS96*s.dpt[3,8])
    ALPHA324 = C24*(ALPHA36+BETA76*s.dpt[1,8]+BETA86*s.dpt[2,8]+BS96*s.dpt[3,8])-S24*(ALPHA25+BETA46*s.dpt[1,8]+ \
 	  BETA66*s.dpt[3,8]+BS56*s.dpt[2,8])
    OM125 = qd[25]+OM124
    OM225 = OM224*C25+OM324*S25
    OM325 = -OM224*S25+OM324*C25
    OMp125 = qdd[25]+OMp124
    OMp225 = C25*(OMp224+qd[25]*OM324)+S25*(OMp324-qd[25]*OM224)
    OMp325 = C25*(OMp324-qd[25]*OM224)-S25*(OMp224+qd[25]*OM324)
    BS225 = OM125*OM225
    BS325 = OM125*OM325
    BS525 = -OM125*OM125-OM325*OM325
    BS625 = OM225*OM325
    BS925 = -OM125*OM125-OM225*OM225
    BETA225 = BS225-OMp325
    BETA325 = BS325+OMp225
    BETA625 = BS625-OMp125
    BETA825 = BS625+OMp125
    ALPHA125 = ALPHA124+BETA224*s.dpt[2,33]
    ALPHA225 = C25*(ALPHA224+BS524*s.dpt[2,33])+S25*(ALPHA324+BETA824*s.dpt[2,33])
    ALPHA325 = C25*(ALPHA324+BETA824*s.dpt[2,33])-S25*(ALPHA224+BS524*s.dpt[2,33])
    OM126 = OM125*C26-OM325*S26
    OM226 = qd[26]+OM225
    OM326 = OM125*S26+OM325*C26
    OMp126 = C26*(OMp125-qd[26]*OM325)-S26*(OMp325+qd[26]*OM125)
    OMp226 = qdd[26]+OMp225
    OMp326 = C26*(OMp325+qd[26]*OM125)+S26*(OMp125-qd[26]*OM325)
    ALPHA126 = C26*(ALPHA125+BETA225*s.dpt[2,35]+BETA325*s.dpt[3,35])-S26*(ALPHA325+BETA825*s.dpt[2,35]+BS925* \
 	  s.dpt[3,35])
    ALPHA226 = ALPHA225+BETA625*s.dpt[3,35]+BS525*s.dpt[2,35]
    ALPHA326 = C26*(ALPHA325+BETA825*s.dpt[2,35]+BS925*s.dpt[3,35])+S26*(ALPHA125+BETA225*s.dpt[2,35]+BETA325* \
 	  s.dpt[3,35])
    OM127 = qd[27]+OM16
    OM227 = OM26*C27+OM36*S27
    OM327 = -OM26*S27+OM36*C27
    OMp127 = qdd[27]+OMp16
    OMp227 = C27*(OMp26+qd[27]*OM36)+S27*(OMp36-qd[27]*OM26)
    OMp327 = C27*(OMp36-qd[27]*OM26)-S27*(OMp26+qd[27]*OM36)
    ALPHA127 = ALPHA16+BETA26*s.dpt[2,10]+BETA36*s.dpt[3,10]+BS16*s.dpt[1,10]
    ALPHA227 = C27*(ALPHA25+BETA46*s.dpt[1,10]+BETA66*s.dpt[3,10]+BS56*s.dpt[2,10])+S27*(ALPHA36+BETA76*s.dpt[1,10]+ \
 	  BETA86*s.dpt[2,10]+BS96*s.dpt[3,10])
    ALPHA327 = C27*(ALPHA36+BETA76*s.dpt[1,10]+BETA86*s.dpt[2,10]+BS96*s.dpt[3,10])-S27*(ALPHA25+BETA46*s.dpt[1,10]+ \
 	  BETA66*s.dpt[3,10]+BS56*s.dpt[2,10])
    OM128 = qd[28]+OM16
    OM228 = OM26*C28+OM36*S28
    OM328 = -OM26*S28+OM36*C28
    OMp128 = qdd[28]+OMp16
    OMp228 = C28*(OMp26+qd[28]*OM36)+S28*(OMp36-qd[28]*OM26)
    OMp328 = C28*(OMp36-qd[28]*OM26)-S28*(OMp26+qd[28]*OM36)
    BS228 = OM128*OM228
    BS528 = -OM128*OM128-OM328*OM328
    BS628 = OM228*OM328
    BETA228 = BS228-OMp328
    BETA828 = BS628+OMp128
    ALPHA128 = ALPHA16+BETA26*s.dpt[2,11]+BETA36*s.dpt[3,11]+BS16*s.dpt[1,11]
    ALPHA228 = C28*(ALPHA25+BETA46*s.dpt[1,11]+BETA66*s.dpt[3,11]+BS56*s.dpt[2,11])+S28*(ALPHA36+BETA76*s.dpt[1,11]+ \
 	  BETA86*s.dpt[2,11]+BS96*s.dpt[3,11])
    ALPHA328 = C28*(ALPHA36+BETA76*s.dpt[1,11]+BETA86*s.dpt[2,11]+BS96*s.dpt[3,11])-S28*(ALPHA25+BETA46*s.dpt[1,11]+ \
 	  BETA66*s.dpt[3,11]+BS56*s.dpt[2,11])
    OM129 = qd[29]+OM128
    OM229 = OM228*C29+OM328*S29
    OM329 = -OM228*S29+OM328*C29
    OMp129 = qdd[29]+OMp128
    OMp229 = C29*(OMp228+qd[29]*OM328)+S29*(OMp328-qd[29]*OM228)
    OMp329 = C29*(OMp328-qd[29]*OM228)-S29*(OMp228+qd[29]*OM328)
    BS229 = OM129*OM229
    BS329 = OM129*OM329
    BS529 = -OM129*OM129-OM329*OM329
    BS629 = OM229*OM329
    BS929 = -OM129*OM129-OM229*OM229
    BETA229 = BS229-OMp329
    BETA329 = BS329+OMp229
    BETA629 = BS629-OMp129
    BETA829 = BS629+OMp129
    ALPHA129 = ALPHA128+BETA228*s.dpt[2,39]
    ALPHA229 = C29*(ALPHA228+BS528*s.dpt[2,39])+S29*(ALPHA328+BETA828*s.dpt[2,39])
    ALPHA329 = C29*(ALPHA328+BETA828*s.dpt[2,39])-S29*(ALPHA228+BS528*s.dpt[2,39])
    OM130 = OM129*C30-OM329*S30
    OM230 = qd[30]+OM229
    OM330 = OM129*S30+OM329*C30
    OMp130 = C30*(OMp129-qd[30]*OM329)-S30*(OMp329+qd[30]*OM129)
    OMp230 = qdd[30]+OMp229
    OMp330 = C30*(OMp329+qd[30]*OM129)+S30*(OMp129-qd[30]*OM329)
    ALPHA130 = C30*(ALPHA129+BETA229*s.dpt[2,41]+BETA329*s.dpt[3,41])-S30*(ALPHA329+BETA829*s.dpt[2,41]+BS929* \
 	  s.dpt[3,41])
    ALPHA230 = ALPHA229+BETA629*s.dpt[3,41]+BS529*s.dpt[2,41]
    ALPHA330 = C30*(ALPHA329+BETA829*s.dpt[2,41]+BS929*s.dpt[3,41])+S30*(ALPHA129+BETA229*s.dpt[2,41]+BETA329* \
 	  s.dpt[3,41])
    OM131 = qd[31]+OM16
    OM231 = OM26*C31+OM36*S31
    OM331 = -OM26*S31+OM36*C31
    OMp131 = qdd[31]+OMp16
    OMp231 = C31*(OMp26+qd[31]*OM36)+S31*(OMp36-qd[31]*OM26)
    OMp331 = C31*(OMp36-qd[31]*OM26)-S31*(OMp26+qd[31]*OM36)
    ALPHA131 = ALPHA16+BETA26*s.dpt[2,13]+BETA36*s.dpt[3,13]+BS16*s.dpt[1,13]
    ALPHA231 = C31*(ALPHA25+BETA46*s.dpt[1,13]+BETA66*s.dpt[3,13]+BS56*s.dpt[2,13])+S31*(ALPHA36+BETA76*s.dpt[1,13]+ \
 	  BETA86*s.dpt[2,13]+BS96*s.dpt[3,13])
    ALPHA331 = C31*(ALPHA36+BETA76*s.dpt[1,13]+BETA86*s.dpt[2,13]+BS96*s.dpt[3,13])-S31*(ALPHA25+BETA46*s.dpt[1,13]+ \
 	  BETA66*s.dpt[3,13]+BS56*s.dpt[2,13])
 
# Backward Dynamics

    Fs131 = -s.frc[1,31]+s.m[31]*ALPHA131
    Fs231 = -s.frc[2,31]+s.m[31]*ALPHA231
    Fs331 = -s.frc[3,31]+s.m[31]*ALPHA331
    Cq131 = -s.trq[1,31]+s.In[1,31]*OMp131-s.In[5,31]*OM231*OM331+s.In[9,31]*OM231*OM331
    Cq231 = -s.trq[2,31]+s.In[1,31]*OM131*OM331+s.In[5,31]*OMp231-s.In[9,31]*OM131*OM331
    Cq331 = -s.trq[3,31]-s.In[1,31]*OM131*OM231+s.In[5,31]*OM131*OM231+s.In[9,31]*OMp331
    Fs130 = -s.frc[1,30]+s.m[30]*ALPHA130
    Fs230 = -s.frc[2,30]+s.m[30]*ALPHA230
    Fs330 = -s.frc[3,30]+s.m[30]*ALPHA330
    Cq130 = -s.trq[1,30]+s.In[1,30]*OMp130-s.In[5,30]*OM230*OM330+s.In[9,30]*OM230*OM330
    Cq230 = -s.trq[2,30]+s.In[1,30]*OM130*OM330+s.In[5,30]*OMp230-s.In[9,30]*OM130*OM330
    Cq330 = -s.trq[3,30]-s.In[1,30]*OM130*OM230+s.In[5,30]*OM130*OM230+s.In[9,30]*OMp330
    Fs129 = -s.frc[1,29]+s.m[29]*ALPHA129
    Fs229 = -s.frc[2,29]+s.m[29]*ALPHA229
    Fs329 = -s.frc[3,29]+s.m[29]*ALPHA329
    Fq129 = Fs129+Fs130*C30+Fs330*S30
    Fq229 = Fs229+Fs230
    Fq329 = Fs329-Fs130*S30+Fs330*C30
    Cq129 = -s.trq[1,29]+s.In[1,29]*OMp129-s.In[5,29]*OM229*OM329+s.In[9,29]*OM229*OM329+Cq130*C30+Cq330*S30-Fs230* \
 	  s.dpt[3,41]+s.dpt[2,41]*(-Fs130*S30+Fs330*C30)
    Cq229 = -s.trq[2,29]+Cq230+s.In[1,29]*OM129*OM329+s.In[5,29]*OMp229-s.In[9,29]*OM129*OM329+s.dpt[3,41]*(Fs130*C30 \
 	  +Fs330*S30)
    Cq329 = -s.trq[3,29]-s.In[1,29]*OM129*OM229+s.In[5,29]*OM129*OM229+s.In[9,29]*OMp329-Cq130*S30+Cq330*C30- \
 	  s.dpt[2,41]*(Fs130*C30+Fs330*S30)
    Fs128 = -s.frc[1,28]+s.m[28]*ALPHA128
    Fs228 = -s.frc[2,28]+s.m[28]*ALPHA228
    Fs328 = -s.frc[3,28]+s.m[28]*ALPHA328
    Fq128 = Fq129+Fs128
    Fq228 = Fs228+Fq229*C29-Fq329*S29
    Fq328 = Fs328+Fq229*S29+Fq329*C29
    Cq128 = -s.trq[1,28]+Cq129+s.In[1,28]*OMp128-s.In[5,28]*OM228*OM328+s.In[9,28]*OM228*OM328+s.dpt[2,39]*(Fq229*S29 \
 	  +Fq329*C29)
    Cq228 = -s.trq[2,28]+s.In[1,28]*OM128*OM328+s.In[5,28]*OMp228-s.In[9,28]*OM128*OM328+Cq229*C29-Cq329*S29
    Cq328 = -s.trq[3,28]-s.In[1,28]*OM128*OM228+s.In[5,28]*OM128*OM228+s.In[9,28]*OMp328+Cq229*S29+Cq329*C29-Fq129* \
 	  s.dpt[2,39]
    Fs127 = -s.frc[1,27]+s.m[27]*ALPHA127
    Fs227 = -s.frc[2,27]+s.m[27]*ALPHA227
    Fs327 = -s.frc[3,27]+s.m[27]*ALPHA327
    Cq127 = -s.trq[1,27]+s.In[1,27]*OMp127-s.In[5,27]*OM227*OM327+s.In[9,27]*OM227*OM327
    Cq227 = -s.trq[2,27]+s.In[1,27]*OM127*OM327+s.In[5,27]*OMp227-s.In[9,27]*OM127*OM327
    Cq327 = -s.trq[3,27]-s.In[1,27]*OM127*OM227+s.In[5,27]*OM127*OM227+s.In[9,27]*OMp327
    Fs126 = -s.frc[1,26]+s.m[26]*ALPHA126
    Fs226 = -s.frc[2,26]+s.m[26]*ALPHA226
    Fs326 = -s.frc[3,26]+s.m[26]*ALPHA326
    Cq126 = -s.trq[1,26]+s.In[1,26]*OMp126-s.In[5,26]*OM226*OM326+s.In[9,26]*OM226*OM326
    Cq226 = -s.trq[2,26]+s.In[1,26]*OM126*OM326+s.In[5,26]*OMp226-s.In[9,26]*OM126*OM326
    Cq326 = -s.trq[3,26]-s.In[1,26]*OM126*OM226+s.In[5,26]*OM126*OM226+s.In[9,26]*OMp326
    Fs125 = -s.frc[1,25]+s.m[25]*ALPHA125
    Fs225 = -s.frc[2,25]+s.m[25]*ALPHA225
    Fs325 = -s.frc[3,25]+s.m[25]*ALPHA325
    Fq125 = Fs125+Fs126*C26+Fs326*S26
    Fq225 = Fs225+Fs226
    Fq325 = Fs325-Fs126*S26+Fs326*C26
    Cq125 = -s.trq[1,25]+s.In[1,25]*OMp125-s.In[5,25]*OM225*OM325+s.In[9,25]*OM225*OM325+Cq126*C26+Cq326*S26-Fs226* \
 	  s.dpt[3,35]+s.dpt[2,35]*(-Fs126*S26+Fs326*C26)
    Cq225 = -s.trq[2,25]+Cq226+s.In[1,25]*OM125*OM325+s.In[5,25]*OMp225-s.In[9,25]*OM125*OM325+s.dpt[3,35]*(Fs126*C26 \
 	  +Fs326*S26)
    Cq325 = -s.trq[3,25]-s.In[1,25]*OM125*OM225+s.In[5,25]*OM125*OM225+s.In[9,25]*OMp325-Cq126*S26+Cq326*C26- \
 	  s.dpt[2,35]*(Fs126*C26+Fs326*S26)
    Fs124 = -s.frc[1,24]+s.m[24]*ALPHA124
    Fs224 = -s.frc[2,24]+s.m[24]*ALPHA224
    Fs324 = -s.frc[3,24]+s.m[24]*ALPHA324
    Fq124 = Fq125+Fs124
    Fq224 = Fs224+Fq225*C25-Fq325*S25
    Fq324 = Fs324+Fq225*S25+Fq325*C25
    Cq124 = -s.trq[1,24]+Cq125+s.In[1,24]*OMp124-s.In[5,24]*OM224*OM324+s.In[9,24]*OM224*OM324+s.dpt[2,33]*(Fq225*S25 \
 	  +Fq325*C25)
    Cq224 = -s.trq[2,24]+s.In[1,24]*OM124*OM324+s.In[5,24]*OMp224-s.In[9,24]*OM124*OM324+Cq225*C25-Cq325*S25
    Cq324 = -s.trq[3,24]-s.In[1,24]*OM124*OM224+s.In[5,24]*OM124*OM224+s.In[9,24]*OMp324+Cq225*S25+Cq325*C25-Fq125* \
 	  s.dpt[2,33]
    Fs123 = -s.frc[1,23]+s.m[23]*ALPHA123
    Fs223 = -s.frc[2,23]+s.m[23]*ALPHA223
    Fs323 = -s.frc[3,23]+s.m[23]*ALPHA323
    Cq123 = -s.trq[1,23]+s.In[1,23]*OMp123-s.In[5,23]*OM223*OM323+s.In[9,23]*OM223*OM323
    Cq223 = -s.trq[2,23]+s.In[1,23]*OM123*OM323+s.In[5,23]*OMp223-s.In[9,23]*OM123*OM323
    Cq323 = -s.trq[3,23]-s.In[1,23]*OM123*OM223+s.In[5,23]*OM123*OM223+s.In[9,23]*OMp323
    Fs122 = -s.frc[1,22]+s.m[22]*ALPHA122
    Fs222 = -s.frc[2,22]+s.m[22]*ALPHA222
    Fs322 = -s.frc[3,22]+s.m[22]*ALPHA321
    Cq122 = -s.trq[1,22]+s.In[1,22]*OMp122-s.In[5,22]*OM222*OM322+s.In[9,22]*OM222*OM322
    Cq222 = -s.trq[2,22]+s.In[1,22]*OM122*OM322+s.In[5,22]*OMp222-s.In[9,22]*OM122*OM322
    Cq322 = -s.trq[3,22]-s.In[1,22]*OM122*OM222+s.In[5,22]*OM122*OM222+s.In[9,22]*OMp322
    Fq121 = Fs122*C22-Fs222*S22
    Fq221 = Fs122*S22+Fs222*C22
    Cq121 = Cq122*C22-Cq222*S22
    Cq221 = Cq122*S22+Cq222*C22
    Fs120 = -s.frc[1,20]+s.m[20]*ALPHA120
    Fs220 = -s.frc[2,20]+s.m[20]*ALPHA220
    Fs320 = -s.frc[3,20]+s.m[20]*ALPHA319
    Cq120 = -s.trq[1,20]+s.In[1,20]*OMp120-s.In[5,20]*OM220*OM320+s.In[9,20]*OM220*OM320
    Cq220 = -s.trq[2,20]+s.In[1,20]*OM120*OM320+s.In[5,20]*OMp220-s.In[9,20]*OM120*OM320
    Cq320 = -s.trq[3,20]-s.In[1,20]*OM120*OM220+s.In[5,20]*OM120*OM220+s.In[9,20]*OMp320
    Fq119 = Fs120*C20-Fs220*S20
    Fq219 = Fs120*S20+Fs220*C20
    Cq119 = Cq120*C20-Cq220*S20
    Cq219 = Cq120*S20+Cq220*C20
    Fs118 = -s.frc[1,18]+s.m[18]*ALPHA118
    Fs218 = -s.frc[2,18]+s.m[18]*ALPHA218
    Fs318 = -s.frc[3,18]+s.m[18]*ALPHA318
    Fq118 = Fq119+Fq121+Fs118
    Fq218 = Fs218+Fq219*C19+Fq221*C21-Fs320*S19-Fs322*S21
    Fq318 = Fs318+Fq219*S19+Fq221*S21+Fs320*C19+Fs322*C21
    Cq118 = -s.trq[1,18]+Cq119+Cq121+s.In[1,18]*OMp16-s.In[5,18]*OM26*OM36+s.In[9,18]*OM26*OM36+s.dpt[2,27]*(Fq219* \
 	  S19+Fs320*C19)+s.dpt[2,28]*(Fq221*S21+Fs322*C21)
    Cq218 = -s.trq[2,18]+s.In[1,18]*OM16*OM36+s.In[5,18]*OMp26-s.In[9,18]*OM16*OM36+Cq219*C19+Cq221*C21-Cq320*S19- \
 	  Cq322*S21
    Cq318 = -s.trq[3,18]-s.In[1,18]*OM16*OM26+s.In[5,18]*OM16*OM26+s.In[9,18]*OMp36+Cq219*S19+Cq221*S21+Cq320*C19+ \
 	  Cq322*C21-Fq119*s.dpt[2,27]-Fq121*s.dpt[2,28]
    Fs117 = -s.frc[1,17]+s.m[17]*ALPHA117
    Fs217 = -s.frc[2,17]+s.m[17]*ALPHA217
    Fs317 = -s.frc[3,17]+s.m[17]*ALPHA317
    Cq117 = -s.trq[1,17]+s.In[1,17]*OMp117-s.In[5,17]*OM217*OM317+s.In[9,17]*OM217*OM317
    Cq217 = -s.trq[2,17]+s.In[1,17]*OM117*OM317+s.In[5,17]*OMp217-s.In[9,17]*OM117*OM317
    Cq317 = -s.trq[3,17]-s.In[1,17]*OM117*OM217+s.In[5,17]*OM117*OM217+s.In[9,17]*OMp317
    Fs116 = -s.frc[1,16]+s.m[16]*ALPHA116
    Fs216 = -s.frc[2,16]+s.m[16]*ALPHA216
    Fs316 = -s.frc[3,16]+s.m[16]*ALPHA316
    Cq116 = -s.trq[1,16]+s.In[1,16]*OMp116-s.In[5,16]*OM216*OM316+s.In[9,16]*OM216*OM316
    Cq216 = -s.trq[2,16]+s.In[1,16]*OM116*OM316+s.In[5,16]*OMp216-s.In[9,16]*OM116*OM316
    Cq316 = -s.trq[3,16]-s.In[1,16]*OM116*OM216+s.In[5,16]*OM116*OM216+s.In[9,16]*OMp316
    Fs115 = -s.frc[1,15]+s.m[15]*ALPHA115
    Fs215 = -s.frc[2,15]+s.m[15]*ALPHA215
    Fs315 = -s.frc[3,15]+s.m[15]*ALPHA314
    Fq115 = Fs115+Fs116*C16+Fs316*S16
    Fq215 = Fs215+Fs216
    Fq315 = Fs315-Fs116*S16+Fs316*C16
    Cq115 = -s.trq[1,15]+s.In[1,15]*OMp115-s.In[5,15]*OM215*OM315+s.In[9,15]*OM215*OM315+Cq116*C16+Cq316*S16-Fs216* \
 	  s.dpt[3,23]+s.dpt[2,23]*(-Fs116*S16+Fs316*C16)
    Cq215 = -s.trq[2,15]+Cq216+s.In[1,15]*OM115*OM315+s.In[5,15]*OMp215-s.In[9,15]*OM115*OM315+s.dpt[3,23]*(Fs116*C16 \
 	  +Fs316*S16)
    Cq315 = -s.trq[3,15]-s.In[1,15]*OM115*OM215+s.In[5,15]*OM115*OM215+s.In[9,15]*OMp315-Cq116*S16+Cq316*C16- \
 	  s.dpt[2,23]*(Fs116*C16+Fs316*S16)
    Fq114 = Fq115*C15-Fq215*S15
    Fq214 = Fq115*S15+Fq215*C15
    Cq114 = Cq115*C15-Cq215*S15
    Cq214 = Cq115*S15+Cq215*C15
    Fq213 = Fq214*C14-Fq315*S14
    Fq313 = Fq214*S14+Fq315*C14
    Cq213 = Cq214*C14-Cq315*S14
    Cq313 = Cq214*S14+Cq315*C14
    Fs112 = -s.frc[1,12]+s.m[12]*ALPHA112
    Fs212 = -s.frc[2,12]+s.m[12]*ALPHA212
    Fs312 = -s.frc[3,12]+s.m[12]*ALPHA312
    Fq112 = Fs112+Fq114*C13+Fq313*S13
    Fq212 = Fq213+Fs212
    Fq312 = Fs312-Fq114*S13+Fq313*C13
    Cq112 = -s.trq[1,12]+s.In[1,12]*OMp112-s.In[5,12]*OM212*OM312+s.In[9,12]*OM212*OM312+Cq114*C13+Cq313*S13+ \
 	  s.dpt[2,20]*(-Fq114*S13+Fq313*C13)
    Cq212 = -s.trq[2,12]+Cq213+s.In[1,12]*OM112*OM312+s.In[5,12]*OMp212-s.In[9,12]*OM112*OM312
    Cq312 = -s.trq[3,12]-s.In[1,12]*OM112*OM212+s.In[5,12]*OM112*OM212+s.In[9,12]*OMp312-Cq114*S13+Cq313*C13- \
 	  s.dpt[2,20]*(Fq114*C13+Fq313*S13)
    Fs111 = -s.frc[1,11]+s.m[11]*ALPHA111
    Fs211 = -s.frc[2,11]+s.m[11]*ALPHA211
    Fs311 = -s.frc[3,11]+s.m[11]*ALPHA311
    Cq111 = -s.trq[1,11]+s.In[1,11]*OMp111-s.In[5,11]*OM211*OM311+s.In[9,11]*OM211*OM311
    Cq211 = -s.trq[2,11]+s.In[1,11]*OM111*OM311+s.In[5,11]*OMp211-s.In[9,11]*OM111*OM311
    Cq311 = -s.trq[3,11]-s.In[1,11]*OM111*OM211+s.In[5,11]*OM111*OM211+s.In[9,11]*OMp311
    Fs110 = -s.frc[1,10]+s.m[10]*ALPHA110
    Fs210 = -s.frc[2,10]+s.m[10]*ALPHA210
    Fs310 = -s.frc[3,10]+s.m[10]*ALPHA39
    Fq110 = Fs110+Fs111*C11+Fs311*S11
    Fq210 = Fs210+Fs211
    Fq310 = Fs310-Fs111*S11+Fs311*C11
    Cq110 = -s.trq[1,10]+s.In[1,10]*OMp110-s.In[5,10]*OM210*OM310+s.In[9,10]*OM210*OM310+Cq111*C11+Cq311*S11-Fs211* \
 	  s.dpt[3,16]+s.dpt[2,16]*(-Fs111*S11+Fs311*C11)
    Cq210 = -s.trq[2,10]+Cq211+s.In[1,10]*OM110*OM310+s.In[5,10]*OMp210-s.In[9,10]*OM110*OM310+s.dpt[3,16]*(Fs111*C11 \
 	  +Fs311*S11)
    Cq310 = -s.trq[3,10]-s.In[1,10]*OM110*OM210+s.In[5,10]*OM110*OM210+s.In[9,10]*OMp310-Cq111*S11+Cq311*C11- \
 	  s.dpt[2,16]*(Fs111*C11+Fs311*S11)
    Fq19 = Fq110*C10-Fq210*S10
    Fq29 = Fq110*S10+Fq210*C10
    Cq19 = Cq110*C10-Cq210*S10
    Cq29 = Cq110*S10+Cq210*C10
    Fq28 = Fq29*C9-Fq310*S9
    Fq38 = Fq29*S9+Fq310*C9
    Cq28 = Cq29*C9-Cq310*S9
    Cq38 = Cq29*S9+Cq310*C9
    Fs17 = -s.frc[1,7]+s.m[7]*ALPHA17
    Fs27 = -s.frc[2,7]+s.m[7]*ALPHA27
    Fs37 = -s.frc[3,7]+s.m[7]*ALPHA37
    Fq17 = Fs17+Fq19*C8+Fq38*S8
    Fq27 = Fq28+Fs27
    Fq37 = Fs37-Fq19*S8+Fq38*C8
    Cq17 = -s.trq[1,7]+s.In[1,7]*OMp17-s.In[5,7]*OM27*OM37+s.In[9,7]*OM27*OM37+Cq19*C8+Cq38*S8+s.dpt[2,14]*(-Fq19*S8+ \
 	  Fq38*C8)
    Cq27 = -s.trq[2,7]+Cq28+s.In[1,7]*OM17*OM37+s.In[5,7]*OMp27-s.In[9,7]*OM17*OM37
    Cq37 = -s.trq[3,7]-s.In[1,7]*OM17*OM27+s.In[5,7]*OM17*OM27+s.In[9,7]*OMp37-Cq19*S8+Cq38*C8-s.dpt[2,14]*(Fq19*C8+ \
 	  Fq38*S8)
    Fs16 = -s.frc[1,6]+s.m[6]*(ALPHA16+BETA36*s.l[3,6]+BS16*s.l[1,6])
    Fs26 = -s.frc[2,6]+s.m[6]*(ALPHA25+BETA46*s.l[1,6]+BETA66*s.l[3,6])
    Fs36 = -s.frc[3,6]+s.m[6]*(ALPHA36+BETA76*s.l[1,6]+BS96*s.l[3,6])
    Fq16 = Fq112+Fq118+Fq124+Fq128+Fq17+Fs117+Fs123+Fs127+Fs131+Fs16
    Fq26 = Fq218+Fs26+Fq212*C12+Fq224*C24+Fq228*C28+Fq27*C7-Fq312*S12-Fq324*S24-Fq328*S28-Fq37*S7+Fs217*C17+Fs223*C23 \
 	  +Fs227*C27+Fs231*C31-Fs317*S17-Fs323*S23-Fs327*S27-Fs331*S31
    Fq36 = Fq318+Fs36+Fq212*S12+Fq224*S24+Fq228*S28+Fq27*S7+Fq312*C12+Fq324*C24+Fq328*C28+Fq37*C7+Fs217*S17+Fs223*S23 \
 	  +Fs227*S27+Fs231*S31+Fs317*C17+Fs323*C23+Fs327*C27+Fs331*C31
    Cq16 = -s.trq[1,6]+Cq112+Cq117+Cq118+Cq123+Cq124+Cq127+Cq128+Cq131+Cq17+q[18]*Fq318+s.In[1,6]*OMp16-s.In[5,6]* \
 	  OM26*OM36+s.In[9,6]*OM26*OM36-Fq218*s.dpt[3,5]-Fs26*s.l[3,6]+s.dpt[2,10]*(Fs227*S27+Fs327*C27)+s.dpt[2,11]*(Fq228*S28+ \
 	  Fq328*C28)+s.dpt[2,13]*(Fs231*S31+Fs331*C31)+s.dpt[2,1]*(Fq27*S7+Fq37*C7)+s.dpt[2,2]*(Fq212*S12+Fq312*C12)+s.dpt[2,4]* \
 	  (Fs217*S17+Fs317*C17)+s.dpt[2,7]*(Fs223*S23+Fs323*C23)+s.dpt[2,8]*(Fq224*S24+Fq324*C24)-s.dpt[3,10]*(Fs227*C27-Fs327* \
 	  S27)-s.dpt[3,11]*(Fq228*C28-Fq328*S28)-s.dpt[3,13]*(Fs231*C31-Fs331*S31)-s.dpt[3,1]*(Fq27*C7-Fq37*S7)-s.dpt[3,2]*( \
 	  Fq212*C12-Fq312*S12)-s.dpt[3,4]*(Fs217*C17-Fs317*S17)-s.dpt[3,7]*(Fs223*C23-Fs323*S23)-s.dpt[3,8]*(Fq224*C24-Fq324*S24 \
 	  )
    Cq26 = -s.trq[2,6]+Cq218+s.In[1,6]*OM16*OM36+s.In[5,6]*OMp26-s.In[9,6]*OM16*OM36+Cq212*C12+Cq217*C17+Cq223*C23+ \
 	  Cq224*C24+Cq227*C27+Cq228*C28+Cq231*C31+Cq27*C7-Cq312*S12-Cq317*S17-Cq323*S23-Cq324*S24-Cq327*S27-Cq328*S28-Cq331*S31- \
 	  Cq37*S7+Fq112*s.dpt[3,2]+Fq118*s.dpt[3,5]+Fq124*s.dpt[3,8]+Fq128*s.dpt[3,11]+Fq17*s.dpt[3,1]-Fq318*s.dpt[1,5]+Fs117* \
 	  s.dpt[3,4]+Fs123*s.dpt[3,7]+Fs127*s.dpt[3,10]+Fs131*s.dpt[3,13]+Fs16*s.l[3,6]-Fs36*s.l[1,6]-s.dpt[1,10]*(Fs227*S27+ \
 	  Fs327*C27)-s.dpt[1,11]*(Fq228*S28+Fq328*C28)-s.dpt[1,13]*(Fs231*S31+Fs331*C31)-s.dpt[1,8]*(Fq224*S24+Fq324*C24)
    Cq36 = -s.trq[3,6]+Cq318-q[18]*Fq118-s.In[1,6]*OM16*OM26+s.In[5,6]*OM16*OM26+s.In[9,6]*OMp36+Cq212*S12+Cq217*S17+ \
 	  Cq223*S23+Cq224*S24+Cq227*S27+Cq228*S28+Cq231*S31+Cq27*S7+Cq312*C12+Cq317*C17+Cq323*C23+Cq324*C24+Cq327*C27+Cq328*C28+ \
 	  Cq331*C31+Cq37*C7-Fq112*s.dpt[2,2]-Fq124*s.dpt[2,8]-Fq128*s.dpt[2,11]-Fq17*s.dpt[2,1]+Fq218*s.dpt[1,5]-Fs117* \
 	  s.dpt[2,4]-Fs123*s.dpt[2,7]-Fs127*s.dpt[2,10]-Fs131*s.dpt[2,13]+Fs26*s.l[1,6]+s.dpt[1,10]*(Fs227*C27-Fs327*S27)+ \
 	  s.dpt[1,11]*(Fq228*C28-Fq328*S28)+s.dpt[1,13]*(Fs231*C31-Fs331*S31)+s.dpt[1,8]*(Fq224*C24-Fq324*S24)
    Fq15 = Fq16*C6+Fq36*S6
    Fq35 = -Fq16*S6+Fq36*C6
    Cq15 = Cq16*C6+Cq36*S6
    Cq35 = -Cq16*S6+Cq36*C6
    Fq24 = Fq26*C5-Fq35*S5
    Fq34 = Fq26*S5+Fq35*C5
    Cq34 = Cq26*S5+Cq35*C5
    Fq13 = Fq15*C4-Fq24*S4
    Fq23 = Fq15*S4+Fq24*C4
 
# Symbolic model output

    Qq[1] = Fq13
    Qq[2] = Fq23
    Qq[3] = Fq34
    Qq[4] = Cq34
    Qq[5] = Cq15
    Qq[6] = Cq26
    Qq[7] = Cq17
    Qq[8] = Cq28
    Qq[9] = Cq19
    Qq[10] = Cq310
    Qq[11] = Cq211
    Qq[12] = Cq112
    Qq[13] = Cq213
    Qq[14] = Cq114
    Qq[15] = Cq315
    Qq[16] = Cq216
    Qq[17] = Cq117
    Qq[18] = Fq218
    Qq[19] = Cq119
    Qq[20] = Cq320
    Qq[21] = Cq121
    Qq[22] = Cq322
    Qq[23] = Cq123
    Qq[24] = Cq124
    Qq[25] = Cq125
    Qq[26] = Cq226
    Qq[27] = Cq127
    Qq[28] = Cq128
    Qq[29] = Cq129
    Qq[30] = Cq230
    Qq[31] = Cq131

# Number of continuation lines = 6


