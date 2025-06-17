# -*- coding: utf-8 -*-
"""
Created on Wed Mar 19 13:46:29 2025

@author: Franz
"""

import numpy as np

np.set_printoptions(precision=5, suppress=True)

# ------------------ Test-Matrix -------------------
debugT = np.array([[11,12,13,14],
                   [21,22,23,24],
                   [31,32,33,34],
                   [41,42,43,44]])
# --------------------------------------------------

# ------------ DH-Paramter [alpha, a, d] -----------
# UR3
dhUR3 = np.array([[1.570796327,  0.0,      0.1519 ],
                 [0.0,          -0.24365, 0.0    ],
                 [0.0,          -0.21325, 0.0    ],
                 [1.570796327,  0.0,      0.11235],
                 [-1.570796327, 0.0,      0.08535],
                 [0.0,          0.0,      0.0819 ]])

qUR3 = np.array([0, -1.570796327, 0, -1.570796327, 0, 0])

# UR5
dhUR5 = np.array([[1.570796327,  0.0,      0.089159 ],
                 [0.0,          -0.42500, 0.0      ],
                 [0.0,          -0.39225, 0.0      ],
                 [1.570796327,  0.0,      0.10915  ],
                 [-1.570796327, 0.0,      0.09465  ],
                 [0.0,          0.0,      0.0823   ]])

qUR5 = np.array([0, -1.570796327, 0, -1.570796327, 0, 0])

#Cobot280
c280 = np.array([[1.5708,  0.0,    131.22 ],
                 [0.0,     -110.4, 0.0    ],
                 [0.0,     -96,    0.0    ],
                 [1.5708,  0.0,    63.4   ],
                 [-1.5708, 0.0,    75.05  ],
                 [0.0,     0.0,    45.6   ]])

qC280 = np.array([0, -1.5708, 0, -1.5708, 1.5708, 0])

# RoArm-M1 
dhRoArm = np.array([[0.0,          0.0,      0.11574 ],
                    [-1.570796327, 0.04105,  0.0     ],
                    [0.0,          0.16886,  0.0     ],
                    [0.0,          0.12792,  -0.01399 ],
                    [1.570796327,  0.10854,  -0.01096 ]])

qRoArm = np.array([0, -1.570796327, 0.0, 0.0, 0.0])

def rotx(a):
    """
    Rotation about x axis
    """
    ca = np.cos(a)
    sa = np.sin(a)
    T = np.array([(1, 0, 0, 0), (0, ca, -sa, 0), (0, sa, ca, 0), (0, 0, 0, 1)])
    return T


def roty(a):
    """
    Rotation about y axis
    """
    ca = np.cos(a)
    sa = np.sin(a)
    T = np.array([(ca, 0, sa, 0), (0, 1, 0, 0), (-sa, 0, ca, 0), (0, 0, 0, 1)])
    return T


def rotz(a):
    """
    Rotation about z axis
    """
    ca = np.cos(a)
    sa = np.sin(a)
    T = np.array([(ca, -sa, 0, 0), (sa, ca, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)])
    return T

def rotz_lh(theta):
    # Rotation um z im linkshändigen System: Vorzeichen invertieren
    c, s = np.cos(-theta), np.sin(-theta)
    return np.array([(c, -s, 0, 0),(s,  c, 0, 0), (0,  0, 1, 0), (0, 0, 0, 1)])

def transl(x, y, z):
    """
    Translation about x,y,z
    """
    T = np.array([(1, 0, 0, x), (0, 1, 0, y), (0, 0, 1, z), (0, 0, 0, 1)])
    return T


def Tinv(T):
    """
    Inverse of homogeneous trafo matrix
    """
    R = T[0:3, 0:3]
    Ri = R.transpose()
    Ti = np.eye(4)
    Ti[0:3, 0:3] = Ri
    #Ti[0:3, 3:4] = -(Ri.dot(T[0:3, 3:4]))
    Ti[0:3, 3:4] = -(Ri @ T[0:3, 3:4])
    return Ti


def T_2_rotvec(T):
    """
    homogeneous trafo matrix to rotation vector representation
    """
    x, y, z = T[0:3, 3] # Verschiebungsvektor
    R = T[0:4, 0:4]     # Rotationsmatrix
    theta = np.arccos((R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2) + 1e-6
    K = 1/(2*np.sin(theta))*np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    pose = np.array([x, y, z, theta*K[0], theta*K[1], theta*K[2]])
    return pose


def rotvec_2_T(xyzrxryrz):
    """
    pose with rotation vector representation to homogeneous trafo matrix
    """
    T = np.zeros((4, 4))
    x, y, z, rx, ry, rz = xyzrxryrz
    T[0:3, 3] = x, y, z     # Verschiebung in Transformationsmatrix
    T[3, 3] = 1     # 1 unter Verschiebungsvektor -> homogene Koordinaten
    theta = np.sqrt(xyzrxryrz[3]**2 + xyzrxryrz[4]**2 + xyzrxryrz[5]**2)    # theta = sqrt(rx**2,ry**2,rz**2) (Folie 11)
    kx = rx/theta
    ky = ry/theta
    kz = rz/theta
    c = np.cos(theta)
    s = np.sin(theta)
    v = 1-c
    R = np.array([[kx*kx*v+c,       kx*ky*v-kz*s,       kx*kz*v+ky*s],
                  [kx*ky*v+kz*s,    ky*ky*v+c,          ky*kz*v-kx*s],
                  [kx*kz*v-ky*s,    ky*kz*v+kx*s,       kz*kz*v+c   ]])    # Rotationsmatrix
    T[0:3, 0:3] = R  # Rotation in Transformationsmatrix
    # T[3,0:3] = 0    # 0 ist schon unter Rotationsmatrix wegen np.zeros()
    return T

def T_2_rpy(T):
    """
    homogeneous trafo matrix to pose with roll-pitch-yaw x,y,z,r,p,y
    """
    # Rotation matrix
    R = T[0:3, 0:3]
    # Translations
    xt = T[0,3]
    yt = T[1,3]
    zt = T[2,3]
    
    # calculate angles
    p = np.arctan2(-R[2,0], np.sqrt(R[0,0]**2 + R[1,0]**2))
    cp = np.cos(p)
    y = np.arctan2(R[1,0]/cp, R[0,0]/cp)
    r = np.arctan2(R[2,1]/cp, R[2,2]/cp)
    
    # build vector
    pose = np.array((xt,yt,zt,r,p,y))
    # todo
    return pose


def rpy_2_T(pose):
    """
    pose with roll-pitch-yaw to homogeneous trafo matrix
    """
    T = np.zeros((4, 4))
    xt = pose[0]
    yt = pose[1]
    zt = pose[2]
    r = pose[3]
    p = pose[4]
    y = pose[5]
    
    cr=np.cos(r)
    cp=np.cos(p)
    cy=np.cos(y)
    sr = np.sin(r)
    sp = np.sin(p)
    sy = np.sin(y)
    
    # calculate rotation matrix
    R = np.array([(cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
                  (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
                  (-sp,   cp*sr,          cp*cr         )])
    
    # build homogeneous trafo matrix
    T[0:3, 0:3] = R
    T[0:4,3] = np.array([xt,yt,zt,1])
    
    return T


def dh(alpha, a, d, theta):
    """
    Denavit-Hartenberg (classic)
    """
    T = np.zeros(4)
    ct=np.cos(theta)
    st=np.sin(theta)
    sa=np.sin(alpha)
    ca=np.cos(alpha)
    T = np.array([[ct,   -st*ca,     st*sa, a*ct ], #
                  [st,   ct*ca,     -ct*sa,a*st ],
                  [0,     sa,              ca    ,      d  ],
                  [0,       0                   ,0         , 1]])
    
    return T


def dhm(alpha, a, d, theta):
    """
    Denavit-Hartenberg (modified)
    """
    
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    ct = np.cos(theta)
    st = np.sin(theta)
    
    T = np.array([(ct,    -st,   0,   a    ),
                  (st*ca, ct*ca, -sa, -d*sa),
                  (st*sa, ct*sa, ca,  d*ca ),
                  (0,     0,     0,   1    )])
    return T


def fk_ur(dh_para, q):
    """
    Forward Kinematics for UR type robots
    """
    
    T_0_6 = np.eye(4)
    i = 0
    
    for row in dh_para:
        alpha, a, d = row[0:3]
        T_0_6 = T_0_6 @ dh(alpha, a, d, q[i])
        i+=1
            
    return T_0_6


def fk_RoArm(alpha,a,d,theta):
    """
    Forward Kinematics for UR type robots
    """
    T_0_5 = np.eye(4)
    for i in range(len(alpha)):
        T_0_5=T_0_5@dh(alpha[i],a[i],d[i],theta[i])
    return T_0_5



def inv_RoArm(alpha,a,d, pose, qGripper):
    """ 
    #Backwards Kinematics for RoArm-M1
    """
    # DH-parameter in geeigente Form bringen
    # Pose in Matrix umwandeln
    T05 = rotvec_2_T(pose)
    # Rotation matrix des TCP
    R = T05[0:3, 0:3]
    # Translation des TCP
    P05 = T05[:4, 3]
    
    # Koordinatentransformation
    theta = -np.sign(R[2,0])*np.arccos(R[2,2])
    q1 = np.arctan2(R[1,0], R[0,0]) + np.pi
    Rz = rotz_lh(q1-np.pi)  
    P04 = T05 @ np.array([a[3], 0, -d[4], 1])
    P04_bo = Rz @ P04  
    P02 = np.array([-a[0], 0, d[0], 1])
    P24 = P04_bo - P02  
    x = P24[0]  
    z = P24[2]  
    
    # Restliche Gelenkwinkel berechnen
    if((x**2 + z**2) <= (-a[1]-a[2])):
        # Erste Lösung:
        cosq31 = (x**2 + z**2 - a[1]**2 - a[2]**2)/(2*a[1]*a[2])
        q31 = np.arccos(cosq31) 
       
        if(q31 < 0):  
            # Hilfswinkel berrechen
            psi = np.arccos((x**2 + z**2 + a[1]**2 - a[2]**2)/(2*a[1]*np.sqrt(x**2 + z**2)))
            beta = np.arctan2(z,x)
            
            q21 = np.pi/2 - (beta - psi); 
            q41 = theta - q31 - q21 + np.pi/2 
            # Zweite Lösung:
            q32 = -q31;
            q22 = np.pi/2 - (beta + psi); 
            q42 = theta - q32 - q22 + np.pi/2
        else:
            # Erste Lösung:
            # Hilfswinkel berrechen
            psi = np.arccos((x**2 + z**2  +a[1]**2 - a[2]**2)/(2*-a[1]*np.sqrt(x**2 + z**2)))
            beta = np.arctan2(z,x)
    
            q21 = np.pi/2 - (beta + psi); 
            q41 = theta - q31 - q21 + np.pi/2 
            # Zweite Lösung:
            q32 = -q31;
            q22 = np.pi/2 - (beta - psi); 
            q42 = theta - q32 - q22 + np.pi/2 
    else:
        print("Nicht lösbar")
        
    Q = np.array([[q1, q21, q31, q41, qGripper], [q1, q22, q32, q42, qGripper]])  
    return Q
  
"""
dhRoArm = np.array([[0.0,          0.0,      0.11574 ],
                    [-1.570796327, 0.04105,  0.0     ],
                    [0.0,          0.16886,  0.0     ],
                    [0.0,          0.12792,  -0.01399 ],
                    [1.570796327,  0.10854,  -0.01096 ]])
a=np.array([-41.05,-168.86,-127.92,-108.54,0])
d=np.array([115.74,0,-13.99,0,-10.96])
"""


def inv_ur(dh_para, pose):
    """
    Backwards Kinematics for UR type robots
    pose = xyzrxryrz
    theta_ber = (shoulder pan(q1), shoulder lift (q2), elbow (q3), 
                 wrist 1 (q4), wrist 2 (q5), and wrist 3 (q6))
    """
    # DH Parameter
    a = dh_para.transpose()[1]
    d = dh_para.transpose()[2]
    alpha = dh_para.transpose()[0]

    x, y, z, rx, ry, rz = pose

    # Koordinaten von Punkt O im Basiskoordinatensystem ausrechnen
    T06 = rotvec_2_T([x, y, z, rx, ry, rz])
    P65 = np.array([0, 0, -d[5], 1]).transpose()
    P05 = T06 @ P65

    # theta 1 (2 Möglichkeiten)
    theta1 = np.zeros(8)
    psi1 = np.arctan2(P05[1], P05[0])
    phi1 = np.arccos(d[3] / np.sqrt(P05[0] ** 2 + P05[1] ** 2))
    theta1[:4] = psi1 + phi1 + np.pi / 2
    theta1[4:] = psi1 - phi1 + np.pi / 2

    # theta 5 (2 Möglichkeiten)
    theta5 = np.zeros(8)
    P16z = [x * np.sin(theta1[0]) - y * np.cos(theta1[0]), x * np.sin(theta1[4]) - y * np.cos(theta1[4])]
    theta5[0:2] = np.arccos((P16z[0] - d[3]) / d[5])
    theta5[2:4] = -np.arccos((P16z[0] - d[3]) / d[5])
    theta5[4:6] = np.arccos((P16z[1] - d[3]) / d[5])
    theta5[6:] = -np.arccos((P16z[1] - d[3]) / d[5])

    # theta 6 (1 Möglichkeit)
    theta6 = np.zeros(8)
    # theta 3 (2 Möglichkeiten)
    theta3 = np.zeros(8)
    # theta 2 (1 Möglichkeit)
    theta2 = np.zeros(8)
    # theta 4 (1 Möglichkeit)
    theta4 = np.zeros(8)
    for i in range(8):
        T01 = dh(alpha[0], a[0], d[0], theta1[i])
        T16 = np.linalg.inv(T01) @ T06
        T61 = np.linalg.inv(T06) @ T01
        (zx61, zy61) = (T61[0, 2], T61[1, 2])
        if zx61 == 0 or zy61 == 0 or np.sin(theta5[0]) == 0:  # Index!
            print("Singularity: theta6 not well defined!")
        theta6[i] = np.arctan2(-zy61 / np.sin(theta5[i]), zx61 / np.sin(theta5[i]))

        T45 = dh(alpha[4], a[4], d[4], theta5[i])
        T56 = dh(alpha[5], a[5], d[5], theta6[i])
        T14 = T16 @ np.linalg.inv(T45 @ T56)
        P13 = (T14 @ np.array([[0, -d[3], 0, 1]]).transpose())[0:3]
        theta3[i] = (-1) ** i * np.arccos((np.linalg.norm(P13) ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2]))
        theta2[i] = -np.arctan2(P13[1][0], -P13[0][0]) + np.arcsin(a[2] * np.sin(theta3[i]) / np.linalg.norm(P13))

        T12 = dh(alpha[1], a[1], d[1], theta2[i])
        T23 = dh(alpha[2], a[2], d[2], theta3[i])
        T34 = np.linalg.inv(T12 @ T23) @ T14
        theta4[i] = np.arctan2(T34[1, 0], T34[0, 0])

    theta_ber = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
    return theta_ber


def closest_q(q_curr, q_new):
    # Vergleicht die momentane Gelenkposition mit den von inv_ur berechneten 8 Möglichkeiten
    # Gibt den Index der nächsten Gelenkposition zurück
    distance = np.zeros(8)
    for i in range(8):
        distance[i] = np.linalg.norm(q_curr - q_new.transpose()[i])
    return np.argmin(distance)


def getJacobi(dh_para, q):
    """
    Berechnung der Jacobimatrix
    """
    T_0_6 = fk_ur(dh_para, q)
    p = T_0_6[0:3, 3]

    J = np.zeros((6, 6))
    T_0_i = np.identity(4)

    for i in range(6):
        if i > 0:       # erster Durchgang: Transformation von 0 nach 0 ist die Einheitsmatrix
            alpha, a, d, q_off = dh_para[i - 1]
            theta = q[i - 1]
            T = dh(alpha, a, d, theta)
            T_0_i = np.dot(T_0_i, T)

        z_i = T_0_i[0:3, 2]
        p_i = T_0_i[0:3, 3]
        r = p - p_i
        J[0:3, i] = np.cross(z_i, r)
        J[3:6, i] = z_i
    return J


def pose_dot_from_q_dot(dh_para, q, q_dot):
    """
    Berechnung der TCP-Geschwindigkeit aus den Gelenkwinkelgeschwindigkeit
    """
    return getJacobi(dh_para, q) @ q_dot


def q_dot_from_pose_dot(dh_para, q, pose_dot):
    """
    Berechnung der Gelenkwinkelgeschwindigkeit aus den TCP-Geschwindigkeit
    """
    return np.linalg.pinv(getJacobi(dh_para, q)) @ pose_dot


#--- Testbereich ---
alpha=np.array([+np.pi/2,0,0,-np.pi/2,0])
a=np.array([-0.04105,-0.16886,-0.12792,-0.10854,0])
d=np.array([0.11574,0,-0.01399,0,-0.01096])

qTest = np.array([180, 40, 90, -60 ,180])*np.pi/180.0 + qRoArm
qRoArm = np.array([0, -1.570796327, 0.0, 0.0, 0.0])

T_0_5 = fk_RoArm(alpha,a,d, qTest)
print("Rotationsmatrix")
print(T_0_5)
pose = T_2_rotvec(T_0_5)
# Verschiebung der pose in Weltkoordinaten, bei gleicher Orientierung
pose_new = T_2_rotvec(transl(0,0,-0.005 ) @ T_0_5)
print("Alte Pose:")
print(pose)
print("Neue Pose:")
print(pose_new)


# inverse kinematic RoArm
q = pose_new
#q = pose
solution = inv_RoArm(alpha,a,d, q, np.pi)
print("GelenkWinkel:")
print(solution[0]*(180/np.pi))
print(solution[1]*(180/np.pi))
