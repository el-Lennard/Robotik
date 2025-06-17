import numpy as np
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
    R = T[0:3, 0:3]
    x, y, z = T[0:3, 3]
    theta=np.arccos((R[0][0]+R[1][1]+R[2][2]-1)/(2))
    k=1/(2*np.sin(theta))*np.array([R[2][1]-R[1][2],R[0][2]-R[2][0],R[1][0]-R[0][1]])
    pose=np.array([x,y,z,k[0]*theta,k[1]*theta,k[2]*theta])
    return pose


def rotvec_2_T(xyzrxryrz):
    """
    pose with rotation vector representation to homogeneous trafo matrix
    """
    T = np.eye(4)
    x, y, z, rx, ry, rz = xyzrxryrz
    theta=np.sqrt(rx**2+ry**2+rz**2)
    kx=rx/theta
    ky=ry/theta
    kz=rz/theta
    c = np.cos(theta)
    s = np.sin(theta)
    v = 1-c
    R = np.array([[kx*kx*v+c,       kx*ky*v-kz*s,       kx*kz*v+ky*s],
                  [kx*ky*v+kz*s,    ky*ky*v+c,          ky*kz*v-kx*s],
                  [kx*kz*v-ky*s,    ky*kz*v+kx*s,       kz*kz*v+c   ]])
    T[0:3,0:3]=R
    T[0:3,3]=x,y,z
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
    T = np.array([[ct,   -st*ca,     st*sa, a*ct ],
                  [st,   ct*ca,     -ct*sa,a*st ],
                  [0,     sa,              ca    ,      d  ],
                  [0,       0                   ,0         , 1]])
    
    return T

def fk_ur(alpha,a,d,theta):
    """
    Forward Kinematics for UR type robots
    """
    T_0_5 = np.eye(4)
    for i in range(len(alpha)-1):
        T_0_5=T_0_5@dh(alpha[i],a[i],d[i],theta[i])
    return T_0_5

def inv_RoArm(dh_para, pose, qGripper):
    """ 
    #Backwards Kinematics for RoArm-M1
    """
    
    # DH-parameter in geeigente Form bringen
    a = dh_para[:,1]
    d = dh_para[:,2]
    alpha = dh_para[:,0]
    
    # Pose in Matrix umwandeln
    T05 = rotvec_2_T(pose)
    # Rotation matrix des TCP
    R = T05[0:3, 0:3]
    # Translation des TCP
    P05 = T05[:4, 3]
    
    # Winkel zwischen zBasis und zTCP
    theta = -np.sign(R[2,0])*np.arccos(R[2,2])
    
    #  Basisrotation q1 bestimmen, sodass Arm auf x-Basis liegt
    # --> Berechnen des Winkels Zwischen xBasis und xTCP, projeziert auf die XY-Ebene
    q1 = np.arctan2(R[1,0], R[0,0]) + np.pi
    
    # Stellung in die XZ-Achse des Basiskoordinatensystems transformeieren,
    # um die Gelenkwinkel für einen 2D 3-Achs Problem zu lösen
    # Rückdrehung auf Basis-X-Achse
    Rz = rotz_lh(q1-np.pi)  
    # Lage von Gelenk 4:
    P04 = T05 @ np.array([-a[4], 0, -d[4], 1])
    # In Basisorientierung drehen:
    P04_bo = Rz @ P04
    # Verschiebe Ursprung von Basis nach Gelenk 2
    P02 = np.array([a[1], 0, d[0], 1])  # Gelenk 2 liegt auf x-Achse um a[1] verschoben und um d[0] angehoben
    P24 = P04_bo - P02  # Punkt von Gelenk 4 relativ zu Gelenk 2 im Basis-x-ausgerichteten System
    
    # Ergebnis:
    x = P24[0]  # Projektion entlang Basis-X
    z = P24[2]  # Höhe
    
    
    if((x**2 + z**2) <= (a[2]+a[3])):
        # Erste Lösung:
        cosq31 = (x**2 + z**2 - a[2]**2 - a[3]**2)/(2*a[2]*a[3])
        q31 = np.arccos(cosq31) 
        
        if(q31 < 0):  
            # Hilfswinkel berrechen
            psi = np.arccos((x**2 + z**2 + a[2]**2 - a[3]**2)/(2*a[2]*np.sqrt(x**2 + z**2)))
            beta = np.arctan2(z,x)
            
            q21 = np.pi/2 - (beta - psi); #Drehrichtung und Homeposition beachten
            q41 = theta - q31 - q21 + np.pi/2 #Homeposition beachten
            # Zweite Lösung:
            q32 = -q31;
            q22 = np.pi/2 - (beta + psi); #Drehrichtung und Homeposition beachten
            q42 = theta - q32 - q22 + np.pi/2 #Homeposition beachten
        else:
            # Erste Lösung:
            # Hilfswinkel berrechen
            psi = np.arccos((x**2 + z**2 + a[2]**2 - a[3]**2)/(2*a[2]*np.sqrt(x**2 + z**2)))
            beta = np.arctan2(z,x)
            
            q21 = np.pi/2 - (beta + psi); #Drehrichtung und Homeposition beachten
            q41 = theta - q31 - q21 + np.pi/2 #Homeposition beachten
            # Zweite Lösung:
            q32 = -q31;
            q22 = np.pi/2 - (beta - psi); #Drehrichtung und Homeposition beachten
            q42 = theta - q32 - q22 + np.pi/2 #Homeposition beachten
    else:
        print()
        print("Nicht lösbar")
        
    Q = np.array([[q1, q21, q31, q41, qGripper], [q1, q22, q32, q42, qGripper]])  
    return Q






   


if __name__ == "__main__":
    alpha=np.array([np.pi/2,0,0,0])
    a=np.array([0,0.16886,0.12792,0.10854])
    d=np.array([0.05624,0,0,0])
    theta=np.array([180,30,90,-60])*(np.pi/180)
    T=fk_ur(alpha,a,d,theta)
    #print(T)
    #print(T_2_rotvec(T))