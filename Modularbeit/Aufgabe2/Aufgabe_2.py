
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
    for i in range(len(alpha)):
        T_0_5=T_0_5@dh(alpha[i],a[i],d[i],theta[i])
    return T_0_5
if __name__ == "__main__":
    alpha=np.array([+np.pi/2,0,0,-np.pi/2,0])
    a=np.array([-41.05,-168.86,-127.92,-108.54,0])
    d=np.array([115.74,0,0,-13.99,-10.96])
    theta=np.array([180,30-90,90,10,180])*(np.pi/180) #Winkel2 -90 Grad Offset
    T=fk_ur(alpha,a,d,theta)
    print(T)
    print(T_2_rotvec(T))