
import numpy as np


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


def T_2_rpy(T):
    """
    homogeneous trafo matrix to pose with roll-pitch-yaw x,y,z,r,p,y
    """
    pose = np.zeros((6))
    R = T[0:3, 0:3]
    
    beta = np.arctan2(-R[2, 1], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    cb = np.cos(beta)
    alpha = np.arctan2(R[1, 0]/cb, R[0, 0]/cb)
    gamma = np.arctan2(R[2, 1]/cb, R[2, 2]/cb)
    x, y, z = T[0:3, 3]
    pose[:6]=x,y,z,gamma,beta,alpha
    return pose


def rpy_2_T(xyzrpy):
    """
    pose with roll-pitch-yaw to homogeneous trafo matrix
    """
    T = np.eye(4)
    x, y, z, r,p, y = xyzrpy
    ca = np.cos(xyzrpy[5])
    sa = np.sin(xyzrpy[5])
    cb = np.cos(xyzrpy[4])
    sb = np.sin(xyzrpy[4])
    cg = np.cos(xyzrpy[3])
    sg = np.sin(xyzrpy[3])
    T[0:3, 3] = xyzrpy[0:3]     
    R = np.array([[ca*cb,   ca*sb*sg-sa*cg,     ca*sb*cg+sa*sg],
                  [sa*cb,   sa*sb*sg+ca*cg,     sa*sb*cg-ca*sg],
                  [-sb,     cb*sg,              cb*cg         ]])
    T[0:3,0:3]=R
    return T
 