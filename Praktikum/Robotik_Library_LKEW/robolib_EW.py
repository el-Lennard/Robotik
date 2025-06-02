import numpy as np


def rotx(a):
    """
    Rotation about x axis
    script 2 page 3
    """
    ca = np.cos(a)
    sa = np.sin(a)
    T = np.array([(1, 0, 0, 0), (0, ca, -sa, 0), (0, sa, ca, 0), (0, 0, 0, 1)])
    return T


def roty(a):
    """
    Rotation about y axis
    script 2 page 3
    """
    ca = np.cos(a)
    sa = np.sin(a)
    T = np.array([(ca, 0, sa, 0), (0, 1, 0, 0), (-sa, 0, ca, 0), (0, 0, 0, 1)])
    return T


def rotz(a):
    """
    Rotation about z axis
    script 2 page 2
    """
    ca = np.cos(a)
    sa = np.sin(a)
    T = np.array([(ca, -sa, 0, 0), (sa, ca, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)])
    return T


def transl(x, y, z):
    """
    Translation about x,y,z
    script 2 page 2
    """
    T = np.array([(1, 0, 0, x), (0, 1, 0, y), (0, 0, 1, z), (0, 0, 0, 1)])
    return T


def Tinv(T):
    """
    Inverse of homogeneous trafo matrix 
    (Only possible when matrix is quadratic and the determinante is not zero)
    A * A^(-1) = I
    """
    R = T[0:3, 0:3]
    Ri = R.transpose()  # inverse of rotation matrix
    Ti = np.eye(4)      # identity matrix   
    Ti[0:3, 0:3] = Ri
    #Ti[0:3, 3:4] = -(Ri.dot(T[0:3, 3:4]))
    Ti[0:3, 3:4] = -(Ri @ T[0:3, 3:4])
    return Ti


def T_2_rotvec(T):
    """
    homogeneous trafo matrix to rotation vector representation
    script 1 page 13
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
    script 1 page 11
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
    script 1 page 18
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
    script 1 page 17
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


def dh(alpha, a, d, theta):
    """
    Denavit-Hartenberg (classic) transforms parameters always ...
    ... from one system to the next system
    script 2 page 16
    a are offsets in [m]
    d are offsets in [m]
    alpha are angles in [rad]
    theta are angles in [rad]
    """
    T = np.zeros(4)
    ct=np.cos(theta)
    st=np.sin(theta)
    sa=np.sin(alpha)
    ca=np.cos(alpha)
    T = np.array([[ct,      -st*ca,     st*sa,      a*ct ],
                  [st,      ct*ca,      -ct*sa,     a*st ],
                  [0,       sa,         ca,         d  ],
                  [0,       0,          0,          1]])
    
    return T


def dhm(alpha, a, d, theta):
    """
    Denavit-Hartenberg (modified)
    script 2 page 22
    """
    T = np.zeros(4)
    ct=np.cos(theta)
    st=np.sin(theta)
    sa=np.sin(alpha)
    ca=np.cos(alpha)
    T[0:4,0:4] = np.array([[ct,         -st,        0,      a ],
                           [st*ca,      ct*ca,      -sa,    -d*sa ],
                           [st*sa,      ct*sa,      ca,     d*ca  ],
                           [0,          0,          0,      1]])
    return T


def fk_ur(dh_para, q):
    """
    Forward Kinematics for UR type robots
    Always transforms from one system to the next system
    returns the transformation matrix from System 6 to System 0
    from the transformation matrix, the rotvec can be calculated using T_2_rotvec
    T_2_rotvec then gives the coordinates of the TCP in System 0 in [m] and [rad]
    """
    T_0_6 = np.eye(4)
    i=0
    for line in dh_para:
        T_0_6=T_0_6@dh(line[0],line[1],line[2],q[i])
        i=i+1
    # todo
    return T_0_6

def inv_ur(dh_para, xyzrxryrz):
    """
    Inverse Kinematics for UR type robots
    """
    # DH Parameter, robot geometry
    a = dh_para.transpose()[1]      # transposed matrices needed for calculations
    d = dh_para.transpose()[2]
    alpha = dh_para.transpose()[0]

    # coordinates of searched TCP in System 0 in [m] and [rad]
    x, y, z, rx, ry, rz = xyzrxryrz     

    # Calculation of origin of System 5 in System 0
    #script 3.1 page 23
    T06 = rotvec_2_T([x, y, z, rx, ry, rz])
    P65 = np.array([0, 0, -d[5], 1]).transpose()
    P05 = T06 @ P65

    # theta 1 (2 options) 
    # script 3.1 page 25
    theta1 = np.zeros(8)
    alpha1 = np.arctan2(P05[1], P05[0])       
    alpha2 = np.arccos(d[3] / np.sqrt(P05[0] ** 2 + P05[1] ** 2))
    theta1[:4] = alpha1 + alpha2 + np.pi / 2        # maybe change offset with pi according to DH-parameters
    theta1[4:] = alpha1 - alpha2 + np.pi / 2

    # theta 5 (2 options)
    # script 3.1 page 25
    theta5 = np.zeros(8)
    help0 = [x * np.sin(theta1[0]) - y * np.cos(theta1[0]), x * np.sin(theta1[4]) - y * np.cos(theta1[4])]
    theta5[0:2] = np.arccos((help0[0] - d[3]) / d[5])   # d[3] = l4 of script
    theta5[2:4] = -np.arccos((help0[0] - d[3]) / d[5])  # d[5] = l6 of script
    theta5[4:6] = np.arccos((help0[1] - d[3]) / d[5])
    theta5[6:] = -np.arccos((help0[1] - d[3]) / d[5])

    # theta 6 (1 option)
    theta6 = np.zeros(8)
    # theta 3 (2 options)
    theta3 = np.zeros(8)
    # theta 2 (1 option)
    theta2 = np.zeros(8)
    # theta 4 (1 option)
    theta4 = np.zeros(8)
    
    for i in range(8):
        T01 = dh(alpha[0], a[0], d[0], theta1[i])   # transform from being in System 1 to 0, dependent of base angle (theta1)
        T16 = np.linalg.inv(T01) @ T06      # inverse(T01) = T10
        
        # script 3.1 page 26
        help1 = -T06[0,1] * np.sin(theta1[i]) + T06[1,1] * np.cos(theta1[i])
        help2 = T06[0,0] * np.sin(theta1[i]) - T06[1,0] * np.cos(theta1[i])
        theta6[i] = np.arctan2(help1/np.sin(theta5[i]), help2/np.sin(theta5[i]))

        # script 3.1 page 9 - axes 2, 3 and 4 are parallel
        T45 = dh(alpha[4], a[4], d[4], theta5[i])   # dimensions of arm 4 and angle 5
        T56 = dh(alpha[5], a[5], d[5], theta6[i])   # dimensions of arm 5 and angle 6
        T14 = T16 @ np.linalg.inv(T45 @ T56)        # script 3.1 page 26, transform from 4 to 1 (4 is computable with theta5 and theta6)
        P13 = (T14 @ np.array([[0, -d[3], 0, 1]]).transpose())[0:3]     # position of P3 in System 1 (computable with length of arm between 4 and 3)
        
        # script 3.1 page 9
        theta3[i] = (-1) ** i * np.arccos((np.linalg.norm(P13) ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2])) 
        
        theta2[i] = -np.arctan2(P13[1][0], -P13[0][0]) + np.arcsin(a[2] * np.sin(theta3[i]) / np.linalg.norm(P13)) # maybe change definition for different robot

        T12 = dh(alpha[1], a[1], d[1], theta2[i])
        T23 = dh(alpha[2], a[2], d[2], theta3[i])
        T34 = np.linalg.inv(T12 @ T23) @ T14        # transform from 4 to 3
        theta4[i] = np.arctan2(T34[1, 0], T34[0, 0])

    theta_ber = (np.array([theta1, theta2, theta3, theta4, theta5, theta6])*180/np.pi).transpose()
    return theta_ber