
import numpy as np

def rotx(a):
    """
    Rotation about x axis
    """
    ca = np.cos(a)
    sa = np.sin(a)
    T = np.array([(1,  0,   0, 0),
                  (0, ca, -sa, 0),
                  (0, sa,  ca, 0),
                  (0,  0,   0, 1)])
    return T


def roty(a):
    """
    Rotation about y axis
    """
    ca = np.cos(a)
    sa = np.sin(a)
    T = np.array([(ca,  0,  sa, 0),
                  (0,   1,  0,  0),
                  (-sa, 0,  ca, 0),
                  (0,   0,  0,  1)])
    return T


def rotz(a):
    """
    Rotation about z axis
    """
    ca = np.cos(a)
    sa = np.sin(a)
    T = np.array([(ca, -sa, 0, 0),
                  (sa, ca,  0, 0),
                  (0,   0,  1, 0),
                  (0,   0,  0, 1)])
    return T


def transl(x, y, z):
    """
    Translation about x,y,z
    """
    T = np.array([(1, 0, 0, x),
                  (0, 1, 0, y),
                  (0, 0, 1, z),
                  (0, 0, 0, 1)])
    return T


def Tinv(T):
    """
    Inverse of homogeneous trafo matrix
    """
    R = T[0:3, 0:3]
    Ri = R.transpose()
    Ti = np.eye(4)
    Ti[0:3, 0:3] = Ri
    Ti[0:3, 3:4] = -(Ri @ T[0:3, 3:4])
    return Ti


def T_2_rotvec(T):
    """
    homogeneous trafo matrix to rotation vector representation
    """
    x, y, z = T[0:3, 3] # Verschiebungsvektor
    R = T[0:4, 0:4]     # Rotationsmatrix
    theta = np.arccos((R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2)
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
    pose = np.zeros((6))
    R = T[0:3, 0:3]     # Rotationsmatrix (in Transformationsmatrix enthalten). Zwischenschritt zur Nachvollziehbarkeit
    beta = np.arctan2(-R[2, 1], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    cb = np.cos(beta)
    alpha = np.arctan2(R[1, 0]/cb, R[0, 0]/cb)
    gamma = np.arctan2(R[2, 1]/cb, R[2, 2]/cb)
    x, y, z = T[0:3, 3]   # Verschiebungsvektor
    pose[0:6] = x, y, z, gamma, beta, alpha
    return pose


def rpy_2_T(xyzrpy):
    """
    pose with roll-pitch-yaw to homogeneous trafo matrix
    3: roll = gamma
    4: pitch = beta
    5: yaw = alpha
    """
    T = np.zeros((4, 4))
    T[0:3, 3] = xyzrpy[0:3]     # x, y, z
    T[3, 3] = 1
    ca = np.cos(xyzrpy[5])
    sa = np.sin(xyzrpy[5])
    cb = np.cos(xyzrpy[4])
    sb = np.sin(xyzrpy[4])
    cg = np.cos(xyzrpy[3])
    sg = np.sin(xyzrpy[3])
    R = np.array([[ca*cb,   ca*sb*sg-sa*cg,     ca*sb*cg+sa*sg],
                  [sa*cb,   sa*sb*sg+ca*cg,     sa*sb*cg-ca*sg],
                  [-sb,     cb*sg,              cb*cg         ]])
    T[0:3, 0:3] = R
    return T


def dh(alpha, a, d, theta):
    """
    Denavit-Hartenberg (classic)
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    calph = np.cos(alpha)
    salph = np.sin(alpha)
    T = np.array([[ct,  -st*calph,  st*salph,   a*ct],
                  [st,  ct*calph,   -ct*salph,  a*st],
                  [0,   salph,      calph,      d],
                  [0,   0,          0,          1]])
    return T


def dhm(alpha, a, d, theta):
    """
    Denavit-Hartenberg (modified)
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    calph = np.cos(alpha)
    salph = np.sin(alpha)
    T = np.array([[ct,          -st,        0,      a],
                  [st*calph,    ct*calph,   -salph, -d*salph],
                  [st*salph,    ct*salph,   calph,  d*calph],
                  [0,           0,          0,      1]])
    return T


def fk_ur(dh_para, q):
    """
    Forward Kinematics for UR type robots
    """
    T_0_6 = np.eye(4)
    i = 0
    for lines in dh_para:
        T_0_6 = T_0_6 @ dh(lines[0], lines[1], lines[2], q[i])
        i = i + 1
    return T_0_6


def inv_ur(dh_para, xyzrxryrz):
    # DH Parameter
    a = dh_para.transpose()[1]
    d = dh_para.transpose()[2]
    alpha = dh_para.transpose()[0]

    x, y, z, rx, ry, rz = xyzrxryrz

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


def ik_ur(dh_para, q_curr, pose_new):
    qs = inv_ur(dh_para, pose_new)
    min_index = closest_q(q_curr, qs)
    return np.array(qs[:, min_index]).flatten()


def ik_traj_ur(dh_para, poses, start_q):
    qs = [start_q]
    for i in range(len(poses) - 1):
        qs.append(ik_ur(dh_para, qs[i], poses[i + 1]))
    return np.array(qs)


def get_trajec_params(q_now, q_target, a_max=1, v_max=0.5, t=0):
    """
    Calculation of the parameters for the trapeziodial or triangle trajectory.
    Parameters: q_now / pose_now, q_target / pose_target, number of axes, velocity, acceleration, time for movement
    """
    # get slowest axes -> determine leading axes
    if v_max > 1.05 or a_max > 1.4:  # deg/s, deg/s**2
        raise ValueError("Given acceleration and speed exceed safety limits")

    t_max = 0
    delta_q_max = 0
    switching_time = v_max / a_max
    max_switching_time = 0
    q_max = (v_max ** 2 / a_max)
    for i in range(0, len(q_now)):
        delta_q = np.absolute(q_target[i] - q_now[i])
        if delta_q > q_max:     # trapeziod
            t_sum_now = delta_q / v_max + switching_time
            if t_sum_now > t_max:
                t_max = t_sum_now
                delta_q_max = delta_q
                max_switching_time = switching_time
        else:   # triangle
            t_sum_now = 2 * np.sqrt(delta_q / a_max)
            if t_sum_now > t_max:
                t_max = t_sum_now
                delta_q_max = delta_q
                max_switching_time = 0.5 * t_sum_now

    # if total time is given
    # 25% of needed time is acceleration
    if t > 0:
        if t > t_max:
            t_max = t
            v_max = (16 * delta_q_max) / (12 * t_max)
            a_max = (16 * delta_q_max) / (12 * t_max ** 2)
        else:
            raise ValueError("The given total time is too short.")

    # slow down faster axes, adapt switching time
    params = np.zeros((len(q_now), 5))   # [t_sw1, t_sw2, delta_q, v_max, a_max]
    switching_time = max_switching_time
    for i in range(0, len(q_now)):
        delta_q = (q_target[i] - q_now[i])
        if (delta_q):
            params[i, 2] = delta_q
            if np.absolute(delta_q) == delta_q_max:
                params[i, 3] = v_max * np.sign(delta_q)
                params[i, 4] = a_max * np.sign(delta_q)
            else:
                params[i, 3] = delta_q / (np.absolute(t_max - switching_time))  # v
                params[i, 4] = delta_q / (np.absolute(t_max * switching_time - switching_time ** 2))  # a
            q_sw = (params[i, 3] ** 2 / params[i, 4])
            if np.absolute(delta_q) > np.absolute(q_sw):
                # trajec_trapeziod
                params[i, 0] = switching_time
                params[i, 1] = np.absolute(delta_q_max / v_max)
            else:
                # trajec_triangle
                params[i, 0] = np.sqrt(delta_q_max / a_max)
                params[i, 1] = 0
        else:
            pass
    return params, t_max


def get_trajec_q_at_t(params, t_max, t):
    """
    Calculation of a trapeziodial or triangle trajectory.
    Parmeters: parameter matrix, time
    """
    # params:   [t_sw1, t_sw2, delta_q, v_max, a_max]
    q_number = len(params)
    qn = np.zeros(q_number)
    qnd = np.zeros(q_number)
    qndd = np.zeros(q_number)
    for i in range(0, q_number):
        if params[i, 0]:
            if params[i, 1] == 0:  # trajec_triangle
                if t > t_max:
                    qn[i] = params[i, 2]
                    qnd[i] = 0
                    qndd[i] = 0
                else:
                    if t > params[i, 0]:
                        qn[i] = 0.5 * params[i, 2] + 2 * np.sign(params[i, 2]) * np.sqrt(params[i, 2] * params[i, 4]) * (
                                    t - params[i, 0]) - 0.5 * params[i, 4] * (t ** 2 - params[i, 0] ** 2)
                        qnd[i] = 2 * np.sign(params[i, 2]) * np.sqrt(params[i, 2] * params[i, 4]) - params[i, 4] * t
                        qndd[i] = -params[i, 4]
                    else:
                        if t >= 0:
                            qn[i] = 0.5 * params[i, 4] * (t ** 2)
                            qnd[i] = params[i, 4] * t
                            qndd[i] = params[i, 3]
                        else:
                            qn[i] = 0
                            qnd[i] = 0
                            qndd[i] = 0
            else:   # trajec_trapeziod
                if t > t_max:
                    qn[i] = params[i, 2]
                    qnd[i] = 0
                    qndd[i] = 0
                else:
                    if t > params[i, 1]:
                        qn[i] = params[i, 2] - 0.5 * (params[i, 3] ** 2 / params[i, 4]) + (
                                    params[i, 3] + params[i, 4] / params[i, 3] * params[i, 2]) * (t - params[i, 1]) - 0.5 * params[
                                    i, 4] * (t ** 2 - params[i, 1] ** 2)
                        qnd[i] = params[i, 3] + params[i, 4] / params[i, 3] * params[i, 2] - params[i, 4] * t
                        qndd[i] = -params[i, 4]
                    else:
                        if t > params[i, 0]:
                            qn[i] = 0.5 * (params[i, 3] ** 2 / params[i, 4]) + params[i, 3] * (t - params[i, 0])
                            qnd[i] = params[i, 3]
                            qndd[i] = 0
                        else:
                            if t >= 0:
                                qn[i] = 0.5 * params[i, 4] * (t ** 2)
                                qnd[i] = params[i, 4] * t
                                qndd[i] = params[i, 4]
                            else:
                                qn[i] = 0
                                qnd[i] = 0
                                qndd[i] = 0
    return np.array([qn, qnd, qndd]).T


def get_traj(q_now, q_target, a, v, t_step=8e-3):
    params, t_max = get_trajec_params(q_now, q_target, a_max=a, v_max=v)
    ts = np.arange(0, t_max, t_step)
    traj = np.array([get_trajec_q_at_t(params, t_max, t) for t in ts])
    qn = q_now + traj[:, :, 0]
    qnd = traj[:, :, 1]
    qndd = traj[:, :, 2]
    return qn, qnd, qndd


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
