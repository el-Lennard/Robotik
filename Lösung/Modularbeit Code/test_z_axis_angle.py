import numpy as np
import robolib


def angle_z_axis_to_xy_plane_understandable(T):
    z_b = np.array([0, 0, 1])
    z_trans = T[:3, 2]
    angle = (np.pi/2 - np.arccos(z_trans@z_b/(np.linalg.norm(z_trans)*np.linalg.norm(z_b)))) / np.pi * 180
    return angle


def angle_z_axis_to_xy_plane(p):
    T = robolib.rotvec_2_T(p)
    return (np.pi/2 - np.arccos(p[2, 2])) / np.pi * 180


if __name__ == '__main__':

    ur5 = np.array([[np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0],
                    [0.00000, -0.42500, -0.39225, 0.00000, 0.00000, 0.0000],
                    [0.089159, 0.00000, 0.00000, 0.10915, 0.09465, 0.0823],
                    [0, -np.pi / 2, 0, -np.pi / 2, 0, 0]]).T

    test_angles = [
        np.array([0, 0, 0, -90, 0, 0])*np.pi/180,   # 0 deg
        np.array([0, 0, 0, -90, 45, 0])*np.pi/180,  # 45 deg
        np.array([0, 0, 0, -90, 90, 0])*np.pi/180,  # 90 deg
        np.array([0, 0, 0, -90, 135, 0])*np.pi/180,  # 135 deg -> 45 deg
        np.array([0, 0, 0, -90, 180, 0])*np.pi/180,   # 180 deg -> 0 deg
        np.array([0, 0, 0, -90, -90, 0])*np.pi/180,  # -90 deg
        np.array([0, 0, 0, -90, -45, 0])*np.pi/180,  # -45 deg
    ]

    # step through in debugger
    for test_angle in test_angles:
        print((np.pi/2 - np.arccos(robolib.fk_ur(ur5, test_angle)[2, 2]))*180/np.pi)    # oder arcsin
        # print(angle_z_axis_to_xy_plane_understandable(robolib.fk_ur(ur5, test_angle)))

    # Output:
    # 0.0
    # 45.0
    # 90.0
    # 45.0
    # 0.0
    # -90.0
    # -45.0
    # correct, as minimal angle is calculated -> 135 is 45 deg, 180 is 0 deg
