import robolib_V2 as robolib
import numpy as np

a = [0.00000, -0.24365, -0.21325, 0.00000, 0.00000, 0.0000]
d = [0.1519, 0.00000, 0.00000, 0.11235, 0.08535, 0.0819]
alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]
q_home_offset = [0, -1.570796327, 0, -1.570796327, 0, 0]
joint_direction = [1, 1, -1, 1, 1, 1]

ur3 = np.array([alpha, a, d, q_home_offset]).T
print(ur3)

q = np.array([-39.77, -122.81, -66.75, -45.66, -4.61, 82.96]) / 180 * np.pi
#q = np.array([10, -30, -100, -120, 30, 60]) / 180 * np.pi

T_0_6 = robolib.fk_ur(ur3, q)
print(T_0_6)
print(robolib.T_2_rotvec(T_0_6))