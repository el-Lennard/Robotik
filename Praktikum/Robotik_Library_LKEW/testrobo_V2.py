
import robolib_EW as robolib
import numpy as np

# values from UR-Sim
a = [0.00000, -0.24365, -0.21325, 0.00000, 0.00000, 0.0000]
d = [0.1519, 0.00000, 0.00000, 0.11235, 0.08535, 0.0819]
alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]
q_home_offset = [0, -1.570796327, 0, -1.570796327, 0, 0]

ur3 = np.array([alpha, a, d, q_home_offset]).transpose()
# print(ur3)

q = np.array([0., 0, 0, 0, 0, 0]) / 180 * np.pi

T = robolib.fk_ur(ur3, q)
# print(T)
# print(robolib.T_2_rotvec(T))

q = np.array([10, -20, 30, -40, 50, -60]) / 180 * np.pi

T = robolib.fk_ur(ur3, q)
# print(T)
# print(robolib.T_2_rotvec(T))

# coordinates of searched TCP in [m] and [rad]
xyzrxryrz = np.array([0.05179, -0.17697, 0.4278, 0.688, -2.721, 1.673])

print(robolib.inv_ur(ur3, xyzrxryrz))