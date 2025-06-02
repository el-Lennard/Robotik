import robolib_V2 as robolib
import numpy as np

# values from UR-Sim
# /home/ur/ursim-current/.urcontrol/urcontrol.conf
a = [0, -0.24365, -0.21325, 0.00000, 0.00000, 0.0000]
d = [0.1519, 0.00000, 0.00000, 0.11235, 0.08535, 0.0819]
alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]
q_home_offset = [0, -1.570796327, 0, -1.570796327, 1.570796327, 0]
joint_direction = [1, 1, -1, 1, 1, 1]

dh = np.array([alpha, a, d, q_home_offset]).T
#q = np.array([35, -60, 50,-100 , -100, 30]) / 180 * np.pi
q = np.array([-41.98,-25.35,37.52,-101.5,-86.06,200.12]) / 180 * np.pi
tcp_1 = np.array([-0.3,0,0.2,2.182,2.182,0])
tcp_2 = np.array([-0.4,0,0.2,2.182,2.182,0])
tcp_3 = np.array([-0.4,-0.1,0.2,2.182,2.182,0])
tcp_4 = np.array([-0.3,-0.1,0.2,2.182,2.182,0])

T_0_6 = robolib.fk_ur(dh, q)
#print(robolib.T_2_rpy(T_0_6))
#print(T_0_6)
angles=robolib.inv_ur(dh, tcp_4)
print(angles)
#print(joint_angles)




