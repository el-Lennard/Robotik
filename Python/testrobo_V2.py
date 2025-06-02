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
#print(ur3)
# Given joint angle data (6-DOF for 8 configurations)
data = [
    [16.3886, -115.8649, 93.2719, 121.494, 28.3967, 72.701],
    [16.3886, -142.5334, 85.2745, -23.8401, -28.3967, -107.299],
    [16.3886, -30.6524, -93.2719, -137.1748, 28.3967, 72.701],
    [16.3886, -64.2703, -85.2745, 68.4458, -28.3967, -107.299],
    [10.0, -120.9326, 100.0, 130.9326, 30.0, 60.0],
    [10.0, -140.2765, 78.5241, -8.2476, -30.0, -120.0],
    [10.0, -30.0, -100.0, -120.0, 30.0, 60.0],
    [10.0, -67.9783, -78.5241, 76.5024, -30.0, -120.0]
]

# Convert to a NumPy array (shape: 8x6)
joint_angles = np.array(data, dtype=np.float64)
#q = np.array([35, -60, 50,-100 , -100, 30]) / 180 * np.pi
q = np.array([-41.98,-25.35,37.52,-101.5,-86.06,200.12]) / 180 * np.pi
tcp = np.array([-0.36993,0.06035,0.35752,2.449,2.097,0.821])
T_0_6 = robolib.fk_ur(dh, q)
#print(robolib.T_2_rpy(T_0_6))
#print(T_0_6)
angles=robolib.inv_ur(dh, tcp)
print(angles)
#print(joint_angles)




