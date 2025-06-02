# Example 
from pymycobot import MyCobot280 
import robolib_V2 as robolib
import numpy as np
 
mc = MyCobot280('COM3', 115200) 
a = [0.00000, -110.4, -96, 0.00000, 0.00000, 0.0000]
d = [131.22, 0.00000, 0.00000, 63.4, 75.05, 45.6]
alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]
q_home_offset = [0, -1.570796327, 0, -1.570796327, 1.570796327, 0]
joint_direction = [1, 1, -1, 1, 1, 1]
mc_dh = np.array([alpha, a, d, q_home_offset]).T
#print(mc.get_system_version()) 
 
# Gets the current angle of all joints 
angles = mc.get_angles() 
print(angles) 
 
# Set 1 joint to move to 40 and speed to 20 
#mc.send_angle(4, 30, 20) 
 
#mc.sync_send_angles([0,0,0,0,0,0],20) 
 
#q = [60,-20,-30,-40,50,60] 
q_robo = [0,0,0,0,0,0] 
q_2robo=[50,0,70,10,165,180]
q_2robo_calc=np.array(q_2robo)/180*np.pi
mc.sync_send_angles(q_2robo, 20) 
T_0_6 = robolib.fk_ur(mc_dh, q_2robo_calc)#+ q_home_offset)
T_0_6_calc= robolib.fk_ur(mc_dh, q_2robo_calc+ q_home_offset)
angles = mc.get_angles() 
print(angles)
#print(T_0_6)
tcp_1=robolib.T_2_rotvec(T_0_6)
tcp_1_calc= robolib.T_2_rotvec(T_0_6_calc)
#print(robolib.inv_ur(mc_dh,tcp_1))
print(tcp_1_calc)
