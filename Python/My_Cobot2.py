# Example 
from pymycobot import MyCobot280 
import robolib_V2 as robolib
import numpy as np
 
#mc = MyCobot280('COM3', 115200) 
a = [0.00000, -110.4, -96, 0.00000, 0.00000, 0.0000]
d = [131.22, 0.00000, 0.00000, 63.4, 75.05, 45.6]
alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]
q_home_offset = [0, -1.570796327, 0, -1.570796327, 1.570796327, 0]
joint_direction = [1, 1, -1, 1, 1, 1]
mc_dh = np.array([alpha, a, d, q_home_offset]).T
#print(mc.get_system_version()) 
 
# Gets the current angle of all joints 
#angles = mc.get_angles() 
#print(angles) 
pos_1=np.array([-70,-165, 250, 2.83752738, -1.0699439,
   -0.46587679])
angles_inverse_1=robolib.inv_ur(mc_dh,pos_1)
print(angles_inverse_1)
 
#
# 
# mc.sync_send_angles(ang,20) 

