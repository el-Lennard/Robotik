
import robolib_V1 as robolib
import numpy as np

T = robolib.roty(-30 / 180 * np.pi)
print(T)

Ti = robolib.Tinv(T)
print(Ti)

u = robolib.T_2_rotvec(T)
print(u)

u = np.array([1, 2, 3, 0.1, 0.2, 0.3])
Tu = robolib.rotvec_2_T(u)
print(Tu)

u2 = robolib.T_2_rotvec(Tu)
print(u2)

rpy = robolib.T_2_rpy(Tu)
print(rpy)

T2 = robolib.rpy_2_T(rpy)
print(T2)
