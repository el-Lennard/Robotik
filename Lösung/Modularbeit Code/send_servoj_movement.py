import numpy as np
import time
import socket
import robolib

ur5 = np.array([[np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0],
                [0.00000, -0.42500, -0.39225, 0.00000, 0.00000, 0.0000],
                [0.089159, 0.00000, 0.00000, 0.10915, 0.09465, 0.0823],
                [0, -np.pi / 2, 0, -np.pi / 2, 0, 0]]).T

HOST = "10.27.200.128"  # robot IP
# HOST = '192.168.52.130'     # new VM

Tx_PORT = 30002

s_Tx = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_Tx.connect((HOST, Tx_PORT))

start_q = np.array([30, -60, 90, 0, 90, 0]) / 180 * np.pi
start_pose = robolib.T_2_rotvec(robolib.fk_ur(ur5, start_q))
stop_pose = start_pose + np.array([-0.05, 0, 0, 0, 0, 0])
v_max = 0.1
a_max = 0.6
t = 0.008

poses, *_ = robolib.get_traj(start_pose, stop_pose, a_max, v_max, 8e-3)
qs = robolib.ik_traj_ur(ur5, poses, start_q)

s_Tx.send(f"movej({start_q.tolist()}, a=0.6, v=0.6)\n".encode('ascii'))

time.sleep(5)

function_string = "def myFunc():\n"
for q in qs:
    function_string += f"\tservoj({q.tolist()}, a={a_max}, v={v_max}, t={t})\n"
function_string += "end\nmyFunc()"
s_Tx.send(function_string.encode('ascii'))
