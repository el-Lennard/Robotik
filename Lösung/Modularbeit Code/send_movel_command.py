import numpy as np
import socket
import time

HOST = "10.27.200.128"      # robot IP
# HOST = '192.168.52.130'     # new VM

Tx_PORT = 30002

s_Tx = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_Tx.connect((HOST, Tx_PORT))

start_q = np.array([30, -60, 90, 0, 90, 0]) / 180 * np.pi
# start_pose = robolib.T_2_rotvec(robolib.fk_ur(ur5, start_q))
delta_pose = np.array([-0.05, 0, 0, 0, 0, 0])
a_max = 0.8
v_max = 0.1

s_Tx.send(f"movej({start_q.tolist()}, a=0.6, v=0.6)\n".encode('ascii'))
time.sleep(5)
s_Tx.send(f"movel(pose_add(get_actual_tcp_pose(), p{delta_pose.tolist()}), a={a_max}, v={v_max})\n".encode('ascii'))

s_Tx.close()
