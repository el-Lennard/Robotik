import numpy as np
import socket
import time

HOST = "10.27.200.128"
# HOST = '192.168.52.130'     # new VM
Tx_PORT = 30002

s_Tx = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_Tx.connect((HOST, Tx_PORT))

a_max = 0.8
v_max = 0.6

q = np.array([30, -60, 90, 0, 90, 0]) / 180 * np.pi
s_Tx.send(f"movej({q.tolist()}, a={a_max}, v={v_max})\n".encode('ascii'))

time.sleep(5)

q = q + np.array([0, -40, -10, 0, 0, 0]) / 180 * np.pi
s_Tx.send(f"movej({q.tolist()}, a={a_max}, v={v_max})\n".encode('ascii'))

s_Tx.close()
