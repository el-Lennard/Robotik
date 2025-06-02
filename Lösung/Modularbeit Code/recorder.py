import socket
import pickle
import numpy as np


HOST = "10.27.200.128"      # robot IP
Rx_PORT = 30013


s_Rx = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_Rx.connect((HOST, Rx_PORT))

t = 15  # seconds
data_array = []
amount_of_points = int(np.ceil(t * 125))  # 125 Hz
for _ in range(amount_of_points):
    data_array.append(s_Rx.recv(1116))      # Lab: 1116! Sim: 1220!

filename = "recording_xyz.pkl"
pickle.dump(data_array, open(filename, "wb"))

s_Rx.close()
