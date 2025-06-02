import numpy as np
import socket

HOST = "10.0.2.15"  # Replace with your URSim or robot IP
PORT = 30002        # Correct port for sending URScript commands

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

q = np.array([10, -20, 30, -40, 50, -60]) / 180 * np.pi

command = "movej([{:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}], a=1.2, v=0.25)\n".format(*q)

s.send(command.encode('utf8'))
