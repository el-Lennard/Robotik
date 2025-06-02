import numpy as np
import socket

HOST = "10.181.111.132"     # robot IP
PORT = 30002                # port

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

command = "movej([0, 0, 0, 0, 0, 0])\n"

s.send(command.encode('ascii'))
