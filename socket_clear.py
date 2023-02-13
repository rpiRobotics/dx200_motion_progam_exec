import numpy as np
import socket
import struct
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0',11000))

hz=[]
buf = s.recv(1024)
data = struct.unpack("<16i",buf)
