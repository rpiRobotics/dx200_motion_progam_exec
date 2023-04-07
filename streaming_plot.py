import numpy as np
import socket
import struct
import time
import matplotlib.pyplot as plt

reading_conversion=np.array([1341.380023,1907.674052,1592.888923,1022.862207,980.2392898,454.754161,1435.350459,1300.317471,1422.222174,969.9555508,980.2392898,454.754161,1994.296925,1376.711214])

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0',11000))

joint_angle1=[]
joint_angle2=[]
timestamp=[]

while True:
    try:
        buf = s.recv(1024)
        data = struct.unpack("<34i",buf)
        timestamp.append(data[0])
        joint_angle1.append(data[2:8])
        # joint_angle2.append(data[20:26]*reading_conversion[:6]/10000)
        joint_angle2.append(data[20:26])
    except:
        break
joint_angle1=np.array(joint_angle1)
joint_angle2=np.array(joint_angle2)

plt.plot(joint_angle1[:,0],label='set1')
plt.plot(joint_angle2[:,0],label='set2')
plt.legend()
plt.show()