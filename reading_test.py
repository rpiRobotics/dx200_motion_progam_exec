from MotomanEthernet import MotomanConnector
import time
import numpy as np

mh=MotomanConnector(IP=ip, PORT=80, S_pulse = 1341.4, L_pulse = 1341.4, U_pulse = 1341.4, R_pulse = 1000, B_pulse = 1000, T_pulse = 622)
mh.connectMH() #Connect to Controller
last_reading=np.zeros(6)
while True:
	try:
		time_now=time.time()
		new_reading=mh.getJointAnglesMH()
		if np.linalg.norm(last_reading-new_reading)>0:
			print('freq: ',1/(time_now-time.time()))
	except:
		mh.disconnectMH()
		break
