import numpy as np

robot_name='D500B'

pulse_deg_recording1=np.loadtxt(robot_name+'_pulse2deg_recording1.csv',delimiter=',')
pulse_deg_recording2=np.loadtxt(robot_name+'_pulse2deg_recording2.csv',delimiter=',')

pulse2deg1=pulse_deg_recording1[:,0]/pulse_deg_recording1[:,1]
pulse2deg2=pulse_deg_recording2[:,0]/pulse_deg_recording2[:,1]

pulse2deg=(pulse2deg1+pulse2deg2)/2
print(pulse2deg1)
print(pulse2deg2)

np.savetxt(robot_name+'_pulse2deg.csv',pulse2deg,delimiter=',')