from robodk import *
import sys

from Motoman import RobotPost, Pose
from MotomanEthernet import MotomanConnector

class MotionProgramExecClient:
    def __init__(self, ip='192.168.55.1'):
        self.ip=ip
        self.mh=MotomanConnector(IP=ip, PORT=80, S_pulse = 1341.4, L_pulse = 1341.4, U_pulse = 1341.4, R_pulse = 1000, B_pulse = 1000, T_pulse = 622)
        self.mh.connectMH() #Connect to Controller
        self.robodk_rob=RobotPost(r"""Motoman""",r"""Motoman MA2010""",6, axes_type=['R','R','R','R','R','R'], ip_com=r"""127.0.0.1""", api_port=20500, prog_ptr=1976622395408, robot_ptr=1976635615216, pulses_x_deg=[1341.4, 1341.4, 1341.4, 1000, 1000, 622])

    def execute_motion_program(self, filename):
        self.robodk_rob.PROG_FILES=[]
        self.robodk_rob.PROG_FILES.append(filename)
        self.robodk_rob.ProgSendRobot(self.ip,'JOB',"ftp","")
        ###TODO: figure out return time
        self.mh.servoMH() #Turn Servo on

        self.mh.startJobMH('AAA')
        time.sleep(5)
        self.mh.servoMH(False) #Turn the Servos of


def main():
    client=MotionProgramExecClient()

    client.robodk_rob.STR_VJ='VJ=10.00'
    client.robodk_rob.ProgStart(r"""AAA""")
    client.robodk_rob.setFrame(Pose([0,0,0,0,0,0]),-1,r"""Motoman MA2010 Base""")
    client.robodk_rob.MoveJ(Pose([1232,0,860,0,0,-180]),[10,0,0,0,0,0],[0,0,0])
    client.robodk_rob.MoveJ(Pose([1232,0,593.478,0,0,-180]),[0,-1.23097,-15.1604,0,13.9294,0],[0,0,0])
    client.robodk_rob.MoveJ(Pose([903.741,0,593.478,0,0,-180]),[0,-27.1205,-36.7057,0,9.58517,0],[0,0,0])
    client.robodk_rob.ProgFinish(r"""AAA""")
    client.robodk_rob.ProgSave(".","AAA",False)

    file=open("AAA.JBI","r")
    file_new=open("AAA2.JBI","w")
    lines=file.readlines()
    for i in range(len(lines)):
        if 'RJ' in lines[i]:
            file_new.write(lines[i][:-4]+lines[i][-1:])
        else:
            file_new.write(lines[i])

    file.close()
    file_new.close()

    client.execute_motion_program("AAA2.JBI")

    self.mh.disconnectMH()

if __name__ == "__main__":
    main()
