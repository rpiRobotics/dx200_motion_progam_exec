from robodk import *
import sys

from Motoman import RobotPost, Pose
from MotomanEthernet import MotomanConnector

class MotionProgramExecClient:
    def __init__(self, ip='192.168.55.1',ROBOT_CHOICE='RB1'):
        self.ip=ip
        self.mh=MotomanConnector(IP=ip, PORT=80, S_pulse = 1341.4, L_pulse = 1341.4, U_pulse = 1341.4, R_pulse = 1000, B_pulse = 1000, T_pulse = 622)
        self.mh.connectMH() #Connect to Controller
        self.robodk_rob=RobotPost(r"""Motoman""",r"""Motoman MA2010""",6, axes_type=['R','R','R','R','R','R'],  pulses_x_deg=[1341.4, 1341.4, 1341.4, 1000, 1000, 622],ROBOT_CHOICE=ROBOT_CHOICE)

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
    client=MotionProgramExecClient(ROBOT_CHOICE='RB2')

    ###TODO: fix tool definition
    # client.robodk_rob.DONT_USE_SETTOOL=False
    # client.robodk_rob.setTool(Pose([0,0,450,0,0,0]), None, 'welder')
    client.robodk_rob.ACTIVE_TOOL=1

    client.robodk_rob.ProgStart(r"""AAA""")
    client.robodk_rob.setFrame(Pose([0,0,0,0,0,0]),-1,r"""Motoman MA2010 Base""")
    client.robodk_rob.MoveJ(Pose([1232,0,860,0,0,-180]),[0,0,0,0,0,0],10,0)
    client.robodk_rob.MoveJ(Pose([1232,0,593.478,0,0,-180]),[0,-1.23097,-15.1604,0,13.9294,0],10,0)
    client.robodk_rob.MoveJ(Pose([903.741,0,593.478,0,0,-180]),[0,-27.1205,-36.7057,0,9.58517,0],10,0)
    client.robodk_rob.ProgFinish(r"""AAA""")
    client.robodk_rob.ProgSave(".","AAA",False)

    client.execute_motion_program("AAA.JBI")
    client.mh.disconnectMH()

if __name__ == "__main__":
    main()
