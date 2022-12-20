from robodk import *
import sys, copy
import numpy as np

from Motoman import RobotPost, Pose
from MotomanEthernet import MotomanConnector
# Disable
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore
def enablePrint():
    sys.stdout = sys.__stdout__
class MotionProgramExecClient:
    def __init__(self, IP='192.168.55.1',ROBOT_CHOICE='RB1',pulse2deg=[1.341416193724337745e+03,1.907685083229250267e+03,1.592916090846681982e+03,1.022871664227330484e+03,9.802549195016306385e+02,4.547554799861444508e+02]):
        self.IP=IP
        self.mh=MotomanConnector(IP=IP, PORT=80, S_pulse = pulse2deg[0], L_pulse = pulse2deg[1], U_pulse = pulse2deg[2], R_pulse = pulse2deg[3], B_pulse = pulse2deg[4], T_pulse = pulse2deg[5])
        self.robodk_rob=RobotPost(r"""Motoman""",r"""Motoman MA2010""",6, axes_type=['R','R','R','R','R','R'],  pulses_x_deg=pulse2deg,ROBOT_CHOICE=ROBOT_CHOICE)


    def execute_motion_program(self, filename):
        self.mh.connectMH() #Connect to Controller
        self.robodk_rob.PROG_FILES=[]
        self.robodk_rob.PROG_FILES.append(filename)
        self.robodk_rob.ProgSendRobot(self.IP,'JOB',"ftp","")
        ###TODO: figure out return time
        self.mh.servoMH() #Turn Servo on

        self.mh.startJobMH('AAA')
        # time.sleep(30)   ###TODO: determine when to turn servo off after job finished
        ###block printing
        blockPrint()
        last_reading=np.zeros(6)
        joint_recording=[]
        timestamps=[]
        while True:
            ###read joint angle
            new_reading=np.array(self.mh.getJointAnglesMH())
            timestamps.append(time.time())
            joint_recording.append(new_reading)

            ###check if robot stop
            if np.linalg.norm(last_reading-new_reading)==0:
                [d1,d2]=self.mh.statusMH()
                d1 = [int(i) for i in bin(int(d1))[2:]]
                if not d1[4]:       #if robot not running
                    break
            last_reading=copy.deepcopy(new_reading)
            
        ###enable printing
        enablePrint()
        self.mh.servoMH(False) #Turn the Servos of
        self.mh.disconnectMH() #DISConnect to Controller
        
        return timestamps, joint_recording


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
