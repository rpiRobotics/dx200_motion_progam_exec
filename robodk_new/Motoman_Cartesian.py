# Copyright 2015-2021 - RoboDK Inc. - https://robodk.com/
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ----------------------------------------------------
# This file is a POST PROCESSOR for Robot Offline Programming to generate programs 
# for a Motoman robot (Inform III programming language)
# This post generates MOVL as Cartesian coordinates. MOVJ are generated in pulses.
#
# To edit/test this POST PROCESSOR script file:
# Select "Program"->"Add/Edit Post Processor", then select your post or create a new one.
# You can edit this file using any text editor or Python editor. Using a Python editor allows to quickly evaluate a sample program at the end of this file.
# Python should be automatically installed with RoboDK
#
# You can also edit the POST PROCESSOR manually:
#    1- Open the *.py file with Python IDLE (right click -> Edit with IDLE)
#    2- Make the necessary changes
#    3- Run the file to open Python Shell: Run -> Run module (F5 by default)
#    4- The "test_post()" function is called automatically
# Alternatively, you can edit this file using a text editor and run it with Python
#
# To use a POST PROCESSOR file you must place the *.py file in "C:/RoboDK/Posts/"
# To select one POST PROCESSOR for your robot in RoboDK you must follow these steps:
#    1- Open the robot panel (double click a robot)
#    2- Select "Parameters"
#    3- Select "Unlock advanced options"
#    4- Select your post as the file name in the "Robot brand" box
#
# To delete an existing POST PROCESSOR script, simply delete this file (.py file)
#
# ----------------------------------------------------
# More information about RoboDK Post Processors and Offline Programming here:
#     https://robodk.com/help#PostProcessor
#     https://robodk.com/doc/en/PythonAPI/postprocessor.html
# ----------------------------------------------------


# ----------------------------------------------------
# Import RoboDK tools
from robodk import *
import sys

# ----------------------------------------------------    
# Object class that handles the robot instructions/syntax
from Motoman import RobotPost as MainPost # sublassing the default Motoman post to change the settings
class RobotPost(MainPost):
    """Robot post object defined for Motoman robots"""
    #--------------------------------------------------------------------------------------
    # ---------------- Customize your post processor for best results ---------------------
    # Set the default maximum number of lines per program. 
    # It will then generate multiple "pages (files)". This can be overriden by RoboDK settings.    
    MAX_LINES_X_PROG = 2000    
    
    # Set to True to use SETTOOL for setting the tool in the JBI program
    #USE_SETTOOL = False   
    USE_SETTOOL = True   # This requires the SETTOOL option from Motoman (paid option)
    
    # Specify the default UTool Id to use (register). 
    # You can also use Numbered tools in RoboDK (for example, a tool named "Tool 2" will use UTOOL number 2)
    ACTIVE_TOOL = 9         
    
    # Set to False to always use pulses (otherwise, it may require a paid option)
    #USE_RELATIVE_JOB = False
    USE_RELATIVE_JOB = True  # This requires the Relative Job option from Motoman
    
    # Force joint movements to be in Cartesian
    MOVEJ_IN_CARTESIAN = False
    
    # Generate sub programs with each program
    INCLUDE_SUB_PROGRAMS = True 
        
    # Specify a spare Position register for calculations (Tool, Reference, ...)
    SPARE_PR = 95
    
    # Set to True to use MFRAME for setting reference frames automatically within the program
    USE_MFRAME = False      
    
    # Specify the default UFrame Id to use (register).     
    # You can also use Numbered References in RoboDK (for example, a reference named "Reference 4" will use UFRAME number 4)
    ACTIVE_FRAME = 9   
    
    # Specify if external axes are defined as a separate variable unit
    # (use EC instead of extending the default C register)
    EXTAXES_USE_EC = True
    
    # Specify if external axes must be moved according to a separate MOVJ command
    EXTAXES_USE_MOVJ = True  # Will output: MOVL C00008 V=166.7  +MOVJ EC00008 VJ=100.00
    #EXTAXES_USE_MOVJ = False # Will output: MOVL C00008 EC00008 V=166.7
    
    # Specify the pulses/degree ratio for the external axes here (index 7,8,...)
    PULSES_X_DEG = [1,1,1,1,1,1,  360/360, 360/360,   1,1] 
    
    # Option to swap a specific axis
    # AXIS_INDEX = [0,1,2,3,4,5,6,7] # example to swap axes 7 and 8 (external axes 1-2)
    AXIS_INDEX = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14] # table is already swaped (final ids)    
    
    
    
#/JOB
#//NAME TESTTCPX
#//POS
#///NPOS 3,0,0,19,0,0
#///TOOL 23
#///POSTYPE PULSE
#///PULSE
#C00000=2730,-84461,-101368,0,-77310,67137
#///TOOL 13
#C00001=76697,-73189,-80544,374,-78336,86207
#C00002=81732,-66267,-100360,-2451,-62876,82497
#///USER 8
#///TOOL 23
#///POSTYPE USER
#///RECTAN
#///RCONF 0,0,0,0,1,0,0,0
#P0010=0.000,0.000,10.205,0.00,0.00,0.00
#P0011=0.001,0.006,-9.794,-170.32,90.00,-170.32
#///RCONF 0,0,0,0,0,0,0,0
#P0012=13.500,0.000,0.000,0.00,0.00,0.00
#///RCONF 0,0,0,0,1,0,0,0
#P0015=0.000,0.000,0.000,180.00,90.00,180.00
#///RCONF 0,0,0,0,0,0,0,0
#P0020=0.000,0.000,0.000,-90.00,0.00,0.00
#P0021=0.000,0.000,-5.000,0.00,0.00,0.00
#///RCONF 1,0,0,0,0,0,0,0
#P0022=0.004,0.003,8.297,-90.00,0.00,-90.00
#P0023=-0.003,-0.003,-9.351,-90.00,0.00,-90.00
#///POSTYPE BASE
#///RCONF 0,0,0,0,0,0,0,0
#P0024=0.000,0.000,8.824,0.00,0.00,0.00
#///TOOL 0
#P0026=-276.205,101.089,162.089,0.08,-83.39,-19.88
#P0027=-276.205,101.634,162.089,179.99,-6.61,160.20
#///USER 8
#///TOOL 23
#///POSTYPE USER
#///RCONF 0,0,0,0,1,0,0,0
#P0028=0.410,-0.016,0.147,0.00,-90.00,0.00
#///RCONF 1,0,0,0,0,0,0,0
#P0030=-0.004,0.001,-0.527,-90.00,0.00,-90.00
#P0100=0.000,0.000,50.000,0.00,90.00,0.00
#P0101=0.000,0.000,-25.000,0.00,90.00,0.00
#P0103=0.000,0.000,25.000,-90.00,0.00,-90.00
#P0104=0.000,0.000,-25.000,-90.00,0.00,-90.00
#P0110=-100.000,0.000,0.000,0.00,0.00,0.00
#P0111=0.000,0.000,200.000,0.00,0.00,0.00
#//INST
#///DATE 2012/04/25 14:11
#///COMM 1ER PROGRAM POUR VERIFIER LE TCP
#///ATTR SC,RW
#///GROUP1 RB1
#NOP
#DOUT OT#(5) ON
#DOUT OT#(2) ON
 



# -------------------------------------------------
# ------------ For testing purposes ---------------   
def Pose(xyzrpw):
    [x,y,z,r,p,w] = xyzrpw
    a = r*math.pi/180
    b = p*math.pi/180
    c = w*math.pi/180
    ca = math.cos(a)
    sa = math.sin(a)
    cb = math.cos(b)
    sb = math.sin(b)
    cc = math.cos(c)
    sc = math.sin(c)
    return Mat([[cb*ca, ca*sc*sb - cc*sa, sc*sa + cc*ca*sb, x],[cb*sa, cc*ca + sc*sb*sa, cc*sb*sa - ca*sc, y],[-sb, cb*sc, cc*cb, z],[0,0,0,1]])

def test_post():
    """Test the post with a basic program"""

    robot = RobotPost('Motomantest', 'Motoman robot', 6)

    robot.ProgStart("Program")
    robot.RunMessage("Program generated by RoboDK", True)
    # robot.setFrame(Pose([807.766544, -963.699898, 41.478944, 0, 0, 0]), None, 0)
    # robot.setTool(Pose([62.5, -108.253175, 100, -60, 90, 0]), None, 0)
    robot.MoveJ(Pose([200, 200, 500, 180, 0, 180]), [-46.18419, -6.77518, -20.54925, 71.38674, 49.58727, -302.54752] )
    robot.MoveL(Pose([200, 250, 348.734575, 180, 0, -150]), [-41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418] )
    robot.MoveL(Pose([200, 200, 262.132034, 180, 0, -150]), [-43.73892, -3.91728, -35.77935, 58.57566, 54.11615, -253.81122] )
    # robot.RunMessage("Setting air valve 1 on")
    # robot.RunCode("TCP_On", True)
    # robot.Pause(1000)
    # robot.MoveL(Pose([200, 250, 348.734575, 180, 0, -150]), [-41.62707, -8.89064, -30.01809, 60.62329, 49.66749, -258.98418] )
    # robot.MoveL(Pose([250, 300, 278.023897, 180, 0, -150]), [-37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677] )
    # robot.MoveL(Pose([250, 250, 191.421356, 180, 0, -150]), [-39.75778, -1.04537, -40.37883, 52.09118, 54.15317, -246.94403] )
    # robot.RunMessage("Setting air valve off")
    # robot.RunCode("TCP_Off", True)
    # robot.Pause(1000)
    # robot.MoveL(Pose([250, 300, 278.023897, 180, 0, -150]), [-37.52588, -6.32628, -34.59693, 53.52525, 49.24426, -251.44677] )
    # robot.MoveL(Pose([250, 200, 278.023897, 180, 0, -150]), [-41.85389, -1.95619, -34.89154, 57.43912, 52.34162, -253.73403] )
    # robot.MoveL(Pose([250, 150, 191.421356, 180, 0, -150]), [-43.82111, 3.29703, -40.29493, 56.02402, 56.61169, -249.23532] )
    robot.ProgFinish("Program")
    robot.ProgSave(".","Program",True)
    
    robot.PROG = robot.PROG_LIST.pop()
    for line in robot.PROG:
        print(line)
    
    if len(robot.LOG) > 0:
        mbox('Program generation LOG:\n\n' + robot.LOG)

    input("Press Enter to close...")

def p(x,y,z,r,p,w):
    a = r*math.pi/180.0
    b = p*math.pi/180.0
    c = w*math.pi/180.0
    ca = math.cos(a)
    sa = math.sin(a)
    cb = math.cos(b)
    sb = math.sin(b)
    cc = math.cos(c)
    sc = math.sin(c)
    return Mat([[cb*ca,ca*sc*sb-cc*sa,sc*sa+cc*ca*sb,x],[cb*sa,cc*ca+sc*sb*sa,cc*sb*sa-ca*sc,y],[-sb,cb*sc,cc*cb,z],[0.0,0.0,0.0,1.0]])

def main():
    r = RobotPost(r"""Motoman""",r"""Motoman MA2010""",6, axes_type=['R','R','R','R','R','R'], ip_com=r"""127.0.0.1""", api_port=20500, prog_ptr=1976622395408, robot_ptr=1976635615216, pulses_x_deg=[1.000000,1.000000,1.000000,1.000000,1.000000,1.000000])

    r.ProgStart(r"""Prog1""")
    r.RunMessage(r"""Program generated by RoboDK v5.3.2 for Motoman MA2010 on 28/10/2022 12:18:47""",True)
    r.RunMessage(r"""Using nominal kinematics.""",True)
    r.setFrame(p(0,0,0,0,0,0),-1,r"""Motoman MA2010 Base""")
    r.MoveJ(p(1232,0,860,0,0,-180),[0,0,0,0,0,0])
    r.MoveJ(p(1232,0,593.478,0,0,-180),[0,-1.23097,-15.1604,0,13.9294,0])
    r.MoveJ(p(903.741,0,593.478,0,0,-180),[0,-27.1205,-36.7057,0,9.58517,0])
    r.ProgFinish(r"""Prog1""")
    r.ProgSave(".","Program",True)

if __name__ == "__main__":
    """Function to call when the module is executed by itself: test"""
    # test_post()
    main()
