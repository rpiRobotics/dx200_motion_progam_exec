import sys, copy, socket, os, time
import numpy as np

# from Motoman import *
# Disable print
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore print
def enablePrint():
    sys.stdout = sys.__stdout__

def DirExists(folder):
    """Returns true if the folder exists"""
    return os.path.isdir(folder)

def get_safe_name(progname, max_chars = 6):
    """Get a safe program name"""
    # Remove special characters
    for c in r'-[]/\;,><&*:%=+@!#^()|?^':
        progname = progname.replace(c,'')
    # Set a program name by default:
    if len(progname) <= 0:
        progname = 'Program'
    # Force the program to start with a letter (not a number)
    if progname[0].isdigit():
        progname = 'P' + progname
    # Set the maximum size of a program (number of characters)
    if len(progname) > max_chars:
        progname = progname[:max_chars]
    return progname

def pose_2_xyzrpw(H):
    """Calculates the equivalent position (mm) and Euler angles (deg) as an [x,y,z,r,p,w] array, given a pose.
    It returns the values that correspond to the following operation:
    transl(x,y,z)*rotz(w*pi/180)*roty(p*pi/180)*rotx(r*pi/180)

    :param H: pose
    :type H: :class:`.Mat`
    :return: [x,y,z,w,p,r] in mm and deg

    .. seealso:: :class:`.Mat`, :func:`~robodk.TxyzRxyz_2_Pose`, :func:`~robodk.Pose_2_TxyzRxyz`, :func:`~robodk.Pose_2_ABB`, :func:`~robodk.Pose_2_Adept`, :func:`~robodk.Pose_2_Comau`, :func:`~robodk.Pose_2_Fanuc`, :func:`~robodk.Pose_2_KUKA`, :func:`~robodk.pose_2_xyzrpw`, :func:`~robodk.Pose_2_Nachi`, :func:`~robodk.Pose_2_Staubli`, :func:`~robodk.Pose_2_UR`, :func:`~robodk.quaternion_2_pose`
    """
    x = H[0, 3]
    y = H[1, 3]
    z = H[2, 3]
    if (H[2, 0] > (1.0 - 1e-10)):
        p = -np.pi / 2
        r = 0
        w = np.arctan2(-H[1, 2], H[1, 1])
    elif H[2, 0] < -1.0 + 1e-10:
        p = np.pi / 2
        r = 0
        w = np.arctan2(H[1, 2], H[1, 1])
    else:
        p = np.arctan2(-H[2, 0], np.sqrt(H[0, 0] * H[0, 0] + H[1, 0] * H[1, 0]))
        w = np.arctan2(H[1, 0], H[0, 0])
        r = np.arctan2(H[2, 1], H[2, 2])
    return [x, y, z, r * 180 / np.pi, p * 180 / np.pi, w * 180 / np.pi]

def Pose(xyzrpw):
    [x,y,z,r,p,w] = xyzrpw
    a = r*np.pi/180
    b = p*np.pi/180
    c = w*np.pi/180
    ca = np.cos(a)
    sa = np.sin(a)
    cb = np.cos(b)
    sb = np.sin(b)
    cc = np.cos(c)
    sc = np.sin(c)
    return np.array([[cb*ca, ca*sc*sb - cc*sa, sc*sa + cc*ca*sb, x],[cb*sa, cc*ca + sc*sb*sa, cc*sb*sa - ca*sc, y],[-sb, cb*sc, cc*cb, z],[0,0,0,1]])

def getFileDir(filepath):
    """Returns the directory of a file path"""
    return os.path.dirname(filepath)
def getBaseName(filepath):
    """Returns the file name and extension of a file path"""
    return os.path.basename(filepath)
def RemoveFileFTP(ftp, filepath):
    """Delete a file on a remote server."""
    import ftplib
    try:
        ftp.delete(filepath)
    except ftplib.all_errors as e:
        import sys
        print('POPUP: Could not remove file {0}: {1}'.format(filepath, e))
        sys.stdout.flush()
def UploadFileFTP(file_path_name, server_ip, remote_path, username, password):
    """Upload a file to a robot through FTP"""
    filepath = getFileDir(file_path_name)
    filename = getBaseName(file_path_name)
    import ftplib
    import os
    import sys
    print("POPUP: <p>Connecting to <strong>%s</strong> using user name <strong>%s</strong> and password ****</p><p>Please wait...</p>" % (server_ip, username))
    sys.stdout.flush()
    try:
        myFTP = ftplib.FTP(server_ip, username, password)
    except:
        error_str = sys.exc_info()[1]
        print("POPUP: <font color=\"red\">Connection to %s failed: <p>%s</p></font>" % (server_ip, error_str))
        sys.stdout.flush()
        time.sleep(4)
        return False

    remote_path_prog = remote_path + '/' + filename
    print("POPUP: Connected. Deleting remote file %s..." % remote_path_prog)
    sys.stdout.flush()
    RemoveFileFTP(myFTP, remote_path_prog)
    print("POPUP: Connected. Uploading program to %s..." % server_ip)
    sys.stdout.flush()
    try:
        myFTP.cwd(remote_path)
    except:
        error_str = sys.exc_info()[1]
        print("POPUP: <font color=\"red\">Remote path not found or can't be created: %s</font>" % (remote_path))
        sys.stdout.flush()
        time.sleep(4)
        #contin = mbox("Remote path\n%s\nnot found or can't create folder.\n\nChange path and permissions and retry." % remote_path)
        return False

    def uploadThis(localfile, filename):
        print('  Sending file: %s' % localfile)
        print("POPUP: Sending file: %s" % filename)
        sys.stdout.flush()
        fh = open(localfile, 'rb')
        myFTP.storbinary('STOR %s' % filename, fh)
        fh.close()

    uploadThis(file_path_name, filename)
    myFTP.close()
    print("POPUP: File trasfer completed: <font color=\"blue\">%s</font>" % remote_path_prog)
    sys.stdout.flush()
    return True

def UploadFTP(program, robot_ip, remote_path, ftp_user, ftp_pass, pause_sec=2):
    """Upload a program or a list of programs to the robot through FTP provided the connection parameters"""
    # Iterate through program list if it is a list of files
    if isinstance(program, list):
        if len(program) == 0:
            print('POPUP: Nothing to transfer')
            sys.stdout.flush()
            time.sleep(pause_sec)
            return

        for prog in program:
            UploadFTP(prog, robot_ip, remote_path, ftp_user, ftp_pass, 0)

        print("POPUP: <font color=\"blue\">Done: %i files and folders successfully transferred</font>" % len(program))
        sys.stdout.flush()
        time.sleep(pause_sec)
        print("POPUP: Done")
        sys.stdout.flush()
        return

    import os
    if os.path.isfile(program):
        print('Sending program file %s...' % program)
        UploadFileFTP(program, robot_ip, remote_path, ftp_user, ftp_pass)
    else:
        print('Sending program folder %s...' % program)
        UploadDirFTP(program, robot_ip, remote_path, ftp_user, ftp_pass)

    time.sleep(pause_sec)
    print("POPUP: Done")
    sys.stdout.flush()

# Object class that handles the robot instructions/syntax
class MotionProgramExecClient(object):
    """Robot post object defined for Motoman robots"""
    PROG_EXT = 'JBI'             # set the program extension
    MAX_LINES_X_PROG = 2000      # maximum number of lines per program. It will then generate multiple "pages (files)". This can be overriden by RoboDK settings.    
    DONT_USE_MFRAME = True       # Set to false to use MFRAME for setting reference frames automatically within the program
    DONT_USE_SETTOOL = True      # Set to false to use SETTOOL for setting the tool within the program
    USE_RELATIVE_JOB = False      # Set to False to always use pulses (Otherwise, it might require a special/paid option
    
    INCLUDE_SUB_PROGRAMS = True # Generate sub programs
    STR_V = 'V=100.0'         # set default cartesian speed
    STR_VJ = 'VJ=50.00'       # set default joints speed
    STR_PL = ''             # Rounding value (from 0 to 4) (in RoboDK, set to 100 mm rounding for PL=4
    ACTIVE_FRAME = 9        # Active UFrame Id (register)
    ACTIVE_TOOL = 9         # Active UTool Id (register)
    SPARE_PR = 95           # Spare Position register for calculations
    
    REGISTER_DIGITS = 5

    # Pulses per degree (provide these in the robot parameters menu: Double click the motoman robot in RoboDK, select "Parameters"
    PULSES_X_DEG = None 

    # PROG specific variables:
    LINE_COUNT = 0      # Count the number of instructions (limited by MAX_LINES_X_PROG)
    P_COUNT = 0         # Count the number of P targets in one file
    C_COUNT = 0         # Count the number of P targets in one file
    nProgs = 0          # Count the number of programs and sub programs
    LBL_ID_COUNT = 0    # Number of labels used
    
    # other variables
    ROBOT_POST = ''
    ROBOT_NAME = ''
    PROG_FILES = [] # List of Program files to be uploaded through FTP
    
    PROG_NAMES = [] # List of PROG NAMES
    PROG_LIST = [] # List of PROG 
    
    PROG_NAME = 'unknown'  # Original name of the current program (example: ProgA)
    PROG_NAME_CURRENT = 'unknown' # Auto generated name (different from PROG_NAME if we have more than 1 page per program. Example: ProgA2)
    
    nPages = 0           # Count the number of pages
    PROG_NAMES_MAIN = [] # List of programs called by a main program due to splitting
    
    PROG = []     # Save the program lines
    PROG_TARGETS = []  # Save the program lines (targets section)
    LOG = '' # Save a log
    
    nAxes = 6 # Important: This is usually provided by RoboDK automatically. Otherwise, override the __init__ procedure. 

    HAS_TRACK = False
    HAS_TURNTABLE = False
    
    # Specific to ARC welding applications
    SPEED_BACKUP = None
    LAST_POSE = None
    POSE_FRAME = np.eye(4)
    POSE_FRAME = np.eye(4)
    LAST_CONFDATA = [None, None, None, None] # [pulses(None, Pulses(0), Cartesian) ,  base(or None), tool, config]

    
    def __init__(self, robotpost=None, robotname=None, robot_axes = 6, **kwargs):
        if self.DONT_USE_MFRAME:
            self.ACTIVE_FRAME = None
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.nAxes = robot_axes
        self.PROG = []
        self.PROG_TARGETS=[]
        self.LOG = ''
        self.s = socket.socket()        #socket connection
        self.IP='192.168.1.31'
        self.PORT=80

        for k,v in kwargs.items():
            if k == 'lines_x_prog':
                self.MAX_LINES_X_PROG = v            
            if k == 'pulse2deg':
                self.PULSES_X_DEG = v
            if k=='ROBOT_CHOICE':
                self.ROBOT_CHOICE = v 
            if k =='IP':
                self.IP=v
            if k=='PORT':
                self.PORT=v
                
                
    def ProgStart(self, progname, new_page = False):
        self.PROG=[]
        self.PROG_TARGETS=[]
        progname = get_safe_name(progname)
        progname_i = progname
        if new_page:
            #nPages = len(self.PROG_LIST)
            if self.nPages == 0:
                if len(self.PROG_NAMES_MAIN) > 0:
                    print("Can't split %s: Two or more programs are split into smaller programs" % progname)
                    print(self.PROG_NAMES_MAIN)
                    raise Exception("Only one program at a time can be split into smaller programs")
                self.PROG_NAMES_MAIN.append(self.PROG_NAME) # add the first program in the list to be genrated as a subprogram call
                self.nPages = self.nPages + 1

            self.nPages = self.nPages + 1
            progname_i = "%s%i" % (self.PROG_NAME, self.nPages)          
            self.PROG_NAMES_MAIN.append(progname_i)
            
        else:
            if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
                return
            self.PROG_NAME = progname
            self.nProgs = self.nProgs + 1
            #self.PROG_NAMES = []
            
        self.PROG_NAME_CURRENT = progname_i
        self.PROG_NAMES.append(progname_i)
        
    def ProgFinish(self, progname, new_page = False):
        progname = get_safe_name(progname)
        if not new_page:
            # Reset page count
            self.nPages = 0
            
        #if self.nPROGS > 1:
        #    # Motoman does not support defining multiple programs in the same file, so one program per file
        #    return
        header = ''
        header += '/JOB' + '\n'
        header += '//NAME %s' % progname + '\n'
        header += '//POS' + '\n'
        header += '///NPOS %i,0,0,%i,0,0' % (self.C_COUNT, self.P_COUNT)
        
        # Targets are added at this point       
        
        import time        
        datestr = time.strftime("%Y/%m/%d %H:%M")
        
        header_ins = ''
        header_ins += '//INST' + '\n'
        header_ins += '///DATE %s' % datestr + '\n'
        #///DATE 2012/04/25 14:11
        header_ins += '///COMM Generated using RoboDK\n' # comment: max 28 chars
        if self.USE_RELATIVE_JOB:
            header_ins += '///ATTR SC,RW,RJ' + '\n'
            if self.ACTIVE_FRAME is not None:
                header_ins += '///FRAME USER %i' % self.ACTIVE_FRAME + '\n'           
        else:
            header_ins += '///ATTR SC,RW' + '\n'

        if self.ROBOT_CHOICE:
            header_ins += '///GROUP1 '+self.ROBOT_CHOICE + '\n'
        else:
            header_ins += '///GROUP1 RB1' + '\n'
        header_ins += 'NOP'
        #if self.HAS_TURNTABLE:
        #    header = header + '/APPL' + '\n'

        self.PROG.insert(0, header_ins)
        self.PROG.append('END')
        
        self.PROG_TARGETS.insert(0, header)
        
        self.PROG = self.PROG_TARGETS + self.PROG        
        
        # Save PROG in PROG_LIST
        self.PROG_LIST.append(self.PROG)
        self.PROG = []
        self.PROG_TARGETS = []
        self.LINE_COUNT = 0
        self.P_COUNT = 0
        self.C_COUNT = 0
        self.LAST_CONFDATA = [None, None, None, None]
        self.LBL_ID_COUNT = 0
        
    def progsave(self, folder, progname, ask_user = False, show_result = False):
        if not folder.endswith('/'):
            folder = folder + '/'
        progname = progname + '.' + self.PROG_EXT
        if ask_user or not DirExists(folder):
            filesave = getSaveFile(folder, progname, 'Save program as...')
            if filesave is not None:
                filesave = filesave.name
            else:
                return
        else:
            filesave = folder + progname
        fid = open(filesave, "w")
        #fid.write(self.PROG)
        for line in self.PROG:
            fid.write(line)
            fid.write('\n')
        fid.close()
        self.PROG_FILES.append(filesave)
        
        # open file with default application
        if show_result:
            if type(show_result) is str:
                # Open file with provided application
                import subprocess
                p = subprocess.Popen([show_result, filesave])
            elif type(show_result) is list:
                import subprocess
                p = subprocess.Popen(show_result + [filesave])   
            else:
                # open file with default application
                import os
                os.startfile(filesave)
            #if len(self.LOG) > 0:
            #    mbox('Program generation LOG:\n\n' + self.LOG)

            
            
    def ProgSave(self, folder, progname, ask_user = False, show_result = False):
        progname = get_safe_name(progname)
        self.PROG = self.PROG_LIST.pop()
        self.progsave(folder, progname, ask_user, show_result)

        if show_result and len(self.LOG) > 0:
            mbox('Program generation LOG:\n\n' + self.LOG)
        
    def ProgSendRobot(self, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        UploadFTP(self.PROG_FILES, self.IP, remote_path, ftp_user, ftp_pass)
        
    def MoveJ(self, pose, joints, speed, zone):
        """Add a joint movement"""
        self.page_size_control() # Important to control the maximum lines per program and not save last target on new program
        target_id = self.add_target_joints(joints)
        self.addline("MOVJ C%05d %s%s" % (target_id, "VJ=%.1f" % speed, ' PL=%i' % round(min(zone, 8))))                    
        
    def MoveL(self, joints, speed, zone, conf_RLF=None):
        """Add a linear movement"""        
        self.page_size_control() # Important to control the maximum lines per program and not save last target on new program
                
        target_id = self.add_target_joints(joints)

        self.addline("MOVL C%05d %s%s" % (target_id, "V=%.1f" % speed, ' PL=%i' % round(min(zone, 8))))        
        
    def MoveC(self, joints1, joints2, joints3, speed, zone, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        self.page_size_control() # Important to control the maximum lines per program and not save last target on new program

        target_id1 = self.add_target_joints(joints1)
        target_id2 = self.add_target_joints(joints2)
        target_id3 = self.add_target_joints(joints3)
            
        self.addline("MOVC C%05d %s%s" % (target_id1, "V=%.1f" % speed, ' PL=%i' % round(min(1, 8))))
        self.addline("MOVC C%05d %s%s" % (target_id2, "V=%.1f" % speed, ' PL=%i' % round(min(1, 8))))
        self.addline("MOVC C%05d %s%s" % (target_id3, "V=%.1f" % speed, ' PL=%i' % round(min(zone, 8))))

    def SetArc(self,on,AC=None,AVP=None,V=None):
        if AC:
            if on:
                self.addline('ARCON '+'AC='+str(AC)+' AVP='+str(AVP)+' V='+str(V))
            else:
                self.addline('ARCOF')
        else:
            if on:
                self.addline('ARCON')
            else:
                self.addline('ARCOF')
        
        
    def setFrame(self, pose, frame_id, frame_name):
        """Change the robot reference frame"""
        xyzwpr = pose_2_xyzrpw(pose)
        if self.DONT_USE_MFRAME:
            self.ACTIVE_FRAME = None
            self.POSE_FRAME = pose
            self.RunMessage('Using %s (targets wrt base):' % (str(frame_name)), True)
            self.RunMessage('%.1f,%.1f,%.1f,%.1f,%.1f,%.1f' % (xyzwpr[0], xyzwpr[1], xyzwpr[2], xyzwpr[3], xyzwpr[4], xyzwpr[5]), True)
        else:
            self.POSE_FRAME = eye(4)
            if frame_id is None or frame_id < 0:
                self.RunMessage('Setting Frame %i (%s):' % (self.ACTIVE_FRAME, str(frame_name)), True)            
                decimals = [1000,1000,1000,100,100,100]
                frame_calc = [eye(4), transl(200,0,0), transl(0,200,0)]
                for m in range(3):
                    xyzwpr_pm = pose_2_xyzrpw(pose*frame_calc[m])
                    for i in range(6):
                        self.addline("SETE P%05d (%i) %i" % (self.SPARE_PR+m, i+1, round(xyzwpr_pm[i]*decimals[i])))
                    for i in range(6,self.nAxes):
                        self.addline("SETE P%05d (%i) %i" % (self.SPARE_PR+m, i+1, 0))
                    
                self.addline("MFRAME UF#(%i) P%05d P%05d P%05d" % (self.ACTIVE_FRAME, self.SPARE_PR, self.SPARE_PR+1, self.SPARE_PR+2))
                    
            else:
                self.ACTIVE_FRAME = frame_id
                self.RunMessage('Frame %i (%s) should be close to:' % (self.ACTIVE_FRAME, str(frame_name)), True)
                self.RunMessage('%.1f,%.1f,%.1f,%.1f,%.1f,%.1f' % (xyzwpr[0], xyzwpr[1], xyzwpr[2], xyzwpr[3], xyzwpr[4], xyzwpr[5]), True)
        
    def setTool(self, pose, tool_id, tool_name):
        """Change the robot TCP"""
        xyzwpr = pose_2_xyzrpw(pose)
        if tool_id is None or tool_id < 0:
            if self.DONT_USE_SETTOOL:
                self.RunMessage('Tool %s should be close to:' % (str(tool_name)), True)
                self.RunMessage('%.1f,%.1f,%.1f,%.1f,%.1f,%.1f' % (xyzwpr[0], xyzwpr[1], xyzwpr[2], xyzwpr[3], xyzwpr[4], xyzwpr[5]), True)
            else:
                self.RunMessage('Setting Tool %i (%s):' % (self.ACTIVE_TOOL, str(tool_name)), True)            
                decimals = [1000,1000,1000,100,100,100]
                for i in range(6):
                    self.addline("SETE P%05d (%i) %i" % (self.SPARE_PR, i+1, round(xyzwpr[i]*decimals[i])))
                for i in range(6,self.nAxes):
                    self.addline("SETE P%05d (%i) %i" % (self.SPARE_PR, i+1, 0))
                    
                self.addline("SETTOOL TL#(%i) P%05d" % (self.ACTIVE_TOOL, self.SPARE_PR))
                
        else:
            self.ACTIVE_TOOL = tool_id
            self.RunMessage('Tool %i (%s) should be close to:' % (self.ACTIVE_TOOL, str(tool_name)), True)
            self.RunMessage('%.1f,%.1f,%.1f,%.1f,%.1f,%.1f' % (xyzwpr[0], xyzwpr[1], xyzwpr[2], xyzwpr[3], xyzwpr[4], xyzwpr[5]), True)

        
    def setDO(self, io_var, io_value):
        """Sets a variable (output) to a given value"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'OT#(%s)' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'ON'
            else:
                io_value = 'OFF'
        
        # at this point, io_var and io_value must be string values
        #DOUT OT#(2) ON
        self.addline('DOUT %s %s' % (io_var, io_value))
        
    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for an input io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = 'IN#(%s)' % str(io_var)        
        if type(io_value) != str: # set default variable value if io_value is a number            
            if io_value > 0:
                io_value = 'ON'
            else:
                io_value = 'OFF'
        
        # at this point, io_var and io_value must be string values
        if timeout_ms < 0:
            #WAIT IN#(12)=ON
            self.addline('WAIT %s=%s' % (io_var, io_value))
        else:
            #self.LBL_ID_COUNT = self.LBL_ID_COUNT + 1
            self.addline('WAIT %s=%s T=%.2f' % (io_var, io_value, timeout_ms*0.001))
       
            
    def RunMessage(self, message, iscomment = False):
        """Add a joint movement"""
        if iscomment:
            for i in range(0,len(message), 29):
                i2 = min(i + 29, len(message))
                self.addline("'%s" % message[i:i2])
                
        else:
            for i in range(0,len(message), 25):
                i2 = min(i + 25, len(message))
                self.addline('MSG "%s"' % message[i:i2])
        
# ------------------ private ----------------------
    def page_size_control(self):
        if self.LINE_COUNT >= self.MAX_LINES_X_PROG:
            self.ProgFinish(self.PROG_NAME, True)
            self.ProgStart(self.PROG_NAME, True)


    def addline(self, newline, movetype = ' '):
        """Add a program line"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        
        self.page_size_control()        
        self.LINE_COUNT = self.LINE_COUNT + 1
        self.PROG.append(newline)
            
    def addline_targets(self, newline):
        """Add a line at the end of the program (used for targets)"""
        self.PROG_TARGETS.append(newline)
        
    def addlog(self, newline):
        """Add a log message"""
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
        self.LOG = self.LOG + newline + '\n'
        
# ------------------ targets ----------------------     

    def setCartesian(self, confdata):
        #self.LAST_CONFDATA = [none/pulses(0)/postype(1), base, tool, config]
        if self.ACTIVE_FRAME is not None and self.ACTIVE_FRAME != self.LAST_CONFDATA[1]:
            self.addline_targets("///USER %i" % self.ACTIVE_FRAME)
            self.LAST_CONFDATA[1] = self.ACTIVE_FRAME        

        if self.ACTIVE_TOOL != self.LAST_CONFDATA[2]:
            self.addline_targets("///TOOL %i" % self.ACTIVE_TOOL)
            self.LAST_CONFDATA[2] = self.ACTIVE_TOOL

        if self.LAST_CONFDATA[0] != 2:
            if self.ACTIVE_FRAME is not None:
                self.addline_targets("///POSTYPE USER")
            else:
                self.addline_targets("///POSTYPE BASE")

            self.addline_targets("///RECTAN")
            self.addline_targets("///RCONF %s" % confdata)
            self.LAST_CONFDATA[3] = confdata
            
            
        elif self.LAST_CONFDATA[3] != confdata:
            self.addline_targets("///RCONF %s" % confdata)
            self.LAST_CONFDATA[3] = confdata

        self.LAST_CONFDATA[0] = 2

    def setPulses(self):
        #self.LAST_CONFDATA = [none/pulses(0)/postype(1), base, tool, config]
        if self.LAST_CONFDATA[0] is None:
            self.addline_targets("///TOOL %i" % self.ACTIVE_TOOL)
            self.LAST_CONFDATA[2] = self.ACTIVE_TOOL
       
        if self.LAST_CONFDATA[0] != 1:
            self.addline_targets("///POSTYPE PULSE")
            self.addline_targets("///PULSE")
            self.LAST_CONFDATA[0] = 1
            
        self.LAST_CONFDATA[0] = 1
        self.LAST_CONFDATA[1] = None
        self.LAST_CONFDATA[2] = None
        self.LAST_CONFDATA[3] = None
        
    def add_target_joints(self, joints):    
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return

        self.setPulses()            
        cid = self.C_COUNT
        self.C_COUNT = self.C_COUNT + 1        
                
        str_pulses=[]        
        for i in range(len(joints)):
            str_pulses.append('%i' % round(joints[i] * self.PULSES_X_DEG[i]))

        self.addline_targets('C%05i=' % cid + ','.join(str_pulses))         
        return cid
    
    def add_target_cartesian(self, pose, joints, conf_RLF):           
        if self.nProgs > 1 and not self.INCLUDE_SUB_PROGRAMS:
            return
            
        if not self.USE_RELATIVE_JOB:
            return self.add_target_joints(joints)            
            
        xyzwpr = pose_2_xyzrpw(pose)
        
        if conf_RLF is None:
            conf_RLF = [0,0,0]

        turns = [0,0,0]
        if len(joints) >= 6:
            turnJ4 = (joints[3]+180)//360
            turnJ6 = (joints[5]+180)//360
            turnJ1 = (joints[0]+180)//360
            turns = [turnJ4, turnJ6, turnJ1]

        confdata = '%i,%i,%i,%i,%i,%i,0,0' % tuple(conf_RLF[:3] + turns[:3])
        self.setCartesian(confdata)            
        cid = self.C_COUNT
        self.C_COUNT = self.C_COUNT + 1        
        self.addline_targets('C%05i=' % cid + '%.3f,%.3f,%.3f,%.2f,%.2f,%.2f' % tuple(xyzwpr))
        return cid

    ##################ADVANCED ETHERNET FUNCTION############################
    def __sendCMD(self,command,payload):
        """INTERNAL - Internal send Function.

        Args:
            command (string): Command - See Yaskawa documentation
            payload (string): Command payload - empty string if no data should be send. If not empty, remember the <CR> at the end!

        Raises:
            Exception: If the Command does not return an ok, an error is raised

        Returns:
            string data: returned data from command transaction
            string data2: returned data from command payload transaction
        """

        print(command,payload)

        self.s.send(bytes(f"HOSTCTRL_REQUEST {command} {len(payload)}\r\n","utf-8"))
        data = self.s.recv(1024)
        print(f'Received: {repr(data)}')

        if data[:2] != b"OK":
            print(f"COMMAND ({command}) ERROR")
            raise Exception("Yaskawa Error!")

        elif len(payload) > 0:
            self.s.send(bytes(f"{payload}","utf-8"))
        
        data2 = self.s.recv(1024)
        print(f'Received: {repr(data2)}')
        
        return data, data2


    def connectMH(self):
        """Connect to the Motoman controller

        Raises:
            Exception: If the connection does not return OK
        """
        self.s.connect((self.IP,self.PORT))
        self.s.send(b"CONNECT Robot_access Keep-Alive:-1\r\n")
        data = self.s.recv(1024)
        print(f'Received: {repr(data)}')
        if data[:2] != b"OK":
            print("Connection Faulty!")
            raise Exception("Yaskawa Connection Error!")

    def disconnectMH(self): #Disconnect
        """Disconnect from the Controller
        """
        self.s.close()

    def getJointAnglesMH(self): #Read Encoder pulses and convert them to Joint Angles
        """Read the Joint Angles

        Returns:
            list: list of the six joint angles
        """
        d1, d2 = self.__sendCMD("RPOSJ","")

        data2_str = d2.decode("utf-8").replace("\r","").split(",")

        data2_arr = [int(data2_str[0])/self.PULSES_X_DEG[0],int(data2_str[1])/self.PULSES_X_DEG[1],int(data2_str[2])/self.PULSES_X_DEG[2],int(data2_str[3])/self.PULSES_X_DEG[3],int(data2_str[4])/self.PULSES_X_DEG[4],int(data2_str[5])/self.PULSES_X_DEG[5]]
        return data2_arr

    def getCoordinatesMH(self,coordinateSystem = 0): #Somehow our controller raises an internal error
        """Read the current Position in reference to a selectable coordinate system, currently Broken on DX Controllers!

        Args:
            coordinateSystem (int, optional): The refereced coordinate System. 0 = Base, 1 = Robot, 2-64 = User. Defaults to 0.

        Returns:
            list: List with the current Positional Values
        """
        d1, d2 = self.__sendCMD("RPOSC","0,0\r")
        return d2.decode("utf-8").replace("\r","").split(",")

    def servoMH(self, state = True): #Enable/Disable Servos
        """Turn on/off the Servo motors

        Args:
            state (bool, optional): Powerstate to set the servos to. Defaults to True.
        """
        time.sleep(0.1)
        self.__sendCMD("SVON",f"{1 if state else 0}\r")


    def moveAngleMH(self, speed,S,L,U,R,B,T):
        """Move the Robot in joint coordinates

        Args:
            speed (float): Speed value - 0% - 100% - It's not recomended to use more than 50%!
            S (float): S angle
            L (float): L angle
            U (float): U angle
            R (float): R angle
            B (float): B angle
            T (float): T angle
        """
        cmd = f"{speed},{int(S*self.PULSES_X_DEG[0])},{int(L*self.PULSES_X_DEG[1])},{int(U*self.PULSES_X_DEG[2])},{int(R*self.PULSES_X_DEG[3])},{int(B*self.PULSES_X_DEG[4])},{int(T*self.PULSES_X_DEG[5])},0,0,0,0,0,0,0\r" #Convert encoder pulses
        self.__sendCMD("PMOVJ",cmd)


    def WriteVariableMH(self,type,number,value):
        """Write a Variable on the controller

        Args:
            type (int): Type of the Variable | 0 = Byte, 1 = Integer, 2 = Double, 3 = Real, 7 = String. Other Values raise an exception
            number (int): variable numer
            value (byte/int/float/string): Variable Value

        Raises:
            Exception: Exception if the type is not allowed
        """
        cmd = f"{type},{number},{value}\r"
        if type in [0,1,2,3,7]: self.__sendCMD("LOADV",cmd) #Check if Variable Type is allowed
        else: raise Exception("Variable Type not supported!")

    def ReadVariableMH(self,type,number):
        """Read a variable from the controller

        Args:
            type (int): Type of the Variable
            number (int): Variable Number

        Returns:
            string: Variable Value
        """
        d1,d2 = self.__sendCMD("SAVEV",f"{type},{number}\r")
        return d2.decode("utf-8").replace("\r","")

    def statusMH(self):
        """Read the Status bytes from the Robot

        Returns:
            list: list containing the two status bytes
        """
        d1,d2 = self.__sendCMD("RSTATS","")
        status = d2.decode("utf-8").replace("\r","").split(",")
        return status

    def readCurrJobMH(self):
        """Read the current Job Name

        Returns:
            string: Current Job Name
        """
        d1,d2 = self.__sendCMD("RJSEQ","")
        return d2
    
    def startJobMH(self,job):
        """Start a Job by its name

        Args:
            job (string): Job Name which to start

        Returns:
            string d1: return of command transaction
            string d2: return of the Payload Transaction
        """
        d1,d2 = self.__sendCMD("START",f"{job}\r")
        return d1, d2

    def readGroup(self):
        """READ current work group

        Returns:
            string d1: Robot control group information
            string d2: Station control group information
        """
        d1,d2 = self.__sendCMD("RGROUP","")

        data2_str = d2.decode("utf-8").replace("\r","").split(",")
        return [eval(i) for i in data2_str]

    def setGroup(self,robot,station=1,task=0):
        """SET current work group
        robot: 1 or 2: 2010 or 1440
        station: 0 or 1: station or not
        task: default at 0
        """

        d1,d2=self.__sendCMD("RPOSC",b"3,1,0\r")
        # if robot==1:
        #     d1,d2=self.__sendCMD("RPOSC",b"1,1,0\r")
        # else:
        #     d1,d2=self.__sendCMD("RPOSC",b"2,1,0\r")

    ##############################EXECUTION############################################
    def execute_motion_program(self, filename):
        self.connectMH() #Connect to Controller
        self.PROG_FILES=[]
        self.PROG_FILES.append(filename)
        self.ProgSendRobot('JOB',"ftp","")
        ###TODO: figure out return time
        self.servoMH() #Turn Servo on

        self.startJobMH('AAA')
        # time.sleep(30)   ###TODO: determine when to turn servo off after job finished
        ###block printing
        blockPrint()
        last_reading=np.zeros(6)
        joint_recording=[]
        timestamps=[]
        while True:
            ###read joint angle
            new_reading=np.array(self.getJointAnglesMH())
            timestamps.append(time.time())
            joint_recording.append(new_reading)

            ###check if robot stop
            if np.linalg.norm(last_reading-new_reading)==0:
                [d1,d2]=self.statusMH()
                d1 = [int(i) for i in bin(int(d1))[2:]]
                if not d1[4]:       #if robot not running
                    break
            last_reading=copy.deepcopy(new_reading)
            
        ###enable printing
        enablePrint()
        self.servoMH(False) #Turn the Servos of
        self.disconnectMH() #DISConnect to Controller
        
        return timestamps, joint_recording



def main():
    client=MotionProgramExecClient(ROBOT_CHOICE='RB1',pulse2deg=[1.341416193724337745e+03,1.907685083229250267e+03,1.592916090846681982e+03,1.022871664227330484e+03,9.802549195016306385e+02,4.547554799861444508e+02])

    ###TODO: fix tool definition
    # client.motoman.DONT_USE_SETTOOL=False
    # client.motoman.setTool(Pose([0,0,450,0,0,0]), None, 'welder')
    client.ACTIVE_TOOL=1

    client.ProgStart(r"""AAA""")
    client.setFrame(Pose([0,0,0,0,0,0]),-1,r"""Motoman MA2010 Base""")
    client.MoveJ(Pose([1232,0,860,0,0,-180]),[0,0,0,0,0,0],10,0)
    client.MoveJ(Pose([1232,0,593.478,0,0,-180]),[0,-1.23097,-15.1604,0,13.9294,0],10,0)
    client.MoveJ(Pose([903.741,0,593.478,0,0,-180]),[0,-27.1205,-36.7057,0,9.58517,0],10,0)
    client.ProgFinish(r"""AAA""")
    client.ProgSave(".","AAA",False)

    print(client.execute_motion_program("AAA.JBI"))

def movec_test():
    client=MotionProgramExecClient(ROBOT_CHOICE='RB1',pulse2deg=[1.341416193724337745e+03,1.907685083229250267e+03,1.592916090846681982e+03,1.022871664227330484e+03,9.802549195016306385e+02,4.547554799861444508e+02])

    ###TODO: fix tool definition
    # client.motoman.DONT_USE_SETTOOL=False
    # client.motoman.setTool(Pose([0,0,450,0,0,0]), None, 'welder')
    client.ACTIVE_TOOL=1

    client.ProgStart(r"""AAA""")
    client.setFrame(Pose([0,0,0,0,0,0]),-1,r"""Motoman MA2010 Base""")
    q1=np.array([-29.3578,31.3077,10.7948,7.6804,-45.9367,-18.5858])
    q2=np.array([-3.7461,37.3931,19.2775,18.7904,-53.9888,-48.712])
    q3=np.array([29.3548,5.8107,-20.41,27.3331,-58.956,-86.4])
    client.MoveJ(None,q1,1,0)
    client.MoveC(q1, q2, q3, 10,0)

    client.ProgFinish(r"""AAA""")
    client.ProgSave(".","AAA",False)

    print(client.execute_motion_program("AAA.JBI"))


if __name__ == "__main__":
    movec_test()
