import sys
import numpy as np
import argparse
import threading
from dx200_motion_program_exec_client_new import*
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil

import traceback
import time
import io
import random


def rr_zone_to_motoman(rr_fine_point,rr_blend_radius):
    #TODO use "extended" fields for external axes
    return rr_blend_radius

def rr_speed_to_motoman(rr_velocity):
    #TODO use "extended" for angular velocity and external axes?
    return rr_velocity

def rr_joints_to_motoman(rr_joints, rr_joint_units):
    #TODO: joint units
    #TODO: use "extended" for external axes
    return np.rad2deg(rr_joints)

def rr_motion_program_to_motoman(rr_mp):
    mp=MotionProgram(ROBOT_CHOICE='RB1',pulse2deg=[1.435355447016790322e+03,1.300329111270902331e+03,1.422225409601069941e+03,9.699560942607320158e+02,9.802408285708806943e+02,4.547552630640436178e+02])

    for cmd in rr_mp.motion_program_commands[1:]:
        if cmd.datatype.endswith("MoveJ"):
            zd = rr_zone_to_motoman(cmd.data.fine_point,cmd.data.blend_radius)
            sd = rr_speed_to_motoman(cmd.data.tcp_velocity)
            rt = rr_joints_to_motoman(cmd.data.joint)
            mp.MoveJ(rt, sd, zd)
        elif cmd.datatype.endswith("MoveL"):
            zd = rr_zone_to_motoman(cmd.data.fine_point,cmd.data.blend_radius)
            sd = rr_speed_to_motoman(cmd.data.tcp_velocity)
            rt = rr_joints_to_motoman(cmd.data.joint)
            mp.MoveL(rt, sd, zd)
        elif cmd.datatype.endswith("MoveC"):
            zd = rr_zone_to_motoman(cmd.data.fine_point,cmd.data.blend_radius)
            sd = rr_speed_to_motoman(cmd.data.tcp_velocity)
            rt = rr_joints_to_motoman(cmd.data.joint)
            rt2 = rr_joints_to_motoman(cmd.data.joint2)
            rt2 = rr_joints_to_motoman(cmd.data.joint3)
            mp.MoveC(rt2, rt,  sd, zd)
        elif cmd.datatype.endswith("WaitTime"):
            mp.setWaitTime(cmd.data.time)
        elif cmd.datatype.endswith("ARC"):
            mp.SetArc(cmd.data.on,cmd.data.jobnum)
        else:
            assert False, f"Invalid motion program command type \"{cmd.datatype}\""
    
    return mp


class MotionExecImpl:
    def __init__(self, mp_robot_info, ip, port):

        self.mp_robot_info = mp_robot_info
        self._motoman_client = MotionProgramExecClient(ip, port)

        self.device_info = mp_robot_info.robot_info.device_info
        self.robot_info = mp_robot_info.robot_info
        self.motion_program_robot_info = mp_robot_info
        self._logs = {}

    def execute_motion_program(self, program):

        gen = ExecuteMotionProgramGen(self, self._motoman_client, rr_motion_program_to_motoman(program))

        return gen


    def read_log(self, log_handle):
        robot_log_np = self._logs.pop(log_handle)
        return RobotLogGen(robot_log_np)

    def clear_logs(self):
        self._logs.clear()



class ExecuteMotionProgramGen:

    def __init__(self, parent, motoman_client, motion_program, save_log = False):
        self._parent = parent
        self._motoman_client = motoman_client
        self._motion_program = motion_program
        self._action_status_code = RRN.GetConstants("com.robotraconteur.action")["ActionStatusCode"]
        self._status = self._action_status_code["queued"]
        self._thread = None
        self._wait_evt = threading.Event()
        self._thread_exp = None
        self._mp_status = RRN.GetStructureType("experimental.robotics.motion_program.MotionProgramStatus")
        self._log_handle = 0
        self._save_log = save_log

    def Next(self):
        if self._thread_exp is not None:
            raise self._thread_exp
        ret = self._mp_status()
        ret.current_command = -1
        if self._status == self._action_status_code["queued"]:
            self._thread = threading.Thread(target=self._run)
            self._status = self._action_status_code["running"]
            self._thread.start()
            ret.action_status = self._status
            return ret
        self._wait_evt.wait(timeout=1)
        if self._thread.is_alive():
            ret.action_status = self._action_status_code["running"]
            return ret
        else:
            if self._log_handle != 0:
                self._status = self._action_status_code["complete"]
                ret.action_status = self._status
                ret.log_handle = self._log_handle
                self._log_handle = 0
                return ret
            if self._thread_exp:
                raise self._thread_exp
            raise RR.StopIterationException()

    def Close(self):
        pass

    def Abort(self):
        if self._status == self._action_status_code["queued"] or self._status == self._action_status_code["running"]:
            self._motoman_client.stop()

    def _run(self):
        try:
            print("Start Motion Program!")
            robot_log_csv = self._motoman_client.execute_motion_program(self._motion_program)
            if self._save_log:
                robot_log_io = io.StringIO(robot_log_csv.decode("ascii"))
                robot_log_io.seek(0)
                robot_log_np = np.genfromtxt(robot_log_io, dtype=np.float64, skip_header=1, delimiter=",")
                log_handle = random.randint(0,0xFFFFFFF)
                self._parent._logs[log_handle] = robot_log_np
                self._log_handle = log_handle
            print("Motion Program Complete!")
        except BaseException as e:
            self._thread_exp = e
            traceback.print_exc()
        self._wait_evt.set()


class RobotLogGen:
    def __init__(self, robot_log_np, ):
        self.robot_log_np = robot_log_np
        self.closed = False
        self.aborted = False
        self.lock = threading.Lock()
        self._mp_log_part = RRN.GetStructureType("experimental.robotics.motion_program.MotionProgramLogPart")

    def Next(self):
        with self.lock:
            if self.aborted:
                raise RR.OperationAbortedException("Log aborted")

            if self.closed:
                raise RR.StopIterationException()

            ret = self._mp_log_part()

            # All current paths expect to be within 10 MB limit
            ret.time = self.robot_log_np[:,0].flatten()
            ret.command_number = self.robot_log_np[:,1].flatten().astype(np.int32)
            ret.joints = self.robot_log_np[:,2:]

            self.closed = True
            return ret

    def Abort(self):
        self.aborted = True

    def Close(self):
        self.closed = True

            


def main():

    parser = argparse.ArgumentParser(description="Motoman Robot motion program driver service for Robot Raconteur")
    parser.add_argument("--mp-robot-info-file", type=argparse.FileType('r'),default=None,required=True,help="Motion program robot info file (required)")
    parser.add_argument("--mp-robot-base-ip", type=str, default='192.168.1.31', help="robot controller ws base ip (default 192.168.1.31)")
    parser.add_argument("--mp-robot-base-port", type=int, default=80, help="robot controller ws base port (default 80)")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM orSIGINT (Linux only)")

    args, _ = parser.parse_known_args()

    RRC.RegisterStdRobDefServiceTypes(RRN)
    RRN.RegisterServiceTypeFromFile('config/experimental.robotics.motion_program')

    with args.mp_robot_info_file:
        mp_robot_info_text = args.mp_robot_info_file.read()

    info_loader = InfoFileLoader(RRN)
    mp_robot_info, mp_robot_ident_fd = info_loader.LoadInfoFileFromString(mp_robot_info_text, "experimental.robotics.motion_program.MotionProgramRobotInfo", "mp_robot")

    attributes_util = AttributesUtil(RRN)
    mp_robot_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(mp_robot_info.robot_info.device_info)

    mp_exec_obj = MotionExecImpl(mp_robot_info,args.mp_robot_base_ip,args.mp_robot_base_port)

    with RR.ServerNodeSetup("experimental.robotics.motion_program",59843,argv=sys.argv):

        service_ctx = RRN.RegisterService("mp_robot","experimental.robotics.motion_program.MotionProgramRobot",mp_exec_obj)
        service_ctx.SetServiceAttributes(mp_robot_attributes)

        if args.wait_signal:  
            #Wait for shutdown signal if running in service mode          
            print("Press Ctrl-C to quit...")
            import signal
            signal.sigwait([signal.SIGTERM,signal.SIGINT])
        else:
            #Wait for the user to shutdown the service
                input("Server started, press enter to quit...")

if __name__ == "__main__":
    main()
