import io
import struct
import motoplus_rr_driver_command_client as command_client
import motoplus_rr_driver_buffer_client as buffer_client
import numpy as np

def _reinterpret_int32(v):
    assert v < 0x7FFFFFFF and v > -0x8000000
    if v >= 0:
        return v
    else:
        return 0xFFFFFFFF + (v+1)

class MotionProgram:
    def __init__(self, controller_info, group_num: int=0, first_cmd_num: int=1):
        self._first_cmd_num = first_cmd_num
        self._buf = []
        self._cmd_num = first_cmd_num
        self._group_num = group_num
        self._info = controller_info

    def MoveAbsJ(self, to_joint_pos, speed, accuracy):

        assert(len(to_joint_pos) == 6)

        # command number
        self._buf.append((self._cmd_num))
        self._cmd_num += 1

        # Opcode for move command
        self._buf.append(1)

        # group count
        self._buf.append(1)

        # group number
        self._buf.append(self._group_num)

        # config (not used for MoveAbsJ)
        self._buf.append(0)

        # accuracy (specified in meters, convert to micrometers)
        self._buf.append(int(accuracy*1e6))

        # speeds (joint, linear, angular)
        self._buf.append(_reinterpret_int32(speed*1e2))
        self._buf.append(_reinterpret_int32(speed*1e4))
        self._buf.append(_reinterpret_int32(speed*1e4))

        # accel (not used)
        self._buf.append(0)

        # decel (not used)
        self._buf.append(0)

        # coord_type (pulse)
        self._buf.append(0)
        self._buf.append(0)

        # interpolation_type
        self._buf.append(1)

        # destination (6 values)
        pulse_to_radians = self._info.control_groups[self._group_num].pulse_to_radians
        for i in range(6):
            j = pulse_to_radians[i] * to_joint_pos[i]
            self._buf.append(_reinterpret_int32(j))

        # filler
        for i in range(10):
            self._buf.append(0)

    def get_program_buf(self):
        return np.array(self._buf, dtype=np.uint32)
    
class MotionProgramExecClient:
    def __init__(self, controller_host):

        self.controller_host = controller_host

    async def init(self, timeout = 300):

        self._command_client = command_client.MotoPlusRRDriverCommandClient()
        self._command_client.start(self.controller_host)

        self._buffer_client = buffer_client.MotoPlusRRDriverBufferClient()
        self._buffer_client.start(self.controller_host)

        await self._command_client.wait_ready(timeout)
        await self._buffer_client.wait_ready(timeout)

        self.controller_info = await self._command_client.get_controller_info()

    async def execute_motion_program(self, motion_program):
        buf =motion_program.get_program_buf()
        print(buf)

        # For now only use zero buffer
        await self._buffer_client.set_buffer(0, buf)

        # Execute the motion program
        await self._command_client.start_motion_program(0, 0, 0, 1)









