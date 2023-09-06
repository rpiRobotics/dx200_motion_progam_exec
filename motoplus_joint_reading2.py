from motoman_robotraconteur_driver.motoplus_rr_driver_feedback_client \
    import MotoPlusRRDriverFeedbackSyncClient
from motoman_robotraconteur_driver.motoplus_rr_driver_command_client \
    import MotoPlusRRDriverCommandClient
import asyncio
import time, pickle

robot_ip = '192.168.1.31'

async def _read_controller_info():
    c = MotoPlusRRDriverCommandClient()
    c.start(robot_ip)
    await c.wait_ready(10)
    controller_info = await c.get_controller_info() 
    return controller_info

def read_controller_info():
    controller_info = asyncio.run(_read_controller_info())
    return controller_info

controller_info = read_controller_info()
pickle.dump(controller_info, open('controller_info.pkl', 'wb'))

fb = MotoPlusRRDriverFeedbackSyncClient()
fb.start(robot_ip)

# Set the feedback timeout to desired time to wait for incoming packet
fb_timeout = 0.001

while True:
    res, fb_data = fb.try_receive_state_sync(controller_info, fb_timeout)
    if res:
        print(fb_data)
    time.sleep(0.004)

# ControllerState(version=256, time=1689862682.104, seqno=1927179, controller_flags=2442, group_state=[GroupState(axes_count=6, group_flags=6, command_position=array([-0.24426384,  0.47705583,  0.08776562,  0.25640856, -0.8645817 ,
#        -0.676863  ]), feedback_position=array([-0.24426384,  0.47705583,  0.08776562,  0.25674982, -0.8645817 ,
#        -0.67690138]), command_speed=array([0., 0., 0., 0., 0., 0.]), feedback_speed=array([ 0.        ,  0.        ,  0.        ,  0.        , -0.00220784,
#         0.00475907]), torque=array([ 0.        ,  0.22127749,  0.18802222, -0.00546022, -0.00398835,
#         0.01596592])), GroupState(axes_count=6, group_flags=6, command_position=array([1.57077762e+00, 0.00000000e+00, 0.00000000e+00, 1.79939095e-05,
#        0.00000000e+00, 0.00000000e+00]), feedback_position=array([ 1.57077762e+00, -1.34223319e-05,  0.00000000e+00,  1.79939095e-05,
#         0.00000000e+00,  0.00000000e+00]), command_speed=array([0., 0., 0., 0., 0., 0.]), feedback_speed=array([0., 0., 0., 0., 0., 0.]), torque=array([ 0.        , -0.10128492,  0.09827295, -0.00863708, -0.01253481,
#         0.01105333])), GroupState(axes_count=2, group_flags=2, command_position=array([-0.26179542,  3.14159265]), feedback_position=array([-0.26179542,  3.14159265]), command_speed=array([0., 0.]), feedback_speed=array([0., 0.]), torque=array([-0.02919534, -0.01391992]))], task_state=[TaskState(task_flags=0, queued_cmd_num=0, completed_cmd_num=0, current_buffer_num=0, current_buffer_seqno=0, last_completed_buffer_seqno=0), TaskState(task_flags=0, queued_cmd_num=0, completed_cmd_num=0, current_buffer_num=0, current_buffer_seqno=0, last_completed_buffer_seqno=0), TaskState(task_flags=0, queued_cmd_num=0, completed_cmd_num=0, current_buffer_num=0, current_buffer_seqno=0, last_completed_buffer_seqno=0)], motion_streaming_state=MotionStreamingState(streaming_flags=3, streaming_error=0))