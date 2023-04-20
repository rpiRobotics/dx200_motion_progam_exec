import numpy as np
from dx200_motion_program_client import MotionProgram, MotionProgramExecClient
import asyncio

j1 = np.deg2rad([0,0,0,0,0,20], dtype=np.float64)
j2 = np.deg2rad([0,0,0,0,0,-20], dtype=np.float64)
j3 = np.deg2rad([0,0,0,0,10,30], dtype=np.float64)
j4 = np.deg2rad([0,0,0,0,-10,-30], dtype=np.float64)

async def amain():

    # client = MotionProgramExecClient('127.0.0.1')
    client = MotionProgramExecClient('192.168.1.31')
    await client.init()

    mp = MotionProgram(client.controller_info)

    mp.MoveAbsJ(j1, 10, 0.01)
    mp.MoveAbsJ(j2, 10, 0.01)
    mp.MoveAbsJ(j3, 10, 0.01)
    mp.MoveAbsJ(j4, 10, 0.01)

    await client.execute_motion_program(mp)

asyncio.run(amain())