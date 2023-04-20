import asyncio

from motoplus_rr_driver_client import MotoPlusRRDriverReverseSocketClient

RR_MOTOPLUS_BUFFER_SERVER_PORT=24580
RR_MOTOPLUS_BUFFER_MAGIC=0x42415252

RR_MOTOPLUS_BUFOP_NOOP=0x0
RR_MOTOPLUS_BUFOP_BEGIN_WRITE=0x1
RR_MOTOPLUS_BUFOP_WRITE_SEG=0x2
RR_MOTOPLUS_BUFOP_FINISH_WRITE=0x3
RR_MOTOPLUS_BUFOP_RESET=0x4

class MotoPlusRRDriverBufferClient(MotoPlusRRDriverReverseSocketClient):

    def __init__(self):
        super().__init__(RR_MOTOPLUS_BUFFER_MAGIC)

    def start(self, host, port = RR_MOTOPLUS_BUFFER_SERVER_PORT):
        return super().start(host, port)
    
    def start_reverse(self, port = RR_MOTOPLUS_BUFFER_SERVER_PORT):
        return super().start_reverse(port)

    def _res_check_error(self, res):
        if res.param1 != 0:
            raise Exception(f"Buffer returned error: {res.param1}")

    async def _send_begin_write(self, buf_index, len_words):        
        res, payload = await self.send_request(RR_MOTOPLUS_BUFOP_BEGIN_WRITE, buf_index, param1=len_words)
        self._res_check_error(res)

    async def _send_finish_write(self, buf_index, len_words):
        res, payload = await self.send_request(RR_MOTOPLUS_BUFOP_FINISH_WRITE, buf_index, param1=len_words)
        self._res_check_error(res)

    async def _send_reset(self, buf_index):
        res, payload = await self.send_request(RR_MOTOPLUS_BUFOP_RESET, buf_index)
        self._res_check_error(res)

    async def _send_segment(self, buf_index, offset, data):
        res, payload = await self.send_request(RR_MOTOPLUS_BUFOP_WRITE_SEG, buf_index, param1=offset, payload=data)
        self._res_check_error(res)

    async def set_buffer(self, buf_index, data):
        self._verify_payload(data)
        len_data = len(data)
        await self._send_reset(buf_index)
        await self._send_begin_write(buf_index, len_data)
        # Send 1000 words per transfer to avoid bogging down controller
        pos = 0
        while pos < len_data:
            l = min(len_data - pos, 1000)
            await self._send_segment(buf_index, pos, data[pos:pos+l])
            pos += l
        await self._send_finish_write(buf_index, len(data))
