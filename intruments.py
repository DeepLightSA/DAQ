import numpy as np 
import pyvisa
import keyoscacquire as koa
import serial


class InstrumentBase:

    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

    def get_address(self):
        return self.address


class Oscilloscope(InstrumentBase):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    
class KeySightOSC(Oscilloscope):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.scope = koa.Oscilloscope(address=self.address)
    
    def get_traces(self):
        return self.scope.get_trace(channels='active')

    def get_trace_from_channels(self, channels=[1, 2]):
        return self.scope.get_trace(channels=channels)


class LaserController(InstrumentBase):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
   

class KoheronCTL200(LaserController):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.controller = serial.Serial(port=self.port,
                                        baudrate=115200,
                                        parity=serial.PARITY_NONE,
                                        stopbits=serial.STOPBITS_ONE,
                                        bytesize=serial.EIGHTBITS)
