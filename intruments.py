import numpy as np 
import pyvisa
import keyoscacquire as koa
import serial
import time


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
        self._controller = serial.Serial(port=self.port,
                                        baudrate=115200,
                                        parity=serial.PARITY_NONE,
                                        stopbits=serial.STOPBITS_ONE,
                                        bytesize=serial.EIGHTBITS)
        self._connected = True
        self._turned_on = False
        self._current = 0 # mA/s
        self._thermistor_resistance = None
        self.current_change_rate = 50 # mA/s
        self.SETTLING_TIME = 0.1
        time.sleep(self.SETTLING_TIME)

    def _send_command(self, input_str):
        self._controller.write(str.encode(input_str + '\r'))
        time.sleep(self.SETTLING_TIME)

    def _read_command(self,skip=-2): # use -2 after query commands (eg rtact, iphd), use -1 after set commands (eg ilaser)
        """
        Function to read a Koheron digital laser controller from serial BUS
        Use skip=-2 after query commands (eg rtact, iphd)
        Use skip=-1 after set commands (eg ilaser)
        Remember to put some waiting time before the read command
        """
        out = ''
        while self._controller.inWaiting() > 0:
            out += self._controller.read(1).decode('utf-8')
        
        out = out.split()[skip]
        return out

    def set_current(self, current):
        if self._turned_on:
            current_steps = np.linspace(self._current, current, int(np.abs(self._current-current)/self.current_change_rate/self.SETTLING_TIME))
            for current_step in current_steps:
                self._controller.write(str.encode('ilaser ' + str(current_step) + '\r'))
                self._current = current_step
                time.sleep(self.SETTLING_TIME) 
            self._current = current         
        else:
            self._current = current
        
    def set_thermistor_resistance(self, thermistor_resistance):
        self._controller.write(str.encode('rtset ' + str(thermistor_resistance) + '\r'))
        self._thermistor_resistance = thermistor_resistance

    def get_thermistor_resistance(self):
        self._send_command('rtact')
        time.sleep(self.SETTLING_TIME)
        self._thermistor_resistance = self._read_command(skip=-2)
        return self._thermistor_resistance
        
    def laser_on(self):
        current = self._current
        self._current = 0
        self._send_command('ilaser ' + str(self._current))
        self._send_command('lason 1')
        self._turned_on = bool(self._read_command(skip=-2))
        self.set_current(current=current)
        self._current = current
        time.sleep(self.SETTLING_TIME)
        
        return self._turned_on, self._current

    def laser_off(self):
        self._send_command('lason 0')
        time.sleep(self.SETTLING_TIME)
        self._turned_on = bool(self._read_command(skip=-2))

        return self._turned_on
        
    def get_phd(self):
        self._send_command('iphd')
        time.sleep(self.SETTLING_TIME)

        return self._read_command(skip=-2)
        
    def get_current(self):
        self._send_command('ilaser')
        time.sleep(self.SETTLING_TIME)

        return self._read_command(skip=-2)
        
    def get_voltage(self):
        self._send_command('vlaser')
        time.sleep(self.SETTLING_TIME)

        return self._read_command(skip=-2)

    def disconnect(self):
        if self._connected:
            self._controller.close()
            self._connected = False
            print("Disconnected.")
        else:
            print("Already disconnected.")
    
    def connect(self):
        if not self._connected:
            self._controller = serial.Serial(port=self.port,
                                            baudrate=115200,
                                            parity=serial.PARITY_NONE,
                                            stopbits=serial.STOPBITS_ONE,
                                            bytesize=serial.EIGHTBITS)
            self._connected = True
            self._turned_on = False
            self._current = 0 # mA/s
            self._thermistor_resistance = None
            print("Connected.")
        else:
            print("Already connected.")


