from pprint import pformat
from time import sleep
from serial import Serial


class MotorController:
    def __init__(self, name, config):
        self.name = name

        # Get serial port parameters and open serial port
        self.port = config['port']
        self.baud = config['baud']
        self.timeout = config['timeout']
        self.serial_port = Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=self.timeout
        )

        # Test connection
        self.getstatus()
    
    def _readuntiltermination(self):
        term = self.serial_port.read_until(b'\r\n\r\n')
        return term

    def _clearallbuffers(self):
        self.serial_port.flush()
        self.serial_port.reset_input_buffer()

    def _sendreturn(self, bytes):
        self._clearallbuffers()
        self.serial_port.write(bytes)
        self.serial_port.flush()
        recv = self._readuntiltermination()
        # print(bytes, recv)
        return int(recv)
    
    def _send(self, bytes):
        self._clearallbuffers()
        self.serial_port.write(bytes)
        self.serial_port.flush()
    
    def wait(self):
        '''
        Waits for a status code response from the motor controller.
        :return: Status code from the motor controller.
        '''
        status = self._readuntiltermination()
        try:
            return int(status)
        except ValueError:
            return 1

    def getstatus(self):
        '''
        Gets the status of the motor controller.
        :return: 0 if ok, 1 otherwise.
        '''
        return self._sendreturn(b'?\r\n\r\n')
    
    def restart(self):
        '''
        Restarts the motor controller.
        :return: Status of the motor controller after restarting.
        '''
        status = self._sendreturn(b'R\r\n\r\n')
        if status != 0:
            raise Exception('Error restarting controller')
        sleep(0.2)
        status = self.getstatus()
        return status
    
    def terminate(self):
        '''
        Resets the motor controllers.
        '''
        self.restart()
        self.serial_port.close()
    
    def setsteps(self, channel, steps):
        '''
        Sets the steps value for a channel.
        '''
        to_send = f'S {channel} {steps}\r\n\r\n'.encode()
        return self._sendreturn(to_send)
    
    def settime(self, channel, time):
        '''
        Sets the time value for a channel.
        '''
        to_send = f'T {channel} {time}\r\n\r\n'.encode()
        return self._sendreturn(to_send)
    
    def enable(self, channel):
        '''
        Enables the channel.
        '''
        to_send = f'E {channel}\r\n\r\n'.encode()
        return self._sendreturn(to_send)

    def disable(self, channel):
        '''
        Disables the channel.
        '''
        to_send = f'D {channel}\r\n\r\n'.encode()
        return self._sendreturn(to_send)
    
    def setpin(self, pin, state):
        '''
        '''
        to_send = f'P {pin} {state}\r\n\r\n'.encode()
        return self._sendreturn(to_send)

    def moveall(self, wait=False):
        '''
        Executes all movements on the motor controller.
        '''
        to_send = b'G\r\n\r\n'
        if wait:
            return self._return()
        self._send(to_send)
    
    def move(self, channel, wait=False):
        '''
        Executes movement on a specific channel of the motor controller.
        :param channel: Channel number
        '''
        to_send = f'G {channel}\r\n\r\n'.encode()
        if wait:
            return self._sendreturn(to_send)
        self._send(to_send)
