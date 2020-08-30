import math
from pprint import pformat


class Motor:
    def __init__(self, name, config, motor_controller):
        '''
        Motor class which controls and keeps track of a motor on a motor controller.
        :param config: A dictionary object with the keys: controller_channel, name, serial_port,
        init_angle, microstep, ratio, max_time, min_time, max_angle, min_angle.
        '''
        self.name = name

        # Get motor controller information
        self.motor_controller = motor_controller
        self.controller_channel = config['controller_channel']
        
        # Get motor parameters
        self.microstep = config['microstep']
        self.ratio = config['ratio']

        # Get angle parameters
        self.min_angle = config['min_angle']
        self.max_angle = config['max_angle']
        self.angle = config['init_angle']
        self.step_range = self.angletosteps(self.max_angle - self.min_angle)

        # Get timing parameters
        self.max_time = config['max_time']        
        self.min_time = config['min_time']
        self.time_range = self.max_time - self.min_time
    
    def __repr__(self):
        return pformat(self.__dict__)

    def angletosteps(self, angle):
        '''
        :param angle: Angle to be converted to steps.
        :return: The number of steps to pulse in order to move the motor the given angle.
        '''
        return int(angle / self.microstep * self.ratio)
    
    def stepstoangle(self, steps):
        '''
        :param steps: Number of steps to be converted to angle.
        :return: The angle that corresponds to the given number of steps.
        '''
        return steps / self.ratio * self.microstep

    def calctime(self, angle):
        '''
        Returns the amount of time the motor should take to move the given angle. Calculation is
        based on a sine function which maps angle to time.
        :param angle: The angle to move.
        :return: The amount of time the motor should take to move the given angle.
        '''
        steps = self.angletosteps(angle)
        time = self.time_range * math.sin((abs(steps) * math.pi) / (self.step_range * 2))
        if time < self.min_time:
            time = self.min_time
        elif time > self.max_time:
            time = self.max_time
        return time

    def moveto(self, angle, time):
        '''
        Queues a move of the given angle in the given time.
        :param angle: The angle to move to with reference to the initial angle of the motor.
        :param time: The amount of time the move should take.
        '''
        angle_to_move = angle - self.angle
        steps = self.angletosteps(angle_to_move)
        self.angle += self.stepstoangle(steps)
        print(f'Moving {self.name} on channel: {self.controller_channel} {steps} in {time}')
        steps_status = self.motor_controller.setsteps(self.controller_channel, steps)
        time_status = self.motor_controller.settime(self.controller_channel, time)
        return max(steps_status, time_status)

    def enable(self):
        '''
        Enables the motor.
        :return: Response code from the motor controller.
        '''
        return self.motor_controller.enable(self.controller_channel)
    
    def disable(self):
        '''
        Disables the motor.
        :return: Response code from the motor controller.
        '''
        return self.motor_controller.disable(self.controller_channel)

    def getstatus(self):
        '''
        Requests and retrieves the status of the motor from the serial port.
        :return: Response code from the motor controller.
        '''
        return self.motor_controller.getstatus()

