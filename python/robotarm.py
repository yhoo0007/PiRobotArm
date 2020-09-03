import json
import math
from time import sleep
from pprint import pprint
from serial import Serial
from python.exception import InvalidConfigurationException, IKError
from python.motor import Motor
from python.motorcontroller import MotorController


class RobotArm:
    def __init__(self, config_path, verbose=False):
        '''
        Initializes the variables and objects needed for a robot arm.
        :param configs: A dictionary object containing motor configurations for the base, arm a,
        arm b, and picker motors.
        :param verbose: Flag to control verbosity.
        '''
        self.verbose = verbose
        self.loadconfig(config_path)
        self.checkpoints = dict()
        self.prev_command = RobotArm.Command('move', [self.x, self.y, self.z])
        if verbose:
            print('Robot arm initialization complete')
            pprint(self.__dict__)
            print()

    def loadconfig(self, config_path):
        '''
        Loads the config from the configuration path.
        :param config_path: Path to the configuration file.
        '''
        if self.verbose:
            print('Loading configuration from', config_path)
        with open(config_path) as config_file:
            config = json.load(config_file)

            # Initialize motor controllers
            if self.verbose:
                print('Setting up motor controllers')
            self.motor_controllers = {
                name: MotorController(name, conf) for name, conf in config['motor_controllers'].items()
            }

            # Initialize motors
            if self.verbose:
                print('Setting up motors')
            self.base_motor = Motor(
                'base_motor',
                config['motors']['base_motor'],
                self.motor_controllers[config['motors']['base_motor']['motor_controller']]
            )
            self.arm_a_motor = Motor(
                'arm_a_motor',
                config['motors']['arm_a_motor'],
                self.motor_controllers[config['motors']['arm_a_motor']['motor_controller']]
            )
            self.arm_b_motor = Motor(
                'arm_b_motor',
                config['motors']['arm_b_motor'],
                self.motor_controllers[config['motors']['arm_b_motor']['motor_controller']]
            )
            self.picker_motor = Motor(
                'picker_motor',
                config['motors']['picker_motor'],
                self.motor_controllers[config['motors']['picker_motor']['motor_controller']]
            )

            # Initialize coordinates, arm lengths, etc
            if self.verbose:
                print('Initializing additional variables')
            self.arm_a = config['arms']['arm_a']
            self.arm_b = config['arms']['arm_b']
            self.arm_a['length2'] = self.arm_a['length'] * self.arm_a['length']
            self.arm_b['length2'] = self.arm_b['length'] * self.arm_b['length']
            self.z_center_to_origin = config['z_center_to_origin']
            self.x, self.y, self.z = self.anglestocoord(
                config['motors']['base_motor']['init_angle'],
                config['motors']['arm_a_motor']['init_angle'],
                config['motors']['arm_b_motor']['init_angle']
            )

    def getstatus(self):
        '''
        Gets the status of each motor controller.
        :return: 0 if all controllers are ok, 1 otherwise.
        '''
        status = 0
        for mc in self.motor_controllers.values():
            mc_status = mc.getstatus()
            if self.verbose:
                print(f'{mc.name}: {mc_status}')
            status = max(mc_status, status)
        return status

    def getcoord(self, coord):
        '''
        Gets the value of the given coordinate.
        :param coord: One of x, y, or z.
        :return: Value of the given coordinate.
        '''
        if coord not in ('x', 'y', 'z'):
            raise ValueError('coord must be one of: x, y, z')
        return getattr(self, coord)

    def setcoord(self, coord, val):
        '''
        Sets the coordinate to the given value.
        :param coord: One of x, y, or z.
        :param val: Value to be set to.
        '''
        if coord not in ('x', 'y', 'z'):
            raise ValueError('coord must be one of: x, y, z')
        if not (isinstance(val, int) or isinstance(val, float)):
            raise ValueError('value must be type float or int')
        setattr(self, coord, val)
        if self.verbose:
            print(f'Set {coord} to {val}')

    def restart(self):
        '''
        Restarts all motor controllers.
        :return: 0 if all controllers are ok, 1 otherwise.
        '''
        if self.verbose:
            print('Restarting all motor controllers')
        for mc in self.motor_controllers.values():
            mc.restart()
        sleep(0.2)
        return self.getstatus()

    def terminate(self):
        '''
        Terminates all motor controllers.
        '''
        if self.verbose:
            print('Terminating all motor controllers')
        for mc in self.motor_controllers.values():
            mc.terminate()
        
    def enable(self, motor):
        '''
        Enables the motor.
        :param motor: Motor to be enabled.
        '''
        return getattr(self, motor).enable()
    
    def disable(self, motor):
        '''
        Disables the motor.
        :param motor: Motor to be disabled.
        '''
        return getattr(self, motor).disable()
    
    def setpin(self, motor_controller, pin, state):
        '''
        Sets the pin on the given motor controller to the given state.
        :param motor controller: Name of motor controller.
        :param pin: Pin number to be set.
        :param state: 0 or 1 representing the states of the pin.
        '''
        return self.motor_controllers[motor_controller].setpin(pin, state)

    def anglestocoord(self, base_angle, arm_a_angle, arm_b_angle):
        '''
        Forward kinematics function the coordinates of the arm given the angles of the base, arm a,
        and arm b.
        :param base_angle: The angle of the base.
        :param arm_a_angle: The angle between arm a and the horizon.
        :param_arm_b_angle: The angle between arm b and the horizon.
        :return: x, y, z coordinate values of the arm.
        '''
        theta1 = 180 - arm_b_angle - arm_a_angle
        h2 = self.arm_b['length'] * math.sin(math.radians(theta1))
        h1 = self.arm_a['length'] * math.sin(math.radians(arm_a_angle))
        z = h1 - h2 + self.z_center_to_origin
        l = self.arm_a['length'] * math.cos(math.radians(arm_a_angle)) + \
            self.arm_b['length'] * math.cos(math.radians(theta1))
        x = math.cos(math.radians(base_angle)) * l
        y = math.sin(math.radians(base_angle)) * l
        return x, y, z

    def coordtoangles(self, x, y, z):
        '''
        Inverse kinematics function to calculate the angles that each motor should be in in order
        for the arm to reach the given coordinates.
        :param x: x coordinate of destination.
        :param y: y coordinate of destination.
        :param z: z coordinate of destination.
        :return: Four values corresponding to the base angle, arm a angle, arm b angle, and picker
        angle.
        '''
        try:
            # Calculate base angle
            if abs(x) == 0:
                base_angle = 90
            elif abs(y) == 0:
                base_angle = 180 if x < 0 else 0
            else:
                base_angle = math.atan(y / x)
                base_angle = math.degrees(base_angle)
                if x < 0:
                    base_angle += 180

            # Calculate required intermediate values
            r1 = math.sqrt(x * x + y * y)
            r2 = z - self.z_center_to_origin
            r3 = math.sqrt(r1 * r1 + r2 * r2)
            r32 = r3 * r3

            # Calculate the remaining angles
            arm_a_angle = math.degrees(
                math.acos((self.arm_b['length2'] - self.arm_a['length2'] - r32) / 
                (-2 * self.arm_a['length'] * r3)) + math.atan(r2 / r1)
            )
            arm_b_angle = math.degrees(math.acos(
                (r32 - self.arm_a['length2'] - self.arm_b['length2']) / 
                (-2 * self.arm_a['length'] * self.arm_b['length'])
            ))
            picker_angle = 270 - arm_b_angle - arm_a_angle

            # Validate calculated angles and raise IKError if any angle is found to be out of range
            if any([
                base_angle < self.base_motor.min_angle, base_angle > self.base_motor.max_angle,
                arm_a_angle < self.arm_a_motor.min_angle, arm_a_angle > self.arm_a_motor.max_angle,
                arm_b_angle < self.arm_b_motor.min_angle, arm_b_angle > self.arm_b_motor.max_angle
            ]):
                raise IKError((
                    'Calculated angles\n'
                    f'base:\t{base_angle}\n'
                    f'arm a:\t{arm_a_angle}\n'
                    f'arm b:\t{arm_b_angle}\n'
                    f'picker:\t{picker_angle}\n'
                    'resultant angle out of range'
                ))
            return base_angle, arm_a_angle, arm_b_angle, picker_angle

        # Invalid mathematical operations from sin, cos, tan, etc. functions will be caught and
        # converted to an IKError
        except ValueError as e:
            raise IKError(f'IK Error {format_exc()}')

    def moveto(self, x, y, z, time=None):
        '''
        Moves the robot arm to the given coordinates.
        :param x: x coordinate of destination.
        :param y: y coordinate of destination.
        :param z: z coordinate of destination.
        :param time: Time in milliseconds the move should take to complete.
        :param wait: Flag whether the function should block until movement is complete.
        '''
        base_angle, arm_a_angle, arm_b_angle, picker_angle = self.coordtoangles(x, y, z)
        if self.verbose:
            print(
                f'Moving arm to: {x} {y} {z}\n'
                f'Angles: {base_angle} {arm_a_angle} {arm_b_angle} {picker_angle}'
            )
        self.x, self.y, self.z = x, y, z
        if time is None:
            time = max([
                    self.base_motor.calctime(base_angle),
                    self.arm_a_motor.calctime(arm_a_angle),
                    self.arm_b_motor.calctime(arm_b_angle),
                    self.picker_motor.calctime(picker_angle)
            ])
        time = round(time, 2)
        base_motor_status = self.base_motor.moveto(base_angle, time)
        arm_a_motor_status = self.arm_a_motor.moveto(arm_a_angle, time)
        arm_b_motor_status = self.arm_b_motor.moveto(arm_b_angle, time)
        picker_motor_status = self.picker_motor.moveto(picker_angle, time)
        if self.verbose:
            print(
                'Queued movements with status code',
                max([
                    base_motor_status, 
                    arm_a_motor_status, 
                    arm_b_motor_status, 
                    picker_motor_status
                    ])
                )
        for mc in self.motor_controllers.values():
            mc.moveall()
        for mc in self.motor_controllers.values():
            mc.wait()
        if self.verbose:
            print('Done')

    def execute(self, command):
        if command.type == 'status':
            print('Robot arm status:', self.getstatus())
            return
        elif command.type == 'restart':
            print('Restarting robot arm')
            print('Robot arm status:', self.restart())
            return
        elif command.type in ('x', 'y', 'z'):
            if command.args:
                self.setcoord(command.type, int(command.args[0]))
            else:
                print(getattr(self, command.type))
            return
        elif command.type in ('move', 'm'):
            x, y, z = map(int, command.args)
            self.moveto(x, y, z)  # blocking function
        elif command.type in ('enable', 'e'):
            self.enable(command.args[0])
        elif command.type in ('disable', 'd'):
            self.disable(command.args[0])
        elif command.type in ('pin', 'p'):
            self.setpin(command.args[0], command.args[1], command.args[2])
        elif command.type in ('checkpoint', 'cp'):
            if len(command.args) > 0:
                if command.args[0] == 'rm':
                    self.checkpoints.pop(int(command.args[1]))
                elif command.args[0] == 'mv':
                    self.checkpoints[int(command.args[2])] = self.checkpoints.pop(int(command.args[1]))
                elif command.args[0] == 'play':
                    print('Executing checkpoints:')
                    for index, cp in sorted(self.checkpoints.items()):
                        print(f'{index} {cp}')
                        self.execute(cp)
                else:
                    cp_num = int(command.args[0])
                    print(f'Setting checkpoint {cp_num} to prev command: {self.prev_command}')
                    self.checkpoints[cp_num] = self.prev_command
            else:
                print('Checkpoints:')
                for index, cp in self.checkpoints.items():
                    print(index, cp)
            return
        elif command.type in ('wait', 'w'):
            sleep(float(command.args[0]))
        elif command.type in ('q', 'quit'):
            self.terminate()
            raise StopIteration
        else:
            print('Unknown command')
        self.prev_command = command

    class Command:
        def __init__(self, type, args):
            self.type = type
            self.args = args
        
        def __str__(self):
            return f'{self.type} {self.args}'
        
        @staticmethod
        def getcommand():
            raw = input('> ').casefold().split(' ')
            if len(raw) >= 2:
                return RobotArm.Command(raw[0], raw[1:])
            return RobotArm.Command(raw[0], [])
