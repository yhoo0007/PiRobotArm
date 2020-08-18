import serial
import math
import sys
from time import sleep


class IKError(Exception):
    pass

class Motor:
    def __init__(self, config):
        self.channel = config['channel']
        self.name = config['name']
        self.serial_port = config['serial_port']
        self.angle = config['init_angle']
        self.microstep = config['microstep']
        self.ratio = config['ratio']
        self.time_range = config['max_time'] - config['min_time']
        self.step_range = self.angletosteps(config['max_angle'] - config['min_angle'])
    
    def angletosteps(self, angle):
        return int(angle / self.microstep * self.ratio)
    
    def stepstoangle(self, steps):
        return steps / self.ratio * self.microstep

    def calctime(self, angle):
        steps = self.angletosteps(angle)
        return self.time_range * math.sin((abs(steps) * math.pi) / (self.step_range * 2)) + 100
    
    def moveto(self, angle, time):
        angle_to_move = angle - self.angle
        steps = self.angletosteps(angle_to_move)
        self.angle += self.stepstoangle(steps)
        print(f'Moving {self.name} on channel: {self.channel} {steps} in {time}')
        self.serial_port.reset_input_buffer()
        
        steps_str = f'S{self.channel}{steps}\n'
        print(repr(steps_str))
        self.serial_port.write(steps_str.encode())
        self.awaitdone()

        time_str = f'T{self.channel}{time}\n'
        print(repr(time_str))
        self.serial_port.write(time_str.encode())
        self.awaitdone()

    def awaitdone(self):
        return self.serial_port.read_until(terminator=b'0\r\n\r\n')

    def getstatus(self):
        self.serial_port.reset_input_buffer()
        self.serial_port.write(b'?\n')
        status = int(readDecode(self.serial_port).strip()[-1])
        return status


def readDecode(ser):
    ret = ser.read(size=5)
    return ret.decode()


class RobotArm:
    ARM_A = 320
    ARM_B = 270
    Z_CENTER_TO_ORIGIN = 50

    BASE_MIN_ANGLE = -45
    BASE_MAX_ANGLE = 225

    ARM_A_MIN_ANGLE = 0
    ARM_A_MAX_ANGLE = 90

    ARM_B_MIN_ANGLE = 20
    ARM_B_MAX_ANGLE = 180

    PICKER_MIN_ANGLE = 0
    PICKER_MAX_ANGLE = 360


    def __init__(self, configs):
        self.base_motor = Motor(configs[0])
        self.arm_a_motor = Motor(configs[1])
        self.arm_b_motor = Motor(configs[2])
        self.picker_motor = Motor(configs[3])

        self.serial_ports = list(set([c['serial_port'] for c in configs]))

        self.x = 0  # TODO:: forward kinematic calculate coordinate from angles
        self.y = 226
        self.z = 35
    
    def getstatus(self):
        base_motor_status = self.base_motor.getstatus()
        arm_a_motor_status = self.arm_a_motor.getstatus()
        arm_b_motor_status = self.arm_b_motor.getstatus()
        picker_status = self.picker_motor.getstatus()
        print('Base motor status:', base_motor_status)
        print('Arm A motor status:', arm_a_motor_status)
        print('Arm B motor status:', arm_b_motor_status)
        print('Picker motor status:', picker_status)
        return 1 if any([base_motor_status, arm_a_motor_status, arm_b_motor_status, picker_status]) else 0
    
    def moveto(self, x, y, z, time=None, wait=False):
        base_angle, arm_a_angle, arm_b_angle, picker_angle = self.coordtoangle(x, y, z)  # throws ValueError in IK error
        print(f'Calculated angles:\nbase:\t{base_angle}\narm a:\t{arm_a_angle}\narm b:\t{arm_b_angle}\npicker:\t{picker_angle}')
        self.x = x
        self.y = y
        self.z = z

        if time is None:
            time = max(
                [
                    self.base_motor.calctime(base_angle),
                    self.arm_a_motor.calctime(arm_a_angle),
                    self.arm_b_motor.calctime(arm_b_angle),
                    self.picker_motor.calctime(picker_angle)
                ]
            )

        self.base_motor.moveto(base_angle, time)
        self.arm_a_motor.moveto(arm_a_angle, time)
        self.arm_b_motor.moveto(arm_b_angle, time)
        self.picker_motor.moveto(picker_angle, time)

        for port in self.serial_ports:  # start all channels on all motor controllers
            print(repr('G\n'))
            port.write(b'G\n')
        
        if wait:
            for port in self.serial_ports:
                print(port.read_until(terminator=b'0\r\n\r\n'))

    
    def coordtoangle(self, x, y, z):
        try:
            if abs(x) == 0:
                base_angle = 90
            elif abs(y) == 0:
                base_angle = 180 if x < 0 else 0
            else:
                base_angle = math.atan(y / x)
                base_angle = math.degrees(base_angle)
            
            r1 = math.sqrt(x * x + y * y)
            r2 = z - RobotArm.Z_CENTER_TO_ORIGIN
            r3 = math.sqrt(r1 * r1 + r2 * r2)
            ARM_A2 = RobotArm.ARM_A * RobotArm.ARM_A
            ARM_B2 = RobotArm.ARM_B * RobotArm.ARM_B
            arm_a_angle = math.degrees(
                math.acos((ARM_B2 - ARM_A2 - r3 * r3) / (-2 * RobotArm.ARM_A * r3) + \
                math.atan(r2 / r1))
            )

            arm_b_angle = math.degrees(
                math.acos((r3 * r3 - ARM_A2 - ARM_B2) / (-2 * RobotArm.ARM_A * RobotArm.ARM_B))
            )

            picker_angle = 270 - arm_b_angle - arm_a_angle

            if (base_angle < RobotArm.BASE_MIN_ANGLE or base_angle > RobotArm.BASE_MAX_ANGLE or\
                arm_a_angle < RobotArm.ARM_A_MIN_ANGLE or arm_a_angle > RobotArm.ARM_A_MAX_ANGLE or\
                arm_b_angle < RobotArm.ARM_B_MIN_ANGLE or arm_b_angle > RobotArm.ARM_B_MAX_ANGLE):
                    raise IKError('resultant angle out of range')
            return base_angle, arm_a_angle, arm_b_angle, picker_angle
        except ValueError as e:
            raise IKError(f'IK Error {str(e)}')


# PORT1 = '/dev/ttyUSB1'
# PORT1 = 'COM3'
BAUD1 = 115200
TIMEOUT1 = 5

# PORT2 = '/dev/ttyUSB0'
# PORT2 = 'COM4'
BAUD2 = 115200
TIMEOUT2 = 5


def getcommand():
    raw = input('> ').split(' ', maxsplit=1)
    if len(raw) > 1:
        command, args = raw
    else:
        command = raw[0]
        args = None
    return command, args


if __name__ == "__main__":
    port1, port2 = sys.argv[1], sys.argv[2]
    print('Opening serial port')
    ser1 = serial.Serial(port1, BAUD1, timeout=TIMEOUT1)
    ser1.reset_output_buffer()
    ser1.reset_input_buffer()

    ser2 = serial.Serial(port2, BAUD2, timeout=TIMEOUT2)
    ser2.reset_output_buffer()
    ser2.reset_input_buffer()
    robot_arm = RobotArm(
        configs=[
            {
                'channel': 0,
                'name': 'Base motor',
                'serial_port': ser1,
                'init_angle': 90,
                'microstep': 0.225,
                'ratio': 7.143,
                'max_time': 3000,
                'min_time': 100,
                'max_angle': RobotArm.BASE_MAX_ANGLE,
                'min_angle': RobotArm.BASE_MIN_ANGLE
            },
            {
                'channel': 1,
                'name': 'Arm A motor',
                'serial_port': ser1,
                'init_angle': 45,
                'microstep': 0.225,
                'ratio': 50,
                'max_time': 3000,
                'min_time': 300,
                'max_angle': RobotArm.ARM_A_MAX_ANGLE,
                'min_angle': RobotArm.ARM_A_MIN_ANGLE
            },
            {
                'channel': 2,
                'name': 'Arm B motor',
                'serial_port': ser1,
                'init_angle': 90,
                'microstep': 0.225,
                'ratio': 50,
                'max_time': 3000,
                'min_time': 300,
                'max_angle': RobotArm.ARM_B_MAX_ANGLE,
                'min_angle': RobotArm.ARM_B_MIN_ANGLE
            },
            {
                'channel': 0,
                'name': 'Picker motor',
                'serial_port': ser2,
                'init_angle': 270 - 90 - 45,
                'microstep': 0.225,
                'ratio': 1,
                'max_time': 3000,
                'min_time': 10,
                'max_angle': RobotArm.PICKER_MAX_ANGLE,
                'min_angle': RobotArm.PICKER_MIN_ANGLE
            }
        ]
    )

    while True:
        print('Getting status of motor controller')
        status = robot_arm.getstatus()
        if status == 0:
            print('Motor controller OK')
            break
        else:
            print('Motor controller error')
            sleep(1)
    
    print('Ready')
    command = ''
    while True:
        command, args = getcommand()
        print('Running:', command, 'with args:', args)
        if command == 'status':
            print(robot_arm.getstatus())
        elif command == 'move':
            try:
                x, y, z = map(int, args.split())
                try:
                    robot_arm.moveto(x, y, z, wait=True)
                except IKError as e:
                    print(e)
            except ValueError:
                try:
                    x, y, z, t = map(int, args.split())
                    try:
                        robot_arm.moveto(x, y, z, time=t, wait=True)
                    except IKError as e:
                        print(e)
                except ValueError:
                    print('Invalid arguments provided')
        elif command == 'current':
            print(f'Current positions:\nx:\t{robot_arm.x}\ny:\t{robot_arm.y}\nz:\t{robot_arm.z}')
        elif command == 'readall':
            print('Serial 1:', ser1.read_until('\r\n\r\n').decode())
            print('Serial 2:', ser2.read_until('\r\n\r\n').decode())
        elif command.casefold() in ('quit', 'q'):
            ser1.close()
            ser2.close()
            break
        else:
            print('Unknown command')
