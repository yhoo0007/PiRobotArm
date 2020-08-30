import serial
import math
from traceback import format_exc
from time import sleep


# PORT1 = '/dev/ttyUSB1'
# PORT1 = 'COM3'
BAUD1 = 115200
TIMEOUT1 = 5

# PORT2 = '/dev/ttyUSB0'
# PORT2 = 'COM4'
BAUD2 = 115200
TIMEOUT2 = 5

def openport(port, baud, timeout):
    port = serial.Serial(port, baud, timeout=timeout)
    port.reset_output_buffer()
    port.reset_input_buffer()
    return port

def getcommand():
    '''
    Receives input from the user and parses it into a command and arguments.
    :return: command and arguments as strings.
    '''
    raw = input('> ').split(' ')
    if len(raw) > 2:
        return raw[0], raw[1:]
    return raw[0], []


if __name__ == "__main__":
    print('Opening serial ports')
    robot_arm = RobotArm(
        base_motor_config={
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
        arm_a_motor_config={
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
        arm_b_motor_config={
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
        picker_motor_config={
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
    )

    while True:
        print('Getting status of motor controller')
        if robot_arm.getstatus() == 0:
            print('Motor controller OK')
            break
        print('Motor controller error')
        sleep(1)
        print('Retrying...')

    print('Ready')
    while True:
        command, args = getcommand()
        command = command.casefold()
        args = list(map(str.casefold, args))
        print('Running:', command, 'with args:', args)
        if command == 'status':
            robot_arm.getstatus()
        elif command == 'move':
            try:
                if len(args) == 3:
                    x, y, z = map(int, args)
                    robot_arm.moveto(x, y, z)
                elif len(args) == 4:
                    x, y, z, t = map(int, args)
                    robot_arm.moveto(x, y, z, time=t)
                else:
                    print('Invalid arguments provided')
            except IKError as e:
                print(e)
            except ValueError:
                print(format_exc())
        elif command == 'set':
            try:
                pin, state = map(int, args)
                robot_arm.set(pin, state)
            except ValueError:
                print('Invalid arguments provided')

        elif command in ('x', 'y', 'z'):
            try:
                val = int(args[0])
                robot_arm.setcoord(command, val)
            except ValueError:
                print('Invalid argument provided')
        elif command in ('enable', 'disable'):
            try:
                if getattr(robot_arm, command)(arg[0]) != 0:
                    print(f'Error enabling {arg[0]}' if command == 'enable' else f'Error disabling {arg[0]}')
            except IndexError:
                print('No motor argument provided')
        elif command == 'current':
            print((
                'Current coordinates:\n'
                f'x:\t{robot_arm.x}\n'
                f'y:\t{robot_arm.y}\n'
                f'z:\t{robot_arm.z}'
            ))
            print((
                'Current angles:\n'
                f'base:\t{robot_arm.base_motor.angle}\n'
                f'arm a:\t{robot_arm.arm_a_motor.angle}\n'
                f'arm b:\t{robot_arm.arm_b_motor.angle}'
            ))
        elif command == 'restart':
            print('Restarting robot arm')
            robot_arm.restart()
        elif command == 'readall':
            print('Serial 1:', ser1.read_until('\r\n\r\n').decode())
            print('Serial 2:', ser2.read_until('\r\n\r\n').decode())
        elif command.casefold() in ('quit', 'q'):
            robot_arm.terminate()
            del(robot_arm)
            break
        else:
            print('Invalid command')
