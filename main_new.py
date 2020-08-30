from traceback import format_exc
from python.robotarm import RobotArm


CONFIG_PATH = './python/robotarmconfig.json'


def getcommand():
    '''
    Receives input from the user and parses it into a command and arguments. All returned values
    are casefolded.
    :return: command and arguments as strings.
    '''
    raw = input('> ').casefold().split(' ')
    if len(raw) >= 2:
        return raw[0], raw[1:]
    return raw[0], []


if __name__ == '__main__':
    try:
        print('Initializing robot arm')
        # Raspberry Pi ports: /dev/ttyUSB0, /dev/ttyUSB1
        # Windows ports: COM3, COM4
        robot_arm = RobotArm(CONFIG_PATH, verbose=True)

        while True:
            print()
            command, args = getcommand()
            print(command, args)
            if command == 'status':
                print('Robot arm status:', robot_arm.getstatus())
            elif command == 'restart':
                print('Restarting robot arm')
                print('Robot arm status:', robot_arm.restart())
            elif command in ('x', 'y', 'z'):
                if args:
                    robot_arm.setcoord(command, int(args[0]))
                else:
                    print(getattr(robot_arm, command))
            elif command in ('move', 'm'):
                x, y, z = map(int, args)
                robot_arm.moveto(x, y, z)  # blocking function
            elif command in ('enable', 'e'):
                robot_arm.enable(args[0])
            elif command in ('disable', 'd'):
                robot_arm.disable(args[0])
            elif command in ('pin', 'p'):
                robot_arm.setpin(args[0], args[1], args[2])
            
            elif command in ('q', 'quit'):
                robot_arm.terminate()
                break
            else:
                print('Unknown command')
    except Exception as e:
        print(format_exc())
