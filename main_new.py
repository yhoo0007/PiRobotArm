from traceback import format_exc
from python.robotarm import RobotArm
# from python.command import Command


CONFIG_PATH = './python/robotarmconfig.json'


if __name__ == '__main__':
    try:
        print('Initializing robot arm')
        # Raspberry Pi ports: /dev/ttyUSB0, /dev/ttyUSB1
        # Windows ports: COM3, COM4
        robot_arm = RobotArm(CONFIG_PATH, verbose=True)

        while True:
            print()
            command = robot_arm.Command.getcommand()
            print(command)
            robot_arm.execute(command)
            
    except Exception as e:
        print(format_exc())
