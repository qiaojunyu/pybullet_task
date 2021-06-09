from box_button import box_button_env
from robot import UR5Robotiq85
from robot import Panda
from utilities import Camera
import math
import time

if __name__ == '__main__':
    camera = Camera((1, 1, 1),
                    (0, 0, 0),
                    (0, 0, 1),
                    0.1, 5, (320, 320), 40)
    # camera = None
    robot = Panda((0, 0.0, 0), (0, 0, math.pi))
    # print(robot.joints)
    # time.sleep(10)
    control_method = 'end'
    env = box_button_env(robot, control_method, camera)

    env.reset()
    # env.SIMULATION_STEP_DELAY = 0
    # action = [0.1, 0.1, 0.2, 0.1, 0.2, 0.3, 0.01]
    while True:
        action = env.read_debug_parameter()
        obs, reward, done, info = env.step(action)
        print(obs, reward, done, info)




