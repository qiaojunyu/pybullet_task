from rl_env import rl_environment
import pybullet as p
import numpy as np

class box_button_env(rl_environment):

    def __init__(self,
                 robot,
                 control_method='end',
                 camera=None,
                 use_gui=True,
                 ):
        super(rl_environment, self).__init__(
            robot,
            control_method,
            camera,
            use_gui,
        )
        self._set_action_space()
        self._set_observation_space()
        self.table_id = None
        self.planeUid = None
        self.boxID = None
        self.xin = None
        self.yin = None
        self.zin = None
        self.rollId = None
        self.pitchId = None
        self.yawId = None
        self.gripper_opening_length_control = None
        self.debug = True

        self._init_env()
        self.box_opened = False
        self.btn_pressed = False
        self.box_closed = False

    def _init_env(self):
        self.table_id = p.loadURDF("table/table.urdf", basePosition=[0.1, 0.0, -0.65])
        self.planeUid = p.loadURDF("plane.urdf", basePosition=[0, 0, -0.65])
        self.boxID = p.loadURDF("urdf/skew-box-button.urdf", basePosition=[0.65, 0.1, 0])
        self.xin = p.addUserDebugParameter("x", -0.7, 0.7, 0)
        self.yin = p.addUserDebugParameter("y", -0.7, 0.7, 0)
        self.zin = p.addUserDebugParameter("z", 0, 1., 0.5)
        self.rollId = p.addUserDebugParameter("roll", -3.14, 3.14, 0)
        self.pitchId = p.addUserDebugParameter("pitch", -3.14, 3.14, np.pi/2)
        self.yawId = p.addUserDebugParameter("yaw", -np.pi / 2, np.pi / 2, np.pi / 2)
        self.gripper_opening_length_control = p.addUserDebugParameter("gripper_opening_length", 0, 0.04, 0.04)

    def read_debug_parameter(self):
        # read the value of task parameter
        x = p.readUserDebugParameter(self.xin)
        y = p.readUserDebugParameter(self.yin)
        z = p.readUserDebugParameter(self.zin)
        roll = p.readUserDebugParameter(self.rollId)
        pitch = p.readUserDebugParameter(self.pitchId)
        yaw = p.readUserDebugParameter(self.yawId)
        gripper_opening_length = p.readUserDebugParameter(self.gripper_opening_length_control)

        return x, y, z, roll, pitch, yaw, gripper_opening_length


    def _get_reward(self):
        reward = 0
        if not self.box_opened:
            if p.getJointState(self.boxID, 1)[0] > 1.9:
                self.box_opened = True
                print('Box opened!')
        elif not self.btn_pressed:
            if p.getJointState(self.boxID, 0)[0] < - 0.02:
                self.btn_pressed = True
                print('Btn pressed!')
        else:
            if p.getJointState(self.boxID, 1)[0] < 0.1:
                print('Box closed!')
                self.box_closed = True
                reward = 1
        return reward

    def _object_reset(self):
        p.setJointMotorControl2(self.boxID, 0, p.POSITION_CONTROL, force=1)
        p.setJointMotorControl2(self.boxID, 1, p.VELOCITY_CONTROL, force=0)

    def _get_info(self):
        return dict(box_opened=self.box_opened, btn_pressed=self.btn_pressed, box_closed=self.box_closed)

    def _is_done(self):
        if self.box_opened and self.btn_pressed and self.box_closed:
            return True
        else:
            return False


















