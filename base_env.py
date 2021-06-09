import pybullet as p
from abc import ABC, abstractmethod
import pybullet_data

class base_environment(ABC):

    def __init__(self,
                 robot,
                 control_method='end',
                 camera=None,
                 use_gui=True,
                 ):
        if use_gui:
            self.server_id = p.connect(p.GUI)
        else:
            self.server_id = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)
        self.robot = robot
        self.robot.load()
        self.control_method = control_method
        self.camera = camera
        assert control_method in ('joint', 'end')
        self.robot_action_space = None
        self.gripper_action_apace = None
        self.observation_space = None
        self._set_action_space()
        self.observation_space = dict()
        self._set_observation_space()
        # self._int_env()

    def step(self, actions):
        self._set_action(actions)
        for i in range(120):
            p.stepSimulation()
        done = self._is_done()
        return self._get_reward(), self._get_observation(), done, self._get_info()

    @abstractmethod
    def _set_action(self, actions):
        pass

    @abstractmethod
    def _get_reward(self):
        pass

    @abstractmethod
    def _get_observation(self):
        pass

    @abstractmethod
    def _set_action_space(self):
        pass

    @abstractmethod
    def _is_done(self):
        pass

    @abstractmethod
    def _get_info(self):
        pass

    @abstractmethod
    def _set_observation_space(self):
        pass

    @abstractmethod
    def _object_reset(self):
        pass

    @abstractmethod
    def _init_env(self):
        pass

    def reset(self):
        self._object_reset()
        self.robot.reset()
        p.stepSimulation()
        return self._get_observation()

    def close(self):
        p.disconnect(self.server_id)







