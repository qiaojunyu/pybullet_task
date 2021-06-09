import pybullet as p
from abc import abstractmethod
from gym import spaces
import numpy as np
import math
from base_env import base_environment

class rl_environment(base_environment):

    def __init__(self,
                 robot,
                 control_method='end',
                 camera=None,
                 use_gui=True,
                 ):
        super(rl_environment, self).__init__(
            robot,
            control_method='end',
            camera=None,
            use_gui=True,
        )
        self._set_action_space()
        self._set_observation_space()

    def step(self, actions):
        self._set_action(actions)
        for i in range(120):
            p.stepSimulation()
        return self._get_reward(), self._get_observation(), self._is_done(), self._get_info()


    def _set_action_space(self):
        if self.control_method == "end":
            action_space_low_limits = [-1.0]*3
            action_space_high_limits = [1.0]*3

            action_space_low_limits.append(-math.pi)
            action_space_low_limits.append(-math.pi)
            action_space_low_limits.append(-math.pi)

            action_space_high_limits.append(math.pi)
            action_space_high_limits.append(math.pi)
            action_space_high_limits.append(math.pi)

            action_space_low_limits.append(-self.robot.gripper_range[1])
            action_space_high_limits.append(self.robot.gripper_range[1])
            print(np.array(action_space_low_limits))
            self.action_space = spaces.Box(np.array(action_space_low_limits), np.array(action_space_high_limits))
            self.robot_action_space = 6
            self.gripper_action_apace = 2

        if self.control_method == 'joint':
            xyz_low_limits = [-1]*len(self.robot.arm_upper_limits)
            xyz_high_limits = [1]*len(self.robot.arm_upper_limits)
            xyz_low_limits.append(-self.robot.gripper_range[1])
            xyz_high_limits.append(self.robot.gripper_range[1])
            self.action_space = spaces.Box(np.array(xyz_low_limits), np.array(xyz_high_limits))
            self.robot_action_space = self.robot.arm_num_dofs
            self.gripper_action_apace = 1

    def _set_observation_space(self):
        if self.camera is not None:
            self.observation_space["rgb"] = spaces.Box(low=0, high=255, shape=(self.camera.height, self.camera.height, 3), dtype=np.uint8)
            self.observation_space["depth"] = spaces.Box(low=0, high=100, shape=(self.camera.height, self.camera.height, 3), dtype=np.uint8)
            self.observation_space["seg"] = spaces.Box(low=0, high=100, shape=(self.camera.height, self.camera.height, 1), dtype=np.uint8)

        if self.control_method == "end":
            xyz_low_limits = [-10, -10, -10]
            xyz_high_limits = [10, 10, 10]
            xyz_low_limits.append(-self.robot.gripper_range[1])
            xyz_high_limits.append(self.robot.gripper_range[1])
            self.observation_space["robot_obs"] = spaces.Box(np.array(xyz_low_limits), np.array(xyz_high_limits))

        if self.control_method == 'joint':
            xyz_low_limits = self.robot.arm_lower_limits
            xyz_high_limits = self.robot.arm_upper_limits
            xyz_low_limits.append(-self.robot.gripper_range[1])
            xyz_high_limits.append(self.robot.gripper_range[1])
            self.observation_space["robot_obs"] = spaces.Box(np.array(xyz_low_limits), np.array(xyz_high_limits))

    def _get_observation(self):
        obs = dict()
        if self.camera is not None:
            rgb, depth, seg = self.camera.shot()
            print(type(depth))
            depth = self.camera.rgbd_2_world_batch(depth)
            obs.update(dict(rgb=rgb, depth=depth, seg=seg))
        ee_pos = []
        if self.control_method == "end":
            xyz = p.getLinkState(self.robot.id, self.robot.eef_id)[4]
            ee_pos = [pos_i for pos_i in xyz]
        else:
            ee_pos, _ = self.robot.get_joint_info()
        gripper_obs = p.getJointState(self.robot.id, self.robot.eef_id)[0]
        ee_pos.append(gripper_obs)
        obs.update(dict(robot_obs=np.array(ee_pos)))
        return obs

    def _set_action(self, actions):
        if self.control_method == "end" and not self.debug:
            currentPose = p.getLinkState(self.robot.id, self.robot.eef_id)
            currentPosition = currentPose[4]
            robot_action = actions[0:self.robot_action_space]
            dv = 0.01
            for i in range(len(currentPosition)):
                robot_action[i] = currentPosition[i] + robot_action[i]*dv
            self.robot.move_ee(robot_action, self.control_method)
            gripper_action = abs(actions[-1])
            self.robot.move_gripper(gripper_action)

        if self.control_method == "end" and self.debug:
            robot_action = actions[0:self.robot_action_space]
            self.robot.move_ee(robot_action, self.control_method)
            gripper_action = abs(actions[-1])
            self.robot.move_gripper(gripper_action)

        if self.control_method == "joint":
            # 这里需要看看别人如何控制机器臂的，增量控制还是绝对控制，感觉是增量控制概率大一点
            robot_action = actions[0:self.robot_action_space]
            joint_state, _ = self.robot.get_joint_info()
            dv = 0.01
            for i in range(len(joint_state)):
                robot_action[i] = joint_state[i] + robot_action[i]*dv
            self.robot.move_ee(robot_action, self.control_method)
            gripper_action = math.abs(actions[self.robot_action_space:])
            self.robot.move_gripper(gripper_action)

    @abstractmethod
    def _get_reward(self):
        pass

    @abstractmethod
    def _is_done(self):
        pass

    @abstractmethod
    def _get_info(self):
        pass

    def reset(self):
        self.robot.reset()
        return self._get_observation()

    def close(self):
        p.disconnect(self.server_id)







