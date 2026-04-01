import math
import os
import json
import numpy as np
from mujococodebase.skills.skill import Skill
from mujococodebase.utils.math_ops import MathOps
from mujococodebase.utils.neural_network import run_network, load_network
from scipy.spatial.transform import Rotation as R

class Walk(Skill):
    def __init__(self, agent):
        super().__init__(agent)
        self.joint_nominal_position = np.array(
            [0, 0, 0, -1.4, 0, -0.4, 0, 1.4, 0, 0.4, 0, -0.4, 0, 0, 0.8, -0.4, 0, -0.4, 0, 0, 0.8, -0.4, 0]
        )
        self.scaling_factor = 0.5

        self.previous_action = np.zeros(len(self.agent.robot.ROBOT_MOTORS))

        self.model = load_network(model_path=os.path.join(os.path.dirname(__file__), "new_nn_walk.onnx"))

        meta_path = os.path.join(os.path.dirname(__file__), "new_nn_walk.onnx.meta.json")
        with open(meta_path) as f:
            _meta = json.load(f)
        self.carry = np.zeros((1, _meta["gru_hidden_dim"]), dtype=np.float32)

        self._gait_period = 1.0
        self._policy_dt = 0.02
        self.gait_phase_offset = np.array([0.0, -np.pi], dtype=np.float32)
        self.gait_phase = self.gait_phase_offset.copy()
        self.gait_phase_dt = (2.0 * np.pi * self._policy_dt) / self._gait_period

    def _wrap_to_pi(self, x):
        return (x + np.pi) % (2.0 * np.pi) - np.pi

    def _get_gait_phase_features(self):
        phase_tp1 = self._wrap_to_pi(self.gait_phase + self.gait_phase_dt)
        return np.concatenate([np.sin(phase_tp1), np.cos(phase_tp1)], axis=-1).astype(np.float32)

    def _step_gait_manager(self):
        self.gait_phase = self._wrap_to_pi(self.gait_phase + self.gait_phase_dt).astype(np.float32)

    def execute(self, reset: bool, target_2d: list, is_target_absolute: bool, orientation: float=None, is_orientation_absolute: bool=True) -> bool:
        
        robot = self.agent.robot
        world = self.agent.world
        
        velocity = None

        if is_target_absolute:
            raw_target = target_2d - world.global_position[:2]
            velocity = MathOps.rotate_2d_vec(raw_target, -robot.global_orientation_euler[2], is_rad=False)
        else:
            velocity = target_2d

        rel_orientation = None
        if orientation is None:
            rel_orientation = MathOps.vector_angle(velocity) * 0.3
        elif is_orientation_absolute:
            rel_orientation = MathOps.normalize_deg(orientation - robot.global_orientation_euler[2])
        else:
            rel_orientation = orientation * 0.3

        rel_orientation = np.clip(rel_orientation, -0.25, 0.25)

        velocity = np.concat([velocity, np.array([rel_orientation])], axis=0)

        velocity[0] = np.clip(velocity[0], -0.5, 0.5)
        velocity[1] = np.clip(velocity[1], -0.25, 0.25)

        radian_joint_positions = np.deg2rad(list(robot.motor_positions.values()))
        radian_joint_speeds = np.deg2rad(list(robot.motor_speeds.values()))

        scaled_joint_pos = (radian_joint_positions - self.joint_nominal_position) / 3.14
        scaled_joint_vel = radian_joint_speeds / 100.0
        scaled_previous_action = self.previous_action / 10.0

        ang_vel = np.clip(np.deg2rad(robot.gyroscope) / 50.0, -1.0, 1.0)
        orientation_quat_inv = R.from_quat(robot._global_cheat_orientation).inv()
        projected_gravity = orientation_quat_inv.apply(np.array([0.0, 0.0, -1.0]))

        gait_phase_features = self._get_gait_phase_features()

        observation = np.concatenate([
            scaled_joint_pos,
            scaled_joint_vel,
            scaled_previous_action,
            ang_vel,
            velocity,
            gait_phase_features,
            projected_gravity,
        ]).astype(np.float32)

        observation = np.nan_to_num(observation, nan=0.0, posinf=0.0, neginf=0.0)
        observation = np.clip(observation, -10.0, 10.0)

        obs_input = observation[np.newaxis, :]
        session = self.model["session"]
        nn_action, next_carry = session.run(
            ["action", "carry_out"],
            {"obs": obs_input, "carry_in": self.carry}
        )
        nn_action = nn_action.flatten().astype(np.float32)
        self.carry = next_carry

        target_joint_positions = self.joint_nominal_position + self.scaling_factor * nn_action

        self.previous_action = nn_action
        self._step_gait_manager()

        for idx, target in enumerate(target_joint_positions):
            robot.set_motor_target_position(
                robot.ROBOT_MOTORS[idx], target*180/math.pi, kp=25, kd=0.6
            )

    def is_ready(self, *args, **kwargs) -> bool:
        return True