from dataclasses import Field
import logging
import os
import threading
from typing import Mapping

import numpy as np
from mujococodebase.utils.math_ops import MathOps
from mujococodebase.world.field import FIFAField, HLAdultField
from mujococodebase.world.play_mode import PlayModeEnum, PlayModeGroupEnum
from mujococodebase.world.grid_world import GridWorld
from mujococodebase.world.planning import ana_theta_star as planner
from mujococodebase.world.other_robot import OtherRobot


logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__file__)


class DecisionMaker:
    """
    Responsible for deciding what the agent should do at each moment.

    This class is called every simulation step to update the agent's behavior
    based on the current state of the world and game conditions.
    """

    BEAM_POSES: Mapping[type[Field], Mapping[int, tuple[float, float, float]]] = {
        FIFAField: {
            1: (2.1, 0, 0),
            2: (22.0, 12.0, 0),
            3: (22.0, 4.0, 0),
            4: (22.0, -4.0, 0),
            5: (22.0, -12.0, 0),
            6: (15.0, 0.0, 0),
            7: (4.0, 16.0, 0),
            8: (11.0, 6.0, 0),
            9: (11.0, -6.0, 0),
            10: (4.0, -16.0, 0),
            11: (7.0, 0.0, 0),
        },
        HLAdultField: {
            1: (7.0, 0.0, 0),
            2: (2.0, -1.5, 0),
            3: (2.0, 1.5, 0),
        }
    } 

    def __init__(self, agent):
        """
        Creates a new DecisionMaker linked to the given agent.

        Args:
            agent: The main agent that owns this DecisionMaker.
        """
        from mujococodebase.agent import Agent  # type hinting

        self.agent: Agent = agent
        self.is_getting_up: bool = False

        # Optional per-player custom beam pose (x, y, rot_deg), typically set
        # via environment variables by the player launcher.
        self.custom_beam_pose: tuple[float, float, float] | None = self._load_custom_beam_pose()

        self.started = False

        # configuration for planning
        self.planning_threads = []
        self.paths = {
            "robot_to_ball": []
        }
        self.path_ready_events = {
            "robot_to_ball": threading.Event()
        }
        self.path_steps = {
            "robot_to_ball": 0
        }

    def start(self):
        """Initialization tasks that require information from the server that isn't prseent when __init__() runs."""
        self.started = True
        self.create_grid_world()
        self.plan_paths()

    def _load_custom_beam_pose(self) -> tuple[float, float, float] | None:
        x_str = os.getenv("PLAYER_SPAWN_X")
        y_str = os.getenv("PLAYER_SPAWN_Y")
        rot_str = os.getenv("PLAYER_SPAWN_ROT")

        if x_str is None or y_str is None or rot_str is None:
            return None

        try:
            x = float(x_str)
            y = float(y_str)
            rot = float(rot_str)
        except ValueError:
            logger.warning("Invalid PLAYER_SPAWN_* environment variables; ignoring custom beam pose.")
            return None

        return (x, y, rot)

    def update_current_behavior(self) -> None:
        """
        Chooses what the agent should do in the current step.

        This function checks the game state and decides which behavior
        or skill should be executed next.
        """

        if self.agent.world.playmode is PlayModeEnum.GAME_OVER:
            return

        if self.agent.world.playmode_group in (
            PlayModeGroupEnum.ACTIVE_BEAM,
            PlayModeGroupEnum.PASSIVE_BEAM,
        ):
            if self.custom_beam_pose is not None:
                pos2d = self.custom_beam_pose[:2]
                rotation = self.custom_beam_pose[2]
            else:
                default_pose = self.BEAM_POSES[type(self.agent.world.field)][self.agent.world.number]
                pos2d = default_pose[:2]
                rotation = default_pose[2]

            self.agent.server.commit_beam(
                pos2d=pos2d,
                rotation=rotation,
            )

        if self.is_getting_up or self.agent.skills_manager.is_ready(skill_name="GetUp"):
            self.is_getting_up = not self.agent.skills_manager.execute(skill_name="GetUp")

        elif self.agent.world.playmode is PlayModeEnum.PLAY_ON:
            self.play()
        elif self.agent.world.playmode in (PlayModeEnum.BEFORE_KICK_OFF, PlayModeEnum.THEIR_GOAL, PlayModeEnum.OUR_GOAL):
            self.agent.skills_manager.execute("Neutral")
        else:
            self.play()

        self.agent.robot.commit_motor_targets_pd()
    
    def play(self):
        """
        Behavior of robot during the game.
        """
        if not self.started:
            self.start()
        
        # TODO: plan paths for robots to future locations on path to goal so they can receive passes (put in plan_paths())

        # if path planning is incomplete, wait
        if not self.path_ready_events["robot_to_ball"].is_set():
            self.agent.skills_manager.execute("Neutral")
            logger.debug("Waiting for path planning...")
            return
        
        # if path has been followed, wait
        if self.path_steps["robot_to_ball"] >= len(self.paths["robot_to_ball"]):
            self.agent.skills_manager.execute("Neutral")
            logger.debug("Done following robot_to_ball path")
            return
        
        # follow robot-to-ball path
        target_location = np.array(self.paths["robot_to_ball"][self.path_steps["robot_to_ball"]], dtype=float) / self.grid_scale
        # TODO: add orientation
        target_orientation = None
        self.agent.skills_manager.execute(
            "Walk",
            target_2d=target_location,
            is_target_absolute=True,
            orientation=target_orientation
        )
        logger.debug(f"Walking to {target_location}")

        # check if we've arrived and should head to the next waypoint
        agent_location = self.agent.world.global_position[:2]
        agent_to_target = target_location - agent_location
        PATH_COMPLETE_THRESHOLD = 0.1
        if np.linalg.norm(agent_to_target) <= PATH_COMPLETE_THRESHOLD:
            self.path_steps["robot_to_ball"] += 1


    def plan_paths(self):
        """
        Plan all necessary paths.
        """
        t = threading.Thread(
            target=planner, 
            args=(self.grid_world, self.agent_grid_pos, self.ball_grid_pos, "robot_to_ball", self.paths, self.path_ready_events)
        )
        self.planning_threads.append(t)
        t.start()

    def create_grid_world(self) -> dict:
        """
        Convert the simulation world to a grid world for planning purposes.
        """
        # each grid cell will represent 10cm
        self.grid_scale: int = 10
        self.grid_world: GridWorld = GridWorld(
            self.agent.world.field.get_length() * self.grid_scale, 
            self.agent.world.field.get_width() * self.grid_scale
        )
        
        # add obstacle locations
        obstacles: list[OtherRobot] = [player for player in self.agent.world.their_team_players if player.last_seen_time is not None]
        obstacles: np.ndarray[float] = [robot.position for robot in obstacles]
        for pos in obstacles:
            logger.debug(f"Obstacle at {pos}")
            self.grid_world.add_obstacle(np.array([round(pos[0] * self.grid_scale), round(pos[1] * self.grid_scale)]))

        # convert location of line in front of the goal to grid coordinates
        goal_world_pos = self.agent.world.field.get_their_goal_position()[:2]
        goal_width = abs(self.agent.world.field.field_landmarks.landmarks["g_lup"][1] 
                         - self.agent.world.field.field_landmarks.landmarks["g_llp"][1])
        offsets = np.array(range(
            round((goal_world_pos[1] - goal_width/2) * self.grid_scale), 
            round((goal_world_pos[1] + goal_width/2) * self.grid_scale)
        ))
        x = round(goal_world_pos[0] * self.grid_scale)
        y = np.round(goal_world_pos[1] * self.grid_scale + offsets)
        self.goal_grid_pos = np.column_stack((np.full(len(offsets), x), y))

        # convert location of ball to grid coordinates
        ball_world_pos = self.agent.world.ball_pos[:2]
        self.ball_grid_pos = np.array([round(ball_world_pos[0] * self.grid_scale), 
                              round(ball_world_pos[1] * self.grid_scale)])

        # convert location of agent to grid coordinates
        agent_world_pos = self.agent.world.global_position[:2] # NOTE: global_position[2] is used for detecting falls and is NOT the 2D orientation
        self.agent_grid_pos = np.array([round(agent_world_pos[0] * self.grid_scale), 
                               round(agent_world_pos[1] * self.grid_scale)])
        # TODO: add orientation information


    def carry_ball(self):
        """
        Basic example of a behavior: moves the robot toward the goal while handling the ball.
        """
        their_goal_pos = self.agent.world.field.get_their_goal_position()[:2]
        ball_pos = self.agent.world.ball_pos[:2]
        my_pos = self.agent.world.global_position[:2]

        ball_to_goal = their_goal_pos - ball_pos
        bg_norm = np.linalg.norm(ball_to_goal)
        if bg_norm == 0:
            return 
        ball_to_goal_dir = ball_to_goal / bg_norm

        dist_from_ball_to_start_carrying = 0.30
        carry_ball_pos = ball_pos - ball_to_goal_dir * dist_from_ball_to_start_carrying

        my_to_ball = ball_pos - my_pos
        my_to_ball_norm = np.linalg.norm(my_to_ball)
        if my_to_ball_norm == 0:
            my_to_ball_dir = np.zeros(2)
        else:
            my_to_ball_dir = my_to_ball / my_to_ball_norm

        cosang = np.dot(my_to_ball_dir, ball_to_goal_dir)
        cosang = np.clip(cosang, -1.0, 1.0)
        angle_diff = np.arccos(cosang)

        ANGLE_TOL = np.deg2rad(7.5)
        aligned = (my_to_ball_norm > 1e-6) and (angle_diff <= ANGLE_TOL)

        behind_ball = np.dot(my_pos - ball_pos, ball_to_goal_dir) < 0
        desired_orientation = MathOps.vector_angle(ball_to_goal)

        if not aligned or not behind_ball:
            self.agent.skills_manager.execute(
                "Walk",
                target_2d=carry_ball_pos,
                is_target_absolute=True,
                orientation=None if np.linalg.norm(my_pos - carry_ball_pos) > 2 else desired_orientation
            )
        else:
            self.agent.skills_manager.execute(
                "Walk",
                target_2d=their_goal_pos,
                is_target_absolute=True,
                orientation=desired_orientation
            )

