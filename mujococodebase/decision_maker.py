from dataclasses import Field
import logging
import os
from enum import Enum
from typing import Mapping

import numpy as np
from mujococodebase.utils.math_ops import MathOps
from mujococodebase.world.field import FIFAField, HLAdultField
from mujococodebase.world.play_mode import PlayModeEnum, PlayModeGroupEnum

logger = logging.getLogger()


class State(Enum):
    BEAMING = 0
    GETTING_UP = 1
    GO_TO_BALL = 2
    DRIBBLE = 3
    NEUTRAL = 4


class DecisionMaker:

    BEAM_POSES: Mapping[type, Mapping[int, tuple[float, float, float]]] = {
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
        from mujococodebase.agent import Agent
        self.agent: Agent = agent
        self.custom_beam_pose = self._load_custom_beam_pose()

        # FSM
        self._current_state = State.NEUTRAL

    # --------------------------------------------------
    # Core Loop
    # --------------------------------------------------

    def update_current_behavior(self) -> None:
        if self.agent.world.playmode is PlayModeEnum.GAME_OVER:
            return

        self._check_global_interrupts()

        match self._current_state:
            case State.BEAMING:
                self._state_beaming()
            case State.GETTING_UP:
                self._state_getting_up()
            case State.GO_TO_BALL:
                self._state_go_to_ball()
            case State.DRIBBLE:
                self._state_dribble()
            case State.NEUTRAL:
                self._state_neutral()

        self.agent.robot.commit_motor_targets_pd()

    # --------------------------------------------------
    # State Transitions
    # --------------------------------------------------

    def _check_global_interrupts(self):
        """Global transitions (higher priority overrides)"""

        if self.agent.world.playmode_group in (
            PlayModeGroupEnum.ACTIVE_BEAM,
            PlayModeGroupEnum.PASSIVE_BEAM,
        ):
            self._enter_state(State.BEAMING)
            return

        if self.agent.skills_manager.is_ready("GetUp"):
            self._enter_state(State.GETTING_UP)
            return

        if self.agent.world.playmode in (
            PlayModeEnum.BEFORE_KICK_OFF,
            PlayModeEnum.THEIR_GOAL,
            PlayModeEnum.OUR_GOAL,
        ):
            self._enter_state(State.NEUTRAL)
            return

        # All other playmodes (including PLAY_ON) → carry ball.
        # Mirrors the original's final `else: carry_ball()` catch-all.
        # GO_TO_BALL / DRIBBLE transition between each other internally.
        if self._current_state not in (State.GO_TO_BALL, State.DRIBBLE):
            self._enter_state(State.GO_TO_BALL)

    # --------------------------------------------------
    # State Functions
    # --------------------------------------------------

    def _state_beaming(self):
        # Exit when the beam phase ends
        if self.agent.world.playmode_group not in (
            PlayModeGroupEnum.ACTIVE_BEAM,
            PlayModeGroupEnum.PASSIVE_BEAM,
        ):
            self._enter_state(State.NEUTRAL)
            return

        if self.custom_beam_pose is not None:
            pos2d = self.custom_beam_pose[:2]
            rotation = self.custom_beam_pose[2]
        else:
            default_pose = self.BEAM_POSES[type(self.agent.world.field)][self.agent.world.number]
            pos2d = default_pose[:2]
            rotation = default_pose[2]

        self.agent.server.commit_beam(pos2d=pos2d, rotation=rotation)

    def _state_getting_up(self):
        finished = self.agent.skills_manager.execute("GetUp")
        if finished:
            self._enter_state(State.GO_TO_BALL)

    def _state_neutral(self):
        self.agent.skills_manager.execute("Neutral")

    def _state_go_to_ball(self):
        # Exit → DRIBBLE if aligned and behind ball

        aligned, behind_ball, carry_pos, goal_pos, desired_orientation = self._compute_ball_geometry()

        if carry_pos is None:  # degenerate case: ball on goal line
            return

        if aligned and behind_ball:
            self._enter_state(State.DRIBBLE)
            return

        self.agent.skills_manager.execute(
            "Walk",
            target_2d=carry_pos,
            is_target_absolute=True,
            orientation=None if np.linalg.norm(self.agent.world.global_position[:2] - carry_pos) > 2 else desired_orientation
        )

    def _state_dribble(self):
        # Exit → GO_TO_BALL if alignment lost

        aligned, behind_ball, _, goal_pos, desired_orientation = self._compute_ball_geometry()

        if goal_pos is None:  # degenerate case: ball on goal line
            return

        if not aligned or not behind_ball:
            self._enter_state(State.GO_TO_BALL)
            return

        self.agent.skills_manager.execute(
            "Walk",
            target_2d=goal_pos,
            is_target_absolute=True,
            orientation=desired_orientation
        )

    # --------------------------------------------------
    # State Entry
    # --------------------------------------------------

    def _enter_state(self, new_state: State):
        if self._current_state == new_state:
            return

        match new_state:
            case State.BEAMING:
                pass  # no entry actions yet
            case State.GETTING_UP:
                pass  # no entry actions yet
            case State.GO_TO_BALL:
                pass  # no entry actions yet
            case State.DRIBBLE:
                pass  # no entry actions yet
            case State.NEUTRAL:
                pass  # no entry actions yet

        self._current_state = new_state

    # --------------------------------------------------
    # Helpers
    # --------------------------------------------------

    def _compute_ball_geometry(self):
        their_goal_pos = self.agent.world.field.get_their_goal_position()[:2]
        ball_pos = self.agent.world.ball_pos[:2]
        my_pos = self.agent.world.global_position[:2]

        ball_to_goal = their_goal_pos - ball_pos
        bg_norm = np.linalg.norm(ball_to_goal)
        if bg_norm == 0:
            return False, False, None, None, None

        ball_to_goal_dir = ball_to_goal / bg_norm

        dist = 0.30
        carry_ball_pos = ball_pos - ball_to_goal_dir * dist

        my_to_ball = ball_pos - my_pos
        my_to_ball_norm = np.linalg.norm(my_to_ball)

        if my_to_ball_norm == 0:
            my_to_ball_dir = np.zeros(2)
        else:
            my_to_ball_dir = my_to_ball / my_to_ball_norm

        cosang = np.dot(my_to_ball_dir, ball_to_goal_dir)
        cosang = np.clip(cosang, -1.0, 1.0)
        angle_diff = np.arccos(cosang)

        aligned = (my_to_ball_norm > 1e-6) and (angle_diff <= np.deg2rad(7.5))
        behind_ball = np.dot(my_pos - ball_pos, ball_to_goal_dir) < 0

        desired_orientation = MathOps.vector_angle(ball_to_goal)

        return aligned, behind_ball, carry_ball_pos, their_goal_pos, desired_orientation

    def _load_custom_beam_pose(self):
        x = os.getenv("PLAYER_SPAWN_X")
        y = os.getenv("PLAYER_SPAWN_Y")
        r = os.getenv("PLAYER_SPAWN_ROT")
        if x is None or y is None or r is None:
            return None
        try:
            return (float(x), float(y), float(r))
        except ValueError:
            logger.warning("Invalid PLAYER_SPAWN_* environment variables; ignoring custom beam pose.")
            return None