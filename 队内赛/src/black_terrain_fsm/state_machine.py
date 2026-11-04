"""Hierarchical state machine for fixed skills and the three Black policies."""

from __future__ import annotations

import math
import queue
import threading
from enum import Enum
from typing import Any

import torch

from .config import (
    APPROACH_WAYPOINTS,
    AUTOPILOT_STAGE_TIMEOUTS,
    BRIDGE_B_MIN_ROOT_Z,
    BRIDGE_B_SPAWN_YAW,
    BRIDGE_B_TIMEOUT_S,
    BRIDGE_B_X_RANGE,
    BRIDGE_B_YAW_TOLERANCE,
    CRAWL_EXIT_Y,
    KEY_STATE_MAP,
    MANUAL_BACKWARD_SPEED_DEFAULT,
    MANUAL_BACKWARD_SPEED_MAX,
    MANUAL_BACKWARD_SPEED_STEP,
    MANUAL_CLIMB_VX,
    MANUAL_FORWARD_VX_DEFAULT,
    MANUAL_FORWARD_VX_MAX,
    MANUAL_FORWARD_VX_STEP,
    MANUAL_STRAFE_VY_DEFAULT,
    MANUAL_STRAFE_VY_MAX,
    MANUAL_STRAFE_VY_MIN,
    MANUAL_STRAFE_VY_STEP,
    MANUAL_TURN_WZ_DEFAULT,
    MANUAL_TURN_WZ_MAX,
    MANUAL_TURN_WZ_MIN,
    MANUAL_TURN_WZ_STEP,
    NAV_GOAL_TOLERANCE,
    NAV_MAX_VX,
    NAV_MAX_WZ,
    NAV_MIN_VX,
    ORIGIN_RESET_POS,
    ORIGIN_RESET_YAW,
    STAGING_TOLERANCE,
    STAND_JOINTS,
    LOW_JOINTS,
    WALL_MIN_FINAL_Z,
    WALL_MIN_SAFE_Z,
    WALL_STAGE_TIMEOUTS,
    WALL_X_RANGE,
    WALL_CLIMB_MAX_PITCH_RAD,
    WALL_CLIMB_MAX_ROLL_RAD,
    WALL_CLIMB_MIN_FINAL_Z,
    WALL_CLIMB_MIN_SAFE_ROOT_Z,
    WALL_CLIMB_STAGE_TIMEOUTS,
    WALL_CLIMB_START_X_RANGE,
    WALL_CLIMB_START_Y_RANGE,
    WALL_CLIMB_START_YAW_TOLERANCE,
)
from .controllers import (
    BridgeBWalkController,
    CrawlGaitController,
    HoldController,
    JointActionMapper,
    NavigationController,
    PoseBlendController,
    WallClimbController,
    WallVaultController,
)
from .policies import PolicyBank
from .terrain import TerrainMonitor, wrap_angle


# GLFW key codes delivered by MuJoCo's passive viewer callback.
KEY_SPACE = 32
KEY_A = 65
KEY_D = 68
KEY_E = 69
KEY_Q = 81
KEY_RIGHT = 262
KEY_LEFT = 263
KEY_DOWN = 264
KEY_UP = 265
KEY_HOME = 268


class RobotState(str, Enum):
    HOLD = "HOLD"
    STAND = "STAND"
    LOCOMOTION = "LOCOMOTION"
    CLIMB = "CLIMB"
    JUMP = "JUMP"
    LIE_DOWN = "LIE_DOWN"
    CRAWL_FORWARD = "CRAWL_FORWARD"
    BRIDGE_B_CROSS = "BRIDGE_B_CROSS"
    WALL_CROSS = "WALL_CROSS"
    WALL_CLIMB = "WALL_CLIMB"
    RECOVER = "RECOVER"
    AUTOPILOT = "AUTOPILOT"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class StateMachinePolicy:
    """Callable policy with keyboard-safe state requests.

    The viewer invokes this object from the simulation loop.  Keyboard callbacks
    only enqueue state changes or update a small velocity command; simulation
    state is never modified from the viewer thread.
    """

    def __init__(
        self,
        env: Any,
        wrapper: Any,
        policy_bank: PolicyBank,
        initial_state: RobotState = RobotState.STAND,
    ) -> None:
        self.env = env
        self.wrapper = wrapper
        self.robot = env.scene["robot"]
        self.policy_bank = policy_bank
        self.mapper = JointActionMapper(env)
        self.monitor = TerrainMonitor(env)
        self.navigation = NavigationController()
        self._requests: queue.SimpleQueue[RobotState] = queue.SimpleQueue()
        self._reset_requested = threading.Event()
        self.state = RobotState.HOLD
        self.state_elapsed = 0.0
        self.transition_count = 0
        self.last_metrics = self.monitor.measure()
        self.last_action_source = "none"
        # Direction keys are throttles: first press -> default, repeats -> +step.
        # All three command axes start disabled; keys enable only their own axis.
        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.manual_wz = 0.0
        self.manual_forward_speed = MANUAL_FORWARD_VX_DEFAULT
        self.manual_backward_speed = MANUAL_BACKWARD_SPEED_DEFAULT
        self.manual_turn_wz = MANUAL_TURN_WZ_DEFAULT
        self.manual_strafe_vy = MANUAL_STRAFE_VY_DEFAULT
        self._manual_vx_mode: str | None = None
        self._manual_vy_mode: str | None = None
        self._manual_wz_mode: str | None = None
        self._manual_strafe_direction = 1
        self._manual_turn_direction = 1
        self._pose_controller: PoseBlendController | None = None
        self._hold_controller: HoldController | None = None
        self._crawl_controller: CrawlGaitController | None = None
        self._crawl_lowering: PoseBlendController | None = None
        self._autopilot_stage = "stand"
        self._autopilot_stage_elapsed = 0.0
        self._autopilot_failure_reason: str | None = None
        self._autopilot_transitions: list[dict[str, float | str]] = []
        self._autopilot_waypoint = 0
        self._align_hold_time = 0.0
        self._approach_best_distance = math.inf
        self._approach_stall_time = 0.0
        self._bridge_stage = "idle"
        self._bridge_stage_elapsed = 0.0
        self._bridge_failure_reason: str | None = None
        self._bridge_transitions: list[dict[str, float | str]] = []
        self._bridge_controller: BridgeBWalkController | None = None
        self._bridge_result_snapshot: dict[str, Any] | None = None
        self._wall_stage = "idle"
        self._wall_stage_elapsed = 0.0
        self._wall_failure_reason: str | None = None
        self._wall_transitions: list[dict[str, float | str]] = []
        self._wall_controller: WallVaultController | None = None
        self._wall_climb_stage = "idle"
        self._wall_climb_stage_elapsed = 0.0
        self._wall_climb_failure_reason: str | None = None
        self._wall_climb_transitions: list[dict[str, float | str]] = []
        self._wall_climb_controller: WallClimbController | None = None
        self._enter(initial_state, reset_elapsed=False)

    @property
    def step_dt(self) -> float:
        return float(self.env.step_dt)

    def request_state(self, state: RobotState | str) -> None:
        self._requests.put(RobotState(state))

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        self.manual_vx = float(vx)
        self.manual_vy = float(vy)
        self.manual_wz = float(wz)
        self.manual_forward_speed = abs(float(vx))
        self.manual_strafe_vy = abs(float(vy))
        self.manual_turn_wz = abs(float(wz))
        self._manual_vx_mode = "external" if abs(float(vx)) > 1e-9 else None
        self._manual_vy_mode = "external" if abs(float(vy)) > 1e-9 else None
        self._manual_wz_mode = "external" if abs(float(wz)) > 1e-9 else None

    def adjust_velocity(self, dvx: float = 0.0, dwz: float = 0.0) -> None:
        self.manual_vx = max(
            NAV_MIN_VX,
            min(NAV_MAX_VX, self.manual_vx + dvx),
        )
        self.manual_wz = max(-NAV_MAX_WZ, min(NAV_MAX_WZ, self.manual_wz + dwz))

    @staticmethod
    def _clamp_speed(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, float(value)))

    def _apply_turn_rate(self, wz: float) -> float:
        self.manual_turn_wz = self._clamp_speed(
            abs(float(wz)),
            MANUAL_TURN_WZ_MIN,
            MANUAL_TURN_WZ_MAX,
        )
        if self._manual_wz_mode in ("turn_left", "turn_right"):
            self.manual_wz = self._manual_turn_direction * self.manual_turn_wz
        return self.manual_turn_wz

    def set_turn_rate(self, wz: float) -> float:
        """Set the base-policy in-place yaw-rate magnitude."""
        return self._apply_turn_rate(wz)

    def adjust_turn_rate(self, dwz: float = MANUAL_TURN_WZ_STEP) -> float:
        """Increase or decrease the target in-place yaw-rate magnitude."""
        return self._apply_turn_rate(self.manual_turn_wz + float(dwz))

    def _apply_strafe_rate(self, vy: float) -> float:
        self.manual_strafe_vy = self._clamp_speed(
            abs(float(vy)),
            MANUAL_STRAFE_VY_MIN,
            MANUAL_STRAFE_VY_MAX,
        )
        if self._manual_vy_mode in ("strafe_left", "strafe_right"):
            self.manual_vy = (
                self._manual_strafe_direction * self.manual_strafe_vy
            )
        return self.manual_strafe_vy

    def set_strafe_rate(self, vy: float) -> float:
        """Set the target magnitude of base-policy lateral velocity."""
        return self._apply_strafe_rate(vy)

    def adjust_strafe_rate(
        self, dvy: float = MANUAL_STRAFE_VY_STEP
    ) -> float:
        """Increase or decrease the target lateral velocity magnitude."""
        return self._apply_strafe_rate(self.manual_strafe_vy + float(dvy))

    def zero_velocity(self) -> None:
        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.manual_wz = 0.0
        self.manual_forward_speed = MANUAL_FORWARD_VX_DEFAULT
        self.manual_backward_speed = MANUAL_BACKWARD_SPEED_DEFAULT
        self.manual_strafe_vy = MANUAL_STRAFE_VY_DEFAULT
        self.manual_turn_wz = MANUAL_TURN_WZ_DEFAULT
        self._manual_vx_mode = None
        self._manual_vy_mode = None
        self._manual_wz_mode = None

    def request_reset(self) -> None:
        """Request a thread-safe reset to the map origin."""
        self._reset_requested.set()

    def request_forward(self, *, increment: bool = True) -> float:
        """Enable forward vx without changing the lateral or yaw commands."""
        if increment and self._manual_vx_mode == "forward":
            self.manual_forward_speed = min(
                MANUAL_FORWARD_VX_MAX,
                self.manual_forward_speed + MANUAL_FORWARD_VX_STEP,
            )
        elif increment:
            self.manual_forward_speed = MANUAL_FORWARD_VX_DEFAULT
        self._manual_vx_mode = "forward"
        self.manual_vx = self.manual_forward_speed
        if self.state != RobotState.LOCOMOTION:
            self.request_state(RobotState.LOCOMOTION)
        return self.manual_forward_speed

    def set_forward_speed(self, speed: float) -> float:
        """Set the base-policy forward speed magnitude."""
        self.manual_forward_speed = self._clamp_speed(
            abs(float(speed)),
            MANUAL_FORWARD_VX_DEFAULT,
            MANUAL_FORWARD_VX_MAX,
        )
        if self._manual_vx_mode == "forward":
            self.manual_vx = self.manual_forward_speed
        return self.manual_forward_speed

    def request_backward(self, *, increment: bool = True) -> float:
        """Enable backward vx without changing lateral or yaw commands."""
        if increment and self._manual_vx_mode == "backward":
            self.manual_backward_speed = min(
                MANUAL_BACKWARD_SPEED_MAX,
                self.manual_backward_speed + MANUAL_BACKWARD_SPEED_STEP,
            )
        elif increment:
            self.manual_backward_speed = MANUAL_BACKWARD_SPEED_DEFAULT
        self._manual_vx_mode = "backward"
        self.manual_vx = -self.manual_backward_speed
        if self.state != RobotState.LOCOMOTION:
            self.request_state(RobotState.LOCOMOTION)
        return self.manual_backward_speed

    def set_backward_speed(self, speed: float) -> float:
        """Set the base-policy backward speed magnitude."""
        self.manual_backward_speed = self._clamp_speed(
            abs(float(speed)),
            MANUAL_BACKWARD_SPEED_DEFAULT,
            MANUAL_BACKWARD_SPEED_MAX,
        )
        if self._manual_vx_mode == "backward":
            self.manual_vx = -self.manual_backward_speed
        return self.manual_backward_speed

    def request_strafe(
        self,
        direction: int,
        *,
        increment: bool = True,
    ) -> float:
        """Enable lateral vy while retaining the current vx and wz."""
        mode = "strafe_left" if direction >= 0 else "strafe_right"
        if increment and self._manual_vy_mode == mode:
            self.manual_strafe_vy = min(
                MANUAL_STRAFE_VY_MAX,
                self.manual_strafe_vy + MANUAL_STRAFE_VY_STEP,
            )
        elif increment:
            self.manual_strafe_vy = MANUAL_STRAFE_VY_DEFAULT
        self._manual_strafe_direction = 1 if direction >= 0 else -1
        self._manual_vy_mode = mode
        self.manual_vy = (
            self._manual_strafe_direction * self.manual_strafe_vy
        )
        if self.state != RobotState.LOCOMOTION:
            self.request_state(RobotState.LOCOMOTION)
        return self.manual_strafe_vy

    def request_turn(
        self,
        direction: int,
        *,
        increment: bool = True,
    ) -> float:
        """Enable signed wz while retaining the current vx and vy."""
        mode = "turn_left" if direction >= 0 else "turn_right"
        if increment and self._manual_wz_mode == mode:
            self.manual_turn_wz = min(
                MANUAL_TURN_WZ_MAX,
                self.manual_turn_wz + MANUAL_TURN_WZ_STEP,
            )
        elif increment:
            self.manual_turn_wz = MANUAL_TURN_WZ_DEFAULT
        self._manual_turn_direction = 1 if direction >= 0 else -1
        self._manual_wz_mode = mode
        self.manual_wz = self._manual_turn_direction * self.manual_turn_wz
        if self.state != RobotState.LOCOMOTION:
            self.request_state(RobotState.LOCOMOTION)
        return self.manual_turn_wz

    def request_turn_in_place(
        self,
        direction: int,
        *,
        increment: bool = True,
    ) -> float:
        """Backward-compatible alias for request_turn."""
        return self.request_turn(direction, increment=increment)
    def request_stop(self) -> None:
        """Stop manual motion and safely recover from a low posture if needed."""
        self.zero_velocity()
        self._discard_state_requests()
        if self.state in (
            RobotState.CRAWL_FORWARD,
            RobotState.LIE_DOWN,
        ):
            self.request_state(RobotState.RECOVER)

    def handle_key(self, key: int) -> str | None:
        """Handle one GLFW key code; safe to call from the viewer thread."""
        if key == KEY_SPACE:
            self.request_stop()
            print("[KEY] stop: velocity=0, throttle reset", flush=True)
            return "STOP"
        if key == KEY_UP:
            speed = self.request_forward()
            print(f"[KEY] forward speed={speed:.2f} m/s", flush=True)
            return "FORWARD"
        if key == KEY_DOWN:
            speed = self.request_backward()
            print(f"[KEY] backward speed={speed:.2f} m/s", flush=True)
            return "BACKWARD"
        if key == KEY_LEFT:
            speed = self.request_strafe(1)
            print(f"[KEY] strafe left speed={speed:.2f} m/s", flush=True)
            return "STRAFE_LEFT"
        if key == KEY_RIGHT:
            speed = self.request_strafe(-1)
            print(f"[KEY] strafe right speed={speed:.2f} m/s", flush=True)
            return "STRAFE_RIGHT"
        if key == KEY_Q:
            rate = self.request_turn(1)
            print(f"[KEY] turn left rate={rate:.2f} rad/s", flush=True)
            return "TURN_LEFT"
        if key == KEY_E:
            rate = self.request_turn(-1)
            print(f"[KEY] turn right rate={rate:.2f} rad/s", flush=True)
            return "TURN_RIGHT"
        # A/D remain aliases for the new Left/Right strafe controls.
        if key == KEY_A:
            speed = self.request_strafe(1)
            print(f"[KEY] strafe left speed={speed:.2f} m/s", flush=True)
            return "STRAFE_LEFT"
        if key == KEY_D:
            speed = self.request_strafe(-1)
            print(f"[KEY] strafe right speed={speed:.2f} m/s", flush=True)
            return "STRAFE_RIGHT"
        if key == KEY_HOME:
            self.request_reset()
            return "RESET_ORIGIN"

        try:
            char = chr(key).upper()
        except (TypeError, ValueError):
            return None
        if char == "R":
            self.request_reset()
            return "RESET_ORIGIN"
        state_name = KEY_STATE_MAP.get(char)
        if state_name is not None:
            state = RobotState(state_name)
            self.request_state(state)
            return state.value
        return None

    def _enter(self, state: RobotState, reset_elapsed: bool = True) -> None:
        state = RobotState(state)
        self.state = state
        if reset_elapsed:
            self.state_elapsed = 0.0
        self.transition_count += 1
        self._pose_controller = None
        self._hold_controller = None
        self._crawl_lowering = None
        self._crawl_controller = None
        self._bridge_controller = None
        self._wall_controller = None
        self._wall_climb_controller = None
        self._align_hold_time = 0.0
        self._autopilot_stage_elapsed = 0.0
        self._autopilot_failure_reason = None
        self._approach_best_distance = math.inf
        self._approach_stall_time = 0.0
        if state == RobotState.STAND:
            self._pose_controller = PoseBlendController(self.mapper, STAND_JOINTS)
        elif state == RobotState.LIE_DOWN:
            self._pose_controller = PoseBlendController(self.mapper, LOW_JOINTS)
        elif state == RobotState.RECOVER:
            self._pose_controller = PoseBlendController(self.mapper, STAND_JOINTS)
        elif state in (RobotState.HOLD, RobotState.EMERGENCY_STOP):
            self._hold_controller = HoldController(self.mapper)
        elif state == RobotState.LOCOMOTION:
            self.policy_bank.activate("base")
        elif state == RobotState.CLIMB:
            self.policy_bank.activate("climb")
        elif state == RobotState.JUMP:
            self.policy_bank.activate("jump")
        elif state == RobotState.CRAWL_FORWARD:
            self._crawl_lowering = PoseBlendController(self.mapper, LOW_JOINTS)
        elif state == RobotState.BRIDGE_B_CROSS:
            self._bridge_stage_elapsed = 0.0
            self._bridge_failure_reason = None
            self._bridge_transitions = []
            self._bridge_result_snapshot = None
            self.policy_bank.activate("base")
            self._bridge_controller = BridgeBWalkController(self.env)
            self._set_bridge_stage(self._bridge_controller.stage)
            metrics = self.monitor.measure()
            start_ok = (
                BRIDGE_B_X_RANGE[0] <= metrics.x <= BRIDGE_B_X_RANGE[1]
                and -1.55 <= metrics.y <= -1.05
                and abs(wrap_angle(metrics.yaw - BRIDGE_B_SPAWN_YAW))
                <= BRIDGE_B_YAW_TOLERANCE
            )
            if not start_ok:
                self._bridge_fail("start_pose_outside_bridge_b_region")
            elif not self._bridge_controller.front_edge_ok:
                self._bridge_fail("front_feet_not_at_bridge_b_edge")
        elif state == RobotState.WALL_CROSS:
            self._wall_stage_elapsed = 0.0
            self._wall_failure_reason = None
            self._wall_transitions = []
            self._wall_controller = WallVaultController(self.env, self.mapper)
            self._set_wall_stage(self._wall_controller.stage)
        elif state == RobotState.WALL_CLIMB:
            self._wall_climb_stage_elapsed = 0.0
            self._wall_climb_failure_reason = None
            self._wall_climb_transitions = []
            self._wall_climb_controller = WallClimbController(
                self.env, self.mapper
            )
            self._set_wall_climb_stage(self._wall_climb_controller.stage)
            metrics = self.monitor.measure()
            start_ok = (
                WALL_CLIMB_START_X_RANGE[0] <= metrics.x
                <= WALL_CLIMB_START_X_RANGE[1]
                and WALL_CLIMB_START_Y_RANGE[0] <= metrics.y
                <= WALL_CLIMB_START_Y_RANGE[1]
                and abs(wrap_angle(metrics.yaw - math.pi / 2.0))
                <= WALL_CLIMB_START_YAW_TOLERANCE
            )
            if not start_ok:
                self._wall_climb_fail(
                    "start_pose_outside_wall_climb_region"
                )
        elif state == RobotState.AUTOPILOT:
            self._autopilot_stage = "stand"
            self._autopilot_waypoint = 0
            self._autopilot_transitions = []
            self._pose_controller = PoseBlendController(self.mapper, STAND_JOINTS)

        print(
            f"[FSM] -> {state.value} (#{self.transition_count}) | "
            f"pos=({self.last_metrics.x:.2f}, {self.last_metrics.y:.2f}) "
            f"yaw={math.degrees(self.last_metrics.yaw):.1f} deg"
        )

    def _drain_requests(self) -> None:
        requested: RobotState | None = None
        while True:
            try:
                requested = self._requests.get_nowait()
            except queue.Empty:
                break
        if requested is not None:
            self._enter(requested)

    def _discard_state_requests(self) -> None:
        while True:
            try:
                self._requests.get_nowait()
            except queue.Empty:
                break

    def _reset_to_origin(self) -> None:
        """Teleport the robot home and clear all controller/policy state."""
        self._reset_requested.clear()
        self._discard_state_requests()
        self.wrapper.place_robot_at_pose(ORIGIN_RESET_POS, ORIGIN_RESET_YAW)
        self.zero_velocity()
        self.policy_bank.reset()
        self.monitor.reset_tracking()
        self.last_metrics = self.monitor.measure()
        self._enter(RobotState.STAND)
        self.last_action_source = "reset_origin"
        print(
            "[FSM] reset -> origin "
            f"pos=({self.last_metrics.x:.2f}, {self.last_metrics.y:.2f}, "
            f"{self.last_metrics.z:.2f}) velocity=0",
            flush=True,
        )

    def _set_command(self, vx: float, vy: float, wz: float) -> None:
        command = self.env.command_manager.get_command("twist")
        command[:] = torch.tensor(
            [[vx, vy, wz]],
            dtype=command.dtype,
            device=command.device,
        )

    def _act_rl(self, obs: Any, policy_name: str) -> torch.Tensor:
        # The command manager resamples after env.step(), so the observation
        # returned by the wrapper can contain a different command than the one
        # just written by this FSM.  The actor layout is
        # [ang_vel(3), gravity(3), q(12), qd(12), action(12), command(3), scan].
        command = self.env.command_manager.get_command("twist")
        actor_obs = obs["actor"]
        if actor_obs.ndim != 2 or actor_obs.shape[-1] < 45:
            raise ValueError(
                f"Unexpected Black actor observation shape: {tuple(actor_obs.shape)}"
            )
        actor_obs[:, 42:45].copy_(command)
        self.last_action_source = policy_name
        return self.policy_bank.act(policy_name, obs)

    def _act_pose(self, controller: PoseBlendController) -> torch.Tensor:
        self.last_action_source = "fixed_pose"
        return controller.act(self.step_dt)

    def _act_crawl(self, moving: bool = True) -> torch.Tensor:
        if self._crawl_controller is None:
            self._crawl_controller = CrawlGaitController(self.env, self.mapper)
        self.last_action_source = "crawl_gait"
        return self._crawl_controller.act(self.step_dt, moving=moving)

    def _set_autopilot_stage(self, stage: str) -> None:
        self._autopilot_stage = stage
        self._autopilot_stage_elapsed = 0.0
        self._approach_best_distance = math.inf
        self._approach_stall_time = 0.0
        self._autopilot_transitions.append(
            {"stage": stage, "time_s": float(self.state_elapsed)}
        )

    def _autopilot_fail(self, reason: str) -> None:
        self._autopilot_failure_reason = reason
        self._set_autopilot_stage("failed")
        self._hold_controller = HoldController(self.mapper)
        print(f"[AUTO] FAILED: {reason}")

    def _set_wall_stage(self, stage: str) -> None:
        if stage == self._wall_stage:
            return
        self._wall_stage = stage
        self._wall_stage_elapsed = 0.0
        self._wall_transitions.append(
            {"stage": stage, "time_s": float(self.state_elapsed)}
        )

    def _wall_fail(self, reason: str) -> None:
        if self._wall_failure_reason is None:
            self._wall_failure_reason = reason
            self._wall_transitions.append(
                {"stage": "failed", "time_s": float(self.state_elapsed)}
            )
        self._wall_stage = "failed"
        if self._wall_controller is not None:
            self._wall_controller.fail(reason)
        self._hold_controller = HoldController(self.mapper)
        print(f"[WALL] FAILED: {reason}", flush=True)

    def _set_bridge_stage(self, stage: str) -> None:
        if stage == self._bridge_stage:
            return
        self._bridge_stage = stage
        self._bridge_stage_elapsed = 0.0
        self._bridge_transitions.append(
            {"stage": stage, "time_s": float(self.state_elapsed)}
        )

    def _bridge_fail(self, reason: str) -> None:
        self._bridge_failure_reason = reason
        self._set_bridge_stage("failed")
        self._hold_controller = HoldController(self.mapper)
        self._bridge_result_snapshot = self._bridge_snapshot()
        print(f"[BRIDGE-B] FAILED: {reason}", flush=True)

    def _bridge_snapshot(self) -> dict[str, Any]:
        controller = self._bridge_controller
        return {
            "stage": self._bridge_stage,
            "failure_reason": self._bridge_failure_reason,
            "transitions": list(self._bridge_transitions),
            "done": bool(controller.done) if controller is not None else False,
            "south_reached": (
                bool(controller.south_reached)
                if controller is not None
                else False
            ),
            "diagnostics": (
                controller.diagnostics if controller is not None else None
            ),
        }

    @property
    def autopilot_result(self) -> dict[str, Any]:
        return {
            "stage": self._autopilot_stage,
            "failure_reason": self._autopilot_failure_reason,
            "transitions": list(self._autopilot_transitions),
        }

    @property
    def wall_result(self) -> dict[str, Any]:
        return {
            "stage": self._wall_stage,
            "failure_reason": self._wall_failure_reason,
            "transitions": list(self._wall_transitions),
        }

    @property
    def wall_climb_result(self) -> dict[str, Any]:
        controller = self._wall_climb_controller
        return {
            "stage": self._wall_climb_stage,
            "failure_reason": self._wall_climb_failure_reason,
            "transitions": list(self._wall_climb_transitions),
            "done": bool(controller.done) if controller is not None else False,
            "diagnostics": (
                controller.diagnostics if controller is not None else None
            ),
        }

    @property
    def bridge_result(self) -> dict[str, Any]:
        if self._bridge_result_snapshot is not None:
            return dict(self._bridge_result_snapshot)
        return self._bridge_snapshot()

    def _bridge_cross_action(self, obs: Any) -> torch.Tensor:
        controller = self._bridge_controller
        if controller is None:
            self._bridge_fail("bridge_b_controller_missing")
            self._set_command(0.0, 0.0, 0.0)
            assert self._hold_controller is not None
            self.last_action_source = "hold"
            return self._hold_controller.act()

        if self._bridge_failure_reason is not None:
            self._set_command(0.0, 0.0, 0.0)
            assert self._hold_controller is not None
            self.last_action_source = "hold"
            return self._hold_controller.act()

        controller.observe()
        self._bridge_stage = controller.stage
        self._bridge_stage_elapsed += self.step_dt
        if controller.failure_reason is not None:
            self._bridge_fail(controller.failure_reason)
        elif self._bridge_stage_elapsed > BRIDGE_B_TIMEOUT_S:
            self._bridge_fail("bridge_b_timeout")
        elif self.last_metrics.z < BRIDGE_B_MIN_ROOT_Z:
            self._bridge_fail("robot_fell_below_safe_height")

        if self._bridge_failure_reason is not None:
            self._set_command(0.0, 0.0, 0.0)
            assert self._hold_controller is not None
            self.last_action_source = "hold"
            return self._hold_controller.act()

        command = controller.command()
        self._set_command(*command)
        action = self._act_rl(obs, "base")
        if controller.done:
            self._set_bridge_stage("complete")
            self._bridge_result_snapshot = self._bridge_snapshot()
            self._set_command(0.0, 0.0, 0.0)
            self.request_state(RobotState.STAND)
            self.last_action_source = "bridge_b_walk_done"
            print(
                "[BRIDGE-B] south platform reached; stopping and returning "
                "to STAND",
                flush=True,
            )
        return action

    def _wall_cross_action(self, obs: Any) -> torch.Tensor:
        del obs
        metrics = self.last_metrics
        self._set_command(0.0, 0.0, 0.0)
        controller = self._wall_controller
        if controller is None:
            self._wall_fail("wall_controller_missing")
            assert self._hold_controller is not None
            return self._hold_controller.act()
        if self._wall_failure_reason is not None:
            self._hold_controller = self._hold_controller or HoldController(self.mapper)
            self.last_action_source = "hold"
            return self._hold_controller.act()

        if metrics.wall_support_contact:
            self._wall_fail("wall_support_collision")
            assert self._hold_controller is not None
            self.last_action_source = "hold"
            return self._hold_controller.act()
        if metrics.z < WALL_MIN_SAFE_Z:
            self._wall_fail("robot_fell_below_safe_height")
            assert self._hold_controller is not None
            self.last_action_source = "hold"
            return self._hold_controller.act()

        self._wall_stage_elapsed += self.step_dt
        timeout = WALL_STAGE_TIMEOUTS.get(self._wall_stage)
        if timeout is not None and self._wall_stage_elapsed > timeout:
            self._wall_fail(f"{self._wall_stage}_timeout")
            assert self._hold_controller is not None
            self.last_action_source = "hold"
            return self._hold_controller.act()

        if controller.done:
            if (
                not metrics.wall_cleared
                or not WALL_X_RANGE[0] <= metrics.x <= WALL_X_RANGE[1]
                or metrics.z < WALL_MIN_FINAL_Z
            ):
                self._wall_fail("wall_exit_not_cleared")
                assert self._hold_controller is not None
                self.last_action_source = "hold"
                return self._hold_controller.act()
            self._set_wall_stage("done")
            self.request_state(RobotState.STAND)
            self.last_action_source = "wall_vault_done"
            return controller.act(self.step_dt)

        action = controller.act(self.step_dt)
        if controller.stage != self._wall_stage:
            self._set_wall_stage(controller.stage)
        self.last_action_source = "wall_vault"
        return action

    def _wall_climb_action(self, obs: Any) -> torch.Tensor:
        del obs
        metrics = self.last_metrics
        self._set_command(0.0, 0.0, 0.0)
        controller = self._wall_climb_controller
        if controller is None:
            self._wall_climb_fail("wall_climb_controller_missing")
            assert self._hold_controller is not None
            return self._hold_controller.act()
        if self._wall_climb_failure_reason is not None:
            self._hold_controller = self._hold_controller or HoldController(
                self.mapper
            )
            self.last_action_source = "hold"
            return self._hold_controller.act()

        self._wall_climb_stage_elapsed += self.step_dt
        timeout = WALL_CLIMB_STAGE_TIMEOUTS.get(self._wall_climb_stage)
        if timeout is not None and self._wall_climb_stage_elapsed > timeout:
            self._wall_climb_fail(f"{self._wall_climb_stage}_timeout")
        elif metrics.wall_support_contact:
            self._wall_climb_fail("trunk_wall_contact")
        elif metrics.z < WALL_CLIMB_MIN_SAFE_ROOT_Z:
            self._wall_climb_fail("robot_fell_below_safe_height")
        elif controller.max_pitch > WALL_CLIMB_MAX_PITCH_RAD:
            self._wall_climb_fail("pitch_limit_exceeded")
        elif controller.max_roll > WALL_CLIMB_MAX_ROLL_RAD:
            self._wall_climb_fail("roll_limit_exceeded")
        if self._wall_climb_failure_reason is not None:
            assert self._hold_controller is not None
            self.last_action_source = "hold"
            return self._hold_controller.act()

        action = controller.act(self.step_dt)
        if controller.stage != self._wall_climb_stage:
            self._set_wall_climb_stage(controller.stage)
        if controller.failure_reason is not None:
            self._wall_climb_fail(controller.failure_reason)
            assert self._hold_controller is not None
            self.last_action_source = "hold"
            return self._hold_controller.act()

        if controller.done:
            if (
                not metrics.wall_cleared
                or metrics.y <= WALL_CLEAR_Y
                or metrics.z < WALL_CLIMB_MIN_FINAL_Z
            ):
                self._wall_climb_fail("wall_climb_exit_not_cleared")
                assert self._hold_controller is not None
                self.last_action_source = "hold"
                return self._hold_controller.act()
            self._set_wall_climb_stage("done")
            self.request_state(RobotState.STAND)
            self.last_action_source = "wall_climb_done"
            return action

        self.last_action_source = "wall_climb_ik"
        return action

    def _autopilot_action(self, obs: Any) -> torch.Tensor:
        metrics = self.last_metrics
        self._autopilot_stage_elapsed += self.step_dt
        timeout = AUTOPILOT_STAGE_TIMEOUTS.get(self._autopilot_stage)
        if timeout is not None and self._autopilot_stage_elapsed > timeout:
            self._autopilot_fail(f"{self._autopilot_stage}_timeout")
            assert self._hold_controller is not None
            return self._hold_controller.act()

        if self._autopilot_stage == "stand":
            assert self._pose_controller is not None
            action = self._act_pose(self._pose_controller)
            if self._pose_controller.done:
                self.policy_bank.activate("base")
                staging = APPROACH_WAYPOINTS[-1]
                staging_distance = math.hypot(
                    staging[0] - metrics.x,
                    staging[1] - metrics.y,
                )
                if staging_distance <= STAGING_TOLERANCE:
                    self._set_autopilot_stage("align")
                    print("[AUTO] already at bar staging; aligning to +y")
                else:
                    self._set_autopilot_stage("approach")
                    print("[AUTO] stand complete; approaching the south side of the bar")
            return action

        if self._autopilot_stage == "approach":
            goal = APPROACH_WAYPOINTS[self._autopilot_waypoint]
            command = self.navigation.command_to_waypoint(
                (metrics.x, metrics.y),
                metrics.yaw,
                goal,
            )
            self._set_command(*command)
            action = self._act_rl(obs, "base")
            distance = math.hypot(goal[0] - metrics.x, goal[1] - metrics.y)
            if distance < self._approach_best_distance - 0.03:
                self._approach_best_distance = distance
                self._approach_stall_time = 0.0
            else:
                self._approach_stall_time += self.step_dt
            if self._approach_stall_time > 12.0:
                self._autopilot_fail("approach_stalled")
                assert self._hold_controller is not None
                return self._hold_controller.act()
            if distance < NAV_GOAL_TOLERANCE:
                self._autopilot_waypoint += 1
                if self._autopilot_waypoint >= len(APPROACH_WAYPOINTS):
                    self._set_autopilot_stage("align")
                    print("[AUTO] reached crawl staging point; aligning to +y")
            return action

        if self._autopilot_stage == "align":
            command = self.navigation.command_forward(
                metrics.yaw,
                math.pi / 2.0,
                0.70,
            )
            self._set_command(*command)
            action = self._act_rl(obs, "base")
            yaw_error = abs(wrap_angle(math.pi / 2.0 - metrics.yaw))
            self._align_hold_time = self._align_hold_time + self.step_dt if yaw_error < 0.10 else 0.0
            if self._align_hold_time >= 0.35:
                self._set_command(0.0, 0.0, 0.0)
                self._set_autopilot_stage("lower")
                self._pose_controller = PoseBlendController(self.mapper, LOW_JOINTS)
                print("[AUTO] heading aligned; lowering posture")
            return action

        if self._autopilot_stage == "lower":
            self._set_command(0.0, 0.0, 0.0)
            assert self._pose_controller is not None
            action = self._act_pose(self._pose_controller)
            if self._pose_controller.done:
                self._set_autopilot_stage("crawl")
                self._crawl_controller = None
                print("[AUTO] low posture reached; starting experimental crawl")
            return action

        if self._autopilot_stage == "crawl":
            self._set_command(0.0, 0.0, 0.0)
            action = self._act_crawl(moving=True)
            if metrics.bar_contact:
                self._autopilot_fail("limit_bar_contact")
                assert self._hold_controller is not None
                return self._hold_controller.act()
            if metrics.bar_clean_crossed and metrics.y > CRAWL_EXIT_Y:
                self._set_autopilot_stage("recover")
                self._pose_controller = PoseBlendController(self.mapper, STAND_JOINTS)
                print("[AUTO] bar crossing detected; recovering stance")
            return action

        if self._autopilot_stage == "recover":
            assert self._pose_controller is not None
            action = self._act_pose(self._pose_controller)
            if self._pose_controller.done:
                self._set_autopilot_stage("complete")
                self.request_state(RobotState.STAND)
                print("[AUTO] sequence complete; returning to STAND")
            return action

        self._hold_controller = self._hold_controller or HoldController(self.mapper)
        self.last_action_source = "hold"
        return self._hold_controller.act()

    def __call__(self, obs: Any) -> torch.Tensor:
        return self.act(obs)

    def act(self, obs: Any) -> torch.Tensor:
        if self._reset_requested.is_set():
            self._reset_to_origin()
        dt = self.step_dt
        self.state_elapsed += dt
        self._drain_requests()
        self.last_metrics = self.monitor.measure()

        if self.state == RobotState.STAND:
            self._set_command(0.0, 0.0, 0.0)
            assert self._pose_controller is not None
            return self._act_pose(self._pose_controller)

        if self.state == RobotState.LIE_DOWN:
            self._set_command(0.0, 0.0, 0.0)
            assert self._pose_controller is not None
            return self._act_pose(self._pose_controller)

        if self.state == RobotState.RECOVER:
            self._set_command(0.0, 0.0, 0.0)
            assert self._pose_controller is not None
            action = self._act_pose(self._pose_controller)
            if self._pose_controller.done:
                self.request_state(RobotState.STAND)
            return action

        if self.state == RobotState.LOCOMOTION:
            self._set_command(self.manual_vx, self.manual_vy, self.manual_wz)
            return self._act_rl(obs, "base")

        if self.state == RobotState.CLIMB:
            self._set_command(MANUAL_CLIMB_VX, 0.0, 0.0)
            return self._act_rl(obs, "climb")

        if self.state == RobotState.JUMP:
            # The jump checkpoint is autonomous-by-observation; this command is
            # intentionally zero rather than pretending it is a bar-cross skill.
            self._set_command(0.0, 0.0, 0.0)
            return self._act_rl(obs, "jump")

        if self.state == RobotState.CRAWL_FORWARD:
            self._set_command(0.0, 0.0, 0.0)
            if self._crawl_lowering is not None and not self._crawl_lowering.done:
                action = self._act_pose(self._crawl_lowering)
            else:
                self._crawl_lowering = None
                action = self._act_crawl(moving=True)
            if (
                self.last_metrics.bar_clean_crossed
                and self.last_metrics.y > CRAWL_EXIT_Y
            ):
                self.request_state(RobotState.RECOVER)
            return action

        if self.state == RobotState.AUTOPILOT:
            return self._autopilot_action(obs)

        if self.state == RobotState.BRIDGE_B_CROSS:
            return self._bridge_cross_action(obs)

        if self.state == RobotState.WALL_CROSS:
            return self._wall_cross_action(obs)

        if self.state == RobotState.WALL_CLIMB:
            return self._wall_climb_action(obs)

        if self.state in (RobotState.HOLD, RobotState.EMERGENCY_STOP):
            self._set_command(0.0, 0.0, 0.0)
            assert self._hold_controller is not None
            self.last_action_source = "hold"
            return self._hold_controller.act()

        raise RuntimeError(f"Unhandled FSM state: {self.state}")

    def reset(self, dones: torch.Tensor | None = None) -> None:
        """Reset GRU buffers after an environment reset, not after every step."""
        self.policy_bank.reset(dones)
        if dones is None or bool(torch.any(dones).item()):
            self.monitor.reset_tracking()
            self.request_state(RobotState.STAND)
