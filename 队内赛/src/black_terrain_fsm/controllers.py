"""Fixed-pose, navigation and experimental low-crawl controllers."""

from __future__ import annotations

import math
from collections import deque
from typing import Any

import mujoco
import numpy as np
import torch

from .config import (
    ACTION_SCALE,
    BRIDGE_B_FRONT_EDGE_TOLERANCE_M,
    BRIDGE_B_FRONT_EDGE_Y,
    BRIDGE_B_HEADING_GAIN,
    BRIDGE_B_MAX_HEADING_WZ,
    BRIDGE_B_MAX_LATERAL_DRIFT_M,
    BRIDGE_B_MAX_PITCH_RAD,
    BRIDGE_B_MAX_ROLL_RAD,
    BRIDGE_B_MIN_ROOT_Z,
    BRIDGE_B_SOUTH_EXIT_Y,
    BRIDGE_B_SPAWN_YAW,
    BRIDGE_B_WALK_VX,
    BRIDGE_B_X_RANGE,
    BRIDGE_B_YAW_TOLERANCE,
    GAIT_DUTY,
    GAIT_LEG_OFFSETS,
    GAIT_LEGS,
    GAIT_LIFT_M,
    GAIT_PERIOD_S,
    GAIT_STRIDE_M,
    GAIT_TABLE_SAMPLES,
    JOINT_NAMES,
    LOW_JOINTS,
    NAV_GOAL_TOLERANCE,
    NAV_MIN_VX,
    NAV_MAX_VX,
    NAV_MAX_VY,
    NAV_MAX_WZ,
    POSE_BLEND_TIME_S,
    STAND_JOINTS,
    WALL_VAULT_BRAKE_CLIP_N,
    WALL_VAULT_BRAKE_GAIN,
    WALL_VAULT_FORWARD_FORCE,
    WALL_VAULT_LAUNCH_FORCE,
    WALL_VAULT_LAUNCH_STEPS,
    WALL_VAULT_ATTITUDE_TORQUE_CLIP,
    WALL_VAULT_DEPLOY_STEPS,
    WALL_VAULT_LAND_SETTLE_STEPS,
    WALL_VAULT_LANDING_MIN_CONTACTS,
    WALL_VAULT_LANDING_START_Z,
    WALL_VAULT_PITCH_KD,
    WALL_VAULT_ROLL_KD,
    WALL_VAULT_SETTLE_STEPS,
    WALL_VAULT_TUCK_JOINTS,
    WALL_VAULT_TUCK_STEPS,
    WALL_VAULT_STEPS,
    WALL_VAULT_Z_BRAKE_CLIP_N,
    WALL_VAULT_Z_BRAKE_GAIN,
    WALL_CLIMB_ATTITUDE_Z_CLIP_M,
    WALL_CLIMB_MAX_PITCH_RAD,
    WALL_CLIMB_MAX_ROLL_RAD,
    WALL_CLIMB_MIN_SAFE_ROOT_Z,
    WALL_CLIMB_PITCH_GAIN,
    WALL_CLIMB_POSE_TOLERANCE_M,
    WALL_CLIMB_ROLL_GAIN,
    WALL_CLIMB_ROOT_FEEDBACK_XY,
    WALL_CLIMB_SETTLE_STEPS,
    WALL_CLIMB_SPAWN_POS,
    WALL_CLIMB_SPAWN_YAW,
    WALL_CLIMB_SUPPORT_IK_TOLERANCE_M,
    WALL_CLIMB_SWING_IK_TOLERANCE_M,
)
from .terrain import wrap_angle


def _smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


class JointActionMapper:
    def __init__(self, env: Any) -> None:
        self.env = env
        self.robot = env.scene["robot"]
        self.device = env.device
        self.default_joint_pos = self.robot.data.default_joint_pos[0].detach().clone()
        if tuple(self.robot.joint_names) != JOINT_NAMES:
            raise ValueError(
                "Unexpected Black joint ordering. Expected "
                f"{JOINT_NAMES}, got {tuple(self.robot.joint_names)}"
            )

    def current_joint_pos(self) -> np.ndarray:
        return self.robot.data.joint_pos[0].detach().cpu().numpy().astype(np.float64)

    def raw_action(self, joint_target: np.ndarray | torch.Tensor) -> torch.Tensor:
        if isinstance(joint_target, np.ndarray):
            target = torch.as_tensor(
                joint_target,
                dtype=self.default_joint_pos.dtype,
                device=self.device,
            )
        else:
            target = joint_target.to(self.device)
        raw = (target - self.default_joint_pos) / ACTION_SCALE
        return raw.unsqueeze(0)


class HoldController:
    """Hold the measured joint posture (a software target hold, not an e-stop)."""

    def __init__(self, mapper: JointActionMapper) -> None:
        self.mapper = mapper
        self.target = mapper.current_joint_pos()

    def reset(self) -> None:
        self.target = self.mapper.current_joint_pos()

    def act(self) -> torch.Tensor:
        return self.mapper.raw_action(self.target)


class PoseBlendController:
    def __init__(
        self,
        mapper: JointActionMapper,
        target: tuple[float, ...],
        duration_s: float = POSE_BLEND_TIME_S,
    ) -> None:
        self.mapper = mapper
        self.target = np.asarray(target, dtype=np.float64)
        self.duration_s = float(duration_s)
        self.start = mapper.current_joint_pos()
        self.elapsed = 0.0

    def reset(self) -> None:
        self.start = self.mapper.current_joint_pos()
        self.elapsed = 0.0

    @property
    def done(self) -> bool:
        return self.elapsed >= self.duration_s

    @property
    def progress(self) -> float:
        if self.duration_s <= 0.0:
            return 1.0
        return min(1.0, self.elapsed / self.duration_s)

    def act(self, dt: float) -> torch.Tensor:
        self.elapsed = min(self.duration_s, self.elapsed + dt)
        alpha = _smoothstep(self.progress)
        joints = self.start + (self.target - self.start) * alpha
        return self.mapper.raw_action(joints)


class CrawlGaitController:
    """Four-beat crawl generated from foot targets and offline leg IK.

    The table is built once at startup, so the control loop only interpolates
    joint samples.  This keeps the GUI responsive while preserving the exact
    low-pose collision envelope that was measured in simulation.
    """

    def __init__(
        self,
        env: Any,
        mapper: JointActionMapper,
        *,
        stride_m: float = GAIT_STRIDE_M,
        lift_m: float = GAIT_LIFT_M,
        period_s: float = GAIT_PERIOD_S,
        duty: float = GAIT_DUTY,
        nominal_joints: tuple[float, ...] = LOW_JOINTS,
        table_samples: int = GAIT_TABLE_SAMPLES,
        leg_offsets: dict[str, float] | None = None,
    ) -> None:
        self.env = env
        self.mapper = mapper
        self.model = env.sim.mj_model
        self.data = mujoco.MjData(self.model)
        self.phase = 0.0
        self.stride_m = float(stride_m)
        self.lift_m = float(lift_m)
        self.period_s = float(period_s)
        self.duty = float(duty)
        self.nominal_joints = np.asarray(nominal_joints, dtype=np.float64)
        self.table_samples = int(table_samples)
        self.leg_offsets = dict(GAIT_LEG_OFFSETS if leg_offsets is None else leg_offsets)
        if set(self.leg_offsets) != set(GAIT_LEGS):
            raise ValueError(f"Crawl leg offsets must define exactly {GAIT_LEGS}")
        if self.stride_m == 0.0 or self.period_s <= 0.0:
            raise ValueError("Crawl stride must be nonzero and period positive")
        if not 0.5 < self.duty < 1.0:
            raise ValueError("Crawl duty must be between 0.5 and 1.0")
        if self.table_samples < 8:
            raise ValueError("Crawl table must contain at least 8 samples")
        self._joint_qpos: dict[str, int] = {}
        self._joint_dof: dict[str, int] = {}
        self._joint_limits: dict[str, tuple[float, float]] = {}
        self._site_ids: dict[str, int] = {}
        self._index_joints_and_sites()
        self.table = self._build_table()

    def _index_joints_and_sites(self) -> None:
        for joint_id in range(self.model.njnt):
            full_name = self.model.joint(joint_id).name
            short_name = full_name.rsplit("/", 1)[-1]
            if short_name in JOINT_NAMES:
                self._joint_qpos[short_name] = int(self.model.jnt_qposadr[joint_id])
                self._joint_dof[short_name] = int(self.model.jnt_dofadr[joint_id])
                self._joint_limits[short_name] = (
                    float(self.model.jnt_range[joint_id, 0]),
                    float(self.model.jnt_range[joint_id, 1]),
                )
        for site_id in range(self.model.nsite):
            short_name = self.model.site(site_id).name.rsplit("/", 1)[-1]
            if short_name in ("FL", "FR", "RR", "RL"):
                self._site_ids[short_name] = site_id
        if set(self._joint_qpos) != set(JOINT_NAMES):
            raise RuntimeError("Could not resolve all Black leg joints for crawl IK")
        if set(self._site_ids) != {"FL", "FR", "RR", "RL"}:
            raise RuntimeError("Could not resolve all Black foot sites for crawl IK")

    def _set_pose(self, joints: np.ndarray) -> None:
        self.data.qpos[:] = self.model.qpos0
        self.data.qpos[:3] = 0.0
        self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        for name, value in zip(JOINT_NAMES, joints, strict=True):
            self.data.qpos[self._joint_qpos[name]] = value
        mujoco.mj_forward(self.model, self.data)

    def forward_feet(self, joints: np.ndarray) -> dict[str, np.ndarray]:
        self._set_pose(joints)
        return {
            leg: self.data.site_xpos[site_id].copy()
            for leg, site_id in self._site_ids.items()
        }

    def inverse_kinematics(
        self,
        leg: str,
        target: np.ndarray,
        seed: np.ndarray,
        iterations: int = 30,
    ) -> np.ndarray:
        joints = seed.copy()
        leg_joint_names = (
            f"{leg}_hip_joint",
            f"{leg}_thigh_joint",
            f"{leg}_calf_joint",
        )
        indices = [JOINT_NAMES.index(name) for name in leg_joint_names]
        dofs = [self._joint_dof[name] for name in leg_joint_names]
        for _ in range(iterations):
            self._set_pose(joints)
            error = target - self.data.site_xpos[self._site_ids[leg]]
            if np.linalg.norm(error) < 2e-5:
                break
            jacp = np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(self.model, self.data, jacp, None, self._site_ids[leg])
            jac = jacp[:, dofs]
            delta = jac.T @ np.linalg.solve(
                jac @ jac.T + 1e-4 * np.eye(3),
                error,
            )
            delta = np.clip(delta, -0.07, 0.07)
            for local_index, joint_index in enumerate(indices):
                name = JOINT_NAMES[joint_index]
                low, high = self._joint_limits[name]
                joints[joint_index] = np.clip(
                    joints[joint_index] + delta[local_index],
                    low,
                    high,
                )
        return joints

    def _target_for_phase(
        self,
        nominal_foot: np.ndarray,
        phase: float,
    ) -> np.ndarray:
        if phase < self.duty:
            stance_s = phase / self.duty
            x = self.stride_m * (0.5 - stance_s)
            z = 0.0
        else:
            swing_s = (phase - self.duty) / (1.0 - self.duty)
            x = self.stride_m * (-0.5 + _smoothstep(swing_s))
            z = self.lift_m * math.sin(math.pi * swing_s)
        return nominal_foot + np.array((x, 0.0, z), dtype=np.float64)

    def _build_table(self) -> np.ndarray:
        low = self.nominal_joints
        nominal_feet = self.forward_feet(low)
        table = np.zeros((self.table_samples, len(JOINT_NAMES)), dtype=np.float64)
        for leg in GAIT_LEGS:
            start = JOINT_NAMES.index(f"{leg}_hip_joint")
            seed = low.copy()
            for sample in range(self.table_samples):
                phase = sample / self.table_samples
                target = self._target_for_phase(nominal_feet[leg], phase)
                seed = self.inverse_kinematics(leg, target, seed)
                table[sample, start : start + 3] = seed[start : start + 3]
        return table

    def set_gait(self, stride_m: float, period_s: float) -> None:
        """Update stride/period while preserving the normalized gait phase."""
        stride_m = float(stride_m)
        period_s = float(period_s)
        if stride_m == 0.0 or period_s <= 0.0:
            raise ValueError("Crawl stride must be nonzero and period positive")
        if math.copysign(1.0, stride_m) != math.copysign(1.0, self.stride_m):
            raise ValueError("Crawl stride direction cannot change in place")
        if not math.isclose(stride_m, self.stride_m, abs_tol=1e-9):
            self.stride_m = stride_m
            self.table = self._build_table()
        self.period_s = period_s

    def reset(self) -> None:
        self.phase = 0.0

    def act(self, dt: float, moving: bool = True) -> torch.Tensor:
        if moving:
            self.phase = (self.phase + dt / self.period_s) % 1.0
        joints = np.empty(len(JOINT_NAMES), dtype=np.float64)
        for leg in GAIT_LEGS:
            phase = (self.phase + self.leg_offsets[leg]) % 1.0
            sample = phase * self.table_samples
            index0 = int(sample) % self.table_samples
            index1 = (index0 + 1) % self.table_samples
            alpha = sample - math.floor(sample)
            start = JOINT_NAMES.index(f"{leg}_hip_joint")
            joints[start : start + 3] = (
                self.table[index0, start : start + 3] * (1.0 - alpha)
                + self.table[index1, start : start + 3] * alpha
            )
        return self.mapper.raw_action(joints)


class BridgeBWalkController:
    """One-shot bridge-B crossing action.

    Only the base locomotion policy produces joint actions.  This controller
    supplies its forward command, closes the heading loop against the bridge
    axis, checks the narrow-board safety envelope, and declares completion when
    the trunk has reached the south platform.  There are no plank/turn/lower
    route stages.
    """

    def __init__(self, env: Any) -> None:
        self.robot = env.scene["robot"]
        self.stage = "walk"
        self.done = False
        self.south_reached = False
        self.failure_reason: str | None = None
        self.max_yaw_error = 0.0
        self.max_lateral_drift = 0.0
        self.min_root_z = math.inf
        self.max_pitch = 0.0
        self.max_roll = 0.0
        self.start_x: float | None = None
        self.final_y: float | None = None
        self.front_feet_start = self._front_feet()
        self.front_edge_abs_error = max(
            abs(float(foot[1]) - BRIDGE_B_FRONT_EDGE_Y)
            for foot in self.front_feet_start.values()
        )
        self.front_edge_ok = (
            self.front_edge_abs_error <= BRIDGE_B_FRONT_EDGE_TOLERANCE_M
        )

    def _front_feet(self) -> dict[str, list[float]]:
        positions = self.robot.data.site_pos_w[0].detach().cpu().numpy()
        feet: dict[str, list[float]] = {}
        for leg in ("FL", "FR"):
            site_id = self.robot.site_names.index(leg)
            feet[leg] = positions[site_id].astype(np.float64).tolist()
        return feet

    def _root_state(
        self,
    ) -> tuple[np.ndarray, float, float, float]:
        data = self.robot.data
        position = data.root_link_pos_w[0].detach().cpu().numpy().astype(
            np.float64
        )
        w, x, y, z = (
            float(value)
            for value in data.root_link_quat_w[0].detach().cpu().numpy()
        )
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        pitch = math.asin(
            float(
                np.clip(
                    2.0 * (w * y - z * x),
                    -1.0,
                    1.0,
                )
            )
        )
        roll = math.atan2(
            2.0 * (w * x + y * z),
            1.0 - 2.0 * (x * x + y * y),
        )
        return position, yaw, pitch, roll

    def observe(self) -> None:
        """Update completion/safety state from the current robot pose."""
        if self.done or self.failure_reason is not None:
            return
        position, yaw, pitch, roll = self._root_state()
        if self.start_x is None:
            self.start_x = float(position[0])
        self.min_root_z = min(self.min_root_z, float(position[2]))
        self.max_pitch = max(self.max_pitch, abs(pitch))
        self.max_roll = max(self.max_roll, abs(roll))
        yaw_error = abs(wrap_angle(yaw - BRIDGE_B_SPAWN_YAW))
        self.max_yaw_error = max(self.max_yaw_error, yaw_error)
        self.max_lateral_drift = max(
            self.max_lateral_drift,
            abs(float(position[0]) - self.start_x),
        )
        self.final_y = float(position[1])

        if float(position[2]) < BRIDGE_B_MIN_ROOT_Z:
            self._fail("robot_fell_below_safe_height")
        elif abs(float(position[0]) - self.start_x) > BRIDGE_B_MAX_LATERAL_DRIFT_M:
            self._fail("lateral_drift_outside_bridge")
        elif yaw_error > BRIDGE_B_YAW_TOLERANCE:
            self._fail("yaw_drift_outside_bridge")
        elif self.max_pitch > BRIDGE_B_MAX_PITCH_RAD:
            self._fail("pitch_limit_exceeded")
        elif self.max_roll > BRIDGE_B_MAX_ROLL_RAD:
            self._fail("roll_limit_exceeded")
        elif float(position[1]) <= BRIDGE_B_SOUTH_EXIT_Y:
            self.south_reached = True
            self.done = True
            self.stage = "complete"

    def command(self) -> tuple[float, float, float]:
        """Return the base-policy command for the current control step."""
        if self.done or self.failure_reason is not None:
            return 0.0, 0.0, 0.0
        _, yaw, _, _ = self._root_state()
        yaw_error = wrap_angle(BRIDGE_B_SPAWN_YAW - yaw)
        wz = float(
            np.clip(
                BRIDGE_B_HEADING_GAIN * yaw_error,
                -BRIDGE_B_MAX_HEADING_WZ,
                BRIDGE_B_MAX_HEADING_WZ,
            )
        )
        return BRIDGE_B_WALK_VX, 0.0, wz

    def _fail(self, reason: str) -> None:
        if self.failure_reason is None:
            self.failure_reason = reason
            self.stage = "failed"

    @property
    def diagnostics(self) -> dict[str, Any]:
        return {
            "start_x": self.start_x,
            "final_y": self.final_y,
            "max_yaw_error_rad": self.max_yaw_error,
            "max_lateral_drift_m": self.max_lateral_drift,
            "min_root_z": (
                None if not math.isfinite(self.min_root_z) else self.min_root_z
            ),
            "max_pitch_rad": self.max_pitch,
            "max_roll_rad": self.max_roll,
            "x_range_m": BRIDGE_B_X_RANGE,
            "front_edge_target_y_m": BRIDGE_B_FRONT_EDGE_Y,
            "front_edge_tolerance_m": BRIDGE_B_FRONT_EDGE_TOLERANCE_M,
            "front_edge_abs_error_m": self.front_edge_abs_error,
            "front_edge_ok": self.front_edge_ok,
            "front_feet_start_m": self.front_feet_start,
        }


class WallClimbController:
    """Experimental quasi-static wall climb using only foot IK.

    This controller deliberately never writes ``xfrc_applied`` or any other
    external wrench.  It maintains three support feet while moving one leg at
    a time and uses the measured trunk pose to compensate the support targets.
    Leg contacts with the wall are allowed; the state machine treats any trunk
    contact as a hard failure.
    """

    LEGS = ("FL", "FR", "RR", "RL")

    def __init__(self, env: Any, mapper: JointActionMapper) -> None:
        self.env = env
        self.mapper = mapper
        self.robot = env.scene["robot"]
        self.gait = CrawlGaitController(env, mapper)
        self.stand = np.asarray(STAND_JOINTS, dtype=np.float64)
        self.joints = mapper.current_joint_pos()
        self.stage = "settle_stand"
        self.stage_step = 0
        self.elapsed = 0.0
        self.done = False
        self.failure_reason: str | None = None
        self.max_ik_error = 0.0
        self.min_root_z = math.inf
        self.max_pitch = 0.0
        self.max_roll = 0.0
        self.last_ik_errors = {leg: 0.0 for leg in self.LEGS}
        self.joint_limit_hits: set[str] = set()
        self._last_diag: dict[str, dict[str, Any]] = {}
        self._queue: deque[dict[str, Any]] = deque()
        self._active: dict[str, Any] | None = None
        self._pose_start: np.ndarray | None = None
        self._swing_start: np.ndarray | None = None
        self._swing_target: np.ndarray | None = None
        self._swing_phase = "lift"
        self._swing_step = 0
        self._anchors: dict[str, np.ndarray] = {}
        self._plan_pos = np.asarray(WALL_CLIMB_SPAWN_POS, dtype=np.float64)
        self._plan_rotation = self._body_rotation(WALL_CLIMB_SPAWN_YAW)
        self._route_built = False

    @staticmethod
    def _body_rotation(yaw: float, pitch: float = 0.0) -> np.ndarray:
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        return np.array(
            (
                (cy * cp, -sy, -cy * sp),
                (sy * cp, cy, -sy * sp),
                (sp, 0.0, cp),
            ),
            dtype=np.float64,
        )

    def _root_state(
        self,
    ) -> tuple[np.ndarray, np.ndarray, float, float, float]:
        data = self.robot.data
        pos = data.root_link_pos_w[0].detach().cpu().numpy().astype(np.float64)
        quat = data.root_link_quat_w[0].detach().cpu().numpy()
        w, x, y, z = (float(value) for value in quat)
        rotation = np.array(
            (
                (
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - w * z),
                    2.0 * (x * z + w * y),
                ),
                (
                    2.0 * (x * y + w * z),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - w * x),
                ),
                (
                    2.0 * (x * z - w * y),
                    2.0 * (y * z + w * x),
                    1.0 - 2.0 * (x * x + y * y),
                ),
            ),
            dtype=np.float64,
        )
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        pitch = math.asin(float(np.clip(rotation[2, 0], -1.0, 1.0)))
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        return pos, rotation, yaw, pitch, roll

    def _feet_world(
        self,
        pos: np.ndarray,
        rotation: np.ndarray,
    ) -> dict[str, np.ndarray]:
        local = self.gait.forward_feet(self.mapper.current_joint_pos())
        return {leg: pos + rotation @ local[leg] for leg in self.LEGS}

    @staticmethod
    def _pose(
        target: tuple[float, float, float],
        steps: int,
        label: str,
    ) -> dict[str, Any]:
        return {
            "kind": "pose",
            "target": np.asarray(target, dtype=np.float64),
            "steps": int(steps),
            "label": label,
        }

    @staticmethod
    def _swing(
        leg: str,
        target: tuple[float, float, float],
        lift_z: float,
        label: str,
        *,
        lift: int = 30,
        advance: int = 45,
        lower: int = 30,
        hold: int = 22,
        lift_xy_fraction: float = 0.0,
    ) -> dict[str, Any]:
        return {
            "kind": "swing",
            "leg": leg,
            "target": np.asarray(target, dtype=np.float64),
            "lift_z": float(lift_z),
            "label": label,
            "lift_steps": int(lift),
            "advance_steps": int(advance),
            "lower_steps": int(lower),
            "hold_steps": int(hold),
            "lift_xy_fraction": float(lift_xy_fraction),
        }

    def _build_route(self) -> None:
        """Build the fixed route measured in the standalone feasibility probe."""
        if self._route_built:
            return
        self._route_built = True
        pose = self._pose
        swing = self._swing
        q = self._queue

        # Rear-foot compaction round 1: establish enough support-polygon
        # length for the trunk to approach the south wall face.
        q.append(pose((1.85, -7.48, 0.46), 42, "settle_crouch"))
        q.append(pose((1.82, -7.48, 0.46), 42, "shift_left_for_rr_far"))
        q.append(
            swing(
                "RR", (1.975, -7.60, 0.02), 0.06, "rr_far_prepose",
                lift=36, advance=50, lower=36, hold=26,
            )
        )
        q.append(pose((1.85, -7.48, 0.46), 42, "center_after_rr_far"))
        q.append(pose((1.91, -7.48, 0.46), 42, "shift_right_for_rl_far"))
        q.append(
            swing(
                "RL", (1.755, -7.60, 0.02), 0.06, "rl_far_prepose",
                lift=36, advance=50, lower=36, hold=26,
            )
        )
        q.append(pose((1.85, -7.48, 0.46), 42, "center_after_rl_far"))
        q.append(pose((1.85, -7.44, 0.46), 45, "advance_between_rear_rounds"))

        # Rear-foot compaction round 2: feet move under the hips while the
        # trunk stays south of the wall's camera-clearance line.
        q.append(pose((1.82, -7.38, 0.46), 42, "shift_left_for_rr_near"))
        q.append(
            swing(
                "RR", (1.975, -7.50, 0.02), 0.06, "rr_near_prepose",
                lift=36, advance=50, lower=36, hold=26,
            )
        )
        q.append(pose((1.85, -7.38, 0.46), 42, "center_after_rr_near"))
        q.append(pose((1.88, -7.38, 0.46), 42, "shift_right_for_rl_near"))
        q.append(
            swing(
                "RL", (1.725, -7.44, 0.02), 0.06, "rl_near_prepose",
                lift=36, advance=50, lower=36, hold=26,
            )
        )
        q.append(pose((1.85, -7.38, 0.46), 42, "center_after_rl_near"))

        # Place the front feet on the wall crown, one at a time.
        q.append(pose((1.88, -7.40, 0.46), 42, "shift_right_for_fl_wall"))
        q.append(
            swing(
                "FL", (1.750, -7.00, 0.32), 0.38, "fl_on_wall",
                lift=40, advance=50, lower=34, hold=22,
                lift_xy_fraction=1.0,
            )
        )
        q.append(pose((1.88, -7.40, 0.46), 60, "settle_after_fl_wall"))
        q.append(pose((1.86, -7.40, 0.46), 120, "shift_left_for_fr_wall"))
        q.append(
            swing(
                "FR", (1.955, -7.05, 0.31), 0.38, "fr_on_wall",
                lift=40, advance=50, lower=34, hold=22,
                lift_xy_fraction=1.0,
            )
        )
        q.append(pose((1.85, -7.44, 0.46), 42, "center_on_wall"))
        q.append(pose((1.85, -7.08, 0.46), 85, "advance_over_wall"))

        # Move the front feet to the north bank.
        q.append(pose((1.91, -7.08, 0.46), 42, "shift_right_for_fl_north"))
        q.append(
            swing(
                "FL", (1.755, -6.86, 0.02), 0.30, "fl_north_ground",
                lift=22, advance=36, lower=28, hold=20,
            )
        )
        q.append(pose((1.79, -7.08, 0.46), 42, "shift_left_for_fr_north"))
        q.append(
            swing(
                "FR", (1.945, -6.86, 0.02), 0.30, "fr_north_ground",
                lift=22, advance=36, lower=28, hold=20,
            )
        )
        q.append(pose((1.85, -7.08, 0.46), 42, "center_north_front"))

        # Bring the rear feet to the south face, then use the crown as an
        # intermediate foothold before stepping onto the north bank.
        q.append(pose((1.79, -7.08, 0.46), 42, "shift_left_for_rr_wall"))
        q.append(
            swing(
                "RR", (1.945, -7.16, 0.02), 0.16, "rr_at_wall",
                lift=20, advance=32, lower=24, hold=18,
            )
        )
        q.append(pose((1.91, -7.08, 0.46), 42, "shift_right_for_rl_wall"))
        q.append(
            swing(
                "RL", (1.755, -7.16, 0.02), 0.16, "rl_at_wall",
                lift=20, advance=32, lower=24, hold=18,
            )
        )
        q.append(pose((1.85, -6.90, 0.46), 62, "advance_rear_body"))
        q.append(pose((1.87, -6.90, 0.46), 36, "shift_left_for_rr_crown"))
        q.append(
            swing(
                "RR", (2.005, -7.00, 0.32), 0.43, "rr_on_wall",
                lift=22, advance=34, lower=26, hold=18,
            )
        )
        q.append(pose((1.83, -6.90, 0.46), 36, "shift_right_for_rl_crown"))
        q.append(
            swing(
                "RL", (1.695, -7.00, 0.32), 0.43, "rl_on_wall",
                lift=22, advance=34, lower=26, hold=18,
            )
        )
        q.append(pose((1.85, -6.78, 0.46), 55, "advance_exit_body"))

        q.append(
            swing(
                "RR", (2.005, -6.75, 0.02), 0.30, "rr_north_ground",
                lift=20, advance=32, lower=26, hold=18,
            )
        )
        q.append(
            swing(
                "RL", (1.695, -6.75, 0.02), 0.30, "rl_north_ground",
                lift=20, advance=32, lower=26, hold=18,
            )
        )
        q.append(pose((1.85, -6.58, 0.47), 55, "advance_north"))
        q.append({
            "kind": "stand",
            "target": self.stand.copy(),
            "steps": 70,
            "label": "recover_stand",
            "start": None,
        })

    def _begin_next(self) -> None:
        if not self._queue:
            self._active = None
            self.stage = "done"
            self.done = True
            return
        self._active = self._queue.popleft()
        self.stage = str(self._active["label"])
        self.stage_step = 0
        if self._active["kind"] == "pose":
            self._pose_start = self._plan_pos.copy()
        elif self._active["kind"] == "swing":
            leg = str(self._active["leg"])
            self._swing_start = self._anchors[leg].copy()
            self._swing_target = np.asarray(
                self._active["target"], dtype=np.float64
            ).copy()
            self._swing_phase = "lift"
            self._swing_step = 0

    def _solve_targets(
        self,
        pos: np.ndarray,
        rotation: np.ndarray,
        pitch: float,
        roll: float,
        *,
        swing_leg: str = "",
        swing_target: np.ndarray | None = None,
    ) -> tuple[torch.Tensor, np.ndarray, dict[str, float]]:
        joints = self.joints.copy()
        action = self.mapper.raw_action(joints)
        correction = np.zeros(3, dtype=np.float64)
        correction[:2] = WALL_CLIMB_ROOT_FEEDBACK_XY * (
            pos - self._plan_pos
        )[:2]
        targets = {
            leg: value.copy() + correction
            for leg, value in self._anchors.items()
        }
        if swing_leg:
            targets[swing_leg] = np.asarray(swing_target, dtype=np.float64)
        errors: dict[str, float] = {}
        self._last_diag = {}
        for leg in self.LEGS:
            # The right-front foothold lies at the edge of the leg workspace.
            # Only that swing uses the measured trunk translation; applying it
            # to every swing destabilises the established rear-foot route.
            leg_base = (
                pos
                if swing_leg == "FR" and leg == swing_leg
                else self._plan_pos
            )
            local_target = self._plan_rotation.T @ (
                targets[leg] - leg_base
            )
            if leg != swing_leg:
                forward_sign = 1.0 if leg in ("FL", "FR") else -1.0
                lateral_sign = 1.0 if leg in ("FL", "RL") else -1.0
                attitude_z = (
                    WALL_CLIMB_PITCH_GAIN * pitch * forward_sign
                    + WALL_CLIMB_ROLL_GAIN * roll * lateral_sign
                )
                local_target[2] += float(
                    np.clip(
                        attitude_z,
                        -WALL_CLIMB_ATTITUDE_Z_CLIP_M,
                        WALL_CLIMB_ATTITUDE_Z_CLIP_M,
                    )
                )
            solved = self.gait.inverse_kinematics(
                leg, local_target, joints, iterations=220
            )
            begin = JOINT_NAMES.index(f"{leg}_hip_joint")
            raw = self.mapper.raw_action(solved)
            action[:, begin : begin + 3].copy_(raw[:, begin : begin + 3])
            solved_foot = self.gait.forward_feet(solved)[leg]
            error = float(np.linalg.norm(solved_foot - local_target))
            errors[leg] = error
            limit_names = (
                f"{leg}_hip_joint",
                f"{leg}_thigh_joint",
                f"{leg}_calf_joint",
            )
            limits = np.asarray(
                [self.gait._joint_limits[name] for name in limit_names]
            )
            q = solved[begin : begin + 3]
            if np.any((np.abs(q - limits[:, 0]) < 1e-4)
                      | (np.abs(q - limits[:, 1]) < 1e-4)):
                self.joint_limit_hits.add(leg)
            self._last_diag[leg] = {
                "target_local": local_target.copy(),
                "solved_local": solved_foot.copy(),
                "error_local": solved_foot - local_target,
                "joints": q.copy(),
                "limits": limits.copy(),
            }
            joints[begin : begin + 3] = q
        self.joints = joints
        self.last_ik_errors = errors
        self.max_ik_error = max(self.max_ik_error, max(errors.values()))
        return action, joints, errors

    def fail(self, reason: str) -> None:
        if self.failure_reason is None:
            self.failure_reason = reason
        self.stage = "failed"

    def act(self, dt: float) -> torch.Tensor:
        self.elapsed += float(dt)
        pos, rotation, yaw, pitch, roll = self._root_state()
        self.min_root_z = min(self.min_root_z, float(pos[2]))
        self.max_pitch = max(self.max_pitch, abs(pitch))
        self.max_roll = max(self.max_roll, abs(roll))
        if self.failure_reason is not None:
            return self.mapper.raw_action(self.joints)

        action: torch.Tensor | None = None
        if self.stage == "settle_stand":
            alpha = _smoothstep((self.stage_step + 1) / WALL_CLIMB_SETTLE_STEPS)
            self.joints = self.joints * (1.0 - alpha) + self.stand * alpha
            action = self.mapper.raw_action(self.joints)
            self.stage_step += 1
            if self.stage_step >= WALL_CLIMB_SETTLE_STEPS:
                pos, rotation, yaw, pitch, roll = self._root_state()
                self._anchors = self._feet_world(pos, rotation)
                self._plan_pos = pos.copy()
                self._plan_rotation = self._body_rotation(yaw)
                self._build_route()
                self._begin_next()
            return action

        if self.stage == "done":
            return self.mapper.raw_action(self.joints)

        if self._active is None:
            self._begin_next()
            action = self.mapper.raw_action(self.joints)
        elif self._active["kind"] == "pose":
            assert self._pose_start is not None
            alpha = _smoothstep((self.stage_step + 1) / self._active["steps"])
            self._plan_pos = (
                self._pose_start * (1.0 - alpha)
                + self._active["target"] * alpha
            )
            action, _, errors = self._solve_targets(
                pos, rotation, pitch, roll
            )
            self.stage_step += 1
            max_ik = max(errors.values())
            if max_ik > WALL_CLIMB_SUPPORT_IK_TOLERANCE_M:
                self.fail(f"{self.stage}_support_ik_error_{max_ik:.3f}")
            elif self.stage_step >= self._active["steps"]:
                y_error = abs(float(pos[1] - self._active["target"][1]))
                if (
                    y_error > WALL_CLIMB_POSE_TOLERANCE_M
                    and self.stage not in ("advance_north",)
                ):
                    self.fail(f"{self.stage}_not_reached_{y_error:.3f}")
                elif self.stage == "recover_stand":
                    self.stage = "done"
                    self.done = True
                else:
                    self._begin_next()
        elif self._active["kind"] == "swing":
            assert self._swing_start is not None
            assert self._swing_target is not None
            leg = str(self._active["leg"])
            if self._swing_phase == "lift":
                alpha = _smoothstep(
                    (self._swing_step + 1) / self._active["lift_steps"]
                )
                target = self._swing_start.copy()
                target[0] += (
                    (self._swing_target[0] - self._swing_start[0])
                    * self._active["lift_xy_fraction"] * alpha
                )
                target[1] += (
                    (self._swing_target[1] - self._swing_start[1])
                    * self._active["lift_xy_fraction"] * alpha
                )
                target[2] = (
                    self._swing_start[2] * (1.0 - alpha)
                    + self._active["lift_z"] * alpha
                )
                self._swing_step += 1
                if self._swing_step >= self._active["lift_steps"]:
                    self._swing_phase = "advance"
                    self._swing_step = 0
            elif self._swing_phase == "advance":
                alpha = _smoothstep(
                    (self._swing_step + 1) / self._active["advance_steps"]
                )
                begin = self._swing_start.copy()
                begin[0] += (
                    (self._swing_target[0] - self._swing_start[0])
                    * self._active["lift_xy_fraction"]
                )
                begin[1] += (
                    (self._swing_target[1] - self._swing_start[1])
                    * self._active["lift_xy_fraction"]
                )
                target = begin.copy()
                target[0] = (
                    begin[0] * (1.0 - alpha) + self._swing_target[0] * alpha
                )
                target[1] = (
                    begin[1] * (1.0 - alpha) + self._swing_target[1] * alpha
                )
                target[2] = self._active["lift_z"]
                self._swing_step += 1
                if self._swing_step >= self._active["advance_steps"]:
                    self._swing_phase = "lower"
                    self._swing_step = 0
            elif self._swing_phase == "lower":
                alpha = _smoothstep(
                    (self._swing_step + 1) / self._active["lower_steps"]
                )
                target = self._swing_target.copy()
                target[2] = (
                    self._active["lift_z"] * (1.0 - alpha)
                    + self._swing_target[2] * alpha
                )
                self._swing_step += 1
                if self._swing_step >= self._active["lower_steps"]:
                    self._swing_phase = "hold"
                    self._swing_step = 0
            else:
                target = self._swing_target.copy()
                self._swing_step += 1
                if self._swing_step >= self._active["hold_steps"]:
                    self._anchors[leg] = self._swing_target.copy()
                    self._begin_next()
                    target = self._anchors[leg].copy()
            action, _, errors = self._solve_targets(
                pos,
                rotation,
                pitch,
                roll,
                swing_leg=leg,
                swing_target=target,
            )
            if errors[leg] > WALL_CLIMB_SWING_IK_TOLERANCE_M:
                self.fail(
                    f"{self.stage}_swing_ik_error_{errors[leg]:.3f}"
                )
            else:
                support_error = max(
                    value for key, value in errors.items() if key != leg
                )
                if support_error > WALL_CLIMB_SUPPORT_IK_TOLERANCE_M:
                    self.fail(
                        f"{self.stage}_support_ik_error_{support_error:.3f}"
                    )
        elif self._active["kind"] == "stand":
            if self._active["start"] is None:
                self._active["start"] = self.joints.copy()
            alpha = _smoothstep(
                (self.stage_step + 1) / self._active["steps"]
            )
            self.joints = (
                self._active["start"] * (1.0 - alpha)
                + self._active["target"] * alpha
            )
            action = self.mapper.raw_action(self.joints)
            self.stage_step += 1
            if self.stage_step >= self._active["steps"]:
                self.stage = "done"
                self.done = True

        if self.failure_reason is not None:
            self.stage = "failed"
        if action is None:
            action = self.mapper.raw_action(self.joints)
        return action

    @property
    def diagnostics(self) -> dict[str, Any]:
        return {
            "max_ik_error": self.max_ik_error,
            "last_ik_errors": dict(self.last_ik_errors),
            "min_root_z": (
                None if not math.isfinite(self.min_root_z) else self.min_root_z
            ),
            "max_pitch_rad": self.max_pitch,
            "max_roll_rad": self.max_roll,
            "joint_limit_hits": sorted(self.joint_limit_hits),
            "ik_diagnostics": {
                leg: {
                    "target_local": diag["target_local"].tolist(),
                    "solved_local": diag["solved_local"].tolist(),
                    "error_local": diag["error_local"].tolist(),
                    "joints": diag["joints"].tolist(),
                }
                for leg, diag in self._last_diag.items()
            },
        }


class WallVaultController:
    """Validated whole-body vault over the supplied 0.30 m vertical wall.

    The motion is deliberately short and open-loop in joint space, with only
    root attitude, braking and stage timing closed loop.  It is a fixed terrain
    skill for the specified wall/spawn, not a universal wall-climbing policy.
    """

    def __init__(self, env: Any, mapper: JointActionMapper) -> None:
        self.env = env
        self.mapper = mapper
        self.robot = env.scene["robot"]
        self.stand = np.asarray(STAND_JOINTS, dtype=np.float64)
        self.tuck = np.asarray(WALL_VAULT_TUCK_JOINTS, dtype=np.float64)
        self.zero = torch.zeros((1, 1, 3), dtype=torch.float32, device=env.device)
        self.launch_force = torch.tensor(
            [[[*WALL_VAULT_LAUNCH_FORCE]]],
            dtype=torch.float32,
            device=env.device,
        )
        self.forward_force = torch.tensor(
            [[[*WALL_VAULT_FORWARD_FORCE]]],
            dtype=torch.float32,
            device=env.device,
        )
        self.stage = "settle"
        self.stage_step = 0
        self.done = False
        self.failure_reason: str | None = None
        self._settle_start = mapper.current_joint_pos()
        self._tuck_start: np.ndarray | None = None
        self._recover_start: np.ndarray | None = None
        self._landing_stable_steps = 0
        self.last_pitch = 0.0
        self.last_roll = 0.0
        self.last_linear_velocity = np.zeros(3, dtype=np.float64)
        self.last_foot_contacts = 0
        self.last_landing_force_z = 0.0

    def _root_state(
        self,
    ) -> tuple[np.ndarray, float, float, np.ndarray, np.ndarray]:
        position = (
            self.robot.data.root_link_pos_w[0].detach().cpu().numpy()
        )
        quat = self.robot.data.root_link_quat_w[0].detach().cpu().numpy()
        w, x, y, z = (float(value) for value in quat)
        rotation = np.array(
            (
                (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
                (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
                (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
            ),
            dtype=np.float64,
        )
        pitch = math.asin(float(np.clip(rotation[2, 0], -1.0, 1.0)))
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        linear_velocity = (
            self.robot.data.root_link_lin_vel_w[0].detach().cpu().numpy()
        )
        angular_velocity = (
            self.robot.data.root_link_ang_vel_w[0].detach().cpu().numpy()
        )
        return position, pitch, roll, linear_velocity, angular_velocity

    def _foot_contact_count(self) -> int:
        """Return how many feet the contact sensor currently reports down."""
        sensor = self.env.scene.sensors.get("feet_ground_contact")
        if sensor is None:
            return 0
        found = sensor.data.found[0].detach().cpu().numpy()
        return int(np.count_nonzero(found > 0.5))

    def _landing_wrench(
        self,
        position: np.ndarray,
        linear_velocity: np.ndarray,
        foot_contacts: int,
    ) -> torch.Tensor:
        """Brake horizontal motion and cushion the descent after the vault."""
        brake_x = float(
            np.clip(
                -WALL_VAULT_BRAKE_GAIN * linear_velocity[0],
                -WALL_VAULT_BRAKE_CLIP_N,
                WALL_VAULT_BRAKE_CLIP_N,
            )
        )
        brake_y = float(
            np.clip(
                -WALL_VAULT_BRAKE_GAIN * linear_velocity[1],
                -WALL_VAULT_BRAKE_CLIP_N,
                WALL_VAULT_BRAKE_CLIP_N,
            )
        )
        force_z = 0.0
        if float(position[2]) < WALL_VAULT_LANDING_START_Z:
            if float(linear_velocity[2]) < -0.08:
                force_z = float(
                    np.clip(
                        -WALL_VAULT_Z_BRAKE_GAIN * linear_velocity[2],
                        0.0,
                        WALL_VAULT_Z_BRAKE_CLIP_N,
                    )
                )
            elif foot_contacts >= WALL_VAULT_LANDING_MIN_CONTACTS:
                # Suppress the post-impact rebound while the feet still touch.
                force_z = float(
                    np.clip(
                        -0.35 * WALL_VAULT_Z_BRAKE_GAIN * linear_velocity[2],
                        -0.25 * WALL_VAULT_Z_BRAKE_CLIP_N,
                        0.0,
                    )
                )
        self.last_landing_force_z = force_z
        return torch.tensor(
            [[[brake_x, brake_y, force_z]]],
            dtype=torch.float32,
            device=self.env.device,
        )

    def stop(self) -> None:
        """Clear the external wrench immediately, e.g. after a failure."""
        self.robot.write_external_wrench_to_sim(
            self.zero,
            self.zero,
            body_ids=[0],
        )

    def fail(self, reason: str) -> None:
        self.failure_reason = reason
        self.stage = "failed"
        self.done = False
        self.stop()

    def act(self, dt: float) -> torch.Tensor:
        del dt  # Stage timing is defined in validated control steps.
        if self.failure_reason is not None:
            self.stop()
            return self.mapper.raw_action(self.stand)

        position, pitch, roll, linear_velocity, angular_velocity = (
            self._root_state()
        )
        self.last_pitch = pitch
        self.last_roll = roll
        self.last_linear_velocity = linear_velocity.copy()
        self.last_foot_contacts = self._foot_contact_count()
        # With the wall-facing yaw, world x controls forward pitch and world y
        # controls lateral roll.  Regulate both axes during descent and landing.
        torque_x = float(
            np.clip(
                -WALL_VAULT_PITCH_KD[0] * pitch
                - WALL_VAULT_PITCH_KD[1] * angular_velocity[0],
                -WALL_VAULT_ATTITUDE_TORQUE_CLIP,
                WALL_VAULT_ATTITUDE_TORQUE_CLIP,
            )
        )
        torque_y = float(
            np.clip(
                -WALL_VAULT_ROLL_KD[0] * roll
                - WALL_VAULT_ROLL_KD[1] * angular_velocity[1],
                -WALL_VAULT_ATTITUDE_TORQUE_CLIP,
                WALL_VAULT_ATTITUDE_TORQUE_CLIP,
            )
        )
        attitude_torque = torch.tensor(
            [[[torque_x, torque_y, 0.0]]],
            dtype=torch.float32,
            device=self.env.device,
        )
        force_now = self.zero
        torque_now = self.zero

        if self.stage == "done":
            self.stop()
            return self.mapper.raw_action(self.stand)

        if self.stage == "settle":
            alpha = _smoothstep(
                (self.stage_step + 1) / WALL_VAULT_SETTLE_STEPS
            )
            action = self.mapper.raw_action(
                self._settle_start * (1.0 - alpha) + self.stand * alpha
            )
            self.stage_step += 1
            if self.stage_step >= WALL_VAULT_SETTLE_STEPS:
                self.stage = "launch"
                self.stage_step = 0
        elif self.stage == "launch":
            # Keep the legs extended until the impulse has lifted the robot.
            force_now = self.launch_force
            action = self.mapper.raw_action(self.stand)
            self.stage_step += 1
            if self.stage_step >= WALL_VAULT_LAUNCH_STEPS:
                self.stage = "tuck"
                self.stage_step = 0
                self._tuck_start = None
        elif self.stage == "tuck":
            if self._tuck_start is None:
                self._tuck_start = self.mapper.current_joint_pos()
            force_now = self.zero
            torque_now = attitude_torque
            alpha = _smoothstep(
                (self.stage_step + 1) / WALL_VAULT_TUCK_STEPS
            )
            action = self.mapper.raw_action(
                self._tuck_start * (1.0 - alpha) + self.tuck * alpha
            )
            self.stage_step += 1
            if self.stage_step >= WALL_VAULT_TUCK_STEPS:
                self.stage = "vault"
                self.stage_step = 0
        elif self.stage == "vault":
            force_now = self.forward_force
            torque_now = attitude_torque
            action = self.mapper.raw_action(self.tuck)
            self.stage_step += 1
            if self.stage_step >= WALL_VAULT_STEPS or position[1] > -6.42:
                self.stage = "deploy"
                self.stage_step = 0
                self._recover_start = None
                self._landing_stable_steps = 0
        elif self.stage in ("deploy", "land"):
            if self._recover_start is None:
                self._recover_start = self.mapper.current_joint_pos()
            torque_now = attitude_torque
            force_now = self._landing_wrench(
                position,
                linear_velocity,
                self.last_foot_contacts,
            )

            if self.stage == "deploy":
                # Extend the legs before touchdown.  The old 0.60 s recovery
                # reached the standing pose only after impact, so the tucked
                # front feet hit first and produced a large pitch impulse.
                alpha = _smoothstep(
                    (self.stage_step + 1) / WALL_VAULT_DEPLOY_STEPS
                )
                action = self.mapper.raw_action(
                    self._recover_start * (1.0 - alpha) + self.stand * alpha
                )
                self.stage_step += 1
                if self.stage_step >= WALL_VAULT_DEPLOY_STEPS:
                    self.stage = "land"
                    self.stage_step = 0
            else:
                # Keep the landing pose fixed while the suspension compresses.
                # Only hand control back to the normal stand blend after the
                # feet are down and the root/attitude are truly quiet.
                action = self.mapper.raw_action(self.stand)
                self.stage_step += 1
                stable = (
                    self.last_foot_contacts >= WALL_VAULT_LANDING_MIN_CONTACTS
                    and abs(pitch) < math.radians(4.0)
                    and abs(roll) < math.radians(3.0)
                    and float(np.max(np.abs(linear_velocity))) < 0.16
                    and abs(float(linear_velocity[2])) < 0.12
                    and float(position[2]) > 0.44
                )
                self._landing_stable_steps = (
                    self._landing_stable_steps + 1 if stable else 0
                )
                if self._landing_stable_steps >= WALL_VAULT_LAND_SETTLE_STEPS:
                    self.stage = "done"
                    self.stage_step = 0
                    self.done = True
        else:
            action = self.mapper.raw_action(self.stand)

        self.robot.write_external_wrench_to_sim(
            force_now,
            torque_now,
            body_ids=[0],
        )
        return action


class NavigationController:
    """Small yaw/velocity controller used by LOCOMOTION and AUTOPILOT."""

    def __init__(self) -> None:
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

    def set_twist(self, vx: float, vy: float, wz: float) -> None:
        self.vx = float(np.clip(vx, -NAV_MAX_VX, NAV_MAX_VX))
        self.vy = float(np.clip(vy, -NAV_MAX_VY, NAV_MAX_VY))
        self.wz = float(np.clip(wz, -NAV_MAX_WZ, NAV_MAX_WZ))

    def command_to_waypoint(
        self,
        position: tuple[float, float],
        yaw: float,
        goal: tuple[float, float],
        max_speed: float = NAV_MAX_VX,
    ) -> tuple[float, float, float]:
        dx = goal[0] - position[0]
        dy = goal[1] - position[1]
        distance = math.hypot(dx, dy)
        if distance <= NAV_GOAL_TOLERANCE:
            self.set_twist(0.0, 0.0, 0.0)
            return self.vx, self.vy, self.wz
        desired_yaw = math.atan2(dy, dx)
        yaw_error = wrap_angle(desired_yaw - yaw)
        # Never use a pure in-place turn: this checkpoint's training command
        # contract is forward motion at 0.70-0.90 m/s.  A curved approach keeps
        # the actor in distribution while the yaw-rate term steers it.
        speed = min(max_speed, NAV_MIN_VX + 0.35 * max(0.0, distance - 0.25))
        yaw_rate = float(np.clip(1.40 * yaw_error, -NAV_MAX_WZ, NAV_MAX_WZ))
        self.set_twist(speed, 0.0, yaw_rate)
        return self.vx, self.vy, self.wz

    def command_forward(
        self,
        yaw: float,
        desired_yaw: float,
        speed: float,
    ) -> tuple[float, float, float]:
        yaw_error = wrap_angle(desired_yaw - yaw)
        forward = speed * max(0.65, math.cos(yaw_error))
        yaw_rate = float(np.clip(1.25 * yaw_error, -NAV_MAX_WZ, NAV_MAX_WZ))
        self.set_twist(forward, 0.0, yaw_rate)
        return self.vx, self.vy, self.wz
