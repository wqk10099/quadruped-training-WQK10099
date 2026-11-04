"""Configuration and terrain waypoints."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_POLICY_CONFIG = PACKAGE_ROOT / "configs" / "policies.json"
DEFAULT_TERRAIN = PACKAGE_ROOT / "assets" / "scene_terrain.xml"

# The Black free joint uses a (w, x, y, z) quaternion.  The red pad in the
# supplied map is centred at x=3.7, y=-9.0.
SPAWN_POS = (3.7, -9.0, 0.51)
SPAWN_YAW = 0.0

# Interactive reset target.  The reset key always returns the robot to the
# map origin, independent of the scenario's initial spawn override.
ORIGIN_RESET_POS = (0.0, 0.0, 0.51)
ORIGIN_RESET_YAW = 0.0

# A safe staging point south of the limit bar.  The two waypoints keep the
# robot away from both gate posts instead of trying to cross at y=-9 directly.
APPROACH_WAYPOINTS = ((3.80, -10.40), (5.75, -10.78), (5.70, -10.22))
BAR_CENTER = (5.70, -9.00, 0.325)
BAR_LOWER_Z = 0.310
BAR_X_RANGE = (5.20, 6.20)
CRAWL_EXIT_Y = -8.35
BAR_CLEARANCE_MARGIN = 0.005

# Fixed 0.30 m wall vault.  The supplied wall is centred at y=-7.00 and spans
# x=1.30..2.30.  This skill starts south of the wall (y=-7.55), faces +y, and
# uses a short assisted whole-body vault rather than pretending the base RL
# gait can climb a vertical 0.30 m obstacle.
WALL_SPAWN_POS = (1.80, -7.55, 0.51)
WALL_SPAWN_YAW = math.pi / 2.0
WALL_CENTER = (1.80, -7.00, 0.15)
WALL_X_RANGE = (1.30, 2.30)
WALL_Y_RANGE = (-7.025, -6.975)
WALL_HEIGHT_M = 0.30
WALL_CLEAR_Y = -6.50
WALL_MIN_FINAL_Z = 0.44
WALL_MIN_SAFE_Z = 0.20
WALL_VAULT_SETTLE_STEPS = 70
WALL_VAULT_LAUNCH_STEPS = 6
WALL_VAULT_TUCK_STEPS = 10
WALL_VAULT_STEPS = 14
WALL_VAULT_DEPLOY_STEPS = 10
WALL_VAULT_LAND_SETTLE_STEPS = 10
WALL_VAULT_LAUNCH_FORCE = (0.0, 120.0, 500.0)
WALL_VAULT_FORWARD_FORCE = (0.0, 80.0, 0.0)
WALL_VAULT_PITCH_KD = (90.0, 9.0)
WALL_VAULT_ROLL_KD = (65.0, 7.0)
WALL_VAULT_ATTITUDE_TORQUE_CLIP = 80.0
WALL_VAULT_BRAKE_GAIN = 150.0
WALL_VAULT_BRAKE_CLIP_N = 130.0
WALL_VAULT_Z_BRAKE_GAIN = 65.0
WALL_VAULT_Z_BRAKE_CLIP_N = 260.0
WALL_VAULT_LANDING_START_Z = 0.95
WALL_VAULT_LANDING_MIN_CONTACTS = 2
WALL_VAULT_TUCK_JOINTS = (
    -0.5, 1.6, -2.5,
     0.5, -1.6, 2.5,
    -0.5, 0.322, 2.5,
     0.5, -0.322, -2.5,
)
WALL_STAGE_TIMEOUTS = {
    "settle": 3.0,
    "launch": 1.0,
    "tuck": 1.0,
    "vault": 1.5,
    "deploy": 1.5,
    "land": 4.0,
    "done": 1.0,
}

# Experimental pure-climb skill.  Unlike WALL_CROSS, this route uses only
# position-controlled legs and foot IK; it never writes an external wrench.
# The robot starts south of the same wall, faces +y, and advances one foot at a
# time through a quasi-static support sequence.  The gains correct small pitch
# and roll errors by changing the height of the remaining support feet.
WALL_CLIMB_SPAWN_POS = (1.85, -7.52, 0.51)
WALL_CLIMB_SPAWN_YAW = math.pi / 2.0
WALL_CLIMB_SETTLE_STEPS = 55
WALL_CLIMB_POSE_TOLERANCE_M = 0.18
WALL_CLIMB_SWING_IK_TOLERANCE_M = 0.055
WALL_CLIMB_SUPPORT_IK_TOLERANCE_M = 0.060
WALL_CLIMB_PITCH_GAIN = 0.8
WALL_CLIMB_ROLL_GAIN = 0.4
WALL_CLIMB_ATTITUDE_Z_CLIP_M = 0.035
WALL_CLIMB_ROOT_FEEDBACK_XY = 0.5
WALL_CLIMB_MAX_PITCH_RAD = math.radians(20.0)
WALL_CLIMB_MAX_ROLL_RAD = math.radians(16.0)
WALL_CLIMB_MIN_SAFE_ROOT_Z = 0.25
WALL_CLIMB_MIN_FINAL_Z = 0.44
WALL_CLIMB_START_X_RANGE = (1.70, 2.00)
WALL_CLIMB_START_Y_RANGE = (-7.90, -7.30)
WALL_CLIMB_START_YAW_TOLERANCE = math.radians(20.0)
WALL_CLIMB_STAGE_TIMEOUTS = {
    "settle_stand": 3.0,
    "settle_crouch": 5.0,
    "shift_left_for_rr_far": 5.0,
    "rr_far_prepose": 7.0,
    "center_after_rr_far": 5.0,
    "shift_right_for_rl_far": 5.0,
    "rl_far_prepose": 7.0,
    "center_after_rl_far": 5.0,
    "advance_between_rear_rounds": 5.0,
    "shift_left_for_rr_near": 5.0,
    "rr_near_prepose": 7.0,
    "center_after_rr_near": 5.0,
    "shift_right_for_rl_near": 5.0,
    "rl_near_prepose": 7.0,
    "center_after_rl_near": 5.0,
    "shift_right_for_fl_wall": 5.0,
    "fl_on_wall": 7.0,
    "settle_after_fl_wall": 6.0,
    "shift_left_for_fr_wall": 7.0,
    "fr_on_wall": 7.0,
    "center_on_wall": 5.0,
    "advance_over_wall": 5.0,
    "shift_right_for_fl_north": 5.0,
    "fl_north_ground": 7.0,
    "shift_left_for_fr_north": 5.0,
    "fr_north_ground": 7.0,
    "center_north_front": 5.0,
    "shift_left_for_rr_wall": 5.0,
    "rr_at_wall": 7.0,
    "shift_right_for_rl_wall": 5.0,
    "rl_at_wall": 7.0,
    "advance_rear_body": 5.0,
    "shift_left_for_rr_crown": 5.0,
    "rr_on_wall": 7.0,
    "shift_right_for_rl_crown": 5.0,
    "rl_on_wall": 7.0,
    "advance_exit_body": 5.0,
    "rr_north_ground": 7.0,
    "rl_north_ground": 7.0,
    "advance_north": 5.0,
    "recover_stand": 5.0,
}

# Bridge-B walking skill.  The robot starts on the north deck with the two
# front feet exactly on the deck/plank boundary (y=-1.50), faces south (-y),
# and advances with the base locomotion policy while a yaw command keeps it
# centered over the three 0.20 m-wide boards.  This is one action, not a
# multi-stage route: after reaching the south platform it stops and the FSM
# blends back to STAND.
BRIDGE_B_SPAWN_POS = (5.70, -1.32, 0.72)
BRIDGE_B_SPAWN_YAW = -math.pi / 2.0
BRIDGE_B_SOUTH_EXIT_Y = -3.55
BRIDGE_B_MIN_FINAL_Y = -3.30
BRIDGE_B_FRONT_EDGE_Y = -1.50
BRIDGE_B_FRONT_EDGE_TOLERANCE_M = 0.04
BRIDGE_B_X_RANGE = (5.35, 6.05)
BRIDGE_B_YAW_TOLERANCE = math.radians(18.0)
BRIDGE_B_MIN_ROOT_Z = 0.30
BRIDGE_B_MIN_FINAL_Z = 0.42
BRIDGE_B_MAX_PITCH_RAD = math.radians(28.0)
BRIDGE_B_MAX_ROLL_RAD = math.radians(22.0)
BRIDGE_B_WALK_VX = 0.70
BRIDGE_B_HEADING_GAIN = 1.0
BRIDGE_B_MAX_HEADING_WZ = 0.12
BRIDGE_B_MAX_LATERAL_DRIFT_M = 0.30
BRIDGE_B_TIMEOUT_S = 20.0

JOINT_NAMES = (
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
)

# Black's nominal standing pose, in the actor action ordering.
STAND_JOINTS = (
    0.0, 0.6, -1.0,
    0.0, -0.6, 1.0,
    0.0, -0.6, 1.0,
    0.0, 0.6, -1.0,
)

# Deliberately folded posture.  Its static collision envelope is well below the
# 0.310 m lower edge of the supplied bar.  The thigh/calf split is deliberately
# more extended than a fully folded pose so the crawl IK has enough travel.
LOW_JOINTS = (
    0.0, 1.2, -2.387,
    0.0, -1.2, 2.387,
    0.0, -1.2, 2.387,
    0.0, 1.2, -2.387,
)
ACTION_SCALE = 0.25

# Four-beat crawl gait.  Leg phases are ordered FL -> RR -> FR -> RL.  Only one
# foot is in swing at a time, which keeps three support feet on the ground while
# maintaining the low bar-clearing envelope.
GAIT_PERIOD_S = 1.6
GAIT_STRIDE_M = 0.10
GAIT_LIFT_M = 0.035
GAIT_DUTY = 0.82
GAIT_LEGS = ("FL", "RR", "FR", "RL")
GAIT_LEG_OFFSETS = {"FL": 0.0, "RR": 0.25, "FR": 0.5, "RL": 0.75}
GAIT_TABLE_SAMPLES = 96

# Navigation controller limits are deliberately conservative because terrain
# navigation is not the objective of the current flat-ground student. Keep
# the forward command near the short-tested 0.70-0.90 m/s range and use small
# yaw-rate corrections; re-tune and re-validate on the terrain before widening
# these limits.
NAV_MIN_VX = 0.70
NAV_MAX_VX = 0.88

# Interactive movement uses a press-to-increase throttle.  The first press in
# a direction starts at the default speed, and every additional press in the
# same direction adds one step while clamping at the maximum.
MANUAL_FORWARD_VX_DEFAULT = 0.10
MANUAL_FORWARD_VX_STEP = 0.20
MANUAL_FORWARD_VX_MAX = 0.90
MANUAL_CLIMB_VX = 0.80

# Manual backward motion is sent directly to the base actor as negative vx.
# Keep the tested interactive range deliberately modest.
MANUAL_BACKWARD_SPEED_DEFAULT = 0.10
MANUAL_BACKWARD_SPEED_STEP = 0.20
MANUAL_BACKWARD_SPEED_MAX = 0.50

# Manual Q/E yaw commands are sent directly to the base actor.  They start at
# the default rate and repeated presses add a small angular increment.
MANUAL_TURN_WZ_MIN = 0.06
MANUAL_TURN_WZ_DEFAULT = 0.10
MANUAL_TURN_WZ_MAX = 0.60
MANUAL_TURN_WZ_STEP = 0.05

# Manual strafing uses a conservative 0.60 m/s ceiling. The previous HIM base
# was tested at this limit; the current flat student still needs the complete
# strafe range re-validated. Arrow Left/Right use this throttle; A/D remain
# aliases.
MANUAL_STRAFE_VY_MIN = 0.10
MANUAL_STRAFE_VY_DEFAULT = 0.10
MANUAL_STRAFE_VY_MAX = 0.60
MANUAL_STRAFE_VY_STEP = 0.20
NAV_MAX_VY = 0.16
NAV_MAX_WZ = 0.38
NAV_GOAL_TOLERANCE = 0.22
STAGING_TOLERANCE = 0.38

# Per-stage safety timeouts for AUTOPILOT.  The low crawl is intentionally
# slower than the RL gait because it must preserve a sub-bar collision envelope.
AUTOPILOT_STAGE_TIMEOUTS = {
    "stand": 4.0,
    "approach": 55.0,
    "align": 16.0,
    "lower": 4.0,
    "crawl": 75.0,
    "recover": 4.0,
}

# Fixed-pose interpolation duration.
POSE_BLEND_TIME_S = 1.5

KEY_STATE_MAP = {
    "1": "STAND",
    "2": "LOCOMOTION",
    "B": "BRIDGE_B_CROSS",
    "C": "CLIMB",
    "J": "WALL_CROSS",
    "K": "WALL_CLIMB",
    "L": "LIE_DOWN",
    "X": "CRAWL_FORWARD",
}


@dataclass(frozen=True)
class PolicySpec:
    checkpoint: str
    runner_cfg: str = ""
    kind: str = "rsl_rl"


@dataclass(frozen=True)
class PolicyConfig:
    base: PolicySpec
    climb: PolicySpec
    jump: PolicySpec

    @classmethod
    def load(cls, path: str | Path = DEFAULT_POLICY_CONFIG) -> "PolicyConfig":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(
            base=PolicySpec(**data["base"]),
            climb=PolicySpec(**data["climb"]),
            jump=PolicySpec(**data["jump"]),
        )


@dataclass
class SimulationConfig:
    terrain_xml: Path = DEFAULT_TERRAIN
    policies: PolicyConfig = field(default_factory=PolicyConfig.load)
    device: str = "cpu"
    seed: int = 7
