"""Environment construction and terrain/perception helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import torch
from tensordict import TensorDict

from .config import (
    BAR_CENTER,
    BAR_CLEARANCE_MARGIN,
    BAR_LOWER_Z,
    BAR_X_RANGE,
    DEFAULT_TERRAIN,
    SPAWN_POS,
    SPAWN_YAW,
    SimulationConfig,
    WALL_CENTER,
    WALL_HEIGHT_M,
    WALL_X_RANGE,
    WALL_Y_RANGE,
    WALL_CLEAR_Y,
)
from .mjlab_compat import black_env_cfgs


def _attach_terrain(spec: Any, terrain_xml: Path) -> None:
    terrain = mujoco.MjSpec.from_file(str(terrain_xml))
    frame = spec.worldbody.add_frame()
    spec.attach(child=terrain, prefix="", frame=frame)


def build_environment(sim_config: SimulationConfig) -> tuple[Any, Any, Any]:
    """Build the local mjlab environment and deployment-policy wrapper."""
    from mjlab.asset_zoo.robots import get_black_climb_robot_cfg
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.envs.mdp.events import reset_scene_to_default
    from mjlab.managers.event_manager import EventTermCfg
    from mjlab.rl import RslRlVecEnvWrapper

    env_cfgs = black_env_cfgs()
    cfg = env_cfgs.black_fixed_platform30_local_teacher_postmount_env_cfg(play=True)
    cfg.scene.num_envs = 1
    cfg.scene.terrain = None
    cfg.scene.extent = 20.0
    terrain_xml = Path(sim_config.terrain_xml).expanduser().resolve()
    if not terrain_xml.is_file():
        raise FileNotFoundError(f"Terrain XML not found: {terrain_xml}")
    cfg.scene.spec_fn = lambda spec: _attach_terrain(spec, terrain_xml)
    cfg.scene.entities["robot"] = get_black_climb_robot_cfg()
    cfg.episode_length_s = 1e5
    cfg.seed = sim_config.seed
    cfg.events = {
        "reset_scene_to_default": EventTermCfg(
            func=reset_scene_to_default,
            mode="reset",
        )
    }
    cfg.terminations = {}

    # The supplied world has no terrain entity.  Contact sensors configured
    # against the generated terrain would otherwise fail during reset.
    for sensor in cfg.scene.sensors or ():
        if sensor.name in ("feet_ground_contact", "feet_wall_contact"):
            sensor.secondary = None

    # Keep the command fixed between state-machine updates.
    twist = cfg.commands["twist"]
    twist.heading_command = False
    twist.ranges.heading = None
    twist.rel_heading_envs = 0.0
    twist.rel_standing_envs = 0.0
    twist.rel_forward_envs = 0.0
    twist.rel_world_envs = 0.0
    twist.resampling_time_range = (1000.0, 1000.0)
    twist.target_top_ranges = None
    twist.target_brake_ranges = None
    twist.lin_vel_x_choices = None
    twist.init_velocity_prob = 0.0

    cfg.viewer.body_name = "trunk"
    cfg.viewer.distance = 1.4
    cfg.viewer.elevation = -16.0

    env = ManagerBasedRlEnv(cfg=cfg, device=sim_config.device)
    wrapper = BlackTerrainVecEnvWrapper(env, clip_actions=None)
    wrapper.place_robot_at_spawn()
    return env, wrapper, cfg


class BlackTerrainVecEnvWrapper:
    """Factory-compatible wrapper that always resets onto the red spawn pad.

    It subclasses the mjlab RSL-RL wrapper lazily at runtime so importing this
    module does not trigger the broken local Black package initializer.
    """

    def __new__(cls, env: Any, clip_actions: float | None = None) -> Any:
        from mjlab.rl import RslRlVecEnvWrapper

        if cls is BlackTerrainVecEnvWrapper:
            dynamic_cls = type(
                "BlackTerrainVecEnvWrapperImpl",
                (_BlackTerrainResetMixin, RslRlVecEnvWrapper),
                {"__module__": cls.__module__},
            )
            return dynamic_cls(env, clip_actions=clip_actions)
        return super().__new__(cls)


class _BlackTerrainResetMixin:
    def reset(self) -> tuple[TensorDict, dict]:
        _, extras = super().reset()
        self.place_robot_at_spawn()
        return self.get_observations(), extras

    def place_robot_at_spawn(self) -> None:
        self.place_robot_at_pose(SPAWN_POS, SPAWN_YAW)

    def place_robot_at_pose(
        self,
        position: tuple[float, float, float],
        yaw: float,
    ) -> None:
        """Teleport the single environment onto a requested root pose."""
        env = self.unwrapped
        robot = env.scene["robot"]
        device = env.device
        root_pose = torch.tensor(
            [[position[0], position[1], position[2], math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]],
            dtype=torch.float32,
            device=device,
        )
        zeros6 = torch.zeros((1, 6), dtype=torch.float32, device=device)
        joint_pos = robot.data.default_joint_pos.clone()
        joint_vel = torch.zeros_like(joint_pos)
        robot.write_root_link_pose_to_sim(root_pose)
        robot.write_root_link_velocity_to_sim(zeros6)
        robot.write_joint_state_to_sim(joint_pos, joint_vel)
        env.command_manager.get_command("twist")[:] = 0.0
        env.scene.write_data_to_sim()
        env.sim.forward()
        env.sim.sense()


def quat_to_yaw(quat: torch.Tensor) -> float:
    w, x, y, z = (float(v) for v in quat)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class TerrainMetrics:
    x: float
    y: float
    z: float
    yaw: float
    support_top_z: float
    support_top_geom: str
    inside_bar_x: bool
    bar_crossed: bool
    bar_contact: bool
    bar_clean_crossed: bool
    bar_clearance_m: float
    bar_max_top_z: float
    wall_support_contact: bool
    wall_leg_contact: bool
    wall_cleared: bool


class TerrainMonitor:
    """Measure the robot's low-pose envelope and bar crossing from simulation."""

    def __init__(self, env: Any) -> None:
        self.env = env
        self.robot = env.scene["robot"]
        self.model = env.sim.mj_model
        self.geom_ids: list[int] = []
        self.support_geom_ids: set[int] = set()
        self.leg_geom_ids: set[int] = set()
        self.robot_contact_geom_ids: set[int] = set()
        for geom_id in range(self.model.ngeom):
            name = self.model.geom(geom_id).name
            if "black" in name.lower():
                self.robot_contact_geom_ids.add(geom_id)
            if "black_support_collision_" in name:
                self.support_geom_ids.add(geom_id)
            elif "_foot_collision" in name:
                self.leg_geom_ids.add(geom_id)
            if (
                "black_support_collision_" in name
                or name.endswith("_foot_collision")
            ):
                self.geom_ids.append(geom_id)
        if not self.geom_ids:
            raise RuntimeError("No Black support collision geoms found")
        self.leg_geom_ids = self.robot_contact_geom_ids - self.support_geom_ids
        self._previous_x: float | None = None
        self._previous_y: float | None = None
        self._bar_crossed = False
        self._bar_contact = False
        self.bar_max_top_z = 0.0
        self._wall_cleared = False
        self._wall_support_contact = False
        self._wall_leg_contact = False
        self._wall_cleared = False
        self._wall_support_contact = False
        self._wall_leg_contact = False
        self._bar_geom_ids: set[int] = set()
        self._wall_geom_ids: set[int] = set()
        for geom_id in range(self.model.ngeom):
            name = self.model.geom(geom_id).name
            if name.startswith("limit_bar") or name.startswith("limit_post"):
                self._bar_geom_ids.add(geom_id)
            geom_pos = self.model.geom_pos[geom_id]
            geom_size = self.model.geom_size[geom_id]
            near_wall = (
                self.model.geom_type[geom_id]
                == mujoco.mjtGeom.mjGEOM_BOX
                and abs(float(geom_pos[0]) - WALL_CENTER[0]) < 0.05
                and abs(float(geom_pos[1]) - WALL_CENTER[1]) < 0.05
                and abs(float(geom_pos[2]) - WALL_CENTER[2]) < 0.02
                and abs(float(geom_size[0]) - 0.025) < 0.01
                and abs(float(geom_size[1]) - 0.5) < 0.01
                and abs(float(geom_size[2]) - WALL_HEIGHT_M / 2.0) < 0.01
            )
            if near_wall or name.startswith("terrain_wall"):
                self._wall_geom_ids.add(geom_id)
        if not self._wall_geom_ids:
            print("[terrain] warning: 0.30 m wall collision geom not found")

    def reset_tracking(self) -> None:
        self._previous_x = None
        self._previous_y = None
        self._bar_crossed = False
        self._bar_contact = False
        self._wall_cleared = False
        self._wall_support_contact = False
        self._wall_leg_contact = False
        self.bar_max_top_z = 0.0

    def _z_extent(self, geom_id: int, position: np.ndarray, rotation: np.ndarray) -> tuple[float, float] | None:
        geom_type = self.model.geom_type[geom_id]
        size = self.model.geom_size[geom_id]
        axis_z = abs(float(rotation[2, 2]))
        radial_z = math.sqrt(max(0.0, 1.0 - axis_z * axis_z))
        if geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
            extent = float(size[0])
        elif geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            extent = float(np.abs(rotation[2, :]) @ size)
        elif geom_type == mujoco.mjtGeom.mjGEOM_CAPSULE:
            extent = axis_z * float(size[1]) + float(size[0])
        elif geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
            extent = axis_z * float(size[1]) + radial_z * float(size[0])
        else:
            return None
        return float(position[2] - extent), float(position[2] + extent)

    def measure(self) -> TerrainMetrics:
        root_pos = self.robot.data.root_link_pos_w[0].detach().cpu().numpy()
        quat = self.robot.data.root_link_quat_w[0].detach().cpu().numpy()
        yaw = quat_to_yaw(torch.from_numpy(quat))
        geom_pos = self.env.sim.data.geom_xpos[0].detach().cpu().numpy()
        geom_mat = self.env.sim.data.geom_xmat[0].detach().cpu().numpy()

        top_z = -math.inf
        top_geom = ""
        for geom_id in self.geom_ids:
            bounds = self._z_extent(
                geom_id,
                geom_pos[geom_id],
                geom_mat[geom_id].reshape(3, 3),
            )
            if bounds is not None and bounds[1] > top_z:
                top_z = bounds[1]
                top_geom = self.model.geom(geom_id).name

        inside_bar_x = BAR_X_RANGE[0] <= float(root_pos[0]) <= BAR_X_RANGE[1]
        if inside_bar_x and abs(float(root_pos[1]) - BAR_CENTER[1]) < 0.45:
            self.bar_max_top_z = max(self.bar_max_top_z, top_z)

        # A root-plane crossing is not enough: reject runs that physically hit
        # the bar or posts at any point in the run.
        robot_geom_ids = set(self.geom_ids)
        try:
            contact_count = int(self.env.sim.data.nacon[0])
            contact_geoms = self.env.sim.data.contact.geom
            for contact_index in range(contact_count):
                pair = {
                    int(contact_geoms[contact_index, 0]),
                    int(contact_geoms[contact_index, 1]),
                }
                if pair & self._bar_geom_ids and pair & robot_geom_ids:
                    self._bar_contact = True
                if pair & self._wall_geom_ids:
                    if pair & self.support_geom_ids:
                        self._wall_support_contact = True
                    if pair & self.leg_geom_ids:
                        self._wall_leg_contact = True
        except (AttributeError, IndexError, RuntimeError, TypeError, ValueError):
            # Contact bookkeeping is unavailable on some backends.  Keep the
            # conservative geometric clearance criterion instead of crashing.
            pass

        if self._previous_x is not None and self._previous_y is not None:
            crossed_plane = self._previous_y < BAR_CENTER[1] <= float(root_pos[1])
            robust_x = (
                BAR_X_RANGE[0] + 0.10
                <= float(root_pos[0])
                <= BAR_X_RANGE[1] - 0.10
            )
            if crossed_plane and robust_x:
                self._bar_crossed = True
            wall_crossed_plane = (
                self._previous_y < WALL_CLEAR_Y <= float(root_pos[1])
            )
            inside_wall_span = (
                WALL_X_RANGE[0]
                <= float(root_pos[0])
                <= WALL_X_RANGE[1]
            )
            if wall_crossed_plane and inside_wall_span:
                self._wall_cleared = True
        self._previous_x = float(root_pos[0])
        self._previous_y = float(root_pos[1])

        clean_crossed = (
            self._bar_crossed
            and not self._bar_contact
            and self.bar_max_top_z > 0.0
            and self.bar_max_top_z
            < BAR_LOWER_Z - BAR_CLEARANCE_MARGIN
        )
        clearance = (
            BAR_LOWER_Z - self.bar_max_top_z
            if self.bar_max_top_z > 0.0
            else math.nan
        )

        return TerrainMetrics(
            x=float(root_pos[0]),
            y=float(root_pos[1]),
            z=float(root_pos[2]),
            yaw=yaw,
            support_top_z=float(top_z),
            support_top_geom=top_geom,
            inside_bar_x=inside_bar_x,
            bar_crossed=self._bar_crossed,
            bar_contact=self._bar_contact,
            bar_clean_crossed=clean_crossed,
            bar_clearance_m=float(clearance),
            bar_max_top_z=float(self.bar_max_top_z),
            wall_support_contact=self._wall_support_contact,
            wall_leg_contact=self._wall_leg_contact,
            wall_cleared=self._wall_cleared,
        )
