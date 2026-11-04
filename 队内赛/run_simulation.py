#!/usr/bin/env python3
"""GUI/headless simulation entry point for the Black terrain state machine."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import threading
import time
import traceback
from dataclasses import asdict
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import torch

from black_terrain_fsm.config import (
    BAR_LOWER_Z,
    BRIDGE_B_MIN_FINAL_Y,
    BRIDGE_B_MIN_FINAL_Z,
    BRIDGE_B_MIN_ROOT_Z,
    BRIDGE_B_SOUTH_EXIT_Y,
    BRIDGE_B_SPAWN_POS,
    BRIDGE_B_SPAWN_YAW,
    BRIDGE_B_X_RANGE,
    BRIDGE_B_YAW_TOLERANCE,
    CRAWL_EXIT_Y,
    DEFAULT_TERRAIN,
    PolicyConfig,
    SPAWN_YAW,
    SimulationConfig,
    WALL_CENTER,
    WALL_CLEAR_Y,
    WALL_HEIGHT_M,
    WALL_CLIMB_MIN_FINAL_Z,
    WALL_CLIMB_SPAWN_POS,
    WALL_CLIMB_SPAWN_YAW,
    WALL_MIN_FINAL_Z,
    WALL_SPAWN_POS,
    WALL_SPAWN_YAW,
    WALL_X_RANGE,
    WALL_Y_RANGE,
)
from black_terrain_fsm.policies import (
    FLAT_RSL_RL_OBSERVATION_DIM,
    HIM_HISTORY_LENGTH,
    HIM_OBSERVATION_DIM,
    PolicyBank,
)
from black_terrain_fsm.state_machine import RobotState, StateMachinePolicy
from black_terrain_fsm.terrain import TerrainMetrics, build_environment


STATE_NAMES = tuple(state.value for state in RobotState)
SCENARIO_DEFAULT_STATE = {
    "bar": RobotState.CRAWL_FORWARD,
    "autopilot": RobotState.AUTOPILOT,
    "bridge": RobotState.BRIDGE_B_CROSS,
    "wall": RobotState.WALL_CROSS,
    "wall_climb": RobotState.WALL_CLIMB,
    "manual": RobotState.STAND,
}
SCENARIO_DEFAULT_DURATION_S = {
    "bar": 70.0,
    "autopilot": 75.0,
    "bridge": 100.0,
    "wall": 15.0,
    "wall_climb": 60.0,
    "manual": 20.0,
}
BAR_STAGING_POS = (5.70, -10.22, 0.51)
BAR_STAGING_YAW = math.pi / 2.0

CSV_FIELDS = (
    "step",
    "sim_time_s",
    "x",
    "y",
    "z",
    "yaw_rad",
    "support_top_z",
    "support_top_geom",
    "inside_bar_x",
    "bar_crossed",
    "bar_contact",
    "bar_clean_crossed",
    "bar_clearance_m",
    "bar_max_top_z",
    "state",
    "policy_active",
    "action_source",
    "autopilot_stage",
    "bridge_stage",
    "wall_stage",
    "wall_climb_stage",
    "wall_support_contact",
    "wall_leg_contact",
    "wall_cleared",
    "cmd_vx",
    "cmd_vy",
    "cmd_wz",
    "action_abs_max",
)


def _finite_or_none(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, Path):
        return str(value)
    return value


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    if isinstance(value, float):
        return _finite_or_none(value)
    return str(value)


class RunRecorder:
    """Collect step telemetry and summarize a single simulation run."""

    def __init__(self) -> None:
        self.rows: list[dict[str, Any]] = []
        self.first_metrics: TerrainMetrics | None = None
        self.last_metrics: TerrainMetrics | None = None
        self.state_sequence: list[str] = []
        self.max_action_abs = 0.0
        self.max_support_top_z = -math.inf
        self.max_bar_envelope_z = 0.0
        self.min_bar_clearance_m = math.inf
        self.bar_crossed = False
        self.bar_contact = False
        self.bar_clean_crossed = False
        self.wall_support_contact = False
        self.wall_leg_contact = False
        self.wall_cleared = False
        self.nonfinite_observation = False
        self.last_state = ""
        self.last_source = ""
        self.min_root_z = math.inf

    def record(
        self,
        *,
        step: int,
        sim_time_s: float,
        metrics: TerrainMetrics,
        state: str,
        policy_active: str | None,
        action_source: str,
        autopilot_stage: str | None,
        bridge_stage: str | None,
        wall_stage: str | None,
        wall_climb_stage: str | None,
        command: tuple[float, float, float],
        action_abs_max: float,
    ) -> None:
        if self.first_metrics is None:
            self.first_metrics = metrics
        self.last_metrics = metrics
        self.last_state = state
        self.last_source = action_source
        self.max_action_abs = max(self.max_action_abs, action_abs_max)
        self.min_root_z = min(self.min_root_z, metrics.z)
        if math.isfinite(metrics.support_top_z):
            self.max_support_top_z = max(
                self.max_support_top_z,
                metrics.support_top_z,
            )
        if metrics.bar_max_top_z > 0.0:
            self.max_bar_envelope_z = max(
                self.max_bar_envelope_z,
                metrics.bar_max_top_z,
            )
        if math.isfinite(metrics.bar_clearance_m):
            self.min_bar_clearance_m = min(
                self.min_bar_clearance_m,
                metrics.bar_clearance_m,
            )
        self.bar_crossed = self.bar_crossed or metrics.bar_crossed
        self.bar_contact = self.bar_contact or metrics.bar_contact
        self.bar_clean_crossed = (
            self.bar_clean_crossed or metrics.bar_clean_crossed
        )
        self.wall_support_contact = (
            self.wall_support_contact or metrics.wall_support_contact
        )
        self.wall_leg_contact = (
            self.wall_leg_contact or metrics.wall_leg_contact
        )
        self.wall_cleared = self.wall_cleared or metrics.wall_cleared
        if not all(
            math.isfinite(value)
            for value in (
                metrics.x,
                metrics.y,
                metrics.z,
                metrics.yaw,
                action_abs_max,
            )
        ):
            self.nonfinite_observation = True
        if state not in self.state_sequence:
            self.state_sequence.append(state)
        self.rows.append(
            {
                "step": step,
                "sim_time_s": sim_time_s,
                "x": metrics.x,
                "y": metrics.y,
                "z": metrics.z,
                "yaw_rad": metrics.yaw,
                "support_top_z": metrics.support_top_z,
                "support_top_geom": metrics.support_top_geom,
                "inside_bar_x": metrics.inside_bar_x,
                "bar_crossed": metrics.bar_crossed,
                "bar_contact": metrics.bar_contact,
                "bar_clean_crossed": metrics.bar_clean_crossed,
                "bar_clearance_m": metrics.bar_clearance_m,
                "bar_max_top_z": metrics.bar_max_top_z,
                "state": state,
                "policy_active": policy_active or "",
                "action_source": action_source,
                "autopilot_stage": autopilot_stage or "",
                "bridge_stage": bridge_stage or "",
                "wall_stage": wall_stage or "",
                "wall_climb_stage": wall_climb_stage or "",
                "wall_support_contact": metrics.wall_support_contact,
                "wall_leg_contact": metrics.wall_leg_contact,
                "wall_cleared": metrics.wall_cleared,
                "cmd_vx": command[0],
                "cmd_vy": command[1],
                "cmd_wz": command[2],
                "action_abs_max": action_abs_max,
            }
        )

    @property
    def peak_envelope_m(self) -> float | None:
        if self.max_support_top_z == -math.inf:
            return None
        return float(self.max_support_top_z)

    @property
    def measured_bar_clearance_m(self) -> float | None:
        if self.max_bar_envelope_z <= 0.0:
            return None
        return float(BAR_LOWER_Z - self.max_bar_envelope_z)


class MonitoredPolicy:
    """Record the post-step result whenever the viewer calls reset(dones)."""

    def __init__(self, target: StateMachinePolicy, recorder: RunRecorder) -> None:
        self.target = target
        self.recorder = recorder
        self.step = 0
        self.pending: dict[str, Any] | None = None

    def __call__(self, obs: Any) -> torch.Tensor:
        action = self.target(obs)
        action_abs_max = float(torch.max(torch.abs(action)).item())
        command = (
            self.target.env.command_manager.get_command("twist")[0]
            .detach()
            .cpu()
            .tolist()
        )
        self.pending = {
            "step": self.step,
            "state": self.target.state.value,
            "policy_active": self.target.policy_bank.active,
            "action_source": self.target.last_action_source,
            "autopilot_stage": (
                self.target.autopilot_result["stage"]
                if self.target.state == RobotState.AUTOPILOT
                else None
            ),
            "bridge_stage": (
                self.target.bridge_result["stage"]
                if self.target.state == RobotState.BRIDGE_B_CROSS
                else None
            ),
            "wall_stage": (
                self.target.wall_result["stage"]
                if self.target.state == RobotState.WALL_CROSS
                else None
            ),
            "wall_climb_stage": (
                self.target.wall_climb_result["stage"]
                if self.target.state == RobotState.WALL_CLIMB
                else None
            ),
            "command": tuple(float(value) for value in command),
            "action_abs_max": action_abs_max,
        }
        self.step += 1
        return action

    def reset(self, dones: torch.Tensor | None = None) -> None:
        self.flush()
        self.target.reset(dones)

    def flush(self) -> None:
        if self.pending is None:
            return
        metrics = self.target.monitor.measure()
        self.target.last_metrics = metrics
        self.recorder.record(
            sim_time_s=self.step * self.target.step_dt,
            metrics=metrics,
            **self.pending,
        )
        self.pending = None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Black state-machine simulation on the supplied terrain. "
            "Use GUI for keyboard control or headless for repeatable runs."
        )
    )
    parser.add_argument("--mode", choices=("headless", "gui"), default="headless")
    parser.add_argument(
        "--scenario",
        choices=("bar", "autopilot", "bridge", "wall", "wall_climb", "manual"),
        default="bar",
        help=(
            "bar: south staging point + direct crawl; autopilot: state-machine "
            "crawl sequence from the staging point; bridge: walk across bridge-B "
            "from its north edge; wall: assisted vault from the south "
            "side of the 0.30 m wall; wall_climb: experimental pure-IK climb "
            "from the south side of the same wall; manual: red spawn + selected state"
        ),
    )
    parser.add_argument(
        "--state",
        type=str.upper,
        choices=STATE_NAMES,
        default=None,
        help="Initial FSM state; defaults depend on the scenario",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Maximum simulated seconds (headless has a scenario default)",
    )
    parser.add_argument("--steps", type=int, default=None, help="Maximum control steps")
    parser.add_argument(
        "--telemetry-every",
        type=int,
        default=1,
        help="Write every Nth control step to CSV",
    )
    parser.add_argument(
        "--spawn",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=None,
        help="Override the scenario root position",
    )
    parser.add_argument(
        "--yaw-deg",
        type=float,
        default=None,
        help="Override the scenario root yaw in degrees",
    )
    parser.add_argument(
        "--forward-speed",
        type=float,
        default=None,
        help=(
            "Initial base-policy forward speed in m/s; clamped to the manual "
            "forward range and composable with --strafe-vy/--turn-wz"
        ),
    )
    parser.add_argument(
        "--turn-wz",
        type=float,
        default=None,
        help=(
            "Initial base-policy yaw-rate magnitude in rad/s; clamped to "
            "the manual turn range and composable with forward/backward/strafe"
        ),
    )
    parser.add_argument(
        "--backward-speed",
        type=float,
        default=None,
        help=(
            "Initial base-policy backward speed magnitude in m/s; "
            "clamped to the manual range"
        ),
    )
    parser.add_argument(
        "--strafe-vy",
        type=float,
        default=None,
        help=(
            "Initial base-policy lateral velocity in m/s; the sign selects "
            "left (+) or right (-), and magnitude is clamped to the manual range"
        ),
    )
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--terrain", type=Path, default=DEFAULT_TERRAIN)
    parser.add_argument(
        "--policy-config",
        type=Path,
        default=ROOT / "configs" / "policies.json",
    )
    parser.add_argument(
        "--output-prefix",
        type=Path,
        default=None,
        help="CSV/JSON path prefix; default is results/<scenario>_<timestamp>",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return a nonzero code when a GUI run is stopped before completion",
    )
    return parser


def _scenario_pose(args: argparse.Namespace) -> tuple[tuple[float, float, float], float]:
    if args.spawn is not None:
        position = tuple(float(value) for value in args.spawn)
    elif args.scenario in ("bar", "autopilot"):
        position = BAR_STAGING_POS
    elif args.scenario == "bridge":
        position = BRIDGE_B_SPAWN_POS
    elif args.scenario == "wall":
        position = WALL_SPAWN_POS
    elif args.scenario == "wall_climb":
        position = WALL_CLIMB_SPAWN_POS
    else:
        position = (3.70, -9.00, 0.51)
    if args.scenario in ("bar", "autopilot"):
        default_yaw = BAR_STAGING_YAW
    elif args.scenario == "bridge":
        default_yaw = BRIDGE_B_SPAWN_YAW
    elif args.scenario == "wall":
        default_yaw = WALL_SPAWN_YAW
    elif args.scenario == "wall_climb":
        default_yaw = WALL_CLIMB_SPAWN_YAW
    else:
        default_yaw = SPAWN_YAW
    yaw = (
        math.radians(float(args.yaw_deg))
        if args.yaw_deg is not None
        else default_yaw
    )
    return position, yaw


def _max_steps(args: argparse.Namespace, step_dt: float) -> int | None:
    step_limit = args.steps
    if args.duration is not None:
        duration_limit = max(1, math.ceil(float(args.duration) / step_dt))
        step_limit = (
            duration_limit
            if step_limit is None
            else min(step_limit, duration_limit)
        )
    if step_limit is None and args.mode == "headless":
        duration = SCENARIO_DEFAULT_DURATION_S[args.scenario]
        step_limit = max(1, math.ceil(duration / step_dt))
    if step_limit is not None and step_limit <= 0:
        raise ValueError("--steps/--duration must describe a positive run length")
    return step_limit


def _default_output_prefix(args: argparse.Namespace) -> Path:
    if args.output_prefix is not None:
        return args.output_prefix.expanduser().resolve()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return (ROOT / "results" / f"{args.scenario}_{stamp}").resolve()


def _safe_output_path(prefix: Path, suffix: str) -> Path:
    return Path(f"{prefix}{suffix}")


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    key: ("" if isinstance(value, float) and not math.isfinite(value) else value)
                    for key, value in row.items()
                }
            )


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(_json_safe(payload), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def _success_and_failures(
    *,
    scenario: str,
    recorder: RunRecorder,
    state_machine: StateMachinePolicy,
    interrupted: bool,
    reached_limit: bool,
) -> tuple[bool, list[str]]:
    failures: list[str] = []
    if interrupted:
        failures.append("interrupted")
    if recorder.nonfinite_observation:
        failures.append("nonfinite_state_or_action")
    if recorder.bar_contact:
        failures.append("limit_bar_contact")
    if recorder.wall_support_contact:
        failures.append("wall_support_collision")

    autopilot = state_machine.autopilot_result
    if scenario in ("bar", "autopilot"):
        if not recorder.bar_clean_crossed:
            failures.append("clean_bar_crossing_not_reached")
        if not recorder.bar_crossed:
            failures.append("bar_plane_not_crossed")
    if scenario == "autopilot":
        if autopilot["stage"] != "complete":
            failures.append(f"autopilot_not_complete:{autopilot['stage']}")
    elif scenario == "bridge":
        bridge = state_machine.bridge_result
        if bridge["stage"] != "complete":
            failures.append(f"bridge_not_complete:{bridge['stage']}")
        if bridge["failure_reason"]:
            failures.append(f"bridge_failed:{bridge['failure_reason']}")
        if not bridge["south_reached"]:
            failures.append("bridge_b_south_platform_not_reached")
        if recorder.min_root_z < BRIDGE_B_MIN_ROOT_Z:
            failures.append("bridge_b_low_root_height")
        if state_machine.state != RobotState.STAND:
            failures.append(f"bridge_b_recovery_not_complete:{state_machine.state.value}")
        final_metrics = recorder.last_metrics
        if final_metrics is not None:
            if final_metrics.y > BRIDGE_B_MIN_FINAL_Y:
                failures.append("bridge_b_final_y_not_past_bridge")
            if final_metrics.z < BRIDGE_B_MIN_FINAL_Z:
                failures.append("bridge_b_final_height_too_low")
            if not BRIDGE_B_X_RANGE[0] <= final_metrics.x <= BRIDGE_B_X_RANGE[1]:
                failures.append("bridge_b_final_x_outside_span")
            if (
                abs(math.atan2(math.sin(final_metrics.yaw - BRIDGE_B_SPAWN_YAW),
                               math.cos(final_metrics.yaw - BRIDGE_B_SPAWN_YAW)))
                > BRIDGE_B_YAW_TOLERANCE
            ):
                failures.append("bridge_b_final_yaw_outside_bridge_axis")
    elif scenario == "wall":
        wall = state_machine.wall_result
        if wall["stage"] != "done":
            failures.append(f"wall_not_complete:{wall['stage']}")
        if not recorder.wall_cleared:
            failures.append("wall_exit_not_cleared")
        if state_machine.state != RobotState.STAND:
            failures.append(f"wall_recovery_not_complete:{state_machine.state.value}")
        final_metrics = recorder.last_metrics
        if final_metrics is not None:
            if not WALL_X_RANGE[0] <= final_metrics.x <= WALL_X_RANGE[1]:
                failures.append("wall_final_x_outside_span")
            if final_metrics.y <= WALL_CLEAR_Y:
                failures.append("wall_final_y_not_past_wall")
            if final_metrics.z < WALL_MIN_FINAL_Z:
                failures.append("wall_final_height_too_low")
    elif scenario == "wall_climb":
        wall_climb = state_machine.wall_climb_result
        if wall_climb["stage"] != "done":
            failures.append(f"wall_climb_not_complete:{wall_climb['stage']}")
        if wall_climb["failure_reason"]:
            failures.append(f"wall_climb_failed:{wall_climb['failure_reason']}")
        if not recorder.wall_cleared:
            failures.append("wall_climb_exit_not_cleared")
        if state_machine.state != RobotState.STAND:
            failures.append(
                f"wall_climb_recovery_not_complete:{state_machine.state.value}"
            )
        final_metrics = recorder.last_metrics
        if final_metrics is not None:
            if final_metrics.y <= WALL_CLEAR_Y:
                failures.append("wall_climb_final_y_not_past_wall")
            if final_metrics.z < WALL_CLIMB_MIN_FINAL_Z:
                failures.append("wall_climb_final_height_too_low")
    elif scenario == "bar":
        if state_machine.state != RobotState.STAND:
            failures.append(f"recovery_not_complete:{state_machine.state.value}")
    else:
        if reached_limit:
            # Reaching the requested duration is a successful manual run.
            pass
    return not failures, failures


def _run_headless(
    wrapper: Any,
    policy: MonitoredPolicy,
    obs: Any,
    max_steps: int | None,
    scenario: str,
) -> tuple[int, bool, bool]:
    if max_steps is None:
        max_steps = 10**9
    steps = 0
    interrupted = False
    wall_start = time.perf_counter()
    next_progress = 100
    try:
        while steps < max_steps:
            action = policy(obs)
            obs, _, dones, _ = wrapper.step(action)
            policy.reset(dones)
            steps += 1

            if steps >= next_progress:
                metrics = policy.recorder.last_metrics
                elapsed_wall = time.perf_counter() - wall_start
                print(
                    f"[RUN] step={steps} sim={steps * policy.target.step_dt:.1f}s "
                    f"state={policy.target.state.value} "
                    f"xy=({metrics.x:.2f},{metrics.y:.2f}) wall={elapsed_wall:.1f}s",
                    flush=True,
                )
                next_progress += 100

            autopilot = policy.target.autopilot_result
            if scenario == "autopilot":
                if autopilot["stage"] == "failed":
                    break
                if (
                    autopilot["stage"] == "complete"
                    and policy.target.state == RobotState.STAND
                ):
                    break
            if scenario == "bridge":
                bridge = policy.target.bridge_result
                if bridge["stage"] == "failed":
                    break
                if (
                    bridge["stage"] == "complete"
                    and policy.target.state == RobotState.STAND
                    and policy.target.state_elapsed >= 1.0
                ):
                    break
            if scenario == "wall":
                if policy.target.wall_result["stage"] == "failed":
                    break
                if (
                    policy.target.wall_result["stage"] == "done"
                    and policy.target.state == RobotState.STAND
                    and policy.target.state_elapsed >= 1.0
                ):
                    break
            if scenario == "wall_climb":
                if policy.target.wall_climb_result["stage"] == "failed":
                    break
                if (
                    policy.target.wall_climb_result["stage"] == "done"
                    and policy.target.state == RobotState.STAND
                    and policy.target.state_elapsed >= 1.0
                ):
                    break
            if (
                scenario == "bar"
                and policy.target.state == RobotState.STAND
                and policy.recorder.bar_clean_crossed
                and policy.target.state_elapsed >= 1.5
            ):
                break
    except KeyboardInterrupt:
        interrupted = True
        print("[RUN] interrupted by user", flush=True)
    finally:
        policy.flush()
    reached_limit = steps >= max_steps and max_steps < 10**9
    return steps, interrupted, reached_limit


class GuiControlPanel:
    """Small topmost control panel for actions that need a real button."""

    def __init__(self, state_machine: StateMachinePolicy) -> None:
        self.state_machine = state_machine
        self._thread = threading.Thread(
            target=self._run,
            name="black-gui-control-panel",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        try:
            import tkinter as tk
        except Exception as exc:  # pragma: no cover - depends on desktop packages
            print(
                f"[GUI] reset button unavailable ({exc}); use R/Home instead",
                flush=True,
            )
            return

        try:
            root = tk.Tk()
            root.title("Black Control")
            root.geometry("390x330+40+80")
            root.resizable(False, False)
            root.attributes("-topmost", True)

            tk.Label(
                root,
                text="Black terrain controls",
                font=("TkDefaultFont", 11, "bold"),
            ).pack(pady=(12, 6))

            status = tk.StringVar(master=root, value="Ready")

            def reset_origin() -> None:
                self.state_machine.request_reset()
                status.set("Reset requested: origin / STAND / zero velocity")

            def zero_velocity() -> None:
                self.state_machine.request_stop()
                status.set("Motion stopped; zero velocity")

            def pure_wall_climb() -> None:
                self.state_machine.request_state(RobotState.WALL_CLIMB)
                status.set("Pure-IK wall climb requested (K)")

            tk.Button(
                root,
                text="RESET TO ORIGIN  (R / Home)",
                command=reset_origin,
                width=32,
                bg="#b71c1c",
                fg="white",
                activebackground="#d32f2f",
                activeforeground="white",
            ).pack(pady=4)
            tk.Button(
                root,
                text="STOP / ZERO SPEED  (Space)",
                command=zero_velocity,
                width=32,
            ).pack(pady=2)
            tk.Button(
                root,
                text="PURE WALL CLIMB  (K)",
                command=pure_wall_climb,
                width=32,
                bg="#1565c0",
                fg="white",
                activebackground="#1e88e5",
                activeforeground="white",
            ).pack(pady=2)
            tk.Label(root, textvariable=status, fg="#1565c0").pack(pady=(6, 4))
            tk.Label(
                root,
                text=(
                    "Up/Down: forward/backward  Left/Right: strafe\n"
                    "Q/E: turn left/right  Space: stop  R/Home: reset\n"
                    "C: climb  J: wall vault  K: wall IK-climb\n"
                    "L: lie  X: crawl  B: bridge-B walk\n"
                    "Repeat a key to increase that axis; motion axes combine"
                ),
                fg="#555555",
            ).pack()
            root.mainloop()
        except Exception as exc:  # pragma: no cover - depends on desktop packages
            print(
                f"[GUI] reset button failed ({exc}); use R/Home instead",
                flush=True,
            )


def _run_gui(
    env: Any,
    wrapper: Any,
    policy: MonitoredPolicy,
    state_machine: StateMachinePolicy,
    max_steps: int | None,
) -> tuple[int, bool, bool]:
    from mjlab.viewer.native.viewer import NativeMujocoViewer

    print(
        "[GUI] controls: Up/Down = forward/backward via base policy, "
        "Left/Right = strafe left/right, Q/E = turn left/right via base policy, "
        "A/D = strafe aliases, Space = stop, R/Home = reset origin; "
        "C = climb, J = wall vault, L = lie down, X = crawl forward, "
        "K = pure wall IK-climb, B = bridge-B walk; motion axes combine, "
        "repeat a key to increase its speed",
        flush=True,
    )
    panel = GuiControlPanel(state_machine)
    panel.start()
    viewer = NativeMujocoViewer(
        wrapper,
        policy,
        frame_rate=60.0,
        key_callback=state_machine.handle_key,
        enable_perturbations=False,
    )

    # mjlab's native viewer maps Space to pause, Right to single-step, A to
    # show-all-environments, and R to debug-visualization toggling before
    # invoking the task callback.  Those defaults conflict with this FSM's
    # Space/Right/A/R controls, so consume the conflicting keys here instead of
    # forwarding them to the viewer defaults.
    from mjlab.viewer.native.keys import KEY_A, KEY_R, KEY_RIGHT, KEY_SPACE

    viewer_key_callback = viewer._safe_key_callback
    conflicting_keys = {KEY_SPACE, KEY_RIGHT, KEY_A, KEY_R}

    def gui_key_callback(key: int) -> None:
        if key in conflicting_keys:
            state_machine.handle_key(key)
            return
        viewer_key_callback(key)

    viewer._safe_key_callback = gui_key_callback

    try:
        viewer.run(num_steps=max_steps)
    except KeyboardInterrupt:
        print("[RUN] GUI interrupted by user", flush=True)
        policy.flush()
        return policy.step, True, False
    policy.flush()
    reached_limit = max_steps is not None and policy.step >= max_steps
    return policy.step, False, reached_limit


def run(args: argparse.Namespace) -> int:
    initial_state = RobotState(args.state) if args.state else SCENARIO_DEFAULT_STATE[args.scenario]
    position, yaw = _scenario_pose(args)
    output_prefix = _default_output_prefix(args)
    csv_path = _safe_output_path(output_prefix, ".csv")
    json_path = _safe_output_path(output_prefix, ".json")
    started_at = time.time()

    policy_config = PolicyConfig.load(args.policy_config)
    sim_config = SimulationConfig(
        terrain_xml=args.terrain.expanduser().resolve(),
        policies=policy_config,
        device=args.device,
        seed=args.seed,
    )

    print(
        f"[RUN] scenario={args.scenario} mode={args.mode} "
        f"initial={initial_state.value} spawn=({position[0]:.2f},"
        f"{position[1]:.2f},{position[2]:.2f}) yaw={math.degrees(yaw):.1f}deg",
        flush=True,
    )

    env, wrapper, _ = build_environment(sim_config)
    try:
        obs, _ = wrapper.reset()
        wrapper.place_robot_at_pose(position, yaw)
        obs = wrapper.get_observations()

        bank = PolicyBank(wrapper, policy_config, device=args.device)
        bank.load_all()
        state_machine = StateMachinePolicy(env, wrapper, bank, initial_state)
        if args.forward_speed is not None:
            state_machine.set_forward_speed(args.forward_speed)
            state_machine.request_forward(increment=False)
        if args.turn_wz is not None:
            state_machine.set_turn_rate(args.turn_wz)
            state_machine.request_turn(
                1 if args.turn_wz >= 0.0 else -1,
                increment=False,
            )
        if args.backward_speed is not None:
            state_machine.set_backward_speed(args.backward_speed)
            state_machine.request_backward(increment=False)
        if args.strafe_vy is not None:
            state_machine.set_strafe_rate(args.strafe_vy)
            state_machine.request_strafe(
                1 if args.strafe_vy >= 0.0 else -1,
                increment=False,
            )
        recorder = RunRecorder()
        monitored_policy = MonitoredPolicy(state_machine, recorder)
        max_steps = _max_steps(args, state_machine.step_dt)

        if args.mode == "gui":
            steps, interrupted, reached_limit = _run_gui(
                env,
                wrapper,
                monitored_policy,
                state_machine,
                max_steps,
            )
        else:
            steps, interrupted, reached_limit = _run_headless(
                wrapper,
                monitored_policy,
                obs,
                max_steps,
                args.scenario,
            )

        success, failures = _success_and_failures(
            scenario=args.scenario,
            recorder=recorder,
            state_machine=state_machine,
            interrupted=interrupted,
            reached_limit=reached_limit,
        )
        incomplete_gui = (
            args.mode == "gui" and not interrupted and not reached_limit
        )
        if incomplete_gui and args.strict:
            success = False
            failures.append("gui_closed_before_completion")
        elif incomplete_gui:
            failures.append("gui_closed_before_completion")

        first = recorder.first_metrics
        final = recorder.last_metrics
        result = {
            "success": success,
            "failure_reasons": failures,
            "scenario": args.scenario,
            "mode": args.mode,
            "initial_state": initial_state.value,
            "final_state": state_machine.state.value,
            "steps": steps,
            "sim_time_s": steps * state_machine.step_dt,
            "wall_time_s": time.time() - started_at,
            "interrupted": interrupted,
            "reached_limit": reached_limit,
            "state_sequence": recorder.state_sequence,
            "autopilot": state_machine.autopilot_result,
            "wall": {
                **state_machine.wall_result,
                "spawn": WALL_SPAWN_POS,
                "center": WALL_CENTER,
                "x_range_m": WALL_X_RANGE,
                "y_range_m": WALL_Y_RANGE,
                "height_m": WALL_HEIGHT_M,
                "clear_y_m": WALL_CLEAR_Y,
                "support_contact": recorder.wall_support_contact,
                "leg_contact": recorder.wall_leg_contact,
                "cleared": recorder.wall_cleared,
            },
            "wall_climb": {
                **state_machine.wall_climb_result,
                "spawn": WALL_CLIMB_SPAWN_POS,
                "center": WALL_CENTER,
                "x_range_m": WALL_X_RANGE,
                "y_range_m": WALL_Y_RANGE,
                "height_m": WALL_HEIGHT_M,
                "clear_y_m": WALL_CLEAR_Y,
                "min_final_z_m": WALL_CLIMB_MIN_FINAL_Z,
                "support_contact": recorder.wall_support_contact,
                "leg_contact": recorder.wall_leg_contact,
                "cleared": recorder.wall_cleared,
                "minimum_root_z_m": (
                    recorder.min_root_z
                    if math.isfinite(recorder.min_root_z)
                    else None
                ),
            },
            "bridge": {
                **state_machine.bridge_result,
                "north_spawn": BRIDGE_B_SPAWN_POS,
                "south_exit_y_m": BRIDGE_B_SOUTH_EXIT_Y,
                "min_final_y_m": BRIDGE_B_MIN_FINAL_Y,
                "min_final_z_m": BRIDGE_B_MIN_FINAL_Z,
                "x_range_m": BRIDGE_B_X_RANGE,
                "minimum_root_z_m": (
                    recorder.min_root_z
                    if math.isfinite(recorder.min_root_z)
                    else None
                ),
            },
            "spawn": {
                "position": position,
                "yaw_rad": yaw,
            },
            "start_metrics": asdict(first) if first is not None else None,
            "final_metrics": asdict(final) if final is not None else None,
            "bar": {
                "lower_z_m": BAR_LOWER_Z,
                "crawl_exit_y_m": CRAWL_EXIT_Y,
                "crossed": recorder.bar_crossed,
                "contact": recorder.bar_contact,
                "clean_crossed": recorder.bar_clean_crossed,
                "peak_collision_envelope_z_m": recorder.max_bar_envelope_z or None,
                "measured_clearance_m": recorder.measured_bar_clearance_m,
                "minimum_reported_clearance_m": (
                    recorder.min_bar_clearance_m
                    if math.isfinite(recorder.min_bar_clearance_m)
                    else None
                ),
            },
            "peak_support_top_z_m": recorder.peak_envelope_m,
            "max_action_abs": recorder.max_action_abs,
            "policies": {
                "observation_dim": (
                    HIM_HISTORY_LENGTH * HIM_OBSERVATION_DIM
                    if policy_config.base.kind == "him_torchscript"
                    else (
                        FLAT_RSL_RL_OBSERVATION_DIM
                        if policy_config.base.kind == "rsl_rl_flat45"
                        else 122
                    )
                ),
                "observation_history_frames": (
                    HIM_HISTORY_LENGTH
                    if policy_config.base.kind == "him_torchscript"
                    else 1
                ),
                "action_dim": 12,
                "clip_actions": None,
                "checkpoints": asdict(policy_config),
            },
            "assets": {
                "terrain_xml": str(sim_config.terrain_xml),
            },
            "telemetry": {
                "csv": str(csv_path),
                "json": str(json_path),
                "rows": len(recorder.rows),
            },
        }
        _write_csv(csv_path, recorder.rows)
        _write_json(json_path, result)

        print(
            f"[RESULT] success={success} final={state_machine.state.value} "
            f"bar_clean={recorder.bar_clean_crossed} "
            f"bar_contact={recorder.bar_contact} "
            f"clearance={recorder.measured_bar_clearance_m} "
            f"bridge_stage={state_machine.bridge_result['stage']} "
            f"bridge_south_reached={state_machine.bridge_result['south_reached']} "
            f"wall_stage={state_machine.wall_result['stage']} "
            f"wall_support_contact={recorder.wall_support_contact} "
            f"wall_leg_contact={recorder.wall_leg_contact} "
            f"wall_cleared={recorder.wall_cleared}",
            flush=True,
        )
        if failures:
            print(f"[RESULT] reasons={'; '.join(failures)}", flush=True)
        print(f"[RESULT] csv={csv_path}", flush=True)
        print(f"[RESULT] json={json_path}", flush=True)

        if args.mode == "headless" or args.strict:
            return 130 if interrupted else (0 if success else 2)
        return 0
    finally:
        env.close()


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return run(args)
    except Exception:
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
