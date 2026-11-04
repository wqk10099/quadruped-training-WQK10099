#!/usr/bin/env python3
"""Short integration smoke test for every FSM skill and policy."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import torch

from black_terrain_fsm.config import PolicyConfig, SimulationConfig
from black_terrain_fsm.policies import PolicyBank
from black_terrain_fsm.state_machine import RobotState, StateMachinePolicy
from black_terrain_fsm.terrain import build_environment


EXPECTED_SOURCE = {
    RobotState.STAND: "fixed_pose",
    RobotState.LIE_DOWN: "fixed_pose",
    RobotState.RECOVER: "fixed_pose",
    RobotState.LOCOMOTION: "base",
    RobotState.CLIMB: "climb",
    RobotState.JUMP: "jump",
    RobotState.HOLD: "hold",
    RobotState.EMERGENCY_STOP: "hold",
}


def _run_case(
    env: Any,
    wrapper: Any,
    policy: StateMachinePolicy,
    obs: Any,
    state: RobotState,
    steps: int,
) -> dict[str, Any]:
    policy.request_state(state)
    sources: set[str] = set()
    action_abs_max = 0.0
    finite = True
    done_seen = False
    for _ in range(steps):
        action = policy.act(obs)
        action_abs = float(torch.max(torch.abs(action)).item())
        finite = finite and math.isfinite(action_abs)
        action_abs_max = max(action_abs_max, action_abs)
        sources.add(policy.last_action_source)
        obs, _, dones, _ = wrapper.step(action)
        done_seen = done_seen or bool(torch.any(dones).item())
        policy.reset(dones)

    metrics = policy.monitor.measure()
    expected = EXPECTED_SOURCE[state]
    passed = finite and expected in sources
    return {
        "requested_state": state.value,
        "final_state": policy.state.value,
        "expected_source": expected,
        "sources": sorted(sources),
        "passed": passed,
        "finite_actions": finite,
        "action_abs_max": action_abs_max,
        "done_seen": done_seen,
        "position": [metrics.x, metrics.y, metrics.z],
        "support_top_z": metrics.support_top_z,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="cpu")
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "results" / "skill_smoke.json",
    )
    args = parser.parse_args()

    cfg = SimulationConfig(policies=PolicyConfig.load(), device=args.device)
    env, wrapper, _ = build_environment(cfg)
    try:
        obs, _ = wrapper.reset()
        bank = PolicyBank(wrapper, cfg.policies, device=args.device)
        bank.load_all()
        policy = StateMachinePolicy(env, wrapper, bank, RobotState.STAND)
        policy.set_velocity(0.80, 0.0, 0.0)

        cases = (
            (RobotState.STAND, 90),
            (RobotState.LIE_DOWN, 90),
            (RobotState.RECOVER, 90),
            (RobotState.LOCOMOTION, 100),
            (RobotState.CLIMB, 100),
            (RobotState.JUMP, 75),
            (RobotState.HOLD, 30),
            (RobotState.EMERGENCY_STOP, 30),
        )
        results = []
        for state, steps in cases:
            result = _run_case(env, wrapper, policy, obs, state, steps)
            results.append(result)
            print(json.dumps(result, ensure_ascii=False), flush=True)

        payload = {
            "success": all(item["passed"] for item in results),
            "policies_loaded": ["base", "climb", "jump"],
            "cases": results,
        }
    finally:
        env.close()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"[SMOKE] success={payload['success']} output={args.output}", flush=True)
    return 0 if payload["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
