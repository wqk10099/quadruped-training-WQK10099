"""Strict loaders and reset-safe dispatch for Black locomotion policies."""

from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from typing import Any

import torch

from .config import ACTION_SCALE
from .mjlab_compat import black_rl_cfg

_PACKAGE_ROOT = Path(__file__).resolve().parents[2]

# The exported HIM policy uses the legged-gym order
# FL, FR, RL, RR while the MuJoCo model is FL, FR, RR, RL.
HIM_POLICY_JOINT_NAMES = (
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
)
HIM_POLICY_DEFAULT_JOINT_POS = (
    0.0,
    0.8014,
    -1.527,
    0.0,
    -0.8014,
    1.527,
    0.0,
    0.8014,
    -1.527,
    0.0,
    -0.8014,
    1.527,
)
HIM_OBSERVATION_DIM = 45
HIM_HISTORY_LENGTH = 6
HIM_COMMAND_SCALE = (2.0, 2.0, 0.25)
HIM_ANG_VEL_SCALE = 0.25
HIM_JOINT_VEL_SCALE = 0.05
HIM_OBSERVATION_CLIP = 100.0

# The supplied flat-ground student checkpoint was trained by mjlab with the
# first six actor terms only.  The terrain environment appends a 77-D height
# scan after that prefix, so the deployment adapter deliberately consumes only
# [:, :45] instead of constructing the 122-D rough-terrain RNN.
FLAT_RSL_RL_OBSERVATION_DIM = 45
FLAT_RSL_RL_ACTION_DIM = 12
FLAT_RSL_RL_HIDDEN_DIMS = (256, 128)
FLAT_RSL_RL_RNN_HIDDEN_DIM = 128


def _resolve_checkpoint(path: str) -> Path:
    checkpoint = Path(path).expanduser()
    if not checkpoint.is_absolute():
        checkpoint = _PACKAGE_ROOT / checkpoint
    return checkpoint.resolve()


class HimTorchScriptPolicy:
    """Deployment adapter for the supplied six-frame HIM TorchScript policy.

    best.pt was exported from legged-gym and is not an RSL-RL runner
    checkpoint. It consumes six 45-D observations ordered as command,
    body angular velocity, projected gravity, joint position, joint velocity,
    and previous policy action. This adapter performs the required scaling,
    history stacking, joint-order conversion, and default-pose offset so the
    mjlab action manager receives actions in its native order.
    """

    def __init__(self, wrapper: Any, checkpoint: Path, device: str) -> None:
        self.env = wrapper.env
        self.robot = self.env.scene["robot"]
        self.device = torch.device(device)
        self.model = torch.jit.load(str(checkpoint), map_location=self.device)
        self.model.eval()
        self.model.to(self.device)

        joint_names = tuple(self.robot.joint_names)
        try:
            self._policy_to_sim = torch.tensor(
                [joint_names.index(name) for name in HIM_POLICY_JOINT_NAMES],
                dtype=torch.long,
                device=self.device,
            )
        except ValueError as exc:
            raise ValueError(
                "Black robot joint names do not match the HIM policy contract: "
                f"{joint_names}"
            ) from exc

        model_default = torch.tensor(
            HIM_POLICY_DEFAULT_JOINT_POS,
            dtype=self.robot.data.default_joint_pos.dtype,
            device=self.device,
        )
        model_default_sim = torch.empty_like(model_default)
        model_default_sim[self._policy_to_sim] = model_default
        sim_default = self.robot.data.default_joint_pos[0]
        # mjlab adds the robot default pose before applying ACTION_SCALE.
        self._action_offset = (model_default_sim - sim_default) / ACTION_SCALE
        self._last_action = torch.zeros(
            self.robot.data.joint_pos.shape[0],
            len(HIM_POLICY_JOINT_NAMES),
            dtype=self.robot.data.joint_pos.dtype,
            device=self.device,
        )
        self._history: list[torch.Tensor] | None = None

    def reset(self, dones: torch.Tensor | None = None) -> None:
        reset_all = dones is None or bool(torch.any(dones).item())
        if not reset_all:
            return
        self._history = None
        self._last_action.zero_()

    def _observation(self, obs: Any) -> torch.Tensor:
        actor_obs = obs["actor"]
        if actor_obs.ndim != 2 or actor_obs.shape[-1] < HIM_OBSERVATION_DIM:
            raise ValueError(
                "Black actor observation must have at least 45 dimensions, "
                f"got {tuple(actor_obs.shape)}"
            )

        command = self.env.command_manager.get_command("twist")
        ang_vel = actor_obs[:, 0:3]
        gravity = actor_obs[:, 3:6]
        joint_pos_rel = actor_obs[:, 6:18]
        joint_vel = actor_obs[:, 18:30]

        sim_default = self.robot.data.default_joint_pos
        joint_pos_abs = joint_pos_rel + sim_default
        policy_joint_pos = joint_pos_abs[:, self._policy_to_sim]
        policy_joint_vel = joint_vel[:, self._policy_to_sim]
        command_scale = torch.tensor(
            HIM_COMMAND_SCALE, dtype=actor_obs.dtype, device=self.device
        )
        model_default = torch.tensor(
            HIM_POLICY_DEFAULT_JOINT_POS,
            dtype=actor_obs.dtype,
            device=self.device,
        )
        return torch.cat(
            (
                command * command_scale,
                ang_vel * HIM_ANG_VEL_SCALE,
                gravity,
                policy_joint_pos - model_default,
                policy_joint_vel * HIM_JOINT_VEL_SCALE,
                self._last_action,
            ),
            dim=-1,
        ).clamp(-HIM_OBSERVATION_CLIP, HIM_OBSERVATION_CLIP)

    def act(self, obs: Any) -> torch.Tensor:
        frame = self._observation(obs)
        if self._history is None:
            self._history = [frame] * HIM_HISTORY_LENGTH
        else:
            self._history = [frame, *self._history[: HIM_HISTORY_LENGTH - 1]]
        history = torch.cat(self._history, dim=-1)
        with torch.no_grad():
            action = self.model(history)
        if action.ndim != 2 or action.shape[-1] != 12:
            raise ValueError(
                "HIM policy output must have shape [num_envs, 12], "
                f"got {tuple(action.shape)}"
            )
        self._last_action = action.detach().clone()

        # Reorder policy outputs to the simulator and cancel the difference
        # between the policy's training default pose and mjlab's robot default.
        sim_action = torch.empty_like(action)
        sim_action[:, self._policy_to_sim] = action
        return sim_action + self._action_offset


class FlatRSLRLPolicy:
    """Deployment adapter for a 45-D flat-ground RSL-RL student checkpoint.

    The checkpoint contains ``obs_normalizer.*``, ``mlp.*``, ``rnn.rnn.*`` and
    ``distribution.std_param``.  Reconstructing ``RNNModel`` and loading its
    actor state strictly preserves the training-time observation normalization
    and GRU recurrence.  Only the first 45 actor terms are used; the remaining
    terrain height scan is intentionally excluded because this student was
    trained on flat ground.
    """

    def __init__(self, checkpoint: Path, device: str) -> None:
        self.device = torch.device(device)

        from tensordict import TensorDict
        from rsl_rl.models import RNNModel

        self._tensor_dict_cls = TensorDict
        checkpoint_data = torch.load(
            str(checkpoint),
            map_location=self.device,
            weights_only=False,
        )
        actor_state = checkpoint_data.get("actor_state_dict")
        if not isinstance(actor_state, dict):
            raise ValueError(
                "Flat RSL-RL checkpoint must contain actor_state_dict: "
                f"{checkpoint}"
            )

        infos = checkpoint_data.get("infos", {})
        distillation = (
            infos.get("distillation", {}) if isinstance(infos, dict) else {}
        )
        observation_dim = distillation.get(
            "actor_observation_dim", FLAT_RSL_RL_OBSERVATION_DIM
        )
        action_dim = distillation.get("action_dim", FLAT_RSL_RL_ACTION_DIM)
        if observation_dim != FLAT_RSL_RL_OBSERVATION_DIM:
            raise ValueError(
                "Expected a 45-D flat student checkpoint, got "
                f"actor_observation_dim={observation_dim}"
            )
        if action_dim != FLAT_RSL_RL_ACTION_DIM:
            raise ValueError(
                "Expected a 12-D Black action checkpoint, got "
                f"action_dim={action_dim}"
            )

        sample_obs = TensorDict(
            {
                "actor": torch.zeros(
                    1,
                    FLAT_RSL_RL_OBSERVATION_DIM,
                    dtype=torch.float32,
                    device=self.device,
                )
            },
            batch_size=[1],
            device=self.device,
        )
        self.model = RNNModel(
            sample_obs,
            {"actor": ["actor"]},
            "actor",
            FLAT_RSL_RL_ACTION_DIM,
            hidden_dims=FLAT_RSL_RL_HIDDEN_DIMS,
            activation="elu",
            obs_normalization=True,
            distribution_cfg={
                "class_name": "GaussianDistribution",
                "init_std": 0.1,
                "std_type": "scalar",
            },
            rnn_type="gru",
            rnn_hidden_dim=FLAT_RSL_RL_RNN_HIDDEN_DIM,
            rnn_num_layers=1,
        ).to(self.device)
        self.model.load_state_dict(actor_state, strict=True)
        self.model.eval()

    def reset(self, dones: torch.Tensor | None = None) -> None:
        self.model.reset(dones)

    def act(self, obs: Any) -> torch.Tensor:
        actor_obs = obs["actor"]
        if (
            actor_obs.ndim != 2
            or actor_obs.shape[-1] < FLAT_RSL_RL_OBSERVATION_DIM
        ):
            raise ValueError(
                "Black actor observation must contain at least the 45-D flat "
                f"policy prefix, got {tuple(actor_obs.shape)}"
            )

        policy_obs = actor_obs[:, :FLAT_RSL_RL_OBSERVATION_DIM]
        if policy_obs.device != self.device:
            policy_obs = policy_obs.to(self.device)
        policy_obs = self._tensor_dict_cls(
            {"actor": policy_obs},
            batch_size=[policy_obs.shape[0]],
            device=self.device,
        )
        with torch.no_grad():
            action = self.model(policy_obs, stochastic_output=False)
        if action.ndim != 2 or action.shape[-1] != FLAT_RSL_RL_ACTION_DIM:
            raise ValueError(
                "Flat RSL-RL policy output must have shape [num_envs, 12], "
                f"got {tuple(action.shape)}"
            )
        return action


class PolicyBank:
    """Lazily load the base, climb, and jump controllers."""

    def __init__(self, wrapper: Any, policy_config: Any, device: str = "cpu") -> None:
        self.wrapper = wrapper
        self.policy_config = policy_config
        self.device = torch.device(device)
        self._runners: dict[str, Any] = {}
        self._policies: dict[str, Any] = {}
        self._active: str | None = None
        self._rl_cfg = black_rl_cfg()

    @property
    def active(self) -> str | None:
        return self._active

    def _build_runner(self, name: str, spec: Any) -> None:
        checkpoint = _resolve_checkpoint(spec.checkpoint)
        if not checkpoint.is_file():
            raise FileNotFoundError(f"{name} checkpoint not found: {checkpoint}")

        if spec.kind == "him_torchscript":
            self._policies[name] = HimTorchScriptPolicy(
                self.wrapper,
                checkpoint,
                str(self.device),
            )
            return
        if spec.kind == "rsl_rl_flat45":
            if name != "base":
                raise ValueError(
                    "rsl_rl_flat45 is only supported for the base policy"
                )
            self._policies[name] = FlatRSLRLPolicy(
                checkpoint,
                str(self.device),
            )
            return
        if spec.kind != "rsl_rl":
            raise ValueError(f"Unknown policy kind {spec.kind!r} for {name!r}")

        from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

        runner_cfg_fn = getattr(self._rl_cfg, spec.runner_cfg, None)
        if runner_cfg_fn is None:
            raise AttributeError(
                f"Unknown runner config {spec.runner_cfg!r} for policy {name!r}"
            )

        runner_cfg = runner_cfg_fn()
        runner = VelocityOnPolicyRunner(
            self.wrapper,
            asdict(runner_cfg),
            device=str(self.device),
        )
        runner.load(
            str(checkpoint),
            load_cfg={"actor": True},
            strict=True,
            map_location=str(self.device),
        )
        policy = runner.get_inference_policy(device=str(self.device))
        self._runners[name] = runner
        self._policies[name] = policy

    def get(self, name: str) -> Any:
        if name not in self._policies:
            spec = getattr(self.policy_config, name, None)
            if spec is None:
                raise KeyError(f"Unknown policy {name!r}")
            self._build_runner(name, spec)
        return self._policies[name]

    def load_all(self) -> None:
        for name in ("base", "climb", "jump"):
            self.get(name)

    def activate(self, name: str) -> Any:
        policy = self.get(name)
        policy.reset()
        self._active = name
        return policy

    def act(self, name: str, obs: Any) -> torch.Tensor:
        policy = self.get(name)
        actor_obs = obs["actor"]
        if isinstance(policy, (HimTorchScriptPolicy, FlatRSLRLPolicy)):
            action = policy.act(obs)
        else:
            if actor_obs.ndim != 2 or actor_obs.shape[-1] != 122:
                raise ValueError(
                    "Black actor observation must have shape [num_envs, 122], "
                    f"got {tuple(actor_obs.shape)}"
                )
            with torch.no_grad():
                action = policy(obs)
        if action.ndim != 2 or action.shape[-1] != 12:
            raise ValueError(
                "Black policy action must have shape [num_envs, 12], "
                f"got {tuple(action.shape)}"
            )
        return action

    def reset(self, dones: torch.Tensor | None = None) -> None:
        """Compatibility reset called directly by the mjlab viewer."""
        for policy in self._policies.values():
            policy.reset(dones)
        if dones is None or bool(torch.any(dones).item()):
            self._active = None

    def reset_active(self) -> None:
        if self._active is not None:
            self._policies[self._active].reset()
