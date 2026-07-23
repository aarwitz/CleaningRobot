

# ===== RoArm sock fine-tune (auto-injected by gpu_bootstrap.sh; safe to delete) =====
# Appended to the END of openpi's src/openpi/training/config.py, so every name it
# needs from that module (TrainConfig, DataConfig, DataConfigFactory,
# ModelTransformFactory, _CONFIGS, _CONFIGS_DICT) is already defined above.
# Structure mirrors LeRobotLiberoDataConfig — if your openpi version differs,
# compare against src/openpi/training/config.py's own libero config and adjust
# the two lines flagged VERSION-SENSITIVE.
import dataclasses as _rdc

import openpi.transforms as _rtf
import openpi.training.weight_loaders as _rwl
from openpi.models import pi0 as _rpi0
from openpi.policies import roarm_policy as _roarm_policy


@_rdc.dataclass(frozen=True)
class RoarmDataConfig(DataConfigFactory):
    def create(self, assets_dirs, model_config):
        # rename our LeRobot dataset keys -> the keys RoarmInputs reads
        repack = _rtf.Group(inputs=[_rtf.RepackTransform({
            "observation/image": "observation.images.exterior",
            "observation/state": "observation.state",
            "actions": "action",
            "prompt": "prompt",
        })])
        # robot <-> model tensors (pad/mask in, slice 4 DoF out)
        data_transforms = _rtf.Group(
            inputs=[_roarm_policy.RoarmInputs(
                action_dim=model_config.action_dim,
                model_type=model_config.model_type)],
            outputs=[_roarm_policy.RoarmOutputs()],
        )
        model_transforms = ModelTransformFactory()(model_config)  # VERSION-SENSITIVE
        return _rdc.replace(
            self.create_base_config(assets_dirs, model_config),   # VERSION-SENSITIVE
            repack_transforms=repack,
            data_transforms=data_transforms,
            model_transforms=model_transforms,
        )


def _roarm_lora_model():
    return _rpi0.Pi0Config(
        paligemma_variant="gemma_2b_lora",
        action_expert_variant="gemma_300m_lora",
    )


_ROARM_CONFIGS = [
    TrainConfig(
        name=f"pi0_roarm_sock_{_sp}_lora" if _sp != "joint" else "pi0_roarm_sock_lora",
        model=_roarm_lora_model(),
        data=RoarmDataConfig(
            repo_id=f"roarm_sock_{_sp}",
            base_config=DataConfig(prompt_from_task=True),
        ),
        weight_loader=_rwl.CheckpointWeightLoader(
            "gs://openpi-assets/checkpoints/pi0_base/params"),
        num_train_steps=30_000,
        freeze_filter=_roarm_lora_model().get_freeze_filter(),
        ema_decay=None,
    )
    for _sp in ("cartesian", "joint")
]

# register (both the list and the name->config lookup openpi resolves against)
_CONFIGS.extend(_ROARM_CONFIGS)
try:
    _CONFIGS_DICT.update({c.name: c for c in _ROARM_CONFIGS})
except NameError:  # dict built lazily in some versions; extend() above suffices
    pass
# ===== end RoArm sock fine-tune =====
