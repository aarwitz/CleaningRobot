"""openpi input/output transforms for the 4-DoF RoArm M2-S sock task.

Copy this into an openpi checkout at `src/openpi/policies/roarm_policy.py`
(mirrors `libero_policy.py`, the canonical bring-your-own-robot example).

The pi0 model works in a fixed padded action/state dimension (`action_dim`, 32
by default) and a 3-camera image dict (base + two wrists). Our robot has 4 real
DoF and ONE camera, so:
  * RoarmInputs pads state/actions 4 -> action_dim and fills only base_0_rgb,
    masking the two wrist slots OFF (the model then ignores them).
  * RoarmOutputs slices the model's action back down to our 4 real dims.
"""
import dataclasses

import numpy as np

from openpi import transforms
from openpi.models import model as _model


def _parse_image(image) -> np.ndarray:
    image = np.asarray(image)
    if np.issubdtype(image.dtype, np.floating):
        image = (255 * image).astype(np.uint8)
    if image.shape[0] == 3:              # CHW -> HWC
        image = np.transpose(image, (1, 2, 0))
    return image


@dataclasses.dataclass(frozen=True)
class RoarmInputs(transforms.DataTransformFn):
    # Model's padded action dimension (pi0: 32). Must match the model config.
    action_dim: int
    model_type: _model.ModelType = _model.ModelType.PI0

    def __call__(self, data: dict) -> dict:
        state = transforms.pad_to_dim(data["observation/state"], self.action_dim)
        base_image = _parse_image(data["observation/image"])

        # One real camera. pi0 wants a 3-cam dict; mask the two we don't have.
        zeros = np.zeros_like(base_image)
        inputs = {
            "state": state,
            "image": {
                "base_0_rgb": base_image,
                "left_wrist_0_rgb": zeros,
                "right_wrist_0_rgb": zeros,
            },
            "image_mask": {
                "base_0_rgb": np.True_,
                "left_wrist_0_rgb": np.False_,
                "right_wrist_0_rgb": np.False_,
            },
        }
        if "actions" in data:
            inputs["actions"] = transforms.pad_to_dim(data["actions"], self.action_dim)
        if "prompt" in data:
            inputs["prompt"] = data["prompt"]
        return inputs


@dataclasses.dataclass(frozen=True)
class RoarmOutputs(transforms.DataTransformFn):
    """Slice the model's padded action back to the 4 real DoF."""

    def __call__(self, data: dict) -> dict:
        return {"actions": np.asarray(data["actions"][:, :4])}
