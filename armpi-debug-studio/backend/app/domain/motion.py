"""Effective motion limits for the G1 supervised write path.

The mapping file keeps the *provisional* limits that both mock and read-only
hardware use. A hardware write path that runs before the limits are formally
verified is only allowed as **supervised commissioning**: the operator starts
the backend with an explicit acknowledgement, every command is single-joint,
the step is capped and the duration floor is raised. This module is the single
source for those effective values so the service, the safety supervisor and
the hardware adapter cannot drift apart.
"""

from __future__ import annotations

from dataclasses import dataclass

from .mapping import MappingConfig

# The acknowledgement is deliberately a fixed phrase and not a boolean. It is
# read from the environment so enabling motion needs an action in the launch
# command, not just a value in a UI form.
SUPERVISED_MOTION_ACK = "I_HAVE_SUPPORT_AND_A_CLEAR_WORKSPACE"

# Supervised commissioning never exceeds these values, even if the mapping's
# safety block is more permissive. 0.10 rad is about 2.4 degrees per command;
# a 1.0 s floor keeps the vendor's pulse interpolation visibly slow.
SUPERVISED_MAX_STEP_RAD = 0.10
SUPERVISED_MIN_DURATION_S = 1.0


@dataclass(frozen=True)
class EffectiveMotionLimits:
    """Limits used by validation, the safety gate and the adapter."""

    max_step_rad: float
    min_duration_s: float
    max_duration_s: float
    single_joint_only: bool
    supervised: bool

    @property
    def mode_label(self) -> str:
        return "supervised" if self.supervised else "standard"


def effective_motion_limits(
    mapping: MappingConfig,
    *,
    supervised: bool,
) -> EffectiveMotionLimits:
    """Return the active step/duration limits.

    ``supervised`` is true only when the operator supplied the fixed
    acknowledgement. It must never be derived from a UI checkbox.
    """
    if supervised:
        # Align the cap down to a whole servo pulse so a target accepted here
        # cannot map to one more pulse than the gateway allows. The gateway
        # uses floor(step / radians_per_tick); a radian cap that sits between
        # two pulses would otherwise validate then fail at dispatch.
        step_pulses = int(SUPERVISED_MAX_STEP_RAD / mapping.radians_per_tick)
        aligned_step = step_pulses * mapping.radians_per_tick
        return EffectiveMotionLimits(
            max_step_rad=min(mapping.safety.max_step_rad, aligned_step),
            min_duration_s=max(
                mapping.safety.min_duration_s, SUPERVISED_MIN_DURATION_S
            ),
            max_duration_s=mapping.safety.max_duration_s,
            single_joint_only=True,
            supervised=True,
        )
    return EffectiveMotionLimits(
        max_step_rad=mapping.safety.max_step_rad,
        min_duration_s=mapping.safety.min_duration_s,
        max_duration_s=mapping.safety.max_duration_s,
        single_joint_only=False,
        supervised=False,
    )
