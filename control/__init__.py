"""Controllers for CompanionBot simulation baselines."""

from .cascade_pid import CascadePID, CascadePIDCommand, CascadePIDConfig, load_config
from .fixed_lqr import ControlCommand, FixedLQR, LQRDesign, design_from_files
from .full_state_identification import (
    DiscreteStateSpaceModel,
    FullStateFit,
    design_discrete_lqr,
    fit_full_state_ridge,
)
from .disturbance_rejection import (
    CompensationCommand,
    DisturbanceObservation,
    DisturbanceRejectionConfig,
    TwoTimescaleDisturbanceCompensator,
    config_from_dict as disturbance_rejection_config_from_dict,
)
from .probing import CenteredPRBS, PRBSConfig
from .probe_manager import (
    AutoProbeConfig,
    AutoProbeManager,
    ProbeTransition,
    auto_probe_config_from_dict,
)

__all__ = [
    "AutoProbeConfig",
    "AutoProbeManager",
    "CascadePID",
    "CascadePIDCommand",
    "CascadePIDConfig",
    "ControlCommand",
    "CompensationCommand",
    "DiscreteStateSpaceModel",
    "DisturbanceObservation",
    "DisturbanceRejectionConfig",
    "FixedLQR",
    "FullStateFit",
    "LQRDesign",
    "CenteredPRBS",
    "PRBSConfig",
    "ProbeTransition",
    "TwoTimescaleDisturbanceCompensator",
    "auto_probe_config_from_dict",
    "design_discrete_lqr",
    "design_from_files",
    "disturbance_rejection_config_from_dict",
    "load_config",
    "fit_full_state_ridge",
]
