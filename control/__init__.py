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
    FilteredDisturbanceCompensator,
    config_from_dict as disturbance_rejection_config_from_dict,
)
from .disturbance_gate import (
    DisturbanceGateConfig,
    DisturbanceGateOutput,
    PersistentDisturbanceGate,
)
from .probing import CenteredPRBS, PRBSConfig
from .probe_manager import (
    AutoProbeConfig,
    AutoProbeManager,
    ProbeTransition,
    auto_probe_config_from_dict,
)
from .motion_reference import (
    JerkLimitedLongitudinalReferenceGenerator,
    LongitudinalReference,
    LongitudinalReferenceGenerator,
)
from .trajectory_feedforward import (
    NominalDiscreteFeedforward,
    NominalFeedforwardCommand,
    NominalTrajectoryProjection,
    project_hidden_reference_and_input,
)
from .reference_lifecycle import (
    FinitePositionPlan,
    FinitePositionReferenceLifecycle,
    PositionLifecycleCommand,
    PositionLifecyclePhase,
    VelocityLifecycleCommand,
    VelocityLifecyclePhase,
    VelocityReferenceLifecycle,
    VelocityTransitionPlan,
)
from .yaw_control import (
    GyroRelativeYawEstimator,
    ParallelYawPD,
    RelativeYawEstimate,
    WheelTorqueAllocation,
    YawPDCommand,
    allocate_longitudinal_priority,
    wrap_to_pi,
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
    "DisturbanceGateConfig",
    "DisturbanceGateOutput",
    "FixedLQR",
    "FullStateFit",
    "LQRDesign",
    "JerkLimitedLongitudinalReferenceGenerator",
    "LongitudinalReference",
    "LongitudinalReferenceGenerator",
    "NominalDiscreteFeedforward",
    "NominalFeedforwardCommand",
    "NominalTrajectoryProjection",
    "CenteredPRBS",
    "PRBSConfig",
    "ProbeTransition",
    "FilteredDisturbanceCompensator",
    "PersistentDisturbanceGate",
    "auto_probe_config_from_dict",
    "design_discrete_lqr",
    "design_from_files",
    "disturbance_rejection_config_from_dict",
    "load_config",
    "fit_full_state_ridge",
    "project_hidden_reference_and_input",
    "FinitePositionPlan",
    "FinitePositionReferenceLifecycle",
    "PositionLifecycleCommand",
    "PositionLifecyclePhase",
    "VelocityLifecycleCommand",
    "VelocityLifecyclePhase",
    "VelocityReferenceLifecycle",
    "VelocityTransitionPlan",
    "GyroRelativeYawEstimator",
    "ParallelYawPD",
    "RelativeYawEstimate",
    "WheelTorqueAllocation",
    "YawPDCommand",
    "allocate_longitudinal_priority",
    "wrap_to_pi",
]
