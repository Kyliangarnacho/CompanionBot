# Robot spatial description

ROS body axes are x forward / y left / z up, with `base_link` at the wheel axle.
This primitive visualization describes the existing MJCF geometry without
redistributing CAD meshes. It is not a second dynamics model or actuator plugin.
Derived dimensions/provenance are checked by `tools/verification/export_spatial_geometry.py --check`.

`base_footprint` is external to the rigid URDF tree: the spatial state node owns
its dynamic translation/roll/pitch to `base_link`. robot_state_publisher owns
wheel joints and rigid sensor mounts only. Never add a fixed footprint joint.
`sim_lidar` defaults off and enables only a declared massless virtual mount;
camera mounts require explicit provisional opt-in. No real mount is calibrated.

Current frames, sources, authority, diagnostics and headless verification:
[SPATIAL_STATE](../../../docs/SPATIAL_STATE.md).
