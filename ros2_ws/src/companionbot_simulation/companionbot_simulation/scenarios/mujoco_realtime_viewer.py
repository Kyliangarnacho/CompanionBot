"""Small passive MuJoCo viewer helper shared by the current interactive demos."""

from __future__ import annotations

import math
import time

import mujoco
import mujoco.viewer


class ViewerClosed(RuntimeError):
    pass


class RealtimeTrackingViewer:
    """Passive MuJoCo viewer paced in wall time with a chassis-tracking camera."""

    def __init__(self, realtime_factor: float) -> None:
        self.realtime_factor = float(realtime_factor)
        self.context = None
        self.viewer = None
        self.start_wall_s: float | None = None
        self.last_sync_sim_s = -math.inf

    def __call__(self, sim) -> None:
        if self.viewer is None:
            self.context = mujoco.viewer.launch_passive(sim.model, sim.data)
            self.viewer = self.context.__enter__()
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.viewer.cam.trackbodyid = sim.model.body("chassis").id
            self.viewer.cam.distance = 1.6
            self.viewer.cam.azimuth = 135.0
            self.viewer.cam.elevation = -18.0
            self.start_wall_s = time.perf_counter() - (
                float(sim.data.time) / self.realtime_factor
            )
        if not self.viewer.is_running():
            raise ViewerClosed

        assert self.start_wall_s is not None
        target_wall_s = self.start_wall_s + (
            float(sim.data.time) / self.realtime_factor
        )
        remaining_s = target_wall_s - time.perf_counter()
        if remaining_s > 0.0:
            time.sleep(remaining_s)
        if float(sim.data.time) - self.last_sync_sim_s >= 1.0 / 60.0:
            self.viewer.sync()
            self.last_sync_sim_s = float(sim.data.time)

    def close(self) -> None:
        if self.context is not None:
            self.context.__exit__(None, None, None)
        self.context = None
        self.viewer = None
