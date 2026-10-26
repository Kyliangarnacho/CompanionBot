"""Real parallel onboard stereo cameras for the existing simulation scene.

Both images are separately rendered from one private integration snapshot.
This module returns RGB pixels only. It has no disparity, target identity,
renderer-depth, controller or actuator interface.
"""
import hashlib
import json
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from companionbot_contracts.paths import repository_root
from companionbot_contracts.spatial_geometry import load_spatial_geometry
from .visual_companion_scene import load_visual_camera, prepare_visual_companion_model

ROOT=repository_root()
STEREO_CAMERA_CONFIG=ROOT/'models/minisegway/native_runtime/config/stereo_camera.json'


def _sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_stereo_camera(path=None):
    path=Path(path or STEREO_CAMERA_CONFIG).resolve()
    config=load_visual_camera(path)  # Top-level fields are the left camera.
    if config.get('sensor_kind')!='stereo_rgb_simulation':
        raise ValueError('explicit_stereo_camera_profile_required')
    right=config['right_camera'];left=config['camera_extrinsic'];other=right['camera_extrinsic']
    rl,rr=np.asarray(left['rotation_body_camera']),np.asarray(other['rotation_body_camera'])
    tl,tr=np.asarray(left['translation_body_camera_m']),np.asarray(other['translation_body_camera_m'])
    if (rr.shape!=(3,3) or tr.shape!=(3,) or not np.isfinite(rr).all() or not np.isfinite(tr).all()
            or not np.allclose(rr.T@rr,np.eye(3),rtol=0,atol=1e-12)
            or not np.isclose(np.linalg.det(rr),1.,rtol=0,atol=1e-12)
            or other['provisional'] or left['provisional'] or right['name']==config['name']):
        raise ValueError('invalid_stereo_right_extrinsic')
    rotation=rr.T@rl;translation=rr.T@(tl-tr)
    baseline=float(config['baseline_m']);source=config['baseline_source']
    geometry_path=(ROOT/source['geometry_path']).resolve()
    geometry=json.loads(geometry_path.read_text())
    width=float(geometry['wheel_separation_m'])
    declared=config['right_from_left']
    if (not np.isfinite(baseline) or not 0<baseline<=width
            or _sha(geometry_path)!=source['geometry_sha256']
            or not np.isclose(np.linalg.norm(tl-tr),baseline,rtol=0,atol=1e-12)
            or max(abs(tl[1]),abs(tr[1]))>width/2
            or not np.allclose(rotation,np.eye(3),rtol=0,atol=1e-12)
            or not np.allclose(translation,[-baseline,0.,0.],rtol=0,atol=1e-12)
            or not np.allclose(declared['rotation'],rotation,rtol=0,atol=1e-12)
            or not np.allclose(declared['translation_m'],translation,rtol=0,atol=1e-12)):
        raise ValueError('stereo_baseline_parallel_geometry_mismatch')
    return config


def prepare_stereo_companion_model(source,destination,*,asset_dir=None,camera_config=None):
    """Reuse original actor/left overlay, add one massless right model camera."""
    source,destination=Path(source).resolve(),Path(destination).resolve()
    config_path=Path(camera_config or STEREO_CAMERA_CONFIG).resolve()
    config=load_stereo_camera(config_path)
    # Existing mono schema deliberately remains a valid left-camera schema.
    # Right is explicit in this new factory; no old mono profile is rewritten.
    prepare_visual_companion_model(source,destination,asset_dir=asset_dir,camera_config=config_path)
    tree=ET.parse(destination);world=tree.getroot().find('worldbody')
    chassis=world.find("body[@name='chassis']")
    if chassis.find("camera[@name='"+config['right_camera']['name']+"']") is not None:
        raise ValueError('duplicate_stereo_right_camera')
    geometry=load_spatial_geometry();mount=config['right_camera']['camera_extrinsic']
    position=geometry.mj_to_ros.T@np.asarray(mount['translation_body_camera_m'])
    rotation=geometry.mj_to_ros.T@np.asarray(mount['rotation_body_camera'])@np.diag([1.,-1.,-1.])
    q=Rotation.from_matrix(rotation).as_quat()
    ET.SubElement(chassis,'camera',name=config['right_camera']['name'],
        pos=' '.join(map(str,position)),quat=' '.join(map(str,[q[3],*q[:3]])),
        fovy=str(config['vertical_fov_deg']),resolution=f"{config['width']} {config['height']}")
    tree.write(destination,encoding='unicode')
    metadata_path=destination.with_suffix('.json');metadata=json.loads(metadata_path.read_text())
    metadata.update(image_source='real_parallel_stereo_same_integration_snapshot',
        stereo_camera_config=str(config_path),stereo_camera_config_sha256=_sha(config_path),
        left_camera=config['name'],right_camera=config['right_camera']['name'],baseline_m=config['baseline_m'],
        left_right_source_time_skew_s=0.,rendered_images='separate MuJoCo RGB renders, never copied left image')
    metadata_path.write_text(json.dumps(metadata,indent=2)+'\n')
    return destination


class StereoRGBRenderer:
    """One worker owns one GL context, two cameras and a frozen pair state.

    render_left_bgr lets the caller immediately submit the original left RGB to
    independent detector/YOLO-depth workers. render_right_bgr then uses the SAME
    private MjData state. Host read-complete timestamps are real and separate.
    """
    def __init__(self,model,*,camera_config=None):
        self.config=load_stereo_camera(camera_config)
        self._thread=threading.get_native_id();self._closed=False;self._pending=None
        self._model=model;self._spec=int(mujoco.mjtState.mjSTATE_INTEGRATION)
        self._state=np.empty(mujoco.mj_stateSize(model,self._spec),dtype=np.float64)
        self.renderer=mujoco.Renderer(model,height=self.config['height'],width=self.config['width'])
        self.option=mujoco.MjvOption();self.option.geomgroup[3:6]=0;self.option.skingroup[2]=1
        self.last_timing={}

    def _check_thread(self):
        if threading.get_native_id()!=self._thread:raise RuntimeError('stereo_gl_owner_thread_mismatch')
        if self._closed:raise RuntimeError('stereo_renderer_closed')

    def _state_key(self,data):
        mujoco.mj_getState(self._model,data,self._state,self._spec)
        return id(data),hashlib.sha256(self._state.tobytes()).digest()

    def _render(self,data,name):
        begin=time.perf_counter();self.renderer.update_scene(data,camera=name,scene_option=self.option)
        flags=self.config.get('render_flags',{})
        for label,key in (('shadows',mujoco.mjtRndFlag.mjRND_SHADOW),('reflections',mujoco.mjtRndFlag.mjRND_REFLECTION)):
            if label in flags:self.renderer.scene.flags[key]=flags[label]
        image=self.renderer.render()[...,::-1].copy();ready=time.perf_counter()
        return image,begin,ready

    def render_left_bgr(self,data):
        self._check_thread()
        if self._pending is not None:raise RuntimeError('stereo_previous_pair_incomplete')
        self._pending=self._state_key(data)
        try:image,begin,ready=self._render(data,self.config['name'])
        except BaseException:self._pending=None;raise
        self.last_timing=dict(left_render_begin_host_s=begin,left_read_complete_host_s=ready,left_render_wall_s=ready-begin)
        return image

    def render_right_bgr(self,data):
        self._check_thread()
        if self._pending is None:raise RuntimeError('stereo_right_without_left_snapshot')
        if self._state_key(data)!=self._pending:
            self._pending=None;raise RuntimeError('stereo_left_right_snapshot_changed')
        try:image,begin,ready=self._render(data,self.config['right_camera']['name'])
        finally:self._pending=None
        self.last_timing.update(right_render_begin_host_s=begin,right_read_complete_host_s=ready,
            right_render_wall_s=ready-begin,pair_wall_s=ready-self.last_timing['left_render_begin_host_s'])
        return image

    def render_bgr(self,data,*,on_left=None):
        left=self.render_left_bgr(data)
        if on_left is not None:
            try:on_left(left,self.last_timing['left_read_complete_host_s'])
            except BaseException:self._pending=None;raise
        return left,self.render_right_bgr(data)

    def close(self):
        if self._closed:return
        self._check_thread();self.renderer.close();self._closed=True;self._pending=None
