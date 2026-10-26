"""Locked human skin/animation and physical onboard camera scene overlay.

Actor state is owned by simulation image/physics generation. This module has no
follow/controller interface and returns no target observation. RGB is the only
visual sensing input; any renderer depth belongs to an external post-hoc sink.
"""
import copy
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from companionbot_contracts.paths import repository_root
from companionbot_contracts.spatial_geometry import load_spatial_geometry

ROOT=repository_root()
CAMERA_CONFIG=ROOT/"models/minisegway/native_runtime/config/visual_camera.json"
ASSET_LOCK=ROOT/"models/minisegway/native_runtime/character_asset.lock.json"


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_character_assets(asset_dir=None):
    cache=Path(asset_dir or ROOT/".venv/assets/visual_companion/derived").resolve()
    manifest_path=cache/"manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError("character_assets_unavailable: explicitly run tools/assets/prepare_visual_companion.py")
    manifest=json.loads(manifest_path.read_text())
    lock=Path(manifest.get("asset_lock_path",ASSET_LOCK)).resolve()
    allowed={ASSET_LOCK.resolve(),(ASSET_LOCK.parent/"gazebo_character_asset.lock.json").resolve()}
    if lock not in allowed or manifest["lock_sha256"]!=_sha(lock):
        raise ValueError("character_asset_lock_mismatch")
    for name,expected in manifest["generated_sha256"].items():
        path=(cache/name).resolve()
        if path.parent!=cache or _sha(path)!=expected:
            raise ValueError("character_derived_asset_hash_mismatch:"+name)
    return cache,manifest


def load_visual_camera(path=None):
    config=json.loads(Path(path or CAMERA_CONFIG).read_text())
    matrix=np.asarray(config["camera_extrinsic"]["rotation_body_camera"],dtype=float)
    translation=np.asarray(config["camera_extrinsic"]["translation_body_camera_m"],dtype=float)
    k=np.asarray(config["K"],dtype=float)
    fovy=float(config["vertical_fov_deg"])
    flags=config.get("render_flags",{})
    if (any(isinstance(config[k],bool) or not isinstance(config[k],int) or config[k]<1
            for k in ("width","height")) or config["width"]>4096 or config["height"]>2160
            or not isinstance(flags,dict) or set(flags)-{"shadows","reflections"}
            or any(not isinstance(v,bool) for v in flags.values())
            or ("offscreen_samples" in config and (isinstance(config["offscreen_samples"],bool)
                or not isinstance(config["offscreen_samples"],int) or not 1<=config["offscreen_samples"]<=8))):
        raise ValueError("invalid_simulation_render_profile")
    expected_focal=config["height"]/2/np.tan(np.deg2rad(fovy)/2)
    expected=np.array([[expected_focal,0,config["width"]/2],[0,expected_focal,config["height"]/2],[0,0,1]])
    if (config["schema_version"]!=1 or matrix.shape!=(3,3) or translation.shape!=(3,)
            or not np.isfinite(matrix).all() or not np.isfinite(translation).all()
            or not np.allclose(matrix.T@matrix,np.eye(3),atol=1e-10)
            or not np.isclose(np.linalg.det(matrix),1.) or not np.allclose(k,expected,rtol=0,atol=1e-10)
            or config["hardware_execution_ready"] or not 0<fovy<180):
        raise ValueError("invalid_simulation_camera_contract")
    return config


def prepare_visual_companion_model(source,destination,*,asset_dir=None,camera_config=None):
    source,destination=Path(source).resolve(),Path(destination).resolve()
    if source==destination:
        raise ValueError("visual_overlay_must_not_overwrite_source_plant")
    cache,manifest=load_character_assets(asset_dir)
    camera=load_visual_camera(camera_config)
    tree=ET.parse(source)
    root=tree.getroot()
    world=root.find("worldbody")
    chassis=world.find("body[@name='chassis']")
    if chassis is None or world.find("body[@name='companion_actor_bone_0']") is not None:
        raise ValueError("visual_overlay_requires_one_original_robot_and_no_existing_actor")
    character=ET.parse(cache/"character.xml").getroot()
    assets=root.find("asset")
    if assets is None:
        assets=ET.SubElement(root,"asset")
    for asset in character.find("asset"):
        assets.append(copy.deepcopy(asset))
    for body in character.find("worldbody"):
        world.append(copy.deepcopy(body))
    deformable=root.find("deformable")
    if deformable is None:
        deformable=ET.SubElement(root,"deformable")
    for skin in character.find("deformable"):
        deformable.append(copy.deepcopy(skin))
    geometry=load_spatial_geometry()
    position=geometry.mj_to_ros.T@camera["camera_extrinsic"]["translation_body_camera_m"]
    # MJ camera looks along -Z with +Y up; input contract is optical +Z/+Y down.
    rotation=geometry.mj_to_ros.T@np.asarray(camera["camera_extrinsic"]["rotation_body_camera"])@np.diag([1.,-1.,-1.])
    q=Rotation.from_matrix(rotation).as_quat()
    ET.SubElement(chassis,"camera",name=camera["name"],pos=" ".join(map(str,position)),
        quat=" ".join(map(str,[q[3],*q[:3]])),fovy=str(camera["vertical_fov_deg"]),
        resolution=str(camera["width"])+" "+str(camera["height"]))
    visual=root.find("visual")
    if visual is None: visual=ET.SubElement(root,"visual")
    settings=visual.find("global")
    if settings is None: settings=ET.SubElement(visual,"global")
    settings.set("offwidth",str(camera["width"]))
    settings.set("offheight",str(camera["height"]))
    if "offscreen_samples" in camera:
        quality=visual.find("quality")
        if quality is None:quality=ET.SubElement(visual,"quality")
        quality.set("offsamples",str(camera["offscreen_samples"]))
    destination.parent.mkdir(parents=True,exist_ok=True)
    tree.write(destination,encoding="unicode")
    manifest_snapshot=destination.with_suffix(".asset_manifest.json")
    manifest_snapshot.write_text(json.dumps(manifest,indent=2)+"\n")
    camera_snapshot=destination.with_suffix(".camera_config.json")
    camera_snapshot.write_text(json.dumps(camera,indent=2)+"\n")
    destination.with_suffix(".json").write_text(json.dumps(dict(source=str(source),source_sha256=_sha(source),
        asset_manifest=str(cache/"manifest.json"),asset_manifest_sha256=_sha(cache/"manifest.json"),
        asset_manifest_snapshot=str(manifest_snapshot),camera_config_snapshot=str(camera_snapshot),
        camera_config=str(Path(camera_config or CAMERA_CONFIG).resolve()),camera_config_sha256=_sha(camera_config or CAMERA_CONFIG),
        robot_plant_parameters_changed=[],new_sensors_mass_kg=0.,
        actor_simulation=("upstream textured skinned CesiumMan" if manifest.get("profile","cesium")=="cesium"
            else "upstream Gazebo walk.dae skinned human with original material colors")+" walking animation; kinematic collision capsule",
        actor_state_controller_input=False,renderer_depth_controller_input=False,hardware_execution_ready=False),indent=2)+"\n")
    return destination


class VisualCompanionActor:
    """Simulation-only mocap driver for native MuJoCo skin and contact bodies.

    The route/pose belongs to scene generation, never the controller observation.
    Caller updates before a physics step or explicit image acquisition. Native
    skinning uses the cached original animation at 60 Hz; no per-vertex Python
    skinning or inference-time rendering correction occurs.
    """
    def __init__(self,model,*,asset_dir=None):
        cache,self.manifest=load_character_assets(asset_dir)
        with np.load(cache/"walking_animation.npz") as animation:
            self.times=animation["time_s"].copy()
            self.positions=animation["positions_native_m"].copy()
            self.rotations=Rotation.from_quat(animation["quaternions_xyzw"].reshape(-1,4)).as_matrix().reshape(len(self.times),-1,3,3)
            self.duration=float(animation["duration_s"])
        self.mocap_ids=np.array([model.body("companion_actor_bone_"+str(index)).mocapid[0]
            for index in range(self.manifest["bones"])],dtype=int)
        self.collision_id=int(model.body("companion_actor_collision").mocapid[0])
        self.geometry=load_spatial_geometry()
        if (self.mocap_ids<0).any() or self.collision_id<0:
            raise ValueError("character_bones_must_be_native_mocap_bodies")

    def set_pose(self,data,animation_time_s,position_ros_m,yaw_rad=0.,*,animate=True):
        position=np.asarray(position_ros_m,dtype=float)
        if position.shape!=(3,) or not np.isfinite(position).all() or not np.isfinite([animation_time_s,yaw_rad]).all():
            raise ValueError("invalid_simulation_actor_pose")
        phase=animation_time_s%self.duration if animate else 0.
        frame=min(len(self.times)-1,int(np.searchsorted(self.times,phase,side="right")-1))
        c=self.geometry.mj_to_ros
        yaw=Rotation.from_euler("z",yaw_rad).as_matrix()
        native_rotation=c.T@yaw@c
        native_position=c.T@position
        data.mocap_pos[self.mocap_ids]=self.positions[frame]@native_rotation.T+native_position
        q=Rotation.from_matrix(native_rotation@self.rotations[frame]).as_quat()
        data.mocap_quat[self.mocap_ids]=np.column_stack((q[:,3],q[:,:3]))
        data.mocap_pos[self.collision_id]=native_position
        proxy_q=Rotation.from_matrix(c.T@yaw).as_quat()
        data.mocap_quat[self.collision_id]=[proxy_q[3],*proxy_q[:3]]


class OnboardRGBRenderer:
    """Native offscreen RGB sensing only; no target pose or depth return API."""
    def __init__(self,model,*,camera_config=None):
        import mujoco
        self.config=load_visual_camera(camera_config)
        self.renderer=mujoco.Renderer(model,height=self.config["height"],width=self.config["width"])
        self.option=mujoco.MjvOption()
        self.option.geomgroup[3]=0  # existing robot contact proxies
        self.option.geomgroup[4]=0  # existing world/actor collision proxies
        self.option.geomgroup[5]=0  # diagnostic sites
        self.option.skingroup[2]=1
        self.last_timing={}

    def render_bgr(self,data):
        import mujoco
        begin=time.perf_counter()
        self.renderer.update_scene(data,camera=self.config["name"],scene_option=self.option)
        updated=time.perf_counter()
        flags=self.config.get("render_flags",{})
        for name,key in (("shadows",mujoco.mjtRndFlag.mjRND_SHADOW),
                         ("reflections",mujoco.mjtRndFlag.mjRND_REFLECTION)):
            if name in flags:self.renderer.scene.flags[key]=bool(flags[name])
        result=self.renderer.render()[...,::-1].copy()
        end=time.perf_counter()
        self.last_timing=dict(scene_update_s=updated-begin,render_and_copy_s=end-updated)
        return result

    def close(self):
        self.renderer.close()
