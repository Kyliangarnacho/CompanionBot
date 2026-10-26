"""Explicit locked-source actor -> native MuJoCo skin/animation cache.

The upstream model, texture and walking animation are reused. pygltflib parses
resources, SciPy supplies quaternion SLERP, and MuJoCo supplies skinning. This
is a narrow asset adapter, not a new renderer or general animation framework.
"""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import struct
import subprocess
from urllib.request import urlopen
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from pygltflib import GLTF2
from scipy.spatial.transform import Rotation, Slerp

from companionbot_contracts.paths import repository_root
from companionbot_contracts.spatial_geometry import load_spatial_geometry

ROOT = repository_root()
LOCK = ROOT/"models/minisegway/native_runtime/character_asset.lock.json"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def convert_gazebo(cache,upstream):
    """Borrow installed Assimp for COLLADA skin/animation conversion offline."""
    tool_dir=cache/"tools";tool_dir.mkdir(parents=True,exist_ok=True)
    intermediate=cache/"intermediate";intermediate.mkdir(parents=True,exist_ok=True)
    source=ROOT/"tools/assets/convert_character_gltf.cpp"
    binary=tool_dir/"convert_character_gltf"
    subprocess.run(["g++","-std=c++17","-O2","-Wall","-Wextra","-Werror",str(source),"-lassimp","-o",str(binary)],check=True)
    result=subprocess.run([str(binary),str(upstream/"media/models/walk.dae"),str(intermediate/"walk.gltf")],
        check=True,capture_output=True,text=True)
    metadata=json.loads(result.stdout)
    metadata.update(converter_source_sha256=sha(source),converter_binary_sha256=sha(binary),
        source_collada_sha256=sha(upstream/"media/models/walk.dae"))
    (intermediate/"conversion.json").write_text(json.dumps(metadata,indent=2)+"\n")
    return intermediate/"walk.gltf"


def acquire(cache, fetch, lock_path=LOCK):
    lock = json.loads(Path(lock_path).read_text())
    upstream = cache/"upstream"
    for row in lock["files"]:
        path = (upstream/row["path"]).resolve()
        if not path.is_relative_to(upstream.resolve()):
            raise ValueError("character_resource_path_outside_cache")
        if not path.is_file():
            if not fetch:
                raise FileNotFoundError("character asset absent; explicitly run --fetch: "+str(path))
            repository=lock["repository"].removeprefix("https://github.com/")
            url = "https://raw.githubusercontent.com/"+repository+"/"+lock["commit"]+"/"+row["path"]
            payload = urlopen(url, timeout=60).read()
            if hashlib.sha256(payload).hexdigest() != row["sha256"]:
                raise ValueError("downloaded_character_source_hash_mismatch:"+row["path"])
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(payload)
        if sha(path) != row["sha256"]:
            raise ValueError("character_source_hash_mismatch:"+row["path"])
    return upstream


def accessor(gltf, index):
    item = gltf.accessors[index]
    if item.sparse is not None:
        raise ValueError("fixed_character_does_not_use_sparse_accessors")
    view = gltf.bufferViews[item.bufferView]
    dtype = np.dtype({5120:"i1",5121:"u1",5122:"<i2",5123:"<u2",5125:"<u4",5126:"<f4"}[item.componentType])
    width = {"SCALAR":1,"VEC2":2,"VEC3":3,"VEC4":4,"MAT4":16}[item.type]
    payload = gltf.get_data_from_buffer_uri(gltf.buffers[view.buffer].uri)
    result = np.ndarray((item.count,width), dtype=dtype, buffer=payload,
        offset=(view.byteOffset or 0)+(item.byteOffset or 0),
        strides=(view.byteStride or dtype.itemsize*width,dtype.itemsize)).copy()
    if item.normalized:
        raise ValueError("fixed_character_does_not_use_normalized_accessors")
    return result


def local_transform(node):
    if node.matrix:
        return np.asarray(node.matrix,dtype=float).reshape(4,4).T.copy()
    matrix = np.eye(4)
    matrix[:3,3] = node.translation or [0.,0.,0.]
    matrix[:3,:3] = Rotation.from_quat(node.rotation or [0.,0.,0.,1.]).as_matrix()@np.diag(node.scale or [1.,1.,1.])
    return matrix


def globals_for(gltf, local):
    result = [None]*len(gltf.nodes)
    def visit(index, parent):
        result[index] = parent@local[index]
        for child in gltf.nodes[index].children or []:
            visit(child,result[index])
    for root in gltf.scenes[gltf.scene or 0].nodes:
        visit(root,np.eye(4))
    if any(value is None for value in result):
        raise ValueError("unexpected_unreachable_character_node")
    return result


def prepare(cache, fetch=False, *, profile="cesium"):
    cache = Path(cache).resolve()
    if profile not in ("cesium","gazebo"):
        raise ValueError("unsupported_locked_character_profile")
    previous=cache/"derived/manifest.json"
    if previous.is_file() and json.loads(previous.read_text()).get("profile","cesium")!=profile:
        raise ValueError("character_cache_belongs_to_different_profile_choose_distinct_cache")
    lock_path=LOCK if profile=="cesium" else ROOT/"models/minisegway/native_runtime/gazebo_character_asset.lock.json"
    upstream = acquire(cache,fetch,lock_path)
    source = (upstream/"Models/CesiumMan/glTF/CesiumMan.gltf" if profile=="cesium"
        else convert_gazebo(cache,upstream))
    gltf = GLTF2.load(source)
    if len(gltf.skins)!=1 or len(gltf.meshes)!=1 or len(gltf.animations)!=1:
        raise ValueError("unexpected_fixed_character_structure")
    primitives=gltf.meshes[0].primitives
    if len(primitives)!=(1 if profile=="cesium" else 6) or any(p.mode!=4 for p in primitives):
        raise ValueError("unexpected_locked_character_primitives")
    primitive=primitives[0]
    vertices = accessor(gltf,primitive.attributes.POSITION).astype(float)
    faces = accessor(gltf,primitive.indices).reshape(-1,3).astype(np.int32)
    joints = accessor(gltf,primitive.attributes.JOINTS_0).astype(np.int32)
    weights = accessor(gltf,primitive.attributes.WEIGHTS_0).astype(float)
    uv = (accessor(gltf,primitive.attributes.TEXCOORD_0).astype(float)
          if primitive.attributes.TEXCOORD_0 is not None else np.empty((0,2)))
    skin = gltf.skins[0]
    inverse_bind = accessor(gltf,skin.inverseBindMatrices).reshape(-1,4,4).transpose(0,2,1)
    if not np.isfinite(weights).all() or (weights<0).any() or not np.allclose(weights.sum(axis=1),1.,atol=1e-6):
        raise ValueError("invalid_upstream_skin_weights")
    if joints.min()<0 or joints.max()>=len(skin.joints):
        raise ValueError("upstream_skin_joint_index")
    local = [local_transform(node) for node in gltf.nodes]
    global_bind_nodes = globals_for(gltf,local)
    mesh_node = next(index for index,node in enumerate(gltf.nodes) if node.mesh==0)
    mesh_global = global_bind_nodes[mesh_node]
    # Cesium walks along glTF +Z. Gazebo's documented walk.dae has forward
    # root motion along source +X, retained by Assimp after its Z-up→Y-up root.
    gltf_to_ros = (np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]]) if profile=="cesium"
        else np.array([[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]]))
    geometry = load_spatial_geometry()
    conversion = geometry.mj_to_ros.T@gltf_to_ros
    all_vertices=np.concatenate([accessor(gltf,p.attributes.POSITION).astype(float) for p in primitives])
    world_vertices = all_vertices@mesh_global[:3,:3].T+mesh_global[:3,3]
    ros_vertices = world_vertices@gltf_to_ros.T
    source_height = float(np.ptp(ros_vertices[:,2]))
    # Explicit new simulation actor size, never a robot plant parameter.
    scale = 1.70/source_height
    native_vertices = ((vertices@mesh_global[:3,:3].T+mesh_global[:3,3])@conversion.T)*scale
    bind = np.asarray([mesh_global@np.linalg.inv(value) for value in inverse_bind])
    bind_positions = bind[:,:3,3]@conversion.T*scale
    bind_rotations = Rotation.from_matrix(conversion@bind[:,:3,:3]).as_quat()
    derived = cache/"derived"
    derived.mkdir(parents=True,exist_ok=True)
    skin_paths=[]
    total_vertices=total_faces=0
    for primitive_index,primitive in enumerate(primitives):
        vertices=accessor(gltf,primitive.attributes.POSITION).astype(float)
        faces=accessor(gltf,primitive.indices).reshape(-1,3).astype(np.int32)
        joints=accessor(gltf,primitive.attributes.JOINTS_0).astype(np.int32)
        weights=accessor(gltf,primitive.attributes.WEIGHTS_0).astype(float)
        uv=(accessor(gltf,primitive.attributes.TEXCOORD_0).astype(float)
            if primitive.attributes.TEXCOORD_0 is not None else np.empty((0,2)))
        if (not np.isfinite(weights).all() or (weights<0).any()
                or not np.allclose(weights.sum(axis=1),1.,atol=1e-6)
                or joints.min()<0 or joints.max()>=len(skin.joints)):
            raise ValueError("invalid_upstream_primitive_skin_weights")
        native_vertices=((vertices@mesh_global[:3,:3].T+mesh_global[:3,3])@conversion.T)*scale
        skin_path=derived/("cesium_man.skn" if profile=="cesium" else "gazebo_walk_"+str(primitive_index)+".skn")
        used_bones=[bone for bone in range(len(skin.joints)) if np.any((joints==bone)&(weights>0))]
        skin_paths.append(skin_path)
        total_vertices+=len(vertices);total_faces+=len(faces)
        with skin_path.open("wb") as out:
            out.write(struct.pack("<4i",len(vertices),len(uv),len(faces),len(used_bones)))
            out.write(np.asarray(native_vertices,dtype="<f4").tobytes())
            # glTF texture v=0 is image top. MuJoCo/OpenGL skin v=0 is bottom.
            out.write(np.asarray(np.column_stack((uv[:,0],1.-uv[:,1])),dtype="<f4").tobytes())
            out.write(np.asarray(faces,dtype="<i4").tobytes())
            for bone in used_bones:
                name=("companion_actor_bone_"+str(bone)).encode()
                out.write(name+b"\0"*(40-len(name)))
                out.write(np.asarray(bind_positions[bone],dtype="<f4").tobytes())
                q=bind_rotations[bone]
                out.write(np.asarray([q[3],q[0],q[1],q[2]],dtype="<f4").tobytes())
                influence=np.sum(np.where(joints==bone,weights,0.),axis=1)
                indices=np.flatnonzero(influence>0).astype("<i4")
                out.write(struct.pack("<i",len(indices)))
                out.write(indices.tobytes())
                out.write(influence[indices].astype("<f4").tobytes())
    # MuJoCo 3.2 accepts PNG skin textures, not the upstream JPEG. Decode once
    # offline without changing texture content; keep original bytes immutable.
    texture=derived/"cesium_man.png"
    if profile=="cesium":
        with Image.open(source.parent/gltf.images[0].uri) as image:
            image.convert("RGB").save(texture)
    animation=gltf.animations[0]
    channels=[]
    duration=0.
    max_scale_deviation=0.
    for channel in animation.channels:
        sampler=animation.samplers[channel.sampler]
        if sampler.interpolation!="LINEAR":
            raise ValueError("fixed_character_animation_expected_LINEAR")
        times=accessor(gltf,sampler.input).ravel().astype(float)
        values=accessor(gltf,sampler.output).astype(float)
        if not (np.diff(times)>0).all():
            raise ValueError("invalid_upstream_animation_times")
        duration=max(duration,float(times[-1]))
        if channel.target.path=="scale":
            max_scale_deviation=max(max_scale_deviation,float(np.max(np.abs(values-1.))))
            if max_scale_deviation>1e-5:
                raise ValueError("nonrigid_animation_scale_cannot_be_represented_by_MuJoCo_bone")
        channels.append((channel.target.node,channel.target.path,times,values,
            Slerp(times,Rotation.from_quat(values)) if channel.target.path=="rotation" else None))
    rate=60.
    sample_times=np.linspace(0,duration,round(duration*rate)+1)
    positions,quaternions=[],[]
    for t in sample_times:
        translations=[np.asarray(node.translation or [0.,0.,0.]) for node in gltf.nodes]
        rotations=[np.asarray(node.rotation or [0.,0.,0.,1.]) for node in gltf.nodes]
        scales=[np.asarray(node.scale or [1.,1.,1.]) for node in gltf.nodes]
        for index,path,times,values,slerp in channels:
            source_t=np.clip(t,times[0],times[-1])
            if path=="rotation": rotations[index]=slerp(source_t).as_quat()
            elif path=="translation": translations[index]=np.array([np.interp(source_t,times,values[:,i]) for i in range(3)])
            elif path=="scale": scales[index]=np.array([np.interp(source_t,times,values[:,i]) for i in range(3)])
            else: raise ValueError("unsupported_fixed_character_animation_channel:"+path)
        sampled=[]
        for index,node in enumerate(gltf.nodes):
            if node.matrix: sampled.append(local[index]); continue
            transform=np.eye(4)
            transform[:3,3]=translations[index]
            transform[:3,:3]=Rotation.from_quat(rotations[index]).as_matrix()@np.diag(scales[index])
            sampled.append(transform)
        global_animation=globals_for(gltf,sampled)
        bones=np.asarray([global_animation[index] for index in skin.joints])
        native_positions=bones[:,:3,3]@conversion.T*scale
        if profile=="gazebo":
            hips=next(i for i,node in enumerate(gltf.nodes) if node.name=="Hips")
            root_position=global_animation[hips][:3,3]@conversion.T*scale
            native_positions[:,:2]-=root_position[:2]
        positions.append(native_positions)
        quaternions.append(Rotation.from_matrix(conversion@bones[:,:3,:3]).as_quat())
    np.savez_compressed(derived/"walking_animation.npz",time_s=sample_times,
        positions_native_m=np.asarray(positions),quaternions_xyzw=np.asarray(quaternions),duration_s=duration)
    # All mocap bones are world children. Native MuJoCo performs skin blending.
    xml=ET.Element("mujoco")
    assets=ET.SubElement(xml,"asset")
    if profile=="cesium":
        ET.SubElement(assets,"texture",name="companion_actor_texture",type="2d",file=str(texture))
        ET.SubElement(assets,"material",name="companion_actor_material",texture="companion_actor_texture",rgba="1 1 1 1",specular="0",shininess="0")
    else:
        for index,material in enumerate(gltf.materials):
            color=material.pbrMetallicRoughness.baseColorFactor
            ET.SubElement(assets,"material",name="companion_actor_material_"+str(index),rgba=" ".join(map(str,color)),specular="0",shininess="0")
    world=ET.SubElement(xml,"worldbody")
    for bone,position in enumerate(bind_positions):
        q=bind_rotations[bone]
        ET.SubElement(world,"body",name="companion_actor_bone_"+str(bone),mocap="true",
            pos=" ".join(map(str,position)),quat=" ".join(map(str,[q[3],*q[:3]])))
    proxy=ET.SubElement(world,"body",name="companion_actor_collision",mocap="true")
    ET.SubElement(proxy,"geom",name="companion_actor_contact_proxy",type="capsule",fromto="0 0 0.20 0 0 1.50",size="0.20",group="4",rgba="0.6 0.2 0.1 1",contype="1",conaffinity="1")
    deformable=ET.SubElement(xml,"deformable")
    for index,path in enumerate(skin_paths):
        material="companion_actor_material" if profile=="cesium" else "companion_actor_material_"+str(primitives[index].material)
        rgba=([1.,1.,1.,1.] if profile=="cesium" else gltf.materials[primitives[index].material].pbrMetallicRoughness.baseColorFactor)
        # Native skin rgba is the solid color; material supplies its texture.
        # Preserve the upstream Gazebo clothing/skin colors explicitly.
        ET.SubElement(deformable,"skin",name="companion_actor_skin_"+str(index),file=str(path),material=material,group="2",rgba=" ".join(map(str,rgba)))
    ET.ElementTree(xml).write(derived/"character.xml",encoding="unicode")
    files=[p.name for p in skin_paths]+(["cesium_man.png"] if profile=="cesium" else [])+["walking_animation.npz","character.xml"]
    manifest=dict(schema_version=1,profile=profile,asset_lock_path=str(lock_path),lock_sha256=sha(lock_path),converter_sha256=sha(__file__),
        upstream_commit=json.loads(lock_path.read_text())["commit"],libraries={name:importlib.metadata.version(name) for name in ("pygltflib","numpy","scipy")},
        generated_sha256={name:sha(derived/name) for name in files},vertices=total_vertices,faces=total_faces,bones=len(skin.joints),
        source_height_m=source_height,actor_height_m=1.70,uniform_scale=scale,
        walking_duration_s=duration,walking_sample_rate_hz=rate,animation_source="upstream walking channels; offline SciPy SLERP/resampling", 
        horizontal_root_motion="upstream in-place" if profile=="cesium" else "Gazebo Hips horizontal root translation extracted; explicit simulation route owns displacement, as Gazebo interpolate_x separates trajectory",
        gltf_to_ros_rotation=gltf_to_ros.tolist(),
        intermediate_sha256={str(source.relative_to(cache)):sha(source),**{str((source.parent/buffer.uri).relative_to(cache)):sha(source.parent/buffer.uri) for buffer in gltf.buffers}},
        source_scale_roundoff_max=max_scale_deviation,skin_backend="native MuJoCo linear blend skinning", 
        physics_proxy="explicit kinematic conservative torso/leg capsule radius0.20m z0.20..1.50m; not skin collision or a biomechanical human model", 
        robot_plant_parameters_changed=[],hardware_execution_ready=False)
    (derived/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache-dir",type=Path,default=None)
    parser.add_argument("--fetch",action="store_true",help="explicitly acquire absent locked upstream resources")
    parser.add_argument("--profile",choices=("cesium","gazebo"),default="cesium")
    args=parser.parse_args()
    cache=args.cache_dir or ROOT/(".venv/assets/visual_companion" if args.profile=="cesium" else ".venv/assets/visual_companion_gazebo")
    print(json.dumps(prepare(cache,args.fetch,profile=args.profile),indent=2))


if __name__=="__main__": main()
