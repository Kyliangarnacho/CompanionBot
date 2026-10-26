"""Official MuJoCo integration snapshots remain render-only, independent state."""
import mujoco
import numpy as np
import pytest

from tools.verification.verify_render_snapshot import capture_state, STATE_SPEC


@pytest.fixture
def model():
    return mujoco.MjModel.from_xml_string('''<mujoco>
      <option timestep="0.001"/><worldbody>
      <body pos="0 0 1"><freejoint/><geom type="sphere" size=".1"/></body>
      <body mocap="true" pos="1 0 .5"><geom type="sphere" size=".1"/></body>
      </worldbody></mujoco>''')


def test_snapshot_state_roundtrip_includes_mocap_and_does_not_advance(model):
    source=mujoco.MjData(model);source.time=.125;source.qvel[:]=np.linspace(0,.01,model.nv)
    source.mocap_pos[:]=[1.2,.3,.8];source.qacc_warmstart[:]=.02
    source.xfrc_applied[1]=[.1,.2,.3,0,0,0]
    snapshot=capture_state(model,source);saved=snapshot.copy()
    private=mujoco.MjData(model);mujoco.mj_setState(model,private,snapshot,STATE_SPEC)
    assert np.array_equal(capture_state(model,private),snapshot)
    source.time=.5;source.qpos[0]=.6;source.mocap_pos[:]=0
    assert np.array_equal(snapshot,saved)
    mujoco.mj_forward(model,private)
    assert private.time==.125
    assert np.array_equal(private.mocap_pos,np.array([[1.2,.3,.8]]))
    assert private.qpos[0]==0 and not np.shares_memory(source.qpos,private.qpos)


def test_snapshot_shape_and_readonly_binding_are_explicit(model):
    snapshot=capture_state(model,mujoco.MjData(model));private=mujoco.MjData(model)
    with pytest.raises(TypeError):mujoco.mj_setState(model,private,snapshot[:-1],STATE_SPEC)
    snapshot.setflags(write=False)
    # Current installed3.2 binding requires a writable Eigen array even though
    # the underlying C function reads const state. Worker ownership copy is safe.
    with pytest.raises(TypeError):mujoco.mj_setState(model,private,snapshot,STATE_SPEC)
    mujoco.mj_setState(model,private,np.array(snapshot,copy=True),STATE_SPEC)
