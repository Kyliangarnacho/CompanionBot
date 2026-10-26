# Stereo rendering: bounded upstream reuse experiment

This experiment changes no production renderer, control algorithm, camera
calibration, asset or freshness threshold. Its purpose is to determine whether
the same immutable integration snapshot needs two complete visualization scene
updates. No controller receives actor state or renderer depth.

The installed MuJoCo 3.2.0 headers and Python `Renderer` implementation were
checked alongside official [visualization documentation](https://mujoco.readthedocs.io/en/3.2.0/programming/visualization.html),
[scene implementation](https://github.com/google-deepmind/mujoco/blob/3.2.0/src/engine/engine_vis_visualize.c)
and [native renderer](https://github.com/google-deepmind/mujoco/blob/3.2.0/src/render/render_gl3.c).

**Borrow:** `mjv_updateCamera` explicitly supports changing the camera while
retaining geometry and lights for an unchanged simulation state.
`mjv_updateScene` additionally updates the geometric lists, lights and active
skins. Therefore a camera-only second update is a supported responsibility
boundary rather than a replacement visualization framework. Recompute lights
with official `mjv_makeLights`, preserving camera-dependent headlight data.

**Borrow:** native `mjSTEREO_SIDEBYSIDE` splits a viewport into equal eye
rectangles. A 1280×360 viewport retains two original 640×360 images and their
original aspect ratio; it does not downsample either image. Supply its two GL
cameras from official `mjv_averageCamera` applied separately to each existing
physical camera. This reproduces the camera used by each original mono render,
which internally averages its eye pair. No third central camera or guessed
eye separation is required.

**Adapt:** the diagnostic composes existing native APIs into three variants:

| Variant | Full scene updates | Draw/read calls | Earliest left image |
| --- | --- | --- | --- |
| Existing two updates | Two | Two | After left draw/read |
| One scene, camera-only second update | One, plus native camera/light update | Two | After left draw/read |
| Native side by side | One, plus native camera/light setup | One drawing both eyes, one full read | After both eyes draw/read |

Side by side uploads dynamic skin data once, but it still draws the scene for
two eyes. Its average head position also participates in transparent geometry
ordering and view-dependent checks. Consequently geometric equivalence does
not by itself prove exact RGB equality with separate renders. Its delayed left
image can also remove useful overlap with the original YOLO workers. Both
effects must be measured, not assumed beneficial.

**Reject:** using the left image twice, changing FOV/image size/physics speed,
skipping scene occluders or skin updates on later snapshots, calling GL from
another thread, and declaring end-to-end success from a rendering microbenchmark.
No change to the frozen 0.2 s acquisition-to-authority limit is part of this work.

`tools/verification/benchmark_stereo_render.py` uses one standard offscreen
MuJoCo context on one owner thread. All variants share an explicit 1280×360
offscreen buffer, original render flags, four identical integration snapshots
with physical pitch/yaw and the mature animated actor. A first comparison must
prove its baseline RGB equals actual `StereoRGBRenderer` RGB. The measured order
is A/B/C/C/B/A, 12 calls per block after four warmups per variant. Quantiles have
only 24 samples per variant and are finite diagnostic evidence. State/camera
and pixel equality, stage wall time, total process CPU, calling-thread CPU and
context switches are recorded. Input restoration/forward time is reported
separately; output pixel copies are included. This fixture omits AI, matching,
ROS and physics concurrency, so it cannot establish online freshness or Pi
performance. The output `protocol.json` is written before model rendering.

The fixed first stereo grid remains a failed accuracy/freshness experiment.
Its one-way error decomposition points to texture-selected torso samples and
background contamination, rather than a uniform metric scale error. Rendering
performance changes do not repair that measurement failure or authorize FOLLOW.
