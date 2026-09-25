# Python examples

Thirty-six complete programs, in book order, against the `ubicoders-vrsdk`
wheel. Each is a real program with a `main()` and a `__main__` guard — **STM32
shape**: `main()` does setup, then owns a plain loop. No base class, no runner,
no callbacks; the SDK never calls your code.

**No `argparse` anywhere.** Every setting is a module-level constant, and a
permutation worth showing is its own file rather than a flag — which is why
ex14, ex15, ex16 and ex17 exist instead of `--save`, `--resolution`, `--camera`
and `--pose` on ex03.

The one exception is the **`sys_id` of a scene-authored robot**. Those robots are
not in any spawn catalog, their ids are handed out at scene load and keep
incrementing, so no file can hard-code one; they read it from `sys.argv[1]` — one
positional argument, still no flags and still no `argparse`. It is **required** by
ex29–ex33 and ex35, and **optional** for ex21, ex22 and ex27, which create
their own multirotor when it is left off:

```bash
python examples/python/ex31_globalhawk_direct.py 15
```

| Example | What it teaches |
|---|---|
| `ex01_hello_states.py` | read the always-fresh snapshot at your own rate |
| `ex02_hello_control.py` | `SET_MR_PWM` in a loop; the actuator echo is the only receipt |
| `ex03_hello_image.py` | camera frames as numpy: freshness, orientation, RGBA vs BGR |
| `ex04_hello_service.py` | create a robot and delete it — what "explicit lifecycle" means |
| `ex05_hello_car.py` | drive the truck with `SET_CAR` |
| `ex06_hello_throttle.py` | `set_mr_throttle`: what a command the sim ignores looks like |
| `ex07_body_wrench.py` | `set_body_force` / `_torque` / `_ft`, and frame-tagged vectors |
| `ex08_generic_cmd.py` | `send_cmd(**payload)`: the raw escape hatch, landed vs ignored |
| `ex09_state_paced_loop.py` | `wait_new_state`: one iteration per sample; a timeout is a status |
| `ex10_sensors_tour.py` | the whole `State` — truth vs measured vs believed |
| `ex11_topic_discovery.py` | `list_topics` and `topics(sys_id)` from code |
| `ex12_version_info.py` | version pins, subscriber stats, `last_error` |
| `ex13_open_camera.py` | attaching to `front_left`, and what a wrong triple looks like |
| `ex14_camera_save.py` | one frame to disk as a PNG (PPM without OpenCV), then exit |
| `ex15_camera_formats.py` | what a format and a resolution cost, and the robot-wide rule |
| `ex16_two_cameras.py` | `front_left` + `front_right`, independent freshness, `t_ns` skew |
| `ex17_camera_pose.py` | the one example that mounts: pose, lens, and reading them back |
| `ex18_multi_robot.py` | two handles in one process — and their frames differ |
| `ex19_robust_loop.py` | survive the simulator stopping and restarting |
| `ex20_logging_tour.py` | diagnostics: raised errors vs the `logging` bridge |
| `ex21_reset.py` | `reset()`: a bare GET, and why "home" is not "where you found it" |
| `ex22_physical_params.py` | change the mass mid-flight; the climb rate is the only receipt |
| `ex23_skins.py` | `set_skin`: the one service that ever answers "no" |
| `ex24_sensor_config.py` | noise models you can *see* — and the block that kills a sensor |
| `ex25_frames.py` | `set_frames` / `scene_frame()`: three levels, most specific wins |
| `ex26_drive_config.py` | retune the truck's drivetrain and measure the turn radius |
| `ex27_rotor_config.py` | replace the rotor list — wrong length is dropped whole |
| `ex28_hello_msd.py` | a mass-spring-damper: the one plant you can predict |
| `ex29_hello_cartpole.py` | balance it — and measure the rail centre the wire omits |
| `ex30_hello_halfdrone.py` | two rotors, and `NO_RESPONDER` as the capability probe |
| `ex31_globalhawk_direct.py` | take the fixed wing's six panels off the autopilot |
| `ex32_fw_rate_controller.py` | your rate loop, the operator's stick, read off `z/cmd` |
| `ex33_fw_est_source.py` | feed the onboard loop an estimate instead of the truth |
| `ex34_camera_view.py` | `front_left` in an OpenCV window: RGBA to BGR, one `imshow` per frame; needs `opencv-python` |
| `ex35_publish_estimate.py` | `publish_estimate`: the onboard loop flies your attitude, and your 5 deg lie with it |
| `ex36_rotations.py` | `vrsdk.rotations`: convert a live truck's state between frames, polar vs axial |

## Running them

Install the wheel (`pip install ubicoders-vrsdk`), start the simulator, then:

```bash
python examples/python/ex01_hello_states.py
python examples/python/ex03_hello_image.py     # an OpenCV window if cv2 is installed
python examples/python/ex10_sensors_tour.py
python examples/python/ex20_logging_tour.py
```

Ctrl-C stops any of them cleanly. Check the sim is actually publishing first
with the command the wheel installs:

```bash
vrobots topic list
```

`numpy` is required by the camera examples (`frame.image`); `opencv-python` is
optional — ex03 prints metadata without it and ex14 falls back to a PPM.

## System ids in the test scene

Sys ids are **allocated at scene load and keep incrementing across scene loads**
— on a fresh boot straight into the Flatworld scene the truck is `0` and the
multirotor `1`, but treat that as a convenience, not a contract. Each example
names its id in a constant at the top; `vrobots topic list` (or `ex11`) shows
what is really there.

The service and type examples (ex23–ex26, ex28) do not use those ids at all: they
**create** their own robot and delete it on the way out, so they leave the scene
as they found it. ex29–ex33 and ex35 do the opposite — their robots are
scene-authored, so they attach to the `sys_id` you pass and never delete anything. ex21, ex22 and
ex27 do either, depending on whether you pass one — and when you do, **ex22 and
ex27 leave their configuration on that robot until the scene is reloaded**, since
neither mass nor rotor geometry can be read back to restore.

## Three things that bite people

- **Every vrobot already has `front_left` and `front_right`** (720p **rgba8**),
  so the camera examples `open_camera` one of them rather than creating their
  own. Opening mutates nothing, so those examples have no cleanup step and
  cannot collide with each other or with anything else reading the same stream.
  `mount_camera` — the add-a-camera API, with its mount pose, lens and matching
  `unmount_camera` — appears in exactly one file, `ex17_camera_pose.py`. That is
  also the only one that must run to completion rather than being Ctrl-C'd, and
  the only one that can move the **resolution** knob, which is robot-wide:
  mounting at 360p restreams every camera on that robot under a new name
  (`front_left/360p_rgba8`), and unmounting does not put it back.
- **The robots do not agree about axes.** The truck publishes `'fru'` while the
  multirotor and the Global Hawk publish `'frd'` (all three verified live), so
  the third component is up for one and down for the others. Read
  `coord_frame_id`; `ex18_multi_robot.py` shows what happens when you do not.
- **Frames are RGBA, not BGR.** Rows are already top-down and the stride is
  tight, but channel order is the renderer's, and the scene's cameras publish
  Unity's native four-channel readback. ex03 converts at the display call —
  `cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)` — which is the only place that wants
  BGR.

## The twins

`examples/cpp/` has the same programs against the same core, and the Rust
examples will follow with the wrapper crate (see `examples/rust/`). They print
the same numbers, which is the point: the bindings add sugar, not behaviour.
`ex20_logging_tour` is the one that legitimately differs — same concept, native
idiom (Python `logging` here, a C callback in C++).

Two smaller divergences in the new set, both because the C++ surface is thinner:
`ex32_fw_rate_controller` reads its own `src_id` back out of `robot.options`
here and sets it explicitly in C++, and the service examples print topic keys
from `vrsdk.topics(sys_id)` where they exist and compose them inline otherwise
(the per-service keys are not in `topics()` yet).
