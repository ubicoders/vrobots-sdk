# C++ examples

Thirty-six complete programs, the same progression as `examples/python`, plus a
sim-less smoke test. Each is an ordinary `int main`:
setup, then a plain loop. **The SDK never calls your code** — no base class, no
runner, no `update()` callback.

**No flags anywhere.** Every setting is a `constexpr` at the top of the file, and
a permutation worth showing is its own file rather than a flag — which is why
ex14, ex15, ex16 and ex17 exist instead of `--save`, `--resolution`, `--camera`
and `--pose` on ex03.

The one exception is the **`sys_id` of a scene-authored robot**. Those robots are
not in any spawn catalog, their ids are handed out at scene load and keep
incrementing, so no file can hard-code one. It is a single positional argument —
**required** by ex29–ex33 and ex35, and **optional** for ex21, ex22 and ex27,
which create their own multirotor when it is left off:

```powershell
target\cpp-build\Release\ex31_globalhawk_direct.exe 15
```

| Example | What it teaches |
|---|---|
| `ex01_hello_states` | read the latest snapshot at your own rate; register a log callback |
| `ex02_hello_control` | send `SET_MR_PWM`, and read the actuator echo back off the state stream |
| `ex03_hello_image` | camera frames alongside states: `fresh()` semantics, orientation, RGBA vs BGR |
| `ex04_hello_service` | create a robot and remove it — what "explicit lifecycle" means |
| `ex05_hello_car` | drive the truck with `SET_CAR` |
| `ex06_hello_throttle` | `set_mr_throttle`: what a command the sim ignores looks like |
| `ex07_body_wrench` | `set_body_force` / `_torque` / `_ft`, and frame-tagged vectors |
| `ex08_generic_cmd` | `send_cmd` + `vrsdk_cmd_args_t`: the raw escape hatch |
| `ex09_state_paced_loop` | `wait_new_state`: one iteration per sample; a timeout is a status |
| `ex10_sensors_tour` | the whole `vrsdk_state_t` — truth vs measured vs believed |
| `ex11_topic_discovery` | `list_topics` from code; `observed` vs registered |
| `ex12_version_info` | version pins (including header-vs-library), stats, `last_error` |
| `ex13_open_camera` | attaching to `front_left`, and what a wrong triple looks like |
| `ex14_camera_save` | one frame to disk as a PPM, then exit |
| `ex15_camera_formats` | what a format and a resolution cost, and the robot-wide rule |
| `ex16_two_cameras` | `front_left` + `front_right`, independent freshness, `t_ns` skew |
| `ex17_camera_pose` | the one example that mounts: `vrsdk_camera_options_t`, pose, lens |
| `ex18_multi_robot` | two handles in one process — and their frames differ |
| `ex19_robust_loop` | survive the simulator stopping and restarting |
| `ex20_logging_tour` | diagnostics: thrown errors vs the log callback |
| `ex21_reset` | `reset()`: a bare GET, and why "home" is not "where you found it" |
| `ex22_physical_params` | change the mass mid-flight; the climb rate is the only receipt |
| `ex23_skins` | `set_skin`: the one service that ever answers "no" |
| `ex24_sensor_config` | noise models you can *see* — and the block that kills a sensor |
| `ex25_frames` | `set_frames` / `scene_frame`: three levels, most specific wins |
| `ex26_drive_config` | retune the truck's drivetrain and measure the turn radius |
| `ex27_rotor_config` | replace the rotor list — wrong length is dropped whole |
| `ex28_hello_msd` | a mass-spring-damper: the one plant you can predict |
| `ex29_hello_cartpole` | balance it — and measure the rail centre the wire omits |
| `ex30_hello_halfdrone` | two rotors, and `NO_RESPONDER` as the capability probe |
| `ex31_globalhawk_direct` | take the fixed wing's six panels off the autopilot |
| `ex32_fw_rate_controller` | your rate loop, the operator's stick, read off `z/cmd` |
| `ex33_fw_est_source` | feed the onboard loop an estimate instead of the truth |
| `ex34_camera_view` | `front_left` in an OpenCV window: RGBA to BGR, one `imshow` per frame; needs `find_package(OpenCV)` |
| `ex35_publish_estimate` | `publish_estimate`: the onboard loop flies your attitude, and your 5 deg lie with it |
| `ex36_rotations` | `vrsdk::rotations`: convert a live truck's state between frames, polar vs axial |
| `smoke_test` | no simulator needed: linkage, ABI version, defaults, error mapping, RAII |

## Scene ids

Sys ids are **allocated at scene load and keep incrementing across scene loads
in one sim session** — on a fresh boot straight into the Flatworld scene the
truck is `0` and the multirotor `1`, but never rely on that. `vrobots topic
list` (or `ex11_topic_discovery`) shows what is actually publishing.

## Build

The examples build against the prebuilt C bundle; nothing is compiled from SDK
sources. Download the bundle for your OS from the
[Releases page](https://github.com/ubicoders/vrobots-sdk/releases):

| OS | Asset |
|---|---|
| Windows x86_64 | `vrobots_sdk-cpp-<version>-windows-x86_64.zip` |
| Linux x86_64 | `vrobots_sdk-cpp-<version>-linux-x86_64.tar.gz` |

The archive has no top-level folder: `include/` holds `vrobots_sdk.h` and
`vrobots_sdk.hpp`, `lib/` holds the `vrobots_sdk_capi` library. Unpack it into a
folder of its own **beside your checkout of this repository**, named
`vrobots_sdk-cpp` or `vrobots_sdk-cpp-<anything>`:

```text
<parent>/
├── vrobots-sdk/                              this repository
│   └── examples/cpp/CMakeLists.txt
└── vrobots_sdk-cpp-<version>-<os>/           the unpacked bundle
    ├── include/
    └── lib/
```

`CMakeLists.txt` finds that folder on its own, relative to its own location, so
from the repository root:

```bash
cmake -S examples/cpp -B target/cpp-build -DCMAKE_BUILD_TYPE=Release
cmake --build target/cpp-build --config Release
ctest --test-dir target/cpp-build --build-config Release --output-on-failure
```

Then, with the sim in Play mode:

```powershell
target\cpp-build\Release\ex01_hello_states.exe    # Windows
./target/cpp-build/ex01_hello_states              # Linux
```

If the bundle lives somewhere else, or more than one matching folder sits beside
the repository, say where it is:

- `-DVROBOTS_SDK_DIR=...` — the unpacked root (the folder holding `include/` and
  `lib/`).
- `-DVROBOTS_SDK_INCLUDE_DIR=...` and `-DVROBOTS_SDK_LIBRARY=...` — the header
  folder and the library file individually (`libvrobots_sdk_capi.so` on Linux,
  the import library `vrobots_sdk_capi.dll.lib` on Windows).
- `-DVROBOTS_SDK_C_INCLUDE_DIR=...` and `-DVROBOTS_SDK_CXX_INCLUDE_DIR=...` — only
  if the two headers sit in different folders.

On Windows the DLL is copied next to each executable after linking, because
Windows has no rpath. On Linux the build rpath points at the bundle's `lib/`, so
no `LD_LIBRARY_PATH` is needed.

Adding an example is one line: drop `exNN_name.cpp` beside this README and add
`exNN_name` to `VROBOTS_EXAMPLES` in `CMakeLists.txt`.

The bundle also ships its own copy of these sources in `examples/`, one level
under the unpacked root, and the same `CMakeLists.txt` detects that layout too:
`cmake -S examples -B build` inside the unpacked bundle needs no cache variables.

## Using the SDK in your own project

Two headers and one library. `vrobots_sdk.hpp` includes `vrobots_sdk.h`, so
keep them in the same directory:

```cmake
add_executable(my_controller main.cpp)
target_include_directories(my_controller PRIVATE /path/to/include)
target_link_libraries(my_controller PRIVATE /path/to/lib/vrobots_sdk_capi.dll.lib)
```

```cpp
#include <vrobots_sdk.hpp>

int main() {
    vrsdk::check_version();          // header and library must be one release
    vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, /*sys_id=*/1);
    robot.connect();
    while (true) {
        vrsdk::State s = robot.states();
        robot.set_mr_pwm({1501, 1501, 1501, 1501});
        robot.rate(100.0);
    }
}
```

Note on output: `std::printf` to a *redirected* stdout is block-buffered, so
piping a long-running example to a file shows nothing until ~4 KB has
accumulated or the program exits. Add `setvbuf(stdout, nullptr, _IOLBF, 0)` if
you want line-at-a-time logs; on a console it already behaves.

C++17 or later. The wrapper is header-only and throws `vrsdk::Error` on
failure; `e.code()` is the SDK's stable numeric code, the same number the Rust,
Python and CLI surfaces report. Prefer the plain C API? Include
`vrobots_sdk.h` instead and check `vrsdk_err_t` return codes — the wrapper adds
nothing you cannot do by hand.

## Things worth knowing

- **Frames are RGB(A), row-major top-down.** The wire is bottom-up and the SDK
  flips while copying, but channels are the renderer's own order and are never
  swapped. The scene's cameras publish Unity's native four-channel readback, so
  OpenCV wants `cv::cvtColor(m, m, cv::COLOR_RGBA2BGR)` before `imshow`.
- **`vrsdk::Frame` owns its pixels.** They are copied out of the SDK's frame, so
  the value outlives the stream and the robot. A 720p RGBA frame is 3.7 MB —
  move it rather than copy it.
- **Robots outlive the process.** Destroying a `VirtualRobot` closes the session
  and leaves the robot running. `remove()` is the only verb that deletes one
  (spelled that way because `delete` is a keyword).
- **Every vrobot already has `front_left` and `front_right`** (720p **rgba8**),
  so the camera examples `open_camera` one of them rather than creating their
  own. `open_camera` never mutates the sim, so those examples have no cleanup
  step and cannot collide with each other or with anything else reading the same
  stream. `mount_camera` — the add-a-camera API, with its mount pose, lens and
  matching `unmount_camera` — appears in exactly one file, `ex17_camera_pose`.
  That is also the only one that must run to completion rather than being
  Ctrl-C'd, and the only one that can move the **resolution** knob, which is
  robot-wide: mounting at 360p restreams every camera on that robot under a new
  name (`front_left/360p_rgba8`), and unmounting does not put it back.
- **The robots do not agree about axes.** The truck publishes `"fru"` while the
  multirotor and the Global Hawk publish `"frd"` (all three verified live), so
  the third component is up for one and down for the others. Read
  `coord_frame_id`; `ex18_multi_robot` shows what happens when you do not.
- **Camera streams are same-host only** — they ride iceoryx2 shared memory. A
  connection to a sim on another machine gets states and commands, no images.

## What the C++ surface does not have

The examples work around three things the Rust and Python surfaces expose and
this one does not. None is a blocker; each costs a line:

- **No `cmd_name(id)`.** The ids themselves *are* here now, as
  `VRSDK_CMD_SET_CAR` and friends (plus `VRSDK_FW_*` for the fixed-wing modes),
  so `ex33_fw_est_source` passes `VRSDK_CMD_SET_ANGVEL` by name — but there is
  no id→string helper the way Rust's `cmd::name` and Python's `cmd.name` are.
  (`ex08_generic_cmd` predates the constants and still spells its numbers out as
  `constexpr`s.)
- **No topic-name builder.** Rust has `vrobots_sdk::topics::state(sys_id)`,
  Python has `vrsdk.topics(sys_id)`; `ex04_hello_service`, `ex23_skins`,
  `ex25_frames` and `ex26_drive_config` compose the keys inline.
- **No accessor for the options in effect**, so `ex07_body_wrench` names the
  default send frame rather than reading it back and `ex32_fw_rate_controller`
  sets its own `src_id` explicitly rather than reading the default back (it has
  to recognise its own traffic on the command bus). No `axes_name()` helper for
  the numeric `axis_convention` tag either — the frame *id* string is on every
  snapshot, which is what the examples print.
