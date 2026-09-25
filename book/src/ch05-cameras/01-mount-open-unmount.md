# Mount, open and unmount

Three verbs with three different effects on the simulator, only one of which changes anything.

```sh
cargo run -p vrobots-examples --bin ex13_open_camera
./target/cpp-build/ex13_open_camera
python examples/python/ex13_open_camera.py
cargo run -p vrobots-examples --bin ex17_camera_pose
./target/cpp-build/ex17_camera_pose
python examples/python/ex17_camera_pose.py
```

## Start by opening what is already there

**Every vrobot ships with `front_left` and `front_right` mounted, at 720p rgba8.** That is
the default assumption behind every camera example in this book: to read images you
`open_camera` one of those two, and the simulator is not changed in any way. Mounting is
for the case the pair cannot serve -- a camera somewhere else on the robot, pointing
somewhere else, through a different lens, or in a different format -- and among the
examples only `ex17_camera_pose` does it.

That is not a limitation of the API, it is an ordering of it: the mutating verb is the one
with a cleanup step, a name collision to avoid and a robot-wide resolution knob behind it,
and none of that is worth paying for a picture the robot is already publishing.

## The three verbs

| Verb | Mutates the simulator | Needs the camera to exist first | Can it undo itself |
|---|---|---|---|
| `mount_camera` / `mount_camera_with` | yes, `srv/cameras` | no, it creates the camera | yes, with `unmount_camera` |
| `open_camera` | no, subscribe only | yes, exactly this name, resolution and format | nothing to undo |
| `unmount_camera` | yes, `srv/cameras` | it must be one this handle mounted | it is the undo |

The signatures, from `crates/vrobots-sdk/src/robot.rs`:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
fn mount_camera(&self, name: &str, resolution: &str, format: &str) -> VrResult<CameraStream>
fn mount_camera_with(&self, name: &str, resolution: &str, format: &str, options: &CameraOptions) -> VrResult<CameraStream>
fn open_camera(&self, name: &str, resolution: &str, format: &str) -> VrResult<CameraStream>
fn unmount_camera(&self, name: &str) -> VrResult<()>
fn mounted_cameras(&self) -> Vec<CameraSpec>
```

{{#endtab }}
{{#tab name="C++" }}

`cpp/include/vrobots_sdk.hpp`:

```cpp
[[nodiscard]] CameraStream mount_camera(const std::string& name,
                                        const std::string& resolution = "720p",
                                        const std::string& format = "rgb8",
                                        const vrsdk_camera_options_t* options = nullptr)
[[nodiscard]] CameraStream open_camera(const std::string& name,
                                       const std::string& resolution = "720p",
                                       const std::string& format = "rgb8")
void unmount_camera(const std::string& name)
[[nodiscard]] std::vector<std::array<std::string, 3>> mounted_cameras() const
```

{{#endtab }}
{{#tab name="Python" }}

`crates/vrobots-sdk-py/python/vrsdk/_vrsdk.pyi`:

```python
def mount_camera(
    self,
    name: str,
    resolution: str = "720p",
    format: str = "rgb8",
    *,
    mount_position: Optional[Sequence[float]] = None,
    mount_euler_deg: Optional[Sequence[float]] = None,
    fx: Optional[float] = None,
    fy: Optional[float] = None,
    near_clip: Optional[float] = None,
    far_clip: Optional[float] = None,
) -> CameraStream: ...
def open_camera(
    self, name: str, resolution: str = "720p", format: str = "rgb8"
) -> CameraStream: ...
def unmount_camera(self, name: str) -> None: ...
def mounted_cameras(self) -> list[CameraSpec]: ...
```

{{#endtab }}
{{#endtabs }}

Four verbs in Rust, three in the bindings: `mount_camera_with` has no counterpart, because
C++ takes the options as an optional fourth argument and Python takes them as keyword-only
arguments. C++ also has no `CameraSpec` type, so `mounted_cameras` gives back
`{name, resolution, format}` as a three-element array of strings. Everything else, including
the defaults of `"720p"` and `"rgb8"`, matches across the three.

The two `mount_*` calls and `unmount_camera` reach the robot's `srv/cameras`; `open_camera`
opens a subscriber, and `mounted_cameras` never leaves the process.

## The lifecycle

A camera is either on the robot or not, and a camera that is on the robot was put there by
this handle, by another client, or by the scene. Those three cases behave differently, and
the difference is the whole page.

```mermaid
stateDiagram-v2
  state "Mounted by you" as Yours
  state "Mounted by someone else" as Theirs
  state "Theirs, and you are reading it" as Opened
  [*] --> Unmounted
  Unmounted --> Yours: mount_camera
  Yours --> Yours: mount_camera (reconfigure)
  Yours --> Unmounted: unmount_camera
  Unmounted --> Theirs: scene default or another client
  Theirs --> Theirs: unmount_camera refused
  Theirs --> Opened: open_camera
  Opened --> Theirs: stop or drop the stream
  Opened --> Opened: unmount_camera refused
```

`Opened` is a state of your subscription, not of the camera: the camera itself does not
notice that you attached, and it keeps publishing for everyone when you drop the stream.

## Mounting adds exactly one camera

`mount_camera` is an **upsert of one camera**. The request names that camera and asks the
simulator to add it, or to reconfigure it if the name is already there. Every other camera
on the robot is left exactly as it was: the scene's own cameras, another client's cameras,
ones you attached with `open_camera`. Their streams do not blip.

Mounting a name that is already mounted reconfigures it, and if the resolution or format
changes then the stream name changes with it, so the old stream ends and a new one begins.
Anything still holding the old handle is reading a service that no longer exists.

From `examples/rust/src/bin/ex17_camera_pose.rs`, the one example that mounts:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

let options = CameraOptions::default()
    .with_mount_position(MOUNT_POSITION)
    .with_mount_euler_deg(MOUNT_EULER_DEG)
    .with_focal_length(FOCAL_PX)
    .with_clip(0.2, 500.0);

// mount_camera_with CREATES the camera on the robot (srv/cameras) and
// subscribes to its iox2 stream in one call.
let cam = robot.mount_camera_with(CAMERA, RESOLUTION, FORMAT, &options)?;
println!("camera stream: {}", cam.service_name());
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex17_camera_pose.cpp`:

```cpp
vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
robot.connect();

vrsdk::CameraStream cam = robot.mount_camera(CAMERA, RESOLUTION, FORMAT, &options);
std::printf("camera stream: %s\n", cam.service_name().c_str());
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex17_camera_pose.py`:

```python
mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=SYS_ID)
mr.connect()

cam = mr.mount_camera(
    CAMERA,
    RESOLUTION,
    FORMAT,
    mount_position=MOUNT_POSITION,
    mount_euler_deg=MOUNT_EULER_DEG,
    fx=FOCAL_PX,
    fy=FOCAL_PX,
    near_clip=0.2,
    far_clip=500.0,
)
print(f"camera stream: {cam.service_name}")
```

{{#endtab }}
{{#endtabs }}

`service_name` is a method in Rust and C++ and a property in Python, and it reports the same
string in all three.

With `SYS_ID = 1`, `CAMERA = "tilt"`, `RESOLUTION = "720p"` and `FORMAT = "rgb8"`, that
prints the iceoryx2 service name, which is what `vrobots topic list` shows for the same
stream:

```text
camera stream: vrobots/1/i/cam/tilt/720p_rgb8
```

The ack from `srv/cameras` is a receipt, not a result. The confirmation that the camera
exists is the stream appearing, which is what `mount_camera` waits for before it returns.

## Opening changes nothing

`open_camera` opens the iceoryx2 subscriber and touches the simulator not at all. Two
processes can open the same stream, neither disturbs the other, and neither has to own the
camera. The price is that **the name, resolution and format must match the publisher
exactly**: on iceoryx2 those three strings are the stream identity, and there is no type
negotiation behind them.

Every vrobot ships `front_left` and `front_right` at 720p **rgba8**, which is Unity's
native readback and not `rgb8` -- a detail worth getting right, since `rgb8` is the
signature default and asking for it here is one of the two ways to earn the timeout below.
Those are the cameras every camera example reads.

From `examples/rust/src/bin/ex13_open_camera.rs`, opening with the failure spelled out:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
let cam = match robot.open_camera(CAMERA, RESOLUTION, FORMAT) {
    Ok(cam) => cam,
    Err(VrError::Timeout(detail)) => {
        // The whole point of the example: nothing is mounted under that
        // exact identity, and there is no way for the SDK to tell you which
        // of the three strings is wrong.
        eprintln!("no publisher for {CAMERA}/{RESOLUTION}_{FORMAT}: {detail}");
        eprintln!(
            "run `vrobots topic list` -- the [i] lines are the streams that \
             do exist. A camera another process mounted then unmounted is gone."
        );
        return Ok(());
    }
    Err(other) => return Err(other),
};
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex13_open_camera.cpp`:

```cpp
vrsdk::CameraStream cam;
try {
    cam = robot.open_camera(CAMERA, RESOLUTION, FORMAT);
} catch (const vrsdk::Error& e) {
    if (e.code() != VRSDK_ERR_TIMEOUT) {
        throw;
    }
    // The whole point of the example: nothing is mounted under that
    // exact identity, and there is no way for the SDK to tell you which
    // of the three strings is wrong.
    std::printf("no publisher for %s/%s_%s: %s\n", CAMERA, RESOLUTION, FORMAT, e.what());
    std::printf(
        "run `vrobots topic list` -- the [i] lines are the streams that do exist. A "
        "camera another process mounted then unmounted is gone.\n");
    return 0;
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex13_open_camera.py`:

```python
try:
    cam = mr.open_camera(CAMERA, RESOLUTION, FORMAT)
except vrsdk.VrError as e:
    if e.code != vrsdk.err.TIMEOUT:
        raise
    # The whole point of the example: nothing is mounted under that exact
    # identity, and there is no way for the SDK to tell you which of the
    # three strings is wrong.
    print(f"no publisher for {CAMERA}/{RESOLUTION}_{FORMAT}: {e.detail}")
    print(
        "run `vrobots topic list` -- the [i] lines are the streams that do "
        "exist. A camera another process mounted then unmounted is gone."
    )
    return
```

{{#endtab }}
{{#endtabs }}

The missing publisher is a timeout in every surface, so it is caught the same way it is on
`wait_new_state`: branch on the code, re-raise anything else. C++ pays one extra line for
it, because `CameraStream` has to be declared outside the `try` to outlive it.

On a running simulator it attaches and reports the stream it found:

```text
attached to vrobots/1/i/cam/front_left/720p_rgba8 (nothing in the sim changed)
spec: name=front_left resolution=720p format=rgba8 (3686400 bytes/frame)
```

## Unmounting removes what you mounted

`unmount_camera` removes exactly the name it is given and stops that stream's reader
thread. Every other camera on the robot keeps streaming. It refuses a name this handle did
not mount, locally, with `VrError::InvalidArgument`, and the message lists what this handle
did mount. That is what the end of `ex13_open_camera` demonstrates against the scene's own
`front_left`:

```text
unmount_camera refused, correctly: [2] invalid_argument: camera "front_left" was not mounted by this handle (mounted: []). unmount_camera only removes what mount_camera added -- a camera attached with open_camera belongs to whoever created it
```

`mounted_cameras()` returns the specs **this handle asked for**, in mount order. It is not
a read-back: `srv/cameras` has no get verb, and the robot may well carry cameras this
handle knows nothing about. `vrobots topic list` is the read-back.

> **Gotcha.** A program that mounts must reach its `unmount_camera` call before exiting,
> which is why `ex17_camera_pose` runs for a fixed frame count rather than looping forever.
> Ctrl-C skips the cleanup and leaves the camera mounted in a simulator that outlives your
> process; unmount it by mounting the same name again from a short program, or restart the
> simulator. The examples that only open have no such deadline.

**Next:** [Formats and resolution](02-formats-and-resolution.md)

**See also:** [Freshness](04-freshness.md), [Lens and mount pose](05-lens-and-pose.md), [The vrobots command](../ch08-tooling/01-cli.md)
