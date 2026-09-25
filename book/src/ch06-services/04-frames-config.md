# Coordinate frames

Three levels of frame override, where the most specific one wins.

```sh
cargo run -p vrobots-examples --bin ex25_frames
./target/cpp-build/ex25_frames
python examples/python/ex25_frames.py
```

## Frames are presentation, never physics

Changing a frame does not move the robot one millimetre differently. The numbers describing it
are permuted, and depending on the pair one of them changes sign. Registered ids are `unity`,
`frd`, `fru` and `cv`, plus anything the scene registers at runtime, which is why
`coord_frame_id` (a string) is authoritative and `axis_convention` (an enum tag) is the
convenience beside it.

## Three levels

From the `examples/rust/src/bin/ex25_frames.rs` header:

```text
device override   (srv/frames, per device)      <- most specific
robot override    (srv/frames, robot_frame_id)
robot default     (truck: fru, multirotor: frd, globalhawk: frd -- regardless of the scene)
scene frame       (scene_frame(); every launch starts at fru)
```

The three robot defaults in that row were measured live against simulator v3.0.0. The
remaining robot types are unconfirmed, and no per-robot default exists anywhere in the SDK
source, so read `State::coord_frame_id` off the snapshot rather than assuming a default
for them.
<!-- VERIFY: the "robot default" row of that ladder for the robot types other than truck, multirotor and Global Hawk. Those three are live-verified (truck fru, multirotor frd, Global Hawk frd as of 2026-08-09, sim v3.0.0); no per-robot default exists anywhere in the SDK source. -->

`srv/frames` writes the top two levels. The bottom one is read-only from the SDK.

## Reading the scene level

`scene_frame()` is a payload-less GET that reads and changes nothing. It is scene scope rather
than robot scope: the answer is the same for every robot loaded, and the method uses this
robot's session only because that is where the wire is.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    let scene = robot.scene_frame()?;
    println!(
        "scene frame: {:?} (axis_convention {}, {:?})",
        scene.coord_frame_id,
        scene.axis_convention.0,
        scene.axis_convention.name()
    );
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex25_frames.cpp`:

```cpp
const vrsdk::SceneFrame scene = robot.scene_frame();
std::printf("scene frame: \"%s\" (axis_convention %d)\n", scene.coord_frame_id.c_str(),
            scene.axis_convention);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex25_frames.py`:

```python
scene = robot.scene_frame()
print(
    f"scene frame: {scene.coord_frame_id!r} "
    f"(axis_convention {scene.axis_convention}, {scene.axis_convention_name!r})"
)
```

{{#endtab }}
{{#endtabs }}

The query is the same everywhere; what you can print of the answer is not. Rust reads the enum
tag's name through `Axes::name()` and Python through `axis_convention_name`, while in C++
`axis_convention` is a plain integer with no name beside it, so the C++ line prints the number
alone.

It returns `SceneFrame { coord_frame_id: String, axis_convention: Axes }`. A frame the scene
registered at runtime has no enum value, so `axis_convention` comes back `UNSPECIFIED` and the
string is the only thing that identifies it.

```text
scene frame: "fru" (axis_convention <n>, <name>)
```

> **Note.** Nothing persists between launches. The scene frame starts at `fru` every time the
> simulator is started, whatever the last session left it at.

## Setting the robot and device levels

`set_frames(robot_frame_id: Option<&str>, devices: &[DeviceFrame])` takes two independent
halves and writes whichever you fill in.

| Value | Meaning |
|---|---|
| `None` for `robot_frame_id` | leave the robot's level alone |
| a registered id | override that level |
| `INHERIT_FRAME` | **clear** the override, so the level below wins again |
| `""` | untouched; the simulator skips it |

`INHERIT_FRAME` is the string `"inherit"`. The distinction between it and an empty string is
the whole of the API here: one erases an override, the other declines to say anything. The SDK
refuses an empty frame id in a `DeviceFrame` entry outright, precisely because the simulator
would skip it and the ack would still say `ok`.

Device names are matched exactly, case included, and live in the `device` module:

| Constant | String |
|---|---|
| `device::ACCELEROMETER` | `accelerometer` |
| `device::GYROSCOPE` | `gyroscope` |
| `device::MAGNETOMETER` | `magnetometer` |
| `device::BAROMETER` | `barometer` |
| `device::GPS` | `gps` |
| `device::OPTICAL_FLOW` | `optical_flow` |
| `device::camera(name)` | `camera/<name>` |

> **Gotcha.** The device the frames service matches is `gps`, while the block it moves is
> called `gnss` in the state message. Use the constants rather than a literal.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex25_frames.rs`:

```rust
    robot.set_frames(
        Some("frd"),
        &[
            // Keep the gyro reading the way it was, while the robot moves to frd.
            DeviceFrame::new(device::GYROSCOPE, "fru"),
            // Clear any override this device had: fall back to the robot's level.
            DeviceFrame::new(device::GPS, INHERIT_FRAME),
            // Deliberate miss: this truck has no camera called "front". The entry
            // is skipped with a log line and an `ok` ack; the others still apply.
            DeviceFrame::new(device::camera("front"), "cv"),
        ],
    )?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex25_frames.cpp`:

```cpp
robot.set_frames("frd",
                 {
                     // Keep the gyro reading the way it was, while the
                     // robot moves to frd.
                     {vrsdk::device::GYROSCOPE, "fru"},
                     // Clear any override this device had: fall back to
                     // the robot's level.
                     {vrsdk::device::GPS, vrsdk::INHERIT_FRAME},
                     // Deliberate miss: this truck has no camera called
                     // "front". The entry is skipped with a log line and
                     // an `ok` ack; the others still apply.
                     {vrsdk::device::camera("front"), "cv"},
                 });
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex25_frames.py`:

```python
robot.set_frames(
    "frd",
    [
        # Keep the gyro reading the way it was, while the robot moves to frd.
        DeviceFrame(device.GYROSCOPE, "fru"),
        # Clear any override this device had: fall back to the robot's level.
        DeviceFrame(device.GPS, vrsdk.INHERIT_FRAME),
        # Deliberate miss: this truck has no camera called "front". The entry
        # is skipped with a log line and an `ok` ack; the others still apply.
        DeviceFrame(device.camera("front"), "cv"),
    ],
)
```

{{#endtab }}
{{#endtabs }}

The device names and `INHERIT_FRAME` are spelled the same in all three. Only the entry
differs: C++ brace-initialises each pair inline, Python constructs a `DeviceFrame`, and Rust
calls `DeviceFrame::new`. The robot half is `Some("frd")` in Rust and a plain `"frd"` in the
other two.

The robot's header frame changes on the next state sample, the gyro keeps its own, and the
missing camera entry is dropped without disturbing the other two:

```text
default    robot=<default> (...) pos=(...)  gyro frame=<default>  gnss frame=<default>
overridden robot="frd"     (frd) pos=(...)  gyro frame="fru"      gnss frame="frd"
```

Unknown ids are skipped **entry by entry**, not request by request. That is different from the
rotor list, where a single bad entry drops everything, and it is why a typo in one device name
is invisible: the other entries land, the ack says `ok`, and the only symptom is one block
still reporting in the old frame.

## Clearing an override

Passing `INHERIT_FRAME` at both levels puts everything back where it started.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    robot.set_frames(
        Some(INHERIT_FRAME),
        &[DeviceFrame::new(device::GYROSCOPE, INHERIT_FRAME)],
    )?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex25_frames.cpp`:

```cpp
robot.set_frames(vrsdk::INHERIT_FRAME,
                 {{vrsdk::device::GYROSCOPE, vrsdk::INHERIT_FRAME}});
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex25_frames.py`:

```python
robot.set_frames(
    vrsdk.INHERIT_FRAME,
    [DeviceFrame(device.GYROSCOPE, vrsdk.INHERIT_FRAME)],
)
```

{{#endtab }}
{{#endtabs }}

`INHERIT_FRAME` clears an override on every surface. What differs is how each says "leave the
robot's level alone": `None` in Rust and Python, an empty string in C++, which the binding
turns into the same absent field. C++ also offers a devices-only overload for that case.

The robot falls back to its own default, and the gyro falls back to the robot:

```text
cleared    robot=<default> (...) pos=(...)  gyro frame=<default>  gnss frame=<default>
```

If the truck's own default and the scene frame happen to be the same id, that one run cannot
tell you which level answered. The robot default outranks the scene either way.

## What the SDK refuses

Three requests never reach the wire: nothing set at all, an entry with an empty device name,
and an entry with an empty frame id. Each returns `VrError::InvalidArgument` naming what was
wrong.

```text
-- refused before anything reaches the wire --
  nothing set                      [<code>] <message>
  an entry with an empty device    [<code>] <message>
  an entry with an empty frame id  [<code>] <message>
(use INHERIT_FRAME to clear an override; "" would be skipped sim-side)
```

## Where the confirmation is

Two places, and neither is the ack: the `coord_frame_id` stamped on every subsequent state
header, and the robot's `z/frames` topic, which republishes the full definition of each frame,
basis matrix included, on change and then at 1 Hz.

**Next:** [The truck drivetrain](05-drive-config.md)

**See also:** [Frames, axes and units](../ch02-concepts/07-frames-and-units.md), [Sensor noise](03-sensor-config.md), [Appendix A: Topic reference](../appendix-a-topics.md)
