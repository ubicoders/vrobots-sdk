# Frames, axes and units

Which vector is in which frame, what coord_frame_id decides, and the conventions that bite.

## Everything is SI

Metres, seconds, kilograms, newtons, radians, pascals, degrees Celsius. There are no
scaled integers anywhere in the state stream: this SDK publishes physical units
directly, so a gyroscope reading is rad/s rather than a count to be divided by
something.

Three places break the pattern, and each is called out where it appears:
`mount_euler_deg` on a camera and `initial_pole_angle_deg` on a cart-pole are in
degrees because the simulator's inspector shows degrees, and pulse widths are in
microseconds because that is what a PWM channel is.

The wire carries `f32`. The SDK widens every value to `f64` on the way in, which is
lossless, and narrows to `f32` on the way out, which is not. A setpoint you send and
read back through the actuator echo can differ in the seventh significant figure. It
is never a control problem and it is occasionally an equality-comparison problem.

## Axes is an integer, deliberately

The declaration is the argument. From `crates/vrobots-sdk/src/state.rs`:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Axes(pub i32);
```

{{#endtab }}
{{#tab name="C++" }}

`crates/vrobots-sdk-capi/include/vrobots_sdk.h`:

```c
/**
 * The axis convention a vector is expressed in -- the enum tag beside
 * `coord_frame_id`, which is the authoritative name.
 */
typedef int32_t vrsdk_axes_t;
```

{{#endtab }}
{{#tab name="Python" }}

`crates/vrobots-sdk-py/python/vrsdk/_vrsdk.pyi`:

```python
class State:
    @property
    def axis_convention(self) -> int: ...
    @property
    def axis_convention_name(self) -> str: ...

def axes_name(value: int) -> str: ...
```

{{#endtab }}
{{#endtabs }}

The openness survives every binding. C++ gets a `typedef` over `int32_t` rather than an
`enum class`, which is the same decision for the same reason, and Python reports it as a
plain `int` with `axes_name` to turn one into a label. It is the one place in the SDK where
C++ does not wrap a C type in something stronger.

That is a type rather than a program, so there is no output to show. Four constants
are defined on it:

| Constant | C++ | Value | Meaning |
|---|---|---|---|
| `Axes::UNSPECIFIED` | `VRSDK_AXES_UNSPECIFIED` | 0 | no convention declared |
| `Axes::UNITY` | `VRSDK_AXES_UNITY` | 1 | left-handed, X right, Y up, Z forward |
| `Axes::FRD` | `VRSDK_AXES_FRD` | 2 | aerospace forward-right-down |
| `Axes::CV` | `VRSDK_AXES_CV` | 3 | computer vision right-down-forward |

Python has no constants for these: compare the integer, or read
`axis_convention_name` and compare the string.

It is not a Rust enum, and the reason is that the set is open. A scene can register a
coordinate frame at runtime, and such a frame has no constant here. A Rust enum would
force every unknown value to collapse into a catch-all on decode, so a frame the SDK
has never heard of would round-trip as "unspecified" and the information would be
gone. A transparent `i32` carries whatever the simulator sent, whether or not this
build knows a name for it, which also keeps the type trivially FFI-safe for the C++
and Python bindings.

`Axes::name()` returns the registry id string for a known constant (`"unity"`,
`"frd"`, `"cv"`) and `""` for anything else, including a runtime-registered frame.

## `coord_frame_id` is the authoritative one

Every message carries both a string frame id and an `Axes` tag. **The string wins.**
It is the only thing that can name a runtime-registered frame, and `Axes` is a
convenience tag beside it for the cases where a numeric comparison is easier.

`"fru"` is a registered id with no `Axes` constant, and it is the scene default on a
fresh launch, which makes this concrete rather than theoretical: a robot can publish
a frame whose `Axes` tag is `UNSPECIFIED` and whose string is meaningful.

Frames are set at three levels, scene, robot and device, and **the most specific
wins**. `scene_frame()` reads the scene level; `srv/frames` sets the robot and device
levels. [Coordinate frames](../ch06-services/04-frames-config.md) is the whole
picture.

The robots do not agree with each other. The truck publishes `"fru"`, while the
multirotor and the Global Hawk publish `"frd"` (the Global Hawk verified live on
2026-08-09 against simulator v3.0.0), so the third component is up for one and down
for the others, and altitude on the multirotor is minus that component.
<!-- VERIFY: the frames of the robot types other than truck, multirotor and Global Hawk. No per-robot frame default exists in the SDK source, and those three are live-verified (truck fru, multirotor frd, Global Hawk frd); the rest still need a live sim. -->

> **Gotcha.** Two handles in one process is the program where this bites hardest,
> because a distance between two positions in different frames is silently wrong
> rather than visibly wrong. Read `coord_frame_id` on each snapshot and convert
> before combining. `examples/rust/src/bin/ex18_multi_robot.rs` prints the tag beside
> each position and marks its own separation calculation as wrong when the two
> disagree.

## Your frame and the robot's frame are different questions

`ConnectOptions::coord_frame_id` (`"unity"` by default) names the convention *your
outgoing* vectors are written in. The robot converts an incoming vector out of that
frame into its own before acting on it, so a body force sent in Unity axes does the
right thing on a robot publishing `frd`.

Nothing converts what you read. Incoming state is in the robot's frame, tagged with
the robot's `coord_frame_id`, and that is the asymmetry to keep in mind: commands are
converted for you, states are not.

## Quaternion order is `[x, y, z, w]`

The scalar part is last. This matches the wire's `Vec4` field order, and it is the
opposite of the `[w, x, y, z]` convention that most textbooks and several
quaternion libraries use. Feeding an array in the wrong order into a rotation routine
does not error; it produces a rotation that looks almost plausible, which is the worst
possible failure mode. Check the order at every boundary where a quaternion enters or
leaves the SDK.

## The world-pose, body-twist split

Within `Kinematics`, half the fields are world frame and half are body frame:

| Field | Units | Frame |
|---|---|---|
| `lin_pos` | m | **world** |
| `quat` | n/a | **world** |
| `lin_vel` | m/s | **body** |
| `ang_vel` | rad/s | **body** |
| `lin_acc` | m/s² | **body** |
| `ang_acc` | rad/s² | **body** |

This is physics, not a configuration choice, and there is no option that changes it.
A position is only meaningful relative to some fixed origin, so pose is world. A
velocity or an acceleration is what an onboard instrument measures, and instruments
are bolted to the vehicle, so twist is body: `ang_vel` is what a gyroscope reads,
which is body rates rather than Euler angle derivatives.

The same split holds in the estimate block, which carries a `Kinematics` of the same
shape, so a comparison between `estimate.kin` and `kin` is field-by-field valid
without any frame juggling. [Kinematics](../ch03-reading-state/02-kinematics.md)
works through what that means for each field.

**Next:** [Rotation conversions](08-rotation-conversions.md)

**See also:** [Coordinate frames](../ch06-services/04-frames-config.md), [Kinematics](../ch03-reading-state/02-kinematics.md), [More than one robot](../ch08-tooling/06-multi-robot.md)
