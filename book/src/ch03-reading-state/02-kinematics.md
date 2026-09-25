# Kinematics

Every field of the pose, twist and acceleration block, the frame each one is expressed in, and the net wrench published beside it.

## One struct, two uses

`Kinematics` appears twice in a snapshot: as `State::kin`, which is simulator truth,
and as `State::estimate.kin`, which is the robot's belief. The fields, units and frames
are identical in both, which is what makes `estimate.kin - kin` a meaningful
subtraction. Everything on this page applies to both instances.

The one distinction inside the struct is the frame. Position and attitude are **world**
quantities: they answer "where is the robot in the scene". Velocity, angular velocity
and both accelerations are **body** quantities: they answer "what is the robot doing to
itself". That split is physics rather than configuration, and no service changes it.

| Field | Type | Units | Frame | Default | Notes |
|---|---|---|---|---|---|
| `lin_pos` | `[f64; 3]` | m | world | `[0.0; 3]` | origin at the scene's world origin |
| `quat` | `[f64; 4]` | | world | `[0.0; 4]` | unit quaternion, ordered `[x, y, z, w]`; body attitude relative to world |
| `lin_vel` | `[f64; 3]` | m/s | body | `[0.0; 3]` | |
| `ang_vel` | `[f64; 3]` | rad/s | body | `[0.0; 3]` | body rates, which is what a gyro measures |
| `lin_acc` | `[f64; 3]` | m/s² | body | `[0.0; 3]` | coordinate acceleration, not specific force |
| `ang_acc` | `[f64; 3]` | rad/s² | body | `[0.0; 3]` | |

The wire carries these components as `f32`. The SDK widens them to `f64`, which is
lossless, so the same struct reads naturally from Rust, C++ and Python.

## The three conventions that catch people

**Quaternion order is `[x, y, z, w]`.** It matches the wire's `Vec4` field order, not
the `[w, x, y, z]` order that several maths libraries use for their constructors.
Feeding the array into a library that expects scalar-first produces a rotation that is
plausible, continuous and wrong, so it survives casual inspection.

**`ang_vel` is not the derivative of Euler angles.** It is the body rate vector, the
quantity a rate gyro integrates. The two agree only in the trivial case of a single-axis
rotation. If your controller wants roll, pitch and yaw rates in the aerospace sense,
`ang_vel` is already that vector; if it wants the time derivatives of the Euler angles
you extracted from `quat`, you must convert, and the conversion is singular near
90 degrees of pitch.

**`lin_acc` is not what the accelerometer reads.** `kin.lin_acc` is coordinate
acceleration in body axes. The accelerometer reports specific force, which includes
gravity, so a robot sitting still on the ground publishes `lin_acc` near zero and an
accelerometer reading near 1 g. See [Sensors](03-sensors.md).

> **Gotcha.** A nested table missing from the wire decodes to that struct's `Default`,
> and `Kinematics::default()` sets `quat` to `[0.0, 0.0, 0.0, 0.0]`, which is not the
> identity rotation `[0, 0, 0, 1]`. A zero quaternion is not a unit quaternion, so
> normalising it divides by zero. Check the norm before you rely on an attitude,
> particularly when decoding recorded payloads whose producer may not have filled the
> block.

## Which frame the numbers are in

The vectors above are expressed in the convention named by the snapshot's own
`axis_convention` and `coord_frame_id`, which is the robot's frame and not yours.
Different robot types genuinely disagree: the examples report the truck publishing
`fru` and the multirotor `frd`, so the third component of `lin_pos` is up for one and
down for the other. Read `coord_frame_id` before you interpret a sign.

| Header field | Type | Default | Notes |
|---|---|---|---|
| `axis_convention` | `Axes` | `Axes::UNSPECIFIED` | transparent `i32`; `UNITY` 1, `FRD` 2, `CV` 3 |
| `coord_frame_id` | `String` | `""` | authoritative, and the only way to name a frame registered at runtime |

`Axes::name()` returns the registry id for the three built-in conventions and `""` for
anything else, so a runtime-registered frame keeps its identity in `coord_frame_id`
while `axis_convention` carries whatever tag the publisher stamped.

## Wrench: the physics engine's bottom line

`State::wrench` is truth, published beside the kinematics, and it is the net effect of
everything acting on the body during that step: rotor thrust, gravity, contacts and
drag combined.

| Field | Type | Units | Frame | Default | Notes |
|---|---|---|---|---|---|
| `force` | `[f64; 3]` | N | body | `[0.0; 3]` | net force this step |
| `torque` | `[f64; 3]` | N·m | body | `[0.0; 3]` | net torque this step |

Its use is diagnostic. Diffing the wrench against the force and torque your commanded
actuation should have produced is how you observe unmodelled forces, and it is
considerably more direct than inferring them from the acceleration.

## Reading the truth block

The tour example prints the whole of `kin` and the wrench together, labelling each
line with its frame.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex10_sensors_tour.rs`:

```rust
// -- truth ----------------------------------------------------------
let k = &s.kin;
println!("TRUTH  kinematics");
println!("  lin_pos   {} m       (world)", v3(k.lin_pos));
println!(
    "  quat      [{:+.3},{:+.3},{:+.3},{:+.3}] (world, xyzw)",
    k.quat[0], k.quat[1], k.quat[2], k.quat[3]
);
println!("  lin_vel   {} m/s     (body)", v3(k.lin_vel));
println!(
    "  ang_vel   {} rad/s   (body -- what a gyro measures)",
    v3(k.ang_vel)
);
println!("  lin_acc   {} m/s^2   (body)", v3(k.lin_acc));
println!("  ang_acc   {} rad/s^2 (body)", v3(k.ang_acc));
println!(
    "  wrench    F={} N  T={} N.m",
    v3(s.wrench.force),
    v3(s.wrench.torque)
);
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex10_sensors_tour.cpp`:

```cpp
// -- truth --------------------------------------------------------
std::printf("TRUTH  kinematics\n");
v3("lin_pos", r.kin.lin_pos, "m", "(world)");
std::printf("  %-9s [%+.3f,%+.3f,%+.3f,%+.3f] (world, xyzw)\n", "quat", r.kin.quat[0],
            r.kin.quat[1], r.kin.quat[2], r.kin.quat[3]);
v3("lin_vel", r.kin.lin_vel, "m/s", "(body)");
v3("ang_vel", r.kin.ang_vel, "rad/s", "(body -- what a gyro measures)");
v3("lin_acc", r.kin.lin_acc, "m/s^2", "(body)");
v3("ang_acc", r.kin.ang_acc, "rad/s^2", "(body)");
v3("force", r.wrench.force, "N", "(total on the body)");
v3("torque", r.wrench.torque, "N.m", "");
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex10_sensors_tour.py`:

```python
# -- truth ----------------------------------------------------------
k = s.kin
print("TRUTH  kinematics")
print(f"  lin_pos   {v3(k.lin_pos)} m       (world)")
quat = ",".join(f"{c:+.3f}" for c in k.quat)
print(f"  quat      [{quat}] (world, xyzw)")
print(f"  lin_vel   {v3(k.lin_vel)} m/s     (body)")
print(f"  ang_vel   {v3(k.ang_vel)} rad/s   (body -- what a gyro measures)")
print(f"  lin_acc   {v3(k.lin_acc)} m/s^2   (body)")
print(f"  ang_acc   {v3(k.ang_acc)} rad/s^2 (body)")
print(f"  wrench    F={v3(s.wrench.force)} N  T={v3(s.wrench.torque)} N.m")
```

{{#endtab }}
{{#endtabs }}

Rust and Python read `quat` as a four-element sequence; C++ indexes the C array. Every field
name, frame and unit is the same in all three, and so is the rule that `lin_pos` and `quat`
are world while the four rate and acceleration vectors are body.

For a stationary robot the twist and acceleration rows sit at zero and the attitude
row shows the identity rotation with its scalar part last:

<!-- VERIFY: printout reconstructed from the format strings; the values are illustrative, not captured from a run. -->

```text
TRUTH  kinematics
  lin_pos   (  +0.031,  +0.852,  -1.204) m       (world)
  quat      [+0.000,+0.000,+0.000,+1.000] (world, xyzw)
  lin_vel   (  +0.000,  +0.000,  +0.000) m/s     (body)
  ang_vel   (  +0.000,  +0.000,  +0.000) rad/s   (body -- what a gyro measures)
  lin_acc   (  +0.000,  +0.000,  +0.000) m/s^2   (body)
  ang_acc   (  +0.000,  +0.000,  +0.000) rad/s^2 (body)
  wrench    F=(  +0.000,  +0.000,  +0.000) N  T=(  +0.000,  +0.000,  +0.000) N.m
```

**Next:** [Sensors](03-sensors.md)

**See also:** [Frames, axes and units](../ch02-concepts/07-frames-and-units.md), [Mass and inertia](../ch06-services/02-physical-params.md), [A tour of the whole snapshot](09-sensors-tour.md)
