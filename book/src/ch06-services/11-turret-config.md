# Turret configuration

Eleven fields: joint limits and rates, the laser's range, and the geometry of the stereo
camera pair.

```sh
cargo run -p vrobots-examples --bin ex41_hello_turret
./target/cpp-build/ex41_hello_turret
python examples/python/ex41_hello_turret.py
```

## Turret only

`configure_turret` sends `srv/turret`, which only a turret serves. Any other robot has no
responder for it, which returns [`VrError::NoResponder`](../appendix-c-errors.md) after the
service timeout.

The service follows the
[rules every config service shares](00-intro.md#rules-every-config-service-shares): the ack
is a receipt, and nothing reads a value back. Only the fields you set are written.
Everything set here survives `reset()`, and nothing survives a simulator restart.

## The eleven fields

Angles are radians, rates are rad/s and lengths are metres, signed as in
[Aiming the turret](../ch04-commands/14-turret.md#sign-conventions).

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `max_heading_rate` | float | rad/s | pi/2 (90 deg/s) | how fast the pylon turns; `<= 0` reverts to the default |
| `max_pitch_rate` | float | rad/s | pi/3 (60 deg/s) | how fast the head tilts; `<= 0` reverts to the default |
| `heading_min` | float | rad | -pi/2 | how far the pylon turns to the left; clamped to [-pi, pi] |
| `heading_max` | float | rad | +pi/2 | how far the pylon turns to the right; clamped to [-pi, pi] |
| `pitch_min` | float | rad | -0.262 (-15 degrees) | how far the barrel goes down; clamped to [-pi/2, pi/2] |
| `pitch_max` | float | rad | +1.178 (+67.5 degrees) | how far the barrel goes up; clamped to [-pi/2, pi/2] |
| `laser_max_range` | float | m | 200 | how far the laser measures; beyond it the range reads 0, as for no hit; `<= 0` or not finite is refused and the current range is kept |
| `stereo_baseline` | float | m | 0.80 | distance between the two optical centres; clamped to [0.05, 0.94], the head being 0.94 m wide |
| `stereo_focal_length` | float | px | 600 | focal length of both cameras, `fx = fy`; clamped to [100, 5000] |
| `stereo_height` | float | m | 0.35 | height of both optical centres above the pivot; clamped to [0.25, 1.0]; not finite is refused and the current value is kept |
| `stereo_forward` | float | m | 0.40 | distance of both optical centres ahead of the pivot along the barrel, negative being behind it; clamped to [-0.79, 0.45]; not finite is refused and the current value is kept |

A minimum sent above the maximum the turret already has is swapped inside the simulator. To
move a range past its other end, send both ends in one request.

## Limits do not snap the pose

Limits that exclude the current pose do not teleport the joint. It walks back to the nearest
limit at its max rate. The motion command stays latched through the change, and the limit
only clamps it, so when you widen the limits again the turret walks back out to its old
target.

From `ex41_hello_turret`, narrowing the heading limits to ±20 degrees while a -30 degree target is latched:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex41_hello_turret.rs`:

```rust
robot.configure_turret(
    &TurretConfig::default()
        .with_heading_min(deg(-NARROW_DEG))
        .with_heading_max(deg(NARROW_DEG)),
)?;
println!(
    "\n-- srv/turret: heading limits +/-{NARROW_DEG} deg; the {AIM_HEADING_DEG} deg target \
     is clamped to them --"
);
watch(&robot)?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex41_hello_turret.cpp`:

```cpp
{
    auto config = vrsdk::turret_config();
    config.has_heading_min = true;
    config.heading_min = deg(-NARROW_DEG);
    config.has_heading_max = true;
    config.heading_max = deg(NARROW_DEG);
    robot.configure_turret(config);
}
std::printf(
    "\n-- srv/turret: heading limits +/-%g deg; the %g deg target is clamped to them --\n",
    NARROW_DEG, AIM_HEADING_DEG);
watch(robot);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex41_hello_turret.py`:

```python
robot.configure_turret(heading_min=deg(-NARROW_DEG), heading_max=deg(NARROW_DEG))
print(
    f"\n-- srv/turret: heading limits +/-{NARROW_DEG:g} deg; the {AIM_HEADING_DEG:g} deg "
    "target is clamped to them --"
)
watch(robot)
```

{{#endtab }}
{{#endtabs }}

`watch` sends nothing for a second and prints the head's pose, so whatever moves it is the
target already latched, clamped by the limits the service set:

```text
-- srv/turret: heading limits +/-20 deg; the -30 deg target is clamped to them --
   heading  -28.2 deg (+100% of max rate)  pitch  -5.0 deg (  +0%)  laser off 0.00 m
   heading  -20.0 deg (  +0% of max rate)  pitch  -5.0 deg (  +0%)  laser off 0.00 m
   heading  -20.0 deg (  +0% of max rate)  pitch  -5.0 deg (  +0%)  laser off 0.00 m
   heading  -20.0 deg (  +0% of max rate)  pitch  -5.0 deg (  +0%)  laser off 0.00 m
   heading  -20.0 deg (  +0% of max rate)  pitch  -5.0 deg (  +0%)  laser off 0.00 m
```

The heading walks from -30 to -20 degrees and stops. The program then sends the shipped ±90
degrees back, and the head walks out to -30 again without a new target being sent.

> **Gotcha.** Nothing reads the limits back, so "restore" means sending the shipped values. A
> scene or another client that had tuned them differently gets the shipped ones.

## The laser range

`laser_max_range` is the distance within which the laser reports a hit. A surface farther
away reads 0 in `laser_range`, exactly as a miss does. Shorten it to make the laser ignore a
far wall; lengthen it to range across a large scene. Reading the range is covered in
[Aiming the turret](../ch04-commands/14-turret.md#the-laser-and-reading-its-range).

## The stereo pair

The four stereo fields act on the turret's two head cameras, named `left` and `right`,
together, so the pair stays rectified.

| Field | Moves |
|---|---|
| `stereo_baseline` | the two optical centres apart, to `-baseline / 2` and `+baseline / 2` across the barrel |
| `stereo_height` | both centres up from the pivot, from the head's top face (0.25 m) to a 1 m mast |
| `stereo_forward` | both centres along the barrel, from the head's back face (0.79 m behind the pivot) to its front face (0.45 m ahead) |
| `stereo_focal_length` | the focal length of both, in pixels at the current resolution |

The focal length is fixed in pixels, so changing a camera's resolution changes its field of
view.

> **Gotcha.** The stereo pair cannot be moved through the cameras service. The turret
> re-stamps the mount pose and focal length of `left` and `right` every time its
> configuration is applied, so a pose or focal length sent with
> [`mount_camera`](../ch05-cameras/01-mount-open-unmount.md) for either name is overwritten.
> The format and the clip planes of those two cameras are still set through `mount_camera`.

Two more limits follow from the same rule. These fields cannot bring back a camera that
`unmount_camera` removed. And each frame carries the pose and focal length it was rendered
with, in its mount and intrinsics blocks (see
[Lens and mount pose](../ch05-cameras/05-lens-and-pose.md)), so read those instead of trusting the
values you sent: a clamped baseline shows up there and nowhere else.

On the Community plan only `left` streams.

## What the SDK refuses

The simulator acks `ok` whatever it does with a value, so `configure_turret` refuses what it
can tell is wrong before sending, with `VrError::InvalidArgument`:

| Refused | Why |
|---|---|
| a request with nothing set | it would change nothing |
| any value that is not finite | the simulator would refuse that field alone and still ack `ok` |
| `max_heading_rate`, `max_pitch_rate`, `stereo_baseline`, `stereo_focal_length` or `laser_max_range` that is not strictly positive | the simulator would substitute a default or refuse the field |
| `heading_min` above `heading_max`, or `pitch_min` above `pitch_max`, in the same request | the simulator would swap them |

In Python a refusal raises `vrsdk.VrError` with `code == vrsdk.err.INVALID_ARGUMENT`; in C++
it throws `vrsdk::Error` with `VRSDK_ERR_INVALID_ARGUMENT`.

**Next:** [Lidar configuration](12-lidar-config.md)

**See also:** [Aiming the turret](../ch04-commands/14-turret.md), [Turret](../ch07-robots/12-turret.md), [Mount, open and unmount](../ch05-cameras/01-mount-open-unmount.md), [Coordinate frames](04-frames-config.md)
