# Forklift and pallet robot configuration

Every tunable number of the two warehouse vehicles, with what the simulator substitutes when
a value is out of range.

```sh
cargo run -p vrobots-examples --bin ex44_robot_properties
./target/cpp-build/ex44_robot_properties
python examples/python/ex44_robot_properties.py
```

`ex44_robot_properties` reads the forklift's properties and then retunes its lift speed
through `configure_forklift`. No example program calls `configure_palletrobot`; the call has
the same shape.

## Two services, one robot type each

| Method | Segment | Served by |
|---|---|---|
| `configure_forklift` | `srv/forklift` | the forklift |
| `configure_palletrobot` | `srv/palletrobot` | the pallet robot |

Ask any other robot and the query finds no responder, which returns
[`VrError::NoResponder`](../appendix-c-errors.md) after the service timeout. Both services
follow the [rules every config service shares](00-intro.md#rules-every-config-service-shares):
the ack is a receipt, the simulator heals an out-of-range value and still acks `ok`, and
nothing reads a value back.

Only the fields you set are written. Everything set here survives `reset()`, and nothing
survives a simulator restart. Mass and inertia are not here: they belong to
[`set_physical_params`](02-physical-params.md).

## Forklift fields

Units are SI: angles in radians, rates in rad/s, and pulse widths in microseconds.

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `max_motor_torque` | float | N·m | 1200 | peak torque per front wheel; `< 0` becomes 0 |
| `no_load_wheel_speed` | float | rad/s | 13.93 | front-wheel speed at full throttle with no load, so it sets the top speed (about 5 m/s); `<= 0` reverts to the default |
| `idle_brake_torque` | float | N·m | 800 | per wheel, while the throttle sits in the deadband; `< 0` becomes 0 |
| `max_brake_torque` | float | N·m | 2000 | per wheel, at a full brake command; `< 0` becomes 0 |
| `max_steer_angle` | float | rad | 0.785 | rear-wheel angle at full stick; clamped to [0, 1.396], which is 0 to 80 degrees |
| `steer_slew_rate` | float | rad/s | 1.047 | steering servo sweep rate; `< 0` becomes 0, and 0 is an instant servo |
| `lift_min` | float | m | -0.215 | carriage height at the bottom of the pulse band; negative is allowed |
| `lift_max` | float | m | 1.11 | carriage height at the top of the band; `lift_max <= lift_min` resets **both** to the defaults |
| `lift_speed` | float | m/s | 0.5 | `<= 0` reverts to the default; capped at 1 |
| `fork_width_min` | float | m | 0.158 | centre to centre at the bottom of the band; `< 0` resets both ends |
| `fork_width_max` | float | m | 1.334 | centre to centre at the top of the band; `max <= min` resets both ends |
| `fork_width_speed` | float | m/s | 0.1 | each fork moves at half of it; `<= 0` reverts to the default; capped at 0.5 |
| `pwm_band` | `PwmBand` | µs | 1100 / 1500 / 1900, deadband 20 | all four numbers move as one group; not `min < neutral < max` resets the whole band; a negative deadband becomes 0 |
| `initial_steer_us` | integer | µs | 1500 | the steer pulse `reset()` re-latches; 0 means the default; clamped into the band |
| `initial_throttle_us` | integer | µs | 1500 | the throttle pulse `reset()` re-latches; same rules |
| `initial_lift_us` | integer | µs | 1100 | the lift pulse `reset()` re-latches and snaps the carriage to; same rules |
| `initial_fork_width_us` | integer | µs | 1301 | the fork-width pulse `reset()` re-latches and snaps the forks to; same rules |

The drive, the steering and the mast read their settings live, so a change takes effect from
the next physics step. The four `initial_*_us` pulses wait for the next `reset()`.

> **Gotcha.** A range is judged one request at a time. To move `lift_min` above the current
> `lift_max`, send both ends in the same request: a request that leaves the range crossed
> resets it to the shipped one, and the ack still says `ok`. The fork width range behaves the
> same way.

The pulse band covers all five `SET_FORKLIFT` channels, and its deadband applies to steer
and throttle only. A band change re-maps the latched pulses on the next step. Narrowing the
band clamps the `initial_*_us` pulses into it, and widening it again does not undo that, so
send them again.

## Measuring a forklift change

There is nothing to read back, so `ex44` halves the lift speed, times the carriage over the
middle of a move, and then restores the shipped value the same way.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex44_robot_properties.rs`:

```rust
robot.configure_forklift(&ForkliftConfig::default().with_lift_speed(SLOW_LIFT_SPEED))?;
println!("acked -- a receipt. The carriage is the proof:");
let slow = lift_speed(&robot, lift_to_us, found_width_us)?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex44_robot_properties.cpp`:

```cpp
{
    auto config = vrsdk::forklift_config();
    config.has_lift_speed = true;
    config.lift_speed = SLOW_LIFT_SPEED;
    robot.configure_forklift(config);
}
std::printf("acked -- a receipt. The carriage is the proof:\n");
const double slow = lift_speed(robot, lift_to_us, found_width_us);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex44_robot_properties.py`:

```python
robot.configure_forklift(lift_speed=SLOW_LIFT_SPEED)
print("acked -- a receipt. The carriage is the proof:")
slow = lift_speed(robot, lift_to_us, found_width_us)
```

{{#endtab }}
{{#endtabs }}

Rust chains one `with_*` setter per field on `ForkliftConfig::default()`, and Python names
the field as a keyword argument. C++ takes an empty request from
`vrsdk::forklift_config()` and sets the value together with its `has_*` flag; a value
written without the flag is not sent. The run ends by comparing both measurements against
what was asked for:

```text
-- srv/forklift: lift_speed 0.25 m/s (shipped: 0.5) --
acked -- a receipt. The carriage is the proof:
   -0.215 m -> +0.430 m: 0.615 m in 2.45 s mid-move = 0.25 m/s

-- srv/forklift: lift_speed back to the shipped 0.5 m/s --
   +0.447 m -> -0.212 m: 0.610 m in 1.22 s mid-move = 0.50 m/s

measured 0.25 m/s after asking for 0.25, 0.50 m/s after asking for 0.5. The forks are back at 1100 / 1301 us, the drive centred. Scene-authored: never deleted.
```

"Restore" means sending the shipped value. A setting another client had changed comes back
as the shipped one, because there was no way to learn what it was.

## Pallet robot fields

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `pwm_band` | `PwmBand` | µs | 1100 / 1500 / 1900, deadband 20 | all six channels; not `min < neutral < max` resets the whole band; a negative deadband becomes 0 |
| `wheel_radius` | float | m | 0.054 | the virtual wheel radius `R`; `<= 0` reverts to the default |
| `half_wheelbase` | float | m | 0.35 | centre to the front and rear axles, `lx`; `<= 0` reverts to the default |
| `half_track` | float | m | 0.30 | centre to the left and right wheels, `ly`; `<= 0` reverts to the default |
| `max_wheel_speed` | float | rad/s | 28 | wheel speed at 400 µs off neutral; `<= 0` reverts to the default |
| `wheel_accel` | float | rad/s² | 60 | wheel motor acceleration limit; `< 0` becomes 0, and 0 is a wheel that jumps to its command |
| `max_traction_accel` | float | m/s² | 6 | ceiling on the body acceleration the drive applies, which also bounds the push against an obstacle; `<= 0` reverts to the default |
| `parked_hold_gain` | float | 1/s | 4 | with all four wheels stopped, the rate the body is pulled back to where it parked; `< 0` becomes 0, and 0 is no hold |
| `pad_lift_max` | float | m | 0.06 | pad height at the top of the band; `<= 0` reverts to the default; capped at 0.08 |
| `pad_lift_speed` | float | m/s | 0.03 | `< 0` becomes 0, and 0 is an instant pad |
| `pad_turn_max_rate` | float | rad/s | 1.571 | pad turn rate at full stick, 90 deg/s; `< 0` becomes 0 |

The call, with two fields set:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

Written against the `vrobots-sdk` crate:

```rust
robot.configure_palletrobot(
    &PalletRobotConfig::default()
        .with_half_wheelbase(0.40) // m
        .with_wheel_accel(30.0), // rad/s^2
)?;
```

{{#endtab }}
{{#tab name="C++" }}

Written from `include/vrobots_sdk.hpp`:

```cpp
auto config = vrsdk::palletrobot_config();
config.has_half_wheelbase = true;
config.half_wheelbase = 0.40;  // m
config.has_wheel_accel = true;
config.wheel_accel = 30.0;     // rad/s^2
robot.configure_palletrobot(config);
```

{{#endtab }}
{{#tab name="Python" }}

Written from `vrsdk/_vrsdk.pyi`:

```python
robot.configure_palletrobot(half_wheelbase=0.40, wheel_accel=30.0)
```

{{#endtab }}
{{#endtabs }}

None of the calls prints anything. The drive reads its settings live, so the change takes effect
from the next physics step, and the proof is the body's response to the same wheel command.

> **Gotcha.** The geometry is what the robot's own drive mixes against, and nothing publishes
> it. After changing `wheel_radius`, `half_wheelbase`, `half_track` or `max_wheel_speed`,
> update the constants in your own [mecanum mix](../ch04-commands/12-palletrobot.md), or the
> twist you ask for and the twist you get part company.

The latched command survives a band change and is re-mapped onto the new band on the next
step, so a latch at the old neutral can start the wheels. Send a fresh frame after changing
the band, or call `reset()`, which re-latches the rest frame from the new band. The deadband
is not capped: one wide enough to reach both ends of the band silences the wheels and the
pad turn.

## What the SDK refuses

Both methods refuse two things before anything reaches the wire, with
`VrError::InvalidArgument`: a request with nothing set, and a value that is not finite. A
value the simulator converts before storing it (an angle to degrees, a wheel speed to rpm)
is also refused when the converted figure would overflow. Everything else in the Notes
columns happens inside the simulator and never reaches the ack.

**Next:** [Arm configuration](10-arm-config.md)

**See also:** [Driving the forklift](../ch04-commands/11-forklift.md), [Driving the pallet robot](../ch04-commands/12-palletrobot.md), [Forklift](../ch07-robots/08-forklift.md), [Pallet robot](../ch07-robots/09-palletrobot.md), [Robot properties](13-properties.md)
