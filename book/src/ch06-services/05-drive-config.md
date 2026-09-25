# The truck drivetrain

Steering limits, motor torque, brake torque and the pulse band, all clamped silently.

```sh
cargo run -p vrobots-examples --bin ex26_drive_config
./target/cpp-build/ex26_drive_config
python examples/python/ex26_drive_config.py
```

## Truck only

`srv/drive` is the truck's own service. Ask any other robot for it and the query finds no
responder: `configure_drive` returns [`VrError::NoResponder`](../appendix-c-errors.md) after
`service_timeout`. That is a capability probe rather than a fault, and it is
indistinguishable from a simulator that is not running, so confirm with `vrobots topic list`
before concluding anything from it.

Every value is read live by the steering servo, the drive motor and the dynamics, so a change
bites from the next physics step with no rebuild and no dropout.

## `DriveConfig`

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `drive_mode` | `Option<u32>` | | `None`, untouched | 2 rear axle, 4 all wheels; **anything else is ignored by the simulator**, so the SDK refuses it first |
| `max_steer_deg` | `Option<f64>` | deg | `None`, untouched | wheel angle at full stick; **hard-clamped to 0 to 60** |
| `steer_rate_dps` | `Option<f64>` | deg/s | `None`, untouched | servo sweep rate; `0` is an ideal, instantaneous servo |
| `max_motor_torque_nm` | `Option<f64>` | N·m | `None`, untouched | peak torque **per driven wheel** |
| `no_load_wheel_rpm` | `Option<f64>` | rpm | `None`, untouched | wheel speed at full throttle with no load, so this is what sets top speed; **`<= 0` becomes 200** |
| `idle_brake_torque_nm` | `Option<f64>` | N·m | `None`, untouched | per wheel, while the throttle sits in the deadband |
| `max_brake_torque_nm` | `Option<f64>` | N·m | `None`, untouched | per wheel, at a full brake command |
| `pwm_band` | `Option<PwmBand>` | | `None`, untouched | all four numbers move as one group |

`configure_drive` refuses an empty config, a `drive_mode` that is not 2 or 4, and any
non-finite value. Nothing else is checked, because nothing else can be: the clamps in the
Notes column happen inside the simulator and never reach the ack.

## `PwmBand`

| Field | Type | Units | Factory value | Notes |
|---|---|---|---|---|
| `min_us` | `u32` | µs | 1100 | full reverse, full left |
| `neutral_us` | `u32` | µs | 1500 | centre stick |
| `max_us` | `u32` | µs | 1900 | full forward, full right |
| `deadband_us` | `u32` | µs | | half-width of the neutral deadband; inside it the throttle is idle and the idle brake torque holds the truck |

If `min_us < neutral_us < max_us` does not hold, the simulator replaces the **whole** band with
1100 / 1500 / 1900. A partially sensible band is not something you can ask for.

> **Gotcha.** The truck's factory band is 1100 / 1500 / 1900, while `set_car` validates against
> the wider 1100 to 2000 the actuators are specified on. So 1950 is accepted by the SDK and is
> past full throttle for this truck. There is no way to move the neutral point alone.

## Measuring instead of reading back

There is nothing to read back, so `ex26` drives the same full-left circle four times and
compares the steady turn radius, `speed / yaw_rate`. A steering limit that halves must roughly
double the radius, or the request did not land.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex26_drive_config.rs`:

```rust
    // ===== run 2: half the steering =====
    robot.configure_drive(&DriveConfig::default().with_max_steer_deg(15.0))?;
    let narrow = circle(&robot, "max_steer_deg = 15")?;

    // ===== run 3: more than the simulator allows =====
    robot.configure_drive(&DriveConfig::default().with_max_steer_deg(90.0))?;
    let clamped = circle(&robot, "max_steer_deg = 90 -> clamped to 60")?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex26_drive_config.cpp`:

```cpp
// ===== run 2: half the steering =====
{
    auto config = vrsdk::drive_config();
    config.has_max_steer_deg = true;
    config.max_steer_deg = 15.0;
    robot.configure_drive(config);
}
const Circle narrow = circle(robot, "max_steer_deg = 15");

// ===== run 3: more than the simulator allows =====
{
    auto config = vrsdk::drive_config();
    config.has_max_steer_deg = true;
    config.max_steer_deg = 90.0;
    robot.configure_drive(config);
}
const Circle clamped = circle(robot, "max_steer_deg = 90 -> clamped to 60");
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex26_drive_config.py`:

```python
# ===== run 2: half the steering =====
robot.configure_drive(max_steer_deg=15.0)
narrow = circle(robot, "max_steer_deg = 15")

# ===== run 3: more than the simulator allows =====
robot.configure_drive(max_steer_deg=90.0)
clamped = circle(robot, "max_steer_deg = 90 -> clamped to 60")
```

{{#endtab }}
{{#endtabs }}

One field costs one call in Rust and Python and three lines in C++, because the C++ surface is a
plain struct: take an empty request from `vrsdk::drive_config()`, then set the value and its
`has_*` flag together. The flag is the load-bearing half. A value written without it goes out as
if the field had never been touched, and the ack looks the same either way.

Run 2 widens the circle. Run 3 asks for 90 degrees, is acked `ok`, and drives the circle of a
truck limited to 60:

```text
steady turn radius (speed / yaw rate), same command every time:
  as spawned                             r=<m>   speed=<m/s>  yaw=<rad/s>  servo=<value>
  max_steer_deg = 15                     r=<m>   speed=<m/s>  yaw=<rad/s>  servo=<value>
  max_steer_deg = 90 -> clamped to 60    r=<m>   speed=<m/s>  yaw=<rad/s>  servo=<value>
  drive_mode = 2, 40 N.m, 30 deg         r=<m>   speed=<m/s>  yaw=<rad/s>  servo=<value>
```

The 90 that came back as 60 is indistinguishable from a 60 that was asked for. The circle is
the only witness.

## Setting the whole drivetrain at once

Run 4 fills in every field, including the factory band restated explicitly.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    robot.configure_drive(
        &DriveConfig::default()
            .with_drive_mode(2) // rear axle only (4 = all wheels)
            .with_max_steer_deg(30.0)
            .with_steer_rate_dps(120.0) // 0 would be an ideal, instant servo
            .with_max_motor_torque_nm(40.0)
            .with_no_load_wheel_rpm(200.0)
            .with_idle_brake_torque_nm(5.0)
            .with_max_brake_torque_nm(150.0)
            // All four numbers move together, and this IS the factory band.
            .with_pwm_band(PwmBand::new(1100, 1500, 1900, 30)),
    )?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex26_drive_config.cpp`:

```cpp
// ===== run 4: rear-wheel drive, softer motor, factory band restated ====
{
    auto config = vrsdk::drive_config();
    config.has_drive_mode = true;
    config.drive_mode = 2;  // rear axle only (4 = all wheels)
    config.has_max_steer_deg = true;
    config.max_steer_deg = 30.0;
    config.has_steer_rate_dps = true;
    config.steer_rate_dps = 120.0;  // 0 would be an ideal, instant servo
    config.has_max_motor_torque_nm = true;
    config.max_motor_torque_nm = 40.0;
    config.has_no_load_wheel_rpm = true;
    config.no_load_wheel_rpm = 200.0;
    config.has_idle_brake_torque_nm = true;
    config.idle_brake_torque_nm = 5.0;
    config.has_max_brake_torque_nm = true;
    config.max_brake_torque_nm = 150.0;
    // All four numbers move together, and this IS the factory band.
    config.has_pwm_band = true;
    config.pwm_band = vrsdk_pwm_band_t{1100, 1500, 1900, 30};
    robot.configure_drive(config);
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex26_drive_config.py`:

```python
# ===== run 4: rear-wheel drive, softer motor, factory band restated =====
robot.configure_drive(
    drive_mode=2,  # rear axle only (4 = all wheels)
    max_steer_deg=30.0,
    steer_rate_dps=120.0,  # 0 would be an ideal, instant servo
    max_motor_torque_nm=40.0,
    no_load_wheel_rpm=200.0,
    idle_brake_torque_nm=5.0,
    max_brake_torque_nm=150.0,
    # All four numbers move together, and this IS the factory band.
    pwm_band=PwmBand(1100, 1500, 1900, 30),
)
```

{{#endtab }}
{{#endtabs }}

The pulse-width band is the one member that is written whole on every surface, because all four
numbers travel as a group: `PwmBand::new(...)` in Rust, `PwmBand(...)` in Python, and a braced
`vrsdk_pwm_band_t` in C++. There is no keep-current for one number inside it, so the C++ flag
`has_pwm_band` decides only whether the whole block is sent.

The truck keeps driving through the change: no dropout, no re-spawn, and one line of echo at
the start of the next circle.

## Reading the drivetrain in the state stream

| Channel | Meaning |
|---|---|
| `actuator.pwm` | the three channels you sent: steer, throttle, brake |
| `actuator.measured[0..3]` | the four wheel speeds in rad/s, FL, FR, RL, RR |
| `actuator.measured[4]` | the steering servo, which is the channel that answers `max_steer_deg` |

An undriven wheel still reports, because the road turns it. So `drive_mode` 2 versus 4 shows up
as *which* wheels lead under power, not as two silent channels.

> **Note.** There is also a selector form of this service (`?mode=2|4`) for clients that cannot
> attach a payload. The SDK can, so it does not use it.

## What the SDK refuses

```text
-- refused before anything reaches the wire --
  drive_mode = 3   [<code>] <message>
  nothing set      [<code>] <message>
```

**Next:** [Rotors and thrust curves](06-rotor-config.md)

**See also:** [Driving the truck](../ch04-commands/03-truck.md), [Truck](../ch07-robots/02-truck.md), [Actuators](../ch03-reading-state/05-actuator.md)
