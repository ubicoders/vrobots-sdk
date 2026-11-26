# Aiming the turret

Two motion commands share one latch, a third switches the laser, and the actuator report
carries both joint angles and the laser's range back.

```sh
cargo run -p vrobots-examples --bin ex41_hello_turret
./target/cpp-build/ex41_hello_turret
python examples/python/ex41_hello_turret.py
```

The turret is a fixed pan-tilt mount: a pylon turns (heading) and a head tilts (pitch), with
a laser along the barrel and a stereo camera pair on the head. The sandbox scene authors one
without pinning its id, so with no argument the example finds it; pass a `sys_id` to skip
the search.

## The three commands

| Method | Command | Payload | Units | What the turret does |
|---|---|---|---|---|
| `set_turret_rates(heading_rate, pitch_rate)` | `SET_TURRET_RATES` (317) | `float_arr` of 2 | rad/s | turns at that rate, clamped to each joint's max rate, until a limit |
| `set_turret_joints(heading, pitch)` | `SET_TURRET_JOINTS` (318) | `float_arr` of 2 | rad | turns to that target at the max rate, clamped to the joint limits, then holds |
| `set_turret_laser(on)` | `SET_TURRET_LASER` (319) | `int_val` 1 or 0 | | switches the laser |

The SDK refuses a value that is not finite before publishing. Everything else is clamped by
the turret. The defaults are heading within ±90 degrees at 90 deg/s and pitch from -15 to
+67.5 degrees at 60 deg/s; [`configure_turret`](../ch06-services/11-turret-config.md) moves
them.

## Sign conventions

| Quantity | Positive means | Zero means |
|---|---|---|
| heading | the pylon turned to the right, seen from above | the rest pose |
| pitch | the barrel raised | the barrel level |
| heading rate, pitch rate | turning in the positive direction above | holding |

`(0.0, 0.0)` as a target is the rest pose: the barrel level along the base's forward axis.

Joint angles carry no axes. These signs hold whatever frame your connection is in and
whatever frame the turret reports in, and **nothing is frame-converted**.

## Rate or target: the last one wins

Both motion commands latch, and they share a single latch. Each ends the other, and whichever
arrived last is in charge.

A rate is the command for closing an aiming loop yourself, sent every iteration in
proportion to the error. A target is the command for "go there". The joints move
independently, so they need not arrive together, and there is no acceleration limit: a rate
applies at once.

From `ex41_hello_turret`, the rate phase, which must end by sending a rate of zero:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex41_hello_turret.rs`:

```rust
// ===== a rate: SET_TURRET_RATES =====
println!(
    "\n-- heading rate {RATE} rad/s for {:.1} s --",
    f64::from(RATE_SAMPLES) / HZ
);
for i in 0..RATE_SAMPLES {
    robot.set_turret_rates(RATE, 0.0)?;
    robot.rate(HZ);
    if i % 10 == 9 {
        println!("   {}", report(&robot.turret_actuators()?));
    }
}
// A rate latches: without this the pylon would keep turning to its limit.
robot.set_turret_rates(0.0, 0.0)?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex41_hello_turret.cpp`:

```cpp
// ===== a rate: SET_TURRET_RATES =====
std::printf("\n-- heading rate %g rad/s for %.1f s --\n", RATE, RATE_SAMPLES / HZ);
for (int i = 0; i < RATE_SAMPLES; ++i) {
    robot.set_turret_rates(RATE, 0.0);
    robot.rate(HZ);
    if (i % 10 == 9) {
        std::printf("   %s\n", report(robot.turret_actuators()).c_str());
    }
}
// A rate latches: without this the pylon would keep turning to its limit.
robot.set_turret_rates(0.0, 0.0);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex41_hello_turret.py`:

```python
# ===== a rate: SET_TURRET_RATES =====
print(f"\n-- heading rate {RATE:g} rad/s for {RATE_SAMPLES / HZ:.1f} s --")
for i in range(RATE_SAMPLES):
    robot.set_turret_rates(RATE, 0.0)
    robot.rate(HZ)
    if i % 10 == 9:
        print(f"   {report(robot.turret_actuators())}")
# A rate latches: without this the pylon would keep turning to its limit.
robot.set_turret_rates(0.0, 0.0)
```

{{#endtab }}
{{#endtabs }}

Each report line shows the heading advancing at a fixed fraction of the max rate:

```text
-- heading rate 0.3 rad/s for 1.5 s --
   heading   +6.9 deg ( +19% of max rate)  pitch  +0.0 deg (  +0%)  laser off 0.00 m
   heading  +13.8 deg ( +19% of max rate)  pitch  +0.0 deg (  +0%)  laser off 0.00 m
   heading  +20.3 deg ( +19% of max rate)  pitch  +0.0 deg (  +0%)  laser off 0.00 m
```

> **Gotcha.** A controller that stops publishing rates leaves the turret slewing to its
> limit, because the last rate is still latched. End every rate phase with
> `set_turret_rates(0.0, 0.0)`, or with a target.

## Aiming at a target

A target is latched, reached and then held. The example confirms arrival by comparing the
measured angles, never by assuming the travel time:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex41_hello_turret.rs`:

```rust
robot.set_turret_joints(heading, pitch)?;
robot.rate(HZ);
let now = robot.turret_actuators()?;
let t = robot.states().elapsed - t0;
if (now.heading - heading).abs() < TOLERANCE && (now.pitch - pitch).abs() < TOLERANCE {
    println!("   t={t:5.2}s  {}  -- there", report(&now));
    return Ok(());
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex41_hello_turret.cpp`:

```cpp
robot.set_turret_joints(heading, pitch);
robot.rate(HZ);
const vrsdk::TurretActuators now = robot.turret_actuators();
const double t = robot.states().elapsed - t0;
if (std::fabs(now.heading - heading) < TOLERANCE &&
    std::fabs(now.pitch - pitch) < TOLERANCE) {
    std::printf("   t=%5.2fs  %s  -- there\n", t, report(now).c_str());
    return;
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex41_hello_turret.py`:

```python
robot.set_turret_joints(heading, pitch)
robot.rate(HZ)
now = robot.turret_actuators()
t = robot.states.elapsed - t0
if abs(now.heading - heading) < TOLERANCE and abs(now.pitch - pitch) < TOLERANCE:
    print(f"   t={t:5.2f}s  {report(now)}  -- there")
    return
```

{{#endtab }}
{{#endtabs }}

A target past a joint limit is clamped to the limit, so the measured angle stops short and
never matches. The loop then runs out and says so, which is the sign to check the limits.

```text
-- aim: heading -30.0 deg, pitch -5.0 deg --
   t= 0.06s  heading  +22.2 deg (-100% of max rate)  pitch  -2.4 deg (-100%)  laser off 0.00 m
   t= 0.26s  heading   +4.2 deg (-100% of max rate)  pitch  -5.0 deg (  +0%)  laser off 0.00 m
   t= 0.45s  heading  -13.8 deg (-100% of max rate)  pitch  -5.0 deg (  +0%)  laser off 0.00 m
   t= 0.66s  heading  -30.0 deg (  +0% of max rate)  pitch  -5.0 deg (  +0%)  laser off 0.00 m  -- there
```

## The laser, and reading its range

The laser has a latch of its own, untouched by the motion commands. The beam leaves the
muzzle along the barrel, so it follows both joints. While it is on, the simulator measures
the distance from the muzzle to the first thing the beam hits, on every physics step,
ignoring the turret's own body. That one number is all that reaches the wire.

| Decoded field | Units | Meaning |
|---|---|---|
| `heading`, `pitch` | rad | the joint angles, signed as above |
| `heading_rate_fraction`, `pitch_rate_fraction` | | the rate over the last physics step as a fraction of the max rate, in [-1, 1]; 0 while holding |
| `laser_on` | | whether the laser is switched on |
| `laser_range` | m | muzzle to the first hit; **0 while the laser is off or nothing is within range** |

`turret_actuators()` returns these from the latest snapshot. In the raw arrays the laser is
index 2: `normalized[2]` is 1 or 0 and `measured[2]` is the range.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex41_hello_turret.rs`:

```rust
for _ in 0..LASER_SAMPLES {
    robot.set_turret_laser(on)?;
    robot.rate(HZ);
}
let now = robot.turret_actuators()?;
println!(
    "   LASER [{}]: on={}, range {:.2} m  (0 = off, or nothing within range)",
    turret_actuator::LASER,
    now.laser_on,
    now.laser_range
);
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex41_hello_turret.cpp`:

```cpp
for (int i = 0; i < LASER_SAMPLES; ++i) {
    robot.set_turret_laser(on);
    robot.rate(HZ);
}
const vrsdk::TurretActuators now = robot.turret_actuators();
std::printf("   LASER [%d]: on=%s, range %.2f m  (0 = off, or nothing within range)\n",
            VRSDK_TURRET_ACT_LASER, now.laser_on ? "true" : "false", now.laser_range);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex41_hello_turret.py`:

```python
for _ in range(LASER_SAMPLES):
    robot.set_turret_laser(on)
    robot.rate(HZ)
now = robot.turret_actuators()
print(
    f"   LASER [{turret_actuator.LASER}]: on={now.laser_on}, range {now.laser_range:.2f} m  "
    "(0 = off, or nothing within range)"
)
```

{{#endtab }}
{{#endtabs }}

The example aims the barrel 5 degrees down first, so that on open, flat ground the beam
meets the floor:

```text
-- laser on --
   LASER [2]: on=true, range 23.92 m  (0 = off, or nothing within range)
```

Python prints `on=True`. The range reaches 200 m by default, and beyond that a target reads
0 exactly as a miss does, so a zero with `laser_on` true means "nothing within range", not
"touching the muzzle".

## The joints are the attitude too

The turret's `kin.quat` is the head relative to its base: heading on the pylon, then pitch
on the head, roll always 0. It is expressed in the turret's frame, which is `cv` unless the
[frames service](../ch06-services/04-frames-config.md) changed it, and it is not a world
attitude. The decoded `heading` and `pitch` are the same two angles with no frame to unpick,
so prefer them.

## What reset re-latches

`reset()` puts both joints back at 0, or at the nearest limit if the configured limits
exclude 0, clears the motion command and switches the laser off.

**Next:** [Cameras and images](../ch05-cameras/00-intro.md)

**See also:** [Commands latch](01-latching.md), [Turret configuration](../ch06-services/11-turret-config.md), [Turret](../ch07-robots/12-turret.md), [Actuators](../ch03-reading-state/05-actuator.md)
