# Commands latch

A command is a setpoint the robot holds, not an event it performs, and understanding that
explains most of the surprises in this chapter.

## A setpoint, not an impulse

The last command a robot received stays in effect until the next one arrives. Nothing
decays it, nothing times it out, and nothing zeroes it when the process that sent it exits.
Run a program that latches a pulse width, kill it, and the rotors keep spinning at that
pulse width: the echo in the state stream reports the robot's holder, not the sender.

There is no watchdog anywhere in the command path. This is a deliberate simulator property,
not an oversight, and it is the same assumption a real ESC makes about the flight
controller upstream of it.

## Your send rate is your own business

Because the value latches, the publish rate carries no meaning. A 5 Hz sender and a 50 Hz
sender produce exactly the same behaviour if they send the same numbers, and physics runs
at its own rate regardless. Publish at whatever rate your controller thinks at.

The practical consequence appears in every loop that waits for state. From
`examples/rust/src/bin/ex29_hello_cartpole.rs`, the response to a stalled state stream is
to do nothing at all:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
        if let Err(VrError::Timeout(_)) = robot.wait_new_state(SAMPLE_TIMEOUT) {
            println!("no new state -- holding the last force (it latches)");
            continue;
        }
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex29_hello_cartpole.cpp`:

```cpp
} catch (const vrsdk::Error& e) {
    if (e.code() != VRSDK_ERR_TIMEOUT) {
        throw;
    }
    std::printf("no new state -- holding the last force (it latches)\n");
    continue;
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex29_hello_cartpole.py`:

```python
except vrsdk.VrError as e:
    if e.code != vrsdk.err.TIMEOUT:
        raise
    print("no new state -- holding the last force (it latches)")
    continue
```

{{#endtab }}
{{#endtabs }}

Skipping the iteration is not a lost command. The previous force is still applied, so
holding is the correct response to silence rather than a degraded one.

## Releasing a command is itself a command

There is no "stop" verb. To stop pushing, send zero. From
`examples/rust/src/bin/ex28_hello_msd.rs`, where the step force has to be explicitly
withdrawn before the plant can ring back to equilibrium:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    // Release. The force LATCHES, so this zero is not optional.
    println!("   release (set_msd_force(0.0)) -- watch it ring back to equilibrium");
    for i in 0..RELEASE_SAMPLES {
        robot.set_msd_force(0.0)?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex28_hello_msd.cpp`:

```cpp
// Release. The force LATCHES, so this zero is not optional.
std::printf("   release (set_msd_force(0.0)) -- watch it ring back to equilibrium\n");
for (int i = 0; i < RELEASE_SAMPLES; ++i) {
    robot.set_msd_force(0.0);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex28_hello_msd.py`:

```python
# Release. The force LATCHES, so this zero is not optional.
print("   release (set_msd_force(0.0)) -- watch it ring back to equilibrium")
for i in range(RELEASE_SAMPLES):
    robot.set_msd_force(0.0)
```

{{#endtab }}
{{#endtabs }}

Leaving that call out does not "let the force fade": the mass sits pushed against its
spring forever, and the printout looks like a plant that will not settle.

> **Gotcha.** A Ctrl-C never releases anything. Examples that hand a robot back do it on
> the normal exit path only, so an interrupted run leaves the last force, the last pulse
> widths or the last deflections applied until someone else overwrites them.

## What each command holds

| Command | What stays latched | How to release it |
|---|---|---|
| `SET_MR_PWM` | one pulse width per rotor, microseconds | send new pulse widths, or `reset()` |
| `SET_CAR` | steer, throttle and brake, microseconds | send new channels, or `reset()` |
| `SET_MSD` | the drive force, newtons | `set_msd_force(0.0)`, or `reset()` |
| `SET_INVPEN` | the cart force, newtons | `set_cartpole_force(0.0)`, or `reset()` |
| `SET_FW_SURFACES` | one deflection per panel, radians | send new deflections, leave direct mode, or `reset()` |
| `SET_FW_THRUST` | engine thrust, newtons | `set_fw_thrust_bias(0.0)` gives the engine back to airspeed hold in onboard mode |
| `SET_ANGVEL` | the rate setpoint, rad/s | send a new one; `reset()` clears this latch rather than re-seeding it |

## Watching a latch hold

`examples/rust/src/bin/ex31_globalhawk_direct.rs` demonstrates the property directly by
sending one pose and then sending nothing for roughly two seconds:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    // ===== latching, and no watchdog =====
    println!("\n-- nothing sent for ~2 s --");
    robot.set_fw_surfaces(&[
        DEFLECT_RAD,
        -DEFLECT_RAD,
        DEFLECT_RAD,
        -DEFLECT_RAD,
        0.0,
        0.0,
    ])?;
    for i in 0..HOLD_SAMPLES {
        if i % 25 == 0 {
            print_echo(&robot, "  latched");
        }
        robot.rate(HZ);
    }
    println!("  unchanged. A command is a setpoint; there is no failsafe behind it.");
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex31_globalhawk_direct.cpp`:

```cpp
// ===== latching, and no watchdog =====
const std::vector<double> latched = {DEFLECT_RAD, -DEFLECT_RAD, DEFLECT_RAD,
                                     -DEFLECT_RAD, 0.0,         0.0};
std::printf("\n-- nothing sent for ~2 s --\n");
robot.set_fw_surfaces(latched);
for (int i = 0; i < HOLD_SAMPLES; ++i) {
    if (i % 25 == 0) {
        print_echo(robot, "  latched");
    }
    robot.rate(HZ);
}
std::printf("  unchanged. A command is a setpoint; there is no failsafe behind it.\n");
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex31_globalhawk_direct.py`:

```python
# ===== latching, and no watchdog =====
latched = [DEFLECT_RAD, -DEFLECT_RAD, DEFLECT_RAD, -DEFLECT_RAD, 0.0, 0.0]
print("\n-- nothing sent for ~2 s --")
robot.set_fw_surfaces(latched)
for i in range(HOLD_SAMPLES):
    if i % 25 == 0:
        print_echo(robot, "  latched")
    robot.rate(HZ)
print("  unchanged. A command is a setpoint; there is no failsafe behind it.")
```

{{#endtab }}
{{#endtabs }}

Each printed line has the shape below, and the point of the passage is that the panel
numbers in successive lines are identical while nothing is being published:

```text
  latched t=<seconds>s panels=[<six deflections, radians>] engine=<newtons> N  rates=(<p>,<q>,<r>) deg/s
```

## What reset does to a latch

`reset()` re-latches the robot's initial command, the same thing the simulator's own Reset
button does. It is a state reset, not a factory reset: configuration sent through the
`srv/*` services (masses, noise models, rotor curves, skins) survives it.

A live publisher wins again one physics step later, so a control loop that keeps running
through a reset barely notices: it sees one snapshot of the initial command and then its
own values again. A program that resets and then stops sending sees the initial command
stand indefinitely.

On the fixed wing, `reset()` also reverts the control mode and the estimate source, which
is a larger change than re-latching a number and has [its own
section](05-fixed-wing.md#what-reset-takes-away).

**Next:** [Driving a multirotor](02-multirotor.md)

**See also:** [Sending commands](00-intro.md), [Robot lifecycle](../ch06-services/01-lifecycle.md), [Pacing your loop](../ch03-reading-state/07-pacing.md)
