# Hello control

Close the loop: send pulse widths to a multirotor and read them back from the state stream.

```sh
cargo run -p vrobots-examples --bin ex02_hello_control
./target/cpp-build/ex02_hello_control
python examples/python/ex02_hello_control.py
```

## The whole program

The same loop as [Hello states](03-hello-states.md), with two lines added: one that
computes a command and one that sends it.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex02_hello_control.rs`:

```rust
use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const PWM_US: f64 = 1501.0; // microseconds per rotor, on the 1100-2000 band
const HZ: f64 = 100.0;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    // ===== loop =====
    loop {
        let s = robot.states();
        let [x, y, z] = s.kin.lin_pos;
        println!(
            "State t={:.3} pos=({x:.3},{y:.2},{z:.2}) echo={:?}",
            s.elapsed, s.actuator.pwm
        );

        // Do some COOL control here and publish -- PID/EKF is user code, NOT the SDK.
        let cool_control_result = [PWM_US; 4];
        robot.set_mr_pwm(cool_control_result)?;

        robot.rate(HZ);
    }
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex02_hello_control.cpp`:

```cpp
constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr double PWM_US = 1501.0;    // microseconds per rotor, 1100-2000 band
constexpr double HZ = 100.0;

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();
        std::printf("connected to sys_id %u\n", robot.sys_id());

        // ===== loop =====
        for (;;) {
            const vrsdk::State s = robot.states();
            const double* p = s.kin().lin_pos;

            // Do some COOL control here and publish -- PID/EKF is user code,
            // NOT the SDK.
            const std::vector<double> cool_control_result = {PWM_US, PWM_US, PWM_US, PWM_US};
            robot.set_mr_pwm(cool_control_result);

            // The echo: what the robot actually latched, from the state stream.
            const std::vector<std::uint32_t> echo = s.pwm();
            std::printf("State t=%.3f pos=(%.3f,%.2f,%.2f)  pwm_echo=[", s.elapsed, p[0], p[1],
                        p[2]);
            for (std::size_t i = 0; i < echo.size(); ++i) {
                std::printf("%s%u", i ? "," : "", echo[i]);
            }
            std::printf("]  rotor0=%.1f rad/s\n",
                        s.actuator().measured_count > 0 ? s.actuator().measured[0] : 0.0);

            robot.rate(HZ);
        }
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex02_hello_control.py`:

```python
SYS_ID = 1  # the multirotor in the test scene
PWM_US = 1501.0  # microseconds per rotor, on the 1100-2000 band
HZ = 100


def main() -> None:
    # ===== setup =====
    vrsdk.init_logging("info")
    mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=SYS_ID)
    mr.connect()

    # ===== loop =====
    while True:
        s = mr.states
        x, y, z = s.kin.lin_pos
        print(
            f"State t={s.elapsed:.3f} pos=({x:.3f},{y:.2f},{z:.2f}) "
            f"echo={s.actuator.pwm}"
        )

        # Do some COOL control here and publish -- PID/EKF is user code, NOT the
        # SDK. `set_mr_pwm(a, b, c, d)` and `set_mr_pwm([a, b, c, d])` are the
        # same call.
        cool_control_result = [PWM_US] * 4
        mr.set_mr_pwm(cool_control_result)

        mr.rate(HZ)
```

{{#endtab }}
{{#endtabs }}

Python accepts the four values loose or as one sequence; C++ takes a `std::vector<double>`
and reads the echo through `s.pwm()`, which is the same `actuator.pwm` array the other two
print directly.

```text
State t=0.000 pos=(0.000,1.05,0.00) echo=[1501, 1501, 1501, 1501]
State t=0.010 pos=(0.000,1.05,0.00) echo=[1501, 1501, 1501, 1501]
State t=0.020 pos=(0.000,1.05,0.00) echo=[1501, 1501, 1501, 1501]
```

The loop prints the snapshot before it sends, so the first line or two echo whatever was
latched on that robot before you started. Your own value appears a physics step after your
first send.

## The 1100 to 2000 microsecond band

`set_mr_pwm` takes four pulse widths in microseconds, one per rotor, and every one must be
finite and inside 1100 to 2000. The SDK checks that client-side and returns
`VrError::InvalidArgument` before anything reaches the wire, so a bad value is one of the
few command mistakes you find out about immediately.

1100 is idle. A flying drone commanded `[1100; 4]` falls. There is no fixed hover value:
hover is wherever total thrust crosses weight for the robot's current mass and thrust
curves, both of which are configurable.

> **Gotcha.** Length is not validated the same way. `set_mr_pwm_n` accepts a slice of any
> non-empty length, and a wrong rotor count is dropped by the simulator with a log line no
> client can see, leaving the previous command latched. See
> [Driving a multirotor](../ch04-commands/02-multirotor.md).

## You are the flight controller

`SET_MR_PWM` is the lowest actuation level the simulator offers. Nothing sits between
these four numbers and the thrust curves: no attitude stabilisation, no rate damping, no
mixer. Four equal pulse widths produce four equal thrusts, and any imbalance in mass or
inertia tips the vehicle over with nothing to catch it.

That is deliberate. The point of the simulator is that the stabilisation is your code. A
PID loop, an EKF, an LQR: all of it lives in your `main`, and the SDK contributes nothing
to it.

## The actuator echo is the only receipt

Commands are published to `vrobots/{sys_id}/z/cmd` and the robot acknowledges nothing. It
drains its command queue at the start of the next physics step and moves on. A wrong
command id, a wrong `sys_id` and a wrong array length all present identically from
outside: the state stream does not change.

So the receipt is `s.actuator.pwm`, the commanded pulse widths echoed back inside the next
snapshot. Watching it settle on `[1501, 1501, 1501, 1501]` is the proof that your command
landed on the robot you meant.

Commands also **latch**. Each one is a setpoint, not an impulse, and the last one received
stays in effect until the next arrives. There is no watchdog, so a 5 Hz sender and a
100 Hz sender are both fine, and a controller that stops sending leaves the robot flying
its final command. This example runs at 100 Hz while physics runs at 50 Hz, which is
harmless.

## 1501 will not lift it, 1700 will

`PWM_US` is 1501, barely off idle, so the printed position does not change: the example is
about seeing the echo, not about flying. Edit the constant to 1700 and run it again to
watch the vertical component of `pos` move.

> **Note.** Which component that is depends on the robot's frame. The multirotor publishes
> `frd`, where the third component points **down**, so climbing makes it more negative.
> Read `s.coord_frame_id` rather than assuming, as
> [Frames, axes and units](../ch02-concepts/07-frames-and-units.md) explains.

**Next:** [Hello car](05-hello-car.md)

**See also:** [Commands latch](../ch04-commands/01-latching.md), [Driving a multirotor](../ch04-commands/02-multirotor.md), [Actuators](../ch03-reading-state/05-actuator.md)
