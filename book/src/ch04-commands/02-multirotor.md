# Driving a multirotor

`SET_MR_PWM` is the lowest actuation level the simulator offers, so flying a multirotor
means writing the flight controller yourself.

```sh
cargo run -p vrobots-examples --bin ex02_hello_control
./target/cpp-build/ex02_hello_control
python examples/python/ex02_hello_control.py
```

## The whole program

`examples/rust/src/bin/ex02_hello_control.rs` is the loop shape from chapter 2 with one
command in it:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

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

One line per iteration. The four numbers at the end are `actuator.pwm`, which is your last
command echoed back, so they are the proof the command landed:

```text
State t=<seconds> pos=(<x>,<y>,<z>) echo=[1501, 1501, 1501, 1501]
```

`PWM_US` is barely off idle and will not lift the airframe. Raise it to 1700 to watch the
drone climb.

> **Note.** Nothing sits between these four pulse widths and the rotor thrust curves: no
> attitude stabilisation, no rate damping, no mixer. Equal values spin every rotor equally;
> differential values roll, pitch and yaw the airframe. Hover is wherever total thrust
> crosses weight for the current mass and curves, so there is no single "hover number".

## The two methods

| Method | Signature | Rotor count | Notes |
|---|---|---|---|
| `set_mr_pwm` | `(&self, pwm: [f64; 4]) -> VrResult<()>` | exactly 4 | fits the simulator's quadrotor; delegates to `set_mr_pwm_n` |
| `set_mr_pwm_n` | `(&self, pwm: &[f64]) -> VrResult<()>` | any, one per rotor | six or eight for bigger airframes; **exactly 2** for a `HalfDrone`, as `[left_us, right_us]` |

Both send command id `SET_MR_PWM` (300) with the values in `int_arr`. The array length must
equal the airframe's rotor count, which is fixed at spawn.

The bindings need only one method each, because neither language has Rust's split between a
fixed-size array and a slice.

{{#tabs global="lang" }}
{{#tab name="C++" }}

`cpp/include/vrobots_sdk.hpp`:

```cpp
void set_mr_pwm(const std::vector<double>& pwm)
```

{{#endtab }}
{{#tab name="Python" }}

`crates/vrobots-sdk-py/python/vrsdk/_vrsdk.pyi`:

```python
def set_mr_pwm(self, *pwm: Union[float, Sequence[float]]) -> None: ...
```

{{#endtab }}
{{#endtabs }}

C++ takes a `std::vector<double>` of any length, so `{1501, 1501, 1501, 1501}` covers the
quadrotor and a two-element vector covers a `HalfDrone`. Python accepts either form:
`set_mr_pwm(a, b, c, d)` and `set_mr_pwm([a, b, c, d])` are the same call.

## The band

| Quantity | Units | Minimum | Maximum | Notes |
|---|---|---|---|---|
| Pulse width per rotor | microseconds | 1100.0 | 2000.0 | 1100 is idle, so `[1100; 4]` lets a flying drone fall |

The band is checked client-side by a shared `check_pwm` helper, so a value outside it, or a
non-finite value, returns `VrError::InvalidArgument` before anything is published. Passing a
normalised 0.7 by mistake names the channel and the band it missed:

```text
pwm[0] = 0.7 is outside the 1100-2000 us pulse-width band (neutral is 1500; values look like microseconds, not normalised units)
```

The refusal is deliberate rather than defensive. The simulator's actuators are specified on
1100 to 2000 microseconds, so a value outside it is far more likely to be a unit mistake (a
normalised 0.7, a raw newton figure) than an intent.

## A wrong rotor count is not an error

The SDK checks each value but not the length, because it does not know the airframe. A
slice of the wrong length is published happily, the simulator logs it and drops it, and the
previously latched pulse widths stay in effect.

> **Gotcha.** Sending five pulse widths to a quadrotor looks exactly like sending nothing:
> `Ok(())` from the call, no error anywhere, and an echo that keeps reporting the old
> values. Compare `s.actuator.pwm.len()` against what you sent before assuming the
> simulator is asleep.

The one client-side length check is emptiness: an empty slice returns
`VrError::InvalidArgument` with the message "set_mr_pwm needs one pulse width per rotor;
got none".

## The half drone

A `HalfDrone` is a two-rotor airframe that takes the same command id through `set_mr_pwm_n`
with exactly two values, `[left_us, right_us]`. It is scene-authored rather than
creatable, so attach to it by `sys_id` rather than asking the manager to spawn one.

**Next:** [Driving the truck](03-truck.md)

**See also:** [Commands latch](01-latching.md), [Actuators](../ch03-reading-state/05-actuator.md), [Rotors and thrust curves](../ch06-services/06-rotor-config.md), [Multirotor](../ch07-robots/01-multirotor.md)
