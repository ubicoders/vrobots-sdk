# Driving the truck

`SET_CAR` carries three pulse-width channels, on a factory band that is not the
multirotor's.

```sh
cargo run -p vrobots-examples --bin ex05_hello_car
./target/cpp-build/ex05_hello_car
python examples/python/ex05_hello_car.py
```

## The whole program

`examples/rust/src/bin/ex05_hello_car.rs` is the same loop shape as the multirotor example
with a different actuator in it:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 0; // the truck in the test scene
const STEER_US: f64 = 1400.0; // left of centre
const THROTTLE_US: f64 = 1650.0; // light forward
const BRAKE_US: f64 = 1100.0; // released
const HZ: f64 = 50.0;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Truck, Some(SYS_ID))?;

    // ===== loop =====
    loop {
        let s = robot.states();
        let [x, y, z] = s.kin.lin_pos;
        // lin_vel is a BODY-frame vector, so no single component is "the speed";
        // its magnitude is.
        let [vx, vy, vz] = s.kin.lin_vel;
        let speed = (vx * vx + vy * vy + vz * vz).sqrt();
        println!(
            "State t={:.3} pos=({x:.3},{y:.2},{z:.2}) speed={speed:.2} m/s echo={:?}",
            s.elapsed, s.actuator.pwm
        );

        // A gentle left arc: steering left of centre, light forward throttle.
        robot.set_car(STEER_US, THROTTLE_US, Some(BRAKE_US))?;

        robot.rate(HZ);
    }
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex05_hello_car.cpp`:

```cpp
constexpr std::uint32_t SYS_ID = 0;      // the truck in the test scene
constexpr double STEER_US = 1400.0;      // left of centre
constexpr double THROTTLE_US = 1650.0;   // light forward
constexpr double BRAKE_US = 1100.0;      // released
constexpr double HZ = 50.0;

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Truck, SYS_ID);
        robot.connect();
        std::printf("connected to sys_id %u\n", robot.sys_id());

        // ===== loop =====
        for (;;) {
            const vrsdk::State s = robot.states();
            const double* p = s.kin().lin_pos;

            // A gentle left arc: steering left of centre, light forward
            // throttle, brake released.
            robot.set_car(STEER_US, THROTTLE_US, BRAKE_US);

            // Speed from the body-frame twist, so it is visible that the truck
            // really is moving rather than that the command was merely accepted.
            const double* v = s.kin().lin_vel;
            const double speed = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

            const std::vector<std::uint32_t> echo = s.pwm();
            std::printf("State t=%.3f pos=(%.3f,%.2f,%.2f) speed=%.2f m/s  pwm_echo=[", s.elapsed,
                        p[0], p[1], p[2], speed);
            for (std::size_t i = 0; i < echo.size(); ++i) {
                std::printf("%s%u", i ? "," : "", echo[i]);
            }
            std::printf("]\n");

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

`examples/python/ex05_hello_car.py`:

```python
SYS_ID = 0  # the truck in the test scene
STEER_US = 1400.0  # left of centre
THROTTLE_US = 1650.0  # light forward
BRAKE_US = 1100.0  # released
HZ = 50


def main() -> None:
    # ===== setup =====
    vrsdk.init_logging("info")
    car = VirtualRobot(RobotType.TRUCK, sys_id=SYS_ID)
    car.connect()

    # ===== loop =====
    while True:
        s = car.states
        x, y, z = s.kin.lin_pos
        # lin_vel is a BODY-frame vector, so no single component is "the speed";
        # its magnitude is.
        speed = math.dist(s.kin.lin_vel, (0.0, 0.0, 0.0))
        print(
            f"State t={s.elapsed:.3f} pos=({x:.3f},{y:.2f},{z:.2f}) "
            f"speed={speed:.2f} m/s echo={s.actuator.pwm}"
        )

        # A gentle left arc: steering left of centre, light forward throttle.
        car.set_car(STEER_US, THROTTLE_US, BRAKE_US)

        car.rate(HZ)
```

{{#endtab }}
{{#endtabs }}

The echo at the end of each line is the three channels coming back in the order they were
sent:

```text
State t=<seconds> pos=(<x>,<y>,<z>) speed=<m/s> echo=[1400, 1650, 1100]
```

## The signature


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
pub fn set_car(&self, steer: f64, throttle: f64, brake: Option<f64>) -> VrResult<()>
```

{{#endtab }}
{{#tab name="C++" }}

`cpp/include/vrobots_sdk.hpp`:

```cpp
void set_car(double steer, double throttle, std::optional<double> brake = std::nullopt)
```

{{#endtab }}
{{#tab name="Python" }}

`crates/vrobots-sdk-py/python/vrsdk/_vrsdk.pyi`:

```python
def set_car(
    self, steer: float, throttle: float, brake: Optional[float] = None
) -> None: ...
```

{{#endtab }}
{{#endtabs }}

The optional third argument is optional in the type system of all three, and C++ and Python
also default it, so `set_car(steer, throttle)` is the two-channel form there.

The three channels go on the wire as `int_arr`, in that order, under command id `SET_CAR`
(304).

## The three channels

| Channel | Units | 1100 | 1500 | 1900 | Notes |
|---|---|---|---|---|---|
| `steer` | microseconds | full left | centre | full right | symmetric about 1500 |
| `throttle` | microseconds | full reverse | stop, idle brake | full forward | symmetric about 1500 |
| `brake` | microseconds | released | part applied | full | **bottom-anchored**: 1100 is off, not 1500 |

The truck's factory band is 1100 to 1900, unlike the multirotor's 1100 to 2000. The
client-side validator is the shared pulse-width check, which accepts the wider band, so a
value between 1900 and 2000 is published rather than refused. What the truck does with one
is not documented.

> **Gotcha.** The brake channel is the one people get wrong, because every other channel in
> the SDK is centred on 1500. Sending 1500 to the brake is not "no brake": it sits part way
> up a scale whose zero is 1100, and the symptom is a truck that accelerates far more slowly
> than its throttle suggests. <!-- VERIFY: whether the brake channel maps linearly from 1100 to 1900 is not documented anywhere in the SDK source. -->


## Passing None

`brake: None` sends the two-channel form, `[steer_us, throttle_us]`, and brakes nothing.
That is a different message from sending `Some(1100.0)` only in what is on the wire: both
leave the truck unbraked.

Use `None` when your controller has no brake concept, and `Some(...)` when it does. Pick one
form and keep to it: the two-channel message says nothing about the brake, and whether the
robot then holds the brake value a previous three-channel message latched is not documented.

## Stopping

Stopping is a command like any other. Throttle at 1500 is the idle brake, which is the
truck's own resting state, and a brake channel at 1900 adds full braking on top of it. A
controller that exits while commanding 1650 microseconds of throttle leaves the truck
driving, because that is the value still latched.

**Next:** [Single degree of freedom plants](04-single-dof.md)

**See also:** [Commands latch](01-latching.md), [The truck drivetrain](../ch06-services/05-drive-config.md), [Truck](../ch07-robots/02-truck.md)
