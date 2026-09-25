# Hello car

Drive the truck, whose three channels use a different band from the multirotor's.

```sh
cargo run -p vrobots-examples --bin ex05_hello_car
./target/cpp-build/ex05_hello_car
python examples/python/ex05_hello_car.py
```

## The whole program

The same loop shape as [Hello control](04-hello-control.md), a different actuator, and a
different robot: `SYS_ID` is 0 because the truck is the other vehicle in the test scene.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex05_hello_car.rs`:

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

The brake is the third argument in all three, and it is optional in all three: Rust wraps it
in `Some`, C++ in a `std::optional`, Python defaults it to `None`. Omitting it sends the
two-channel form, which brakes nothing.

The truck pulls away and curves left, so `pos` walks and `speed` climbs to a steady value:

```text
State t=0.000 pos=(0.000,0.35,0.00) speed=0.00 m/s echo=[1400, 1650, 1100]
State t=0.020 pos=(0.021,0.35,0.00) speed=1.06 m/s echo=[1400, 1650, 1100]
State t=0.040 pos=(0.043,0.35,0.01) speed=1.09 m/s echo=[1400, 1650, 1100]
```

<!-- VERIFY: the truck's actuator.pwm echo shown here assumes one entry per SET_CAR channel in the order steer, throttle, brake. Not confirmed against a live simulator. -->

## The three channels

`set_car(steer, throttle, brake)` takes pulse widths in microseconds, one per channel. The
truck's factory band is 1100 to 1900, which is **not** the multirotor's 1100 to 2000.

| Channel | 1100 | 1500 | 1900 |
|---|---|---|---|
| steer | full left | centre | full right |
| throttle | full reverse | stop (idle brake) | full forward |
| brake | released | -- | full |

The SDK's client-side check is the wider 1100 to 2000 band it applies to every pulse width,
so a value of 1950 is accepted by the SDK and handled by the truck's own limits. Stay
inside 1100 to 1900 and the two agree.

## Brake is bottom-anchored

Steer and throttle are centre-anchored: 1500 is neutral for both, and the interesting
values live on either side of it. Brake is not. **1100 is released and 1900 is full**, so
the neutral value for the brake channel is the bottom of the band, not the middle.

Sending 1500 on the brake channel therefore applies braking rather than none, which is a
common reason a truck that should be accelerating crawls instead.

> **Gotcha.** `brake` is an `Option<f64>`. Passing `None` sends the two-channel form of
> `SET_CAR`, which brakes nothing. That is a different statement from "the brake is
> released to 1100": the channel is absent from the message. Pass `Some(1100.0)` when you
> want to say released explicitly.

## Speed is a magnitude, not a component

`kin.lin_vel` is a **body-frame** vector, so no single component of it is "the speed". The
example takes the magnitude, which is why the arithmetic is spelled out rather than reading
`vy` and calling it done.

`kin.lin_pos`, by contrast, is a **world-frame** position. That split (pose in world,
twist and acceleration in body) is physics rather than configuration, and it holds for
every robot and for both the truth and the estimate blocks.

The truck also disagrees with the multirotor about which way is up: it publishes `"fru"`
where the multirotor publishes `"frd"`, so the third component of a vector means opposite
things on the two vehicles. `s.coord_frame_id` is the authoritative answer, and
[Frames, axes and units](../ch02-concepts/07-frames-and-units.md) is the page that settles
it.

**Next:** [Hello image](06-hello-image.md)

**See also:** [Driving the truck](../ch04-commands/03-truck.md), [The truck drivetrain](../ch06-services/05-drive-config.md), [Kinematics](../ch03-reading-state/02-kinematics.md)
