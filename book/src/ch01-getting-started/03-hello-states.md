# Hello states

Your first program: connect to a robot and print its position at your own rate.

```sh
cargo run -p vrobots-examples --bin ex01_hello_states
./target/cpp-build/ex01_hello_states
python examples/python/ex01_hello_states.py
```

## The whole program

There is no framework here. `main` does setup, then owns a plain infinite loop.

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex01_hello_states.rs`:

```rust
use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const HZ: f64 = 50.0;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    // ===== loop =====
    loop {
        let s = robot.states(); // immutable latest snapshot, never torn
        let [x, y, z] = s.kin.lin_pos;
        println!("State t={:.3} pos=({x:.3},{y:.2},{z:.2})", s.elapsed);
        robot.rate(HZ); // drift-compensated pacing, Hz
    }
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex01_hello_states.cpp`:

```cpp
#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr double HZ = 50.0;

/// Everything the SDK waits on, retries or drops shows up here. Registering it
/// before connect() is the point: connect is the noisiest moment, and a hang
/// with no log is the hardest thing to debug.
static void on_log(vrsdk::LogLevel level, const char* target, const char* message) {
    std::printf("[%-5s %s] %s\n", vrsdk::to_string(level), target, message);
}

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();  // header and library must be the same release
        vrsdk::set_log_callback(on_log);

        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();  // blocks until the first state snapshot arrives
        std::printf("connected to sys_id %u\n", robot.sys_id());

        // ===== loop =====
        for (;;) {
            const vrsdk::State s = robot.states();  // latest snapshot, never torn
            const double* p = s.kin().lin_pos;
            std::printf("State t=%.3f pos=(%.3f,%.2f,%.2f)\n", s.elapsed, p[0], p[1], p[2]);
            robot.rate(HZ);  // drift-compensated pacing, Hz
        }
    } catch (const vrsdk::Error& e) {
        // `code()` is the SDK's stable number -- the same one Python's
        // VrError.code and the CLI's `error [N]` report.
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex01_hello_states.py`:

```python
import vrsdk
from vrsdk import RobotType, VirtualRobot

SYS_ID = 1  # the multirotor in the test scene
HZ = 50


def main() -> None:
    # ===== setup =====
    vrsdk.init_logging("info")
    mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=SYS_ID)
    mr.connect()

    # ===== loop =====
    while True:
        s = mr.states  # immutable latest snapshot, never torn
        x, y, z = s.kin.lin_pos
        print(f"State t={s.elapsed:.3f} pos=({x:.3f},{y:.2f},{z:.2f})")
        mr.rate(HZ)  # drift-compensated pacing, Hz


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped.")
    except vrsdk.VrError as e:
        # Every SDK failure is one exception type carrying a stable code.
        raise SystemExit(f"error [{e.code} {e.kind}] {e.detail}")
```

{{#endtab }}
{{#endtabs }}

Three surfaces, one shape: setup, then a loop you own. The C++ version registers a log
callback where Rust and Python call `init_logging`, and `states` is a method in Rust and C++
but a property in Python. Fifty lines a second, until you press Ctrl-C:

```text
State t=0.000 pos=(0.000,1.05,0.00)
State t=0.020 pos=(0.000,1.05,0.00)
State t=0.040 pos=(0.000,1.05,0.00)
State t=0.060 pos=(0.000,1.05,0.00)
```

`init_logging("info")` also puts the SDK's own connect progress on stderr. Your position
numbers will differ; a robot sitting still on the ground is the expected first sight.

## The STM32 shape, line by line

Read the program as four moves.

1. **`init_logging("info")`** installs a `tracing` subscriber. `RUST_LOG` overrides the
   filter you pass, and the call is a no-op if a subscriber is already installed.
2. **`VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))`** attaches to a robot the
   scene already contains. `Some(id)` never touches the manager's create service. It
   subscribes the state topic and blocks until the first snapshot arrives, so `states()` is
   valid the instant `connect` returns.
3. **`robot.states()`** hands back the latest snapshot. This is the whole read API.
4. **`robot.rate(HZ)`** sleeps until the next tick, compensating for drift, so the loop
   runs at the rate you asked for rather than that rate minus your own work.

There is no base class, no runner, no `setup()` and no `update()` callback. The SDK never
calls your code. This is the single biggest difference from the older Python client, and
[The shape of a program](../ch02-concepts/04-program-shape.md) is the page that argues it.

## What states() guarantees

Three guarantees, and one absence that surprises people.

- **It never blocks.** An SDK-owned subscriber thread keeps a snapshot fresh; you read it
  at whatever rate suits your controller. A 50 Hz loop against a 25 Hz stream is legal and
  sees each sample twice.
- **It is never torn.** You get a whole snapshot or the previous whole snapshot, never a
  position from one sample and a velocity from the next.
- **It never fails.** There is no error return. If the simulator stops, `states()` keeps
  handing back the last snapshot forever, so a frozen robot and a stopped simulator look
  identical from inside the loop.
- **There is no "is it new" flag.** Nothing on the snapshot says whether you have already
  seen it.

> **Gotcha.** Because a stall presents as unchanging numbers rather than an error, polling
> `states()` cannot detect one. `wait_new_state(timeout)` blocks for a genuinely newer
> sample and returns `VrError::Timeout` when none arrives, which is a status rather than a
> failure. That is the detector, and `ex09_state_paced_loop` is the example.

## elapsed is not a wall clock

`s.elapsed` is seconds since **this robot's first state sample**, as an `f64`, monotonic,
and shared by every one of that robot's streams including its cameras. It is the field to
print and to plot against.

It is not the time of day and it is not your process's uptime. When you need to compare
across streams, use `s.t_ns`, which is nanoseconds since the unix epoch on the simulator's
clock and is directly subtractable from a camera frame's `t_ns`. `s.seq` is the per-topic
sequence number: a jump larger than one means a sample was dropped.

## Which id is the multirotor

`SYS_ID` is 1 because in the test scene, booted straight into the Flatworld scene,
**sys_id 1 is the multirotor and sys_id 0 is the truck**.

That is a convenience, not a contract. Ids are allocated at scene load and keep
incrementing across loads, so the same scene reloaded gives you different numbers. A
`const` at the top of an example is there so the example has something to run against, and
[`vrobots topic list`](02-first-contact.md) is what tells you the truth.

**Next:** [Hello control](04-hello-control.md)

**See also:** [Truth, measured and believed](../ch03-reading-state/01-truth-measured-believed.md), [Pacing your loop](../ch03-reading-state/07-pacing.md), [Timestamps and sequence numbers](../ch03-reading-state/06-timestamps.md)
