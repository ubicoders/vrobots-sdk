# More than one robot

You hold two handles in one process, and you find out why `coord_frame_id` is on every snapshot.

```sh
cargo run -p vrobots-examples --bin ex18_multi_robot
./target/cpp-build/ex18_multi_robot
python examples/python/ex18_multi_robot.py
```

One program is one embedded system bound to one robot is the shape the SDK is built
around, and nothing enforces it. A `VirtualRobot` is a handle. Construct as many as
you like.

## Two connects, two sessions

Each `connect` opens its own zenoh session, its own subscriber thread and its own
snapshot, and each blocks until its own robot's first sample arrives. From
`examples/rust/src/bin/ex18_multi_robot.rs`:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    let truck = VirtualRobot::connect(RobotType::Truck, Some(TRUCK_ID))?;
    let drone = VirtualRobot::connect(RobotType::Multirotor, Some(DRONE_ID))?;
    println!(
        "truck sys_id={} ({:?}), drone sys_id={} ({:?})",
        truck.sys_id(),
        truck.robot_type(),
        drone.sys_id(),
        drone.robot_type()
    );
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex18_multi_robot.cpp`:

```cpp
        // Two connects, two sessions. Each blocks until *its* robot's first
        // state snapshot arrives, so both are live by the time the loop starts.
        vrsdk::VirtualRobot truck(vrsdk::RobotType::Truck, TRUCK_ID);
        truck.connect();
        vrsdk::VirtualRobot drone(vrsdk::RobotType::Multirotor, DRONE_ID);
        drone.connect();
        std::printf("truck sys_id=%u, drone sys_id=%u\n", truck.sys_id(), drone.sys_id());
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex18_multi_robot.py`:

```python
    truck = VirtualRobot(RobotType.TRUCK, sys_id=TRUCK_ID)
    truck.connect()
    drone = VirtualRobot(RobotType.MULTIROTOR, sys_id=DRONE_ID)
    drone.connect()
    print(
        f"truck sys_id={truck.sys_id} ({truck.robot_type.key}), "
        f"drone sys_id={drone.sys_id} ({drone.robot_type.key})"
    )
```

{{#endtab }}
{{#endtabs }}

Rust's `connect` constructs and connects in one call; C++ and Python construct the handle
first and then call `connect()` on it, which is two statements per robot. `sys_id` and the
robot type are methods in Rust (`robot_type()`) and C++ (`type()`) and properties in
Python, and the C++ example prints only the ids because its `RobotType` is a plain enum
with no string form.

```text
truck sys_id=0 (Truck), drone sys_id=1 (Multirotor)
```

Both handles are live by the time the loop starts, so the first `states()` on either
is valid.

| Property | Consequence |
|---|---|
| Each robot listens only on its own `cmd` topic | The `sys_id` in the topic is the routing. A command addressed to the wrong id produces silence, never a different robot moving. |
| `rate()` paces the calling loop | Call it on exactly one handle. Calling it on both sleeps twice per iteration and halves the loop rate. |
| `t_ns` is sim capture time for both | Directly comparable between robots. |
| `elapsed` counts from each robot's own first sample | Not comparable. The two differ by however far apart the two `connect` calls were. |

Two handles is also two sessions and two subscriber threads. That is fine for a
handful of robots. A swarm of fifty wants one subscriber on `vrobots/*/z/state`,
which is a different program.

## One commanded, one observed

The loop drives the truck and reads the multirotor. Nothing pairs the two snapshots:
each is whatever its own subscriber last received.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
        // One robot commanded ...
        truck.set_car(STEER_US, THROTTLE_US, Some(1100.0))?;
        let t = truck.states();

        // ... the other only observed. Nothing pairs the two snapshots: they are
        // whatever each subscriber last received.
        let d = drone.states();
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex18_multi_robot.cpp`:

```cpp
            // One robot commanded ...
            truck.set_car(STEER_US, THROTTLE_US, 1100.0);
            const vrsdk::State t = truck.states();

            // ... the other only observed. Nothing pairs the two snapshots:
            // they are whatever each subscriber last received.
            const vrsdk::State d = drone.states();
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex18_multi_robot.py`:

```python
        # One robot commanded ...
        truck.set_car(STEER_US, THROTTLE_US, 1100.0)
        t = truck.states

        # ... the other only observed. Nothing pairs the two snapshots: they are
        # whatever each subscriber last received.
        d = drone.states
```

{{#endtab }}
{{#endtabs }}

The brake argument is optional in Rust, so it is `Some(1100.0)`; C++ and Python take the
plain number. `states()` is a method in Rust and C++ and a property in Python, and in all
three it is a non-blocking read of whatever that robot's subscriber last received.

Pacing happens once, at the bottom, on one handle:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
        // Paced once, on one handle.
        truck.rate(HZ);
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex18_multi_robot.cpp`:

```cpp
            // Paced once, on one handle.
            truck.rate(HZ);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex18_multi_robot.py`:

```python
        # Paced once, on one handle.
        truck.rate(HZ)
```

{{#endtab }}
{{#endtabs }}

`rate` is identical across the three, and so is the rule: it sleeps the calling thread, so
calling it on both handles halves the loop rate.

## Their frames disagree

This is the trap, and it is the practical reason `coord_frame_id` rides on every
snapshot instead of being something you configure once and assume.

Measured live in the test scene, the truck publishes `fru` and the multirotor
publishes `frd`. Same third component, opposite sign: up for one, down for the other.
A program that mixes the two positions without converting has a sign error nothing
will report.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
        // Each snapshot names its own frame, and here they differ: the truck is
        // "fru" (third component UP) and the drone is "frd" (third component
        // DOWN). Print the tag beside every position rather than assuming one.
        println!(
            "truck[{}] pos=({tx:.2},{ty:.2},{tz:.2}) [{:?}] echo={:?}  |  \
             drone[{}] pos=({dx:.2},{dy:.2},{dz:.2}) [{:?}] alt={:.2} m",
            t.sys_id,
            t.coord_frame_id,
            t.actuator.pwm,
            d.sys_id,
            d.coord_frame_id,
            -dz // "frd": altitude is minus the down component
        );
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex18_multi_robot.cpp`:

```cpp
            // Each snapshot names its own frame, and here they differ: the
            // truck is "fru" (third component UP) and the drone is "frd" (third
            // component DOWN). Print the tag beside every position.
            std::printf(
                "truck[%u] pos=(%.2f,%.2f,%.2f) [%s]  |  drone[%u] pos=(%.2f,%.2f,%.2f) [%s] "
                "alt=%.2f m\n",
                t.sys_id, tp[0], tp[1], tp[2], t.coord_frame_id.c_str(), d.sys_id, dp[0], dp[1],
                dp[2], d.coord_frame_id.c_str(),
                -dp[2]);  // "frd": altitude is minus the down component
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex18_multi_robot.py`:

```python
        # Each snapshot names its own frame, and here they differ: the truck is
        # "fru" (third component UP) and the drone is "frd" (third component
        # DOWN). Print the tag beside every position rather than assuming one.
        print(
            f"truck[{t.sys_id}] pos=({tx:.2f},{ty:.2f},{tz:.2f}) "
            f"[{t.coord_frame_id!r}] echo={t.actuator.pwm}  |  "
            f"drone[{d.sys_id}] pos=({dx:.2f},{dy:.2f},{dz:.2f}) "
            f"[{d.coord_frame_id!r}] alt={-dz:.2f} m"
        )
```

{{#endtab }}
{{#endtabs }}

The frame tag is a plain string in all three, read off the snapshot rather than assumed.
C++ reaches the position through `t.kin().lin_pos`, a fixed-size array it indexes, where
Rust and Python unpack the three components into named variables; the C++ example also
omits the PWM echo the other two print.

```text
truck[0] pos=(1.42,-0.30,0.11) ["fru"] echo=[1500, 1650, 1100]  |  drone[1] pos=(0.00,0.00,-2.50) ["frd"] alt=2.50 m
    naive separation=3.15 m (WRONG: mixed frames, convert first)  snapshot skew=+12.3 ms  (elapsed: truck 4.21s vs drone 4.19s -- different epochs)
```

<!-- VERIFY: the position, echo, separation and skew values in the block above are illustrative and need a live-simulator capture; the `fru` and `frd` frame ids come from the example's own measured note. -->

The example computes the separation anyway and labels it `WRONG`, because the point
of the page is that the arithmetic runs happily and produces a number. The check that
catches it is one comparison:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
            if t.coord_frame_id == d.coord_frame_id {
                ""
            } else {
                " (WRONG: mixed frames, convert first)"
            },
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex18_multi_robot.cpp`:

```cpp
                t.coord_frame_id == d.coord_frame_id ? "" : " (WRONG: mixed frames, convert first)",
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex18_multi_robot.py`:

```python
        warn = "" if t.coord_frame_id == d.coord_frame_id else " (WRONG: mixed frames, convert first)"
```

{{#endtab }}
{{#endtabs }}

One string comparison in every surface. The check costs nothing and is the only thing
standing between you and a sign error that no error path reports.

> **Gotcha.** The frames a robot type publishes are its own, not yours. A program
> that holds one robot can get away with assuming; a program that holds two cannot.
> Read `coord_frame_id` from the snapshot and branch on it.

The skew line uses `t_ns`, which is the shared clock, so that difference is real.
`elapsed` appears beside it only to show that it is not: the two robots count from
different epochs.

**Next:** [Recording and testing without the simulator](07-recording-and-testing.md)

**See also:** [Frames, axes and units](../ch02-concepts/07-frames-and-units.md), [System ids, and the two kinds of robot](../ch02-concepts/03-sys-id.md), [Timestamps and sequence numbers](../ch03-reading-state/06-timestamps.md)
