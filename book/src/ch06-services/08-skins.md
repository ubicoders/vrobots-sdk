# Skins

The only service that ever tells you no, and the one where a wrong name looks like success.

```sh
cargo run -p vrobots-examples --bin ex23_skins
./target/cpp-build/ex23_skins
python examples/python/ex23_skins.py
```

## The catalogs

`set_skin(&str)` takes a catalog key and dresses the robot in it. The catalogs belong to the
robot type and are matched case-insensitively.

| Robot | Keys |
|---|---|
| Multirotor | `blue`, `desert`, `gold`, `green`, `mono`, `pink`, `snow`, `white` |
| Truck | `black`, `blue`, `camouflage`, `gray`, `red` |
| everything else | none, so every request is a no-op |

An empty or whitespace-only name is refused client-side with `VrError::InvalidArgument`,
because on the wire an empty payload is a read-back probe rather than a skin.

## The one refusal in the whole API

Skins are tier-gated inside the simulator. A tier refusal comes back as an honest
`ok = false` **with a reason**, which the SDK surfaces as
[`VrError::Service`](../appendix-c-errors.md) carrying the simulator's own message. That is the
single place in this API surface where a service says no.

| Request | Reply | What actually happened |
|---|---|---|
| a key your tier allows | `ok` | the skin changed |
| any key, tier too low | `ok = false` plus a reason | `VrError::Service`; **do not retry** |
| `gold` on a truck (a multirotor key) | `ok` | nothing, logged inside the simulator |
| `chartreuse` (in no catalog) | `ok` | nothing, logged inside the simulator |

**Do not retry a `VrError::Service` from this service.** It is tier-gated rather than
transient, so the answer will not change. `ex23` treats it as final and stops walking the list
rather than asking four more times.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex23_skins.rs`:

```rust
    match robot.set_skin(skin) {
        Ok(()) => println!("set_skin({skin:?}) -> ok"),
        Err(VrError::Service(reason)) => {
            // The sim's own words. This is the ONLY service that ever gets here.
            println!("set_skin({skin:?}) -> REFUSED by the sim: {reason}");
            return Ok(false);
        }
        Err(other) => return Err(other),
    }
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex23_skins.cpp`:

```cpp
try {
    robot.set_skin(skin);
    std::printf("set_skin(\"%s\") -> ok\n", skin.c_str());
} catch (const vrsdk::Error& e) {
    // The sim's own words. This is the ONLY service that ever gets here, and
    // only for a tier refusal -- anything else is a real failure.
    if (e.code() != VRSDK_ERR_SERVICE) {
        throw;
    }
    std::printf("set_skin(\"%s\") -> REFUSED by the sim: %s\n", skin.c_str(), e.what());
    return false;
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex23_skins.py`:

```python
try:
    robot.set_skin(skin)
    print(f"set_skin({skin!r}) -> ok")
except vrsdk.VrError as e:
    # The sim's own words. This is the ONLY service that ever gets here, and
    # only for a tier refusal -- anything else is a real failure.
    if e.code != vrsdk.err.SERVICE:
        raise
    print(f"set_skin({skin!r}) -> REFUSED by the sim: {e.detail}")
    return False
```

{{#endtab }}
{{#endtabs }}

Rust matches the `VrError::Service` variant and hands every other variant back to the caller. C++
and Python have one error type each, so they catch it, compare the code against `VRSDK_ERR_SERVICE`
or `vrsdk.err.SERVICE`, and rethrow anything that is not a tier refusal.

A permitted key prints one line and the truck changes colour. A refused one prints the
simulator's own explanation and the run stops:

```text
set_skin("black") -> ok
set_skin("blue") -> REFUSED by the sim: <the simulator's message>

Stopping: the tier gate does not open on a retry.
```

> **Note.** Every other refusal in this chapter is silent. This one is the exception, and it is
> worth knowing precisely because of what the rest do instead.

## A typo looks like success

An unknown key on a robot that has a catalog is acked `ok` and dropped with a log line no
client can see. So is a key from another robot's catalog. `ex23` demonstrates both, after
walking the five real truck keys:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    println!("\n-- keys that are acked `ok` and dropped inside the simulator --");
    wear(&robot, WRONG_TYPE_SKIN)?; // a multirotor key, on a truck
    wear(&robot, UNKNOWN_SKIN)?; // no catalog has it
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex23_skins.cpp`:

```cpp
std::printf("\n-- keys that are acked `ok` and dropped inside the simulator --\n");
wear(robot, WRONG_TYPE_SKIN);  // a multirotor key, on a truck
wear(robot, UNKNOWN_SKIN);     // no catalog has it
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex23_skins.py`:

```python
print("\n-- keys that are acked `ok` and dropped inside the simulator --")
wear(robot, WRONG_TYPE_SKIN)  # a multirotor key, on a truck
wear(robot, UNKNOWN_SKIN)  # no catalog has it
```

{{#endtab }}
{{#endtabs }}

Rust's `wear` returns a `Result`, so these two calls still carry `?` to propagate a genuine error
even though the bool is dropped. The C++ and Python helpers return a plain bool and let an
unexpected error unwind on its own. Neither key raises anything here: both come back `ok`.

Both return `Ok(())`, and the truck is still wearing the last key that worked:

```text
-- keys that are acked `ok` and dropped inside the simulator --
set_skin("gold") -> ok
set_skin("chartreuse") -> ok
Both returned Ok. The truck is still wearing "red" -- the ack was a receipt for a request the robot then refused with a log line no client can see.
```

The confirmation is the robot in front of you. There is no read-back and no state field
carrying the current skin.

## On a truck a skin is not only cosmetic

The wheel colliders travel with the skin prefab, so a swap **rebinds the physics wheels**.
`ex23` keeps the truck rolling across every change so that shows up on the wire.

| Channel | Meaning |
|---|---|
| `actuator.measured[0..3]` | the four wheel speeds, FL, FR, RL, RR, in rad/s |
| `actuator.measured[4]` | the steering servo |


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    for i in 0..HOLD_SAMPLES {
        robot.set_car(STEER_US, THROTTLE_US, Some(BRAKE_US))?;
        if i % 15 == 0 {
            let s = robot.states();
            let [vx, vy, vz] = s.kin.lin_vel;
            println!(
                "    t={:6.2}s speed={:5.2} m/s wheels={:?} steer_servo={:?}",
                s.elapsed,
                (vx * vx + vy * vy + vz * vz).sqrt(),
                // 0..3 are FL, FR, RL, RR in rad/s; they must keep turning
                // across the swap, because the colliders were just rebound.
                &s.actuator.measured[..s.actuator.measured.len().min(4)],
                s.actuator.measured.get(4)
            );
        }
        robot.rate(HZ);
    }
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex23_skins.cpp`:

```cpp
for (int i = 0; i < HOLD_SAMPLES; ++i) {
    robot.set_car(STEER_US, THROTTLE_US, BRAKE_US);
    if (i % 15 == 0) {
        const vrsdk::State s = robot.states();
        const double* v = s.kin().lin_vel;
        const double speed = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        // 0..3 are FL, FR, RL, RR in rad/s; they must keep turning across
        // the swap, because the colliders were just rebound.
        std::printf("    t=%6.2fs speed=%5.2f m/s wheels=%s steer_servo=%s\n", s.elapsed,
                    speed, channels(s.actuator(), 0, 4).c_str(),
                    channels(s.actuator(), 4, 5).c_str());
    }
    robot.rate(HZ);
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex23_skins.py`:

```python
for i in range(HOLD_SAMPLES):
    robot.set_car(STEER_US, THROTTLE_US, BRAKE_US)
    if i % 15 == 0:
        s = robot.states
        speed = math.dist(s.kin.lin_vel, (0.0, 0.0, 0.0))
        m = s.actuator.measured
        # 0..3 are FL, FR, RL, RR in rad/s; they must keep turning across
        # the swap, because the colliders were just rebound.
        wheels = [round(v, 3) for v in m[:4]]
        servo = [round(v, 3) for v in m[4:5]]
        print(
            f"    t={s.elapsed:6.2f}s speed={speed:5.2f} m/s "
            f"wheels={wheels} steer_servo={servo}"
        )
    robot.rate(HZ)
```

{{#endtab }}
{{#endtabs }}

The brake is an `Option` in Rust and a plain number in the other two. Reading the wheel channels
also differs: Rust and Python slice a growable list, while C++ has a fixed array with a separate
`measured_count`, which is why `ex23` gives its `channels` helper a first and last index to stay
inside it.

The four wheel channels must keep turning across each swap. A wheel that flatlines is a rebind
that did not take:

```text
set_skin("camouflage") -> ok
    t=<seconds>s speed=<m/s> wheels=[<w0>, <w1>, <w2>, <w3>] steer_servo=Some(<value>)
```

That is also why a skin swap is worth doing while the truck is stationary if you care about
repeatability: it is a change to the physics rig, not a texture swap.

## The empty key

The name is trimmed before it is checked, so whitespace does not sneak past.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    match robot.set_skin("   ") {
        Ok(()) => println!("\nUNEXPECTED: an empty key was accepted"),
        Err(e) => println!("\nempty key -> [{}] {}", e.code(), e.detail()),
    }
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex23_skins.cpp`:

```cpp
try {
    robot.set_skin("   ");
    std::printf("\nUNEXPECTED: an empty key was accepted\n");
} catch (const vrsdk::Error& e) {
    std::printf("\nempty key -> [%d] %s\n", e.code(), e.what());
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex23_skins.py`:

```python
try:
    robot.set_skin("   ")
    print("\nUNEXPECTED: an empty key was accepted")
except vrsdk.VrError as e:
    print(f"\nempty key -> [{e.code} {e.kind}] {e.detail}")
```

{{#endtab }}
{{#endtabs }}

The error carries the same code and message everywhere, but you read it differently: `e.code()`
with `e.detail()` in Rust, `e.code()` with `e.what()` in C++, and the attributes `e.code`,
`e.kind` and `e.detail` in Python.

```text
empty key -> [<code>] <message>
```

**Next:** [Supported virtual robots](../ch07-robots/00-intro.md)

**See also:** [Truck](../ch07-robots/02-truck.md), [Actuators](../ch03-reading-state/05-actuator.md), [Appendix C: Error reference](../appendix-c-errors.md)
