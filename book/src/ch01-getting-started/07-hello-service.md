# Hello service

Create a robot from code and delete it again, which is what explicit lifecycle means.

```sh
cargo run -p vrobots-examples --bin ex04_hello_service
./target/cpp-build/ex04_hello_service
python examples/python/ex04_hello_service.py
```

## The whole program

Lifecycle and configuration are one-shot request and response, so this is the first example
with no loop in it.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex04_hello_service.rs`:

```rust
use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const ROBOT_TYPE: RobotType = RobotType::Multirotor;

fn main() -> Result<(), VrError> {
    vrobots_sdk::init_logging("info");

    // Create a NEW robot in the sim (no sys_id -> manager create; reply carries the id).
    let robot = VirtualRobot::connect(ROBOT_TYPE, None)?;
    let sys_id = robot.sys_id();
    println!("created sys_id = {sys_id}");

    // The create reply is a receipt; the robot *exists* once its state topic
    // publishes, which connect() already waited for -- so this is real data.
    let s = robot.states();
    println!(
        "first state: t={:.3} seq={} name={:?}",
        s.elapsed, s.seq, s.name
    );
    println!("its state topic: {}", vrobots_sdk::topics::state(sys_id));

    // Deletion is explicit and never implicit. delete() waits for the state topic
    // to fall silent: the manager's ack is only a receipt, absence is the proof.
    robot.delete()?;
    println!(
        "deleted sys_id = {sys_id} (is_deleted={})",
        robot.is_deleted()
    );

    // The handle is spent. Commands do not silently do nothing -- they say why.
    match robot.set_mr_pwm([1500.0; 4]) {
        Ok(()) => println!("unexpected: a deleted robot accepted a command"),
        Err(e) => println!("the handle is spent, as expected: [{}] {e}", e.code()),
    }
    Ok(())
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex04_hello_service.cpp`:

```cpp
int main() {
    try {
        vrsdk::check_version();

        // Create a NEW robot in the sim: `create` means "no sys_id", so the
        // manager assigns one and the reply carries it. (A constructor would be
        // ambiguous with the attach form -- see the header.)
        vrsdk::VirtualRobot robot = vrsdk::VirtualRobot::create(vrsdk::RobotType::Multirotor);
        robot.connect();
        const std::uint32_t sys_id = robot.sys_id();
        std::printf("created sys_id = %u\n", sys_id);

        // The create reply is a receipt; the robot *exists* once its state topic
        // publishes, which connect() already waited for -- so this is real data.
        const vrsdk::State s = robot.states();
        std::printf("first state: t=%.3f seq=%llu name=\"%s\"\n", s.elapsed,
                    static_cast<unsigned long long>(s.seq), s.name.c_str());
        // The C++ surface has no topic-name builder (Rust has
        // `vrobots_sdk::topics`, Python has `vrsdk.topics`), and the shape is
        // fixed by the wire, so compose it here.
        std::printf("its state topic: vrobots/%u/z/state\n", sys_id);

        // Deletion is explicit and never implicit. The manager's ack is only a
        // receipt, so remove() also waits for the robot's state topic to fall
        // silent -- that is the real confirmation.
        robot.remove();
        std::printf("deleted sys_id = %u (removed=%s)\n", sys_id,
                    robot.removed() ? "true" : "false");

        // The handle is spent. Commands do not silently do nothing -- they say
        // why.
        try {
            robot.set_mr_pwm({1500.0, 1500.0, 1500.0, 1500.0});
            std::printf("unexpected: a deleted robot accepted a command\n");
        } catch (const vrsdk::Error& e) {
            std::printf("the handle is spent, as expected: [%d] %s\n", e.code(), e.what());
        }
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex04_hello_service.py`:

```python
ROBOT_TYPE = RobotType.MULTIROTOR  # or RobotType.TRUCK / RobotType.from_key("truck")


def main() -> None:
    vrsdk.init_logging("info")

    # Create a NEW robot in the sim (no sys_id -> manager create; the reply
    # carries the assigned id).
    robot = VirtualRobot(ROBOT_TYPE)
    robot.connect()
    sys_id = robot.sys_id
    print(f"created sys_id = {sys_id}")

    # The create reply is a receipt; the robot *exists* once its state topic
    # publishes, which connect() already waited for -- so this is real data.
    s = robot.states
    print(f"first state: t={s.elapsed:.3f} seq={s.seq} name={s.name!r}")
    print(f"its state topic: {vrsdk.topics(sys_id)['state']}")

    # Deletion is explicit and never implicit. delete() waits for the state topic
    # to fall silent: the manager's ack is only a receipt, absence is the proof.
    robot.delete()
    print(f"deleted sys_id = {sys_id} (is_deleted={robot.is_deleted})")

    # The handle is spent. Commands do not silently do nothing -- they say why.
    try:
        robot.set_mr_pwm(1500, 1500, 1500, 1500)
        print("unexpected: a deleted robot accepted a command")
    except vrsdk.VrError as e:
        print(f"the handle is spent, as expected: [{e.code} {e.kind}] {e.detail}")
```

{{#endtab }}
{{#endtabs }}

Three differences worth naming. C++ spells creation `VirtualRobot::create` rather than a
one-argument constructor, because a literal `0` would be ambiguous between "attach to sys_id
0" and a null options pointer. C++ spells deletion `remove()`, because `delete` is a keyword.
And C++ has no topic-name builder, so it composes `vrobots/<id>/z/state` inline where Rust
calls `topics::state` and Python calls `vrsdk.topics`.

The run takes a second or two, most of it spent waiting for the new robot's state topic to
start and then to stop:

```text
created sys_id = 7
first state: t=0.000 seq=0 name="multirotor"
its state topic: vrobots/7/z/state
deleted sys_id = 7 (is_deleted=true)
the handle is spent, as expected: [8] deleted: sys_id 7 was deleted from the sim; this handle is spent
```

<!-- VERIFY: the created robot's `name` field and the id it is allocated both come from the simulator; `"multirotor"` and `7` above are illustrative. -->

## Create versus attach

The second argument to `connect` decides which of two quite different things happens.

| Argument | What it does | Touches `srv/create` |
|---|---|---|
| `Some(id)` | attaches to a robot the scene already contains | no |
| `None` | asks the manager to spawn a new one; the reply carries the id | yes |

Every example so far passed `Some(SYS_ID)`. This one passes `None`, so the manager
allocates the id and `robot.sys_id()` is the only way to learn it.

Creating is limited by the scene's catalog, not the SDK's. The sandbox scene registers
`multirotor`, `truck` and `msd`; any other key is refused with a message naming the ones it
does know, which is also the only live way to enumerate the catalog. Robot types that are
scene-authored only, such as the cart pole and the Global Hawk, can be attached to but
never created.

> **Note.** Create is the one non-idempotent service in the system. The SDK sends it
> exactly once and never retries, because every retry that reaches the manager reserves
> another id and spawns another robot. [What connect actually does](../ch02-concepts/05-connect.md)
> walks the four steps.

## Robots outlive the process

Dropping a `VirtualRobot` closes its zenoh session. It does not delete the robot. The
robot keeps flying its last latched command, keeps publishing state, and is still there
after your program exits, after you rebuild, and after you run something else.

This is why the example calls `delete()`: without it, every run would leave another
multirotor in the scene. Comment the `delete()` line out and run it twice to see exactly
that, then attach `ex01_hello_states` to one of the ids it printed. Create in one process,
attach from another, is the normal shape of a multi-program session.

The way back from a littered scene is to reload it. Ids are allocated at scene load and
keep incrementing, so the reloaded scene numbers its robots differently.

## Absence is the proof, both ways

`connect(type, None)` returns only once the new robot's state topic has published, and
`delete()` returns only once that topic has fallen silent. Neither waits on the service
acknowledgement, because an acknowledgement is packed the instant the request arrives and
says nothing about whether the work happened.

That principle runs through the whole services chapter: an ack is a receipt, not a result,
and the state stream is the confirmation. It is rule two of
[Five rules that explain everything](../ch02-concepts/06-five-rules.md).

After `delete()` the handle is spent, and it says so rather than failing quietly. Every
command on it returns `VrError::Deleted`, error code 8, with a message naming the id.

**Next:** [When nothing happens](08-troubleshooting.md)

**See also:** [Robot lifecycle](../ch06-services/01-lifecycle.md), [System ids, and the two kinds of robot](../ch02-concepts/03-sys-id.md), [Appendix C: Error reference](../appendix-c-errors.md)
