# Safety events

A robot reports each near miss, contact and strike it is involved in as one event, and this page is how you subscribe to them and read every field.

```sh
cargo run -p vrobots-examples --bin ex43_safety_events
./target/cpp-build/ex43_safety_events
python examples/python/ex43_safety_events.py
```

## What a safety event is

A safety event is one record of one incident between a robot and something else: a person,
another robot, or an object. The robot publishes it on its own topic,
`vrobots/{sys_id}/z/event/safety`, over zenoh, so it reaches a program on another machine
exactly as states do.

Two properties set this stream apart from every other one in the SDK.

**Events queue.** A state snapshot is superseded by the next one, so the state stream keeps
only the latest. An incident is superseded by nothing: a contact that turns into a strike is
two records. The stream therefore holds every event until you read it and hands each one out
once, oldest first.

**Silence is the normal state.** The topic is not periodic. One sample is published per
incident and nothing in between, so there is no rate to watch, and a stream that stays quiet
for an hour is a stream on which nothing happened. Waiting for an event and getting nothing
is an answer, not an error.

## Who produces them

| Incident | Reported when | `other_kind` | `impulse` | `robot_at_fault` |
|---|---|---|---|---|
| A robot and a person | any part of any robot type comes close to, touches or hits a person: a chassis, the forks, an arm link | person | measured; 0 for a near miss and for an arm link | measured: true when the robot was the one moving in |
| An arm link and an object | an arm's protective stop finds one of its links against something | object | always 0 | always true |
| An arm link and another robot | the same stop, when what the link met belongs to another robot | robot | always 0 | always true |

People are the pedestrians of the Warehouse scene, who walk between the racks. An incident
with a person is detected on the person's side, which is why every robot type reports it
alike. One event is published per contact episode, and one per approach that passes close
at speed without touching.

The arm rows come from the [rail arm](../ch07-robots/10-rail-arm.md) and the
[table arm](../ch07-robots/11-table-arm.md). An arm link is positioned rather than pushed,
so it transfers no momentum and its impulse is always 0. The arm's tool meeting its load is
a guarded stop, the normal end of a descent, and publishes no event.

## How the simulator picks the tier

The tier is the simulator's verdict, reached with the thresholds below. They are the
simulator's current values, not a contract: every event also carries the measured
`closing_speed` and `impulse`, so a program that wants different limits applies its own to
those two numbers and ignores the tier.

For an incident with a person, who is a 75 kg body:

| Threshold | Value | What it decides |
|---|---|---|
| Near-miss distance | 1.0 m of clearance from the person's body | an approach counts only inside it |
| Near-miss speed | 0.5 m/s toward the person | an approach inside the distance and faster than this, that never touches, is a `near_miss` |
| Near-miss rearm distance | 1.5 m of clearance | the robot must back off this far before another approach can count |
| Strike speed | 1.0 m/s of closing speed | a touch faster than this is a `strike` |
| Strike impulse | 150 N s, summed over the last 3 physics steps of touching | a touch that delivers more is a `strike`, however slow |
| Assessment window | 3 physics steps of touching | a contact is watched this long before it is reported, so the impulse covers the hit and not only its first step |
| Contact rearm time | 1 s | the robot must stay off the person this long before its next touch is a new incident |
| Fault speed | 0.2 m/s toward the person at the contact point | above it `robot_at_fault` is true |

A touch that passes neither strike threshold is a `contact`. The two strike rules are an
"or": a fast tap is a strike on speed alone, and a slow push that pins a person is a strike
on impulse alone. 150 N s is a 2 m/s change of velocity for a 75 kg person, above the
roughly 110 N s a person brings into a bump at walking pace, so only a hit the robot drives,
or a pin, reaches it. A `contact` can escalate to a `strike` once, when a push becomes a pin,
and that is the case in which one incident is two records. A near miss is reported when the
approach ends, and only if it never touched.

For an arm link meeting an object or another robot there is one rule:

| Threshold | Value | What it decides |
|---|---|---|
| Strike speed | 1 m/s, the link point's speed into what it met | above it the event is a `strike`, otherwise a `contact` |
| Collision rearm time | 1 s | a party met again within it belongs to the same episode and sends no new event |

Both arm values are settable per arm, as `strike_speed` and `collision_rearm` on
[Arm configuration](../ch06-services/10-arm-config.md). The person thresholds belong to the
scene and have no service. An arm event has an impulse of 0, so `closing_speed` is the only
number to apply a limit of your own to.

## Two ways to subscribe

| | One robot | Every robot |
|---|---|---|
| Call | `subscribe_safety_events()` on a robot handle | `subscribe_all_safety_events()`, a free function |
| Topic | `vrobots/{sys_id}/z/event/safety` | `vrobots/*/z/event/safety` |
| Needs a robot handle | yes | no; it opens a connection of its own and closes it when the stream ends |
| `elapsed` on an event | seconds since that robot's first state sample | always 0, since there is no one robot to count from |
| Told apart by | not needed | `sys_id` and `robot_name` on each event |

Both return the same stream type. The example uses the swarm-wide form:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex43_safety_events.rs`:

```rust
let events = match subscribe_all_safety_events() {
    Ok(events) => events,
    Err(e) => {
        eprintln!("could not subscribe: [{}] {e}", e.code());
        std::process::exit(1);
    }
};
println!("subscribed to {} for {LISTEN:?}\n", events.key());
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex43_safety_events.cpp`:

```cpp
vrsdk::SafetyEventStream events;
try {
    events = vrsdk::subscribe_all_safety_events();
} catch (const vrsdk::Error& e) {
    std::fprintf(stderr, "could not subscribe: [%d] %s\n", e.code(), e.what());
    return 1;
}
std::printf("subscribed to %s for %gs\n\n", events.key().c_str(), LISTEN_S);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex43_safety_events.py`:

```python
try:
    events = subscribe_all_safety_events()
except VrError as e:
    print(f"could not subscribe: [{e.code}] {e}", file=sys.stderr)
    sys.exit(1)
print(f"subscribed to {events.key} for {LISTEN:g}s\n")
```

{{#endtab }}
{{#endtabs }}

With the example's 30 second listening window, all three print the key they subscribed to:

```text
subscribed to vrobots/*/z/event/safety for 30s
```

> **Gotcha.** Only incidents that happen after the subscription is live arrive. Nothing is
> replayed, and a fresh connection needs a moment of discovery before the simulator's
> publishers reach it, so an incident in that first moment is not seen. Subscribe before
> you start the motion you want to observe.

Because silence cannot tell "nothing happened" from "nobody is publishing", the example
first lists the robots that are publishing a state topic, and ends at once if there are
none.

## Reading the queue

| Method | Returns | Blocks |
|---|---|---|
| `try_next()` | the oldest unread event, or nothing | no |
| `next_timeout(timeout)` | the oldest unread event, or nothing if none arrives in time | up to `timeout` |
| `drain()` | every unread event, oldest first, leaving the queue empty | no |
| `pending` | how many events are waiting | no |

`drain` exists in Rust and Python. In C++ the same read is a loop,
`while (auto e = events.try_next())`. Draining once per iteration is the read for a control
loop; `next_timeout` is the read for a program with nothing else to do. An empty result
from `next_timeout` is not a timeout error in any language: it returns nothing and does not
raise. A timeout of zero does not wait, which makes it the same read as `try_next()`. Each event goes to exactly one reader, so read a stream from one thread.

The example waits for one event at a time, and reports a quiet stretch as what it is:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex43_safety_events.rs`:

```rust
let Some(event) = events.next_timeout(wait) else {
    println!(
        "t={:5.1}s  quiet: no incident, which is the normal state",
        start.elapsed().as_secs_f64()
    );
    continue;
};
print_event(start.elapsed(), &event);
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex43_safety_events.cpp`:

```cpp
const std::optional<vrsdk::SafetyEvent> event = events.next_timeout(wait);
if (!event) {
    std::printf("t=%5.1fs  quiet: no incident, which is the normal state\n",
                seconds_since(start));
    continue;
}
print_event(seconds_since(start), *event);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex43_safety_events.py`:

```python
event = events.next_timeout(wait)
if event is None:
    print(f"t={time.monotonic() - start:5.1f}s  quiet: no incident, which is the normal state")
    continue
print_event(time.monotonic() - start, event)
```

{{#endtab }}
{{#endtabs }}

A run during which a link of the Warehouse rail arm is driven into the building prints
quiet lines until the incident, then two lines for it:

```text
t=  5.0s  quiet: no incident, which is the normal state
t= 10.0s  quiet: no incident, which is the normal state
t= 15.0s  quiet: no incident, which is the normal state
t= 19.1s  robot 2 'kuka_kr210_rail' event #0: strike with object 'Building_Collider'
          closing +2.82 m/s, impulse 0.0 N*s, robot at fault: true, at (+15.90, +35.00, +0.17) m in "fru"
t= 24.1s  quiet: no incident, which is the normal state
t= 29.1s  quiet: no incident, which is the normal state
t= 30.0s  quiet: no incident, which is the normal state
```

That is the Rust output. Python prints `True` and puts the frame id in single quotes. The
event is the arm row of the table above: the other party is an object, the impulse is 0 and
the robot is at fault. A run in which nothing touches anything prints quiet lines only, and
that is an equally normal run.

## Capacity and what gets lost

The queue holds 1024 unread events: `SAFETY_EVENT_QUEUE_CAPACITY` in Rust and Python,
`VRSDK_SAFETY_EVENT_QUEUE_CAPACITY` in C++. When it is full, the **oldest** unread event is
evicted to make room for the new one and counted. Incidents are rare, so a program that
reads once per loop iteration never comes near the limit.

On a topic with no rate, a lost event looks exactly like no event. The counters from
`stats` are the only way to know the record is complete.

| Field | Notes |
|---|---|
| `received` | events that arrived and decoded, read or not |
| `dropped` | events evicted unread from a full queue. Non-zero means nothing read the stream for a long time |
| `seq_gaps` | times a robot's `seq` jumped forward by more than one: events lost between the simulator and this process. Tracked per robot, so robots interleaving on the swarm-wide stream are not gaps |
| `missed_events` | total events those jumps imply missing |
| `decode_errors` | payloads that did not decode; counted, never fatal |

`dropped` is what this process discarded, and `missed_events` is what never reached it. The
example closes with both:

```text
1 event(s) in 30s: 0 near miss, 0 contact, 1 strike, 0 unspecified
dropped 0 (queue full, nobody reading), missed 0 in 0 seq gap(s) (lost on the way), decode errors 0, still queued 0
```

A silent run closes with `0 event(s) in 30s: 0 near miss, 0 contact, 0 strike, 0 unspecified`
and the same second line.

## Every field of an event

| Field | Type | Units | Notes |
|---|---|---|---|
| `sys_id` | `u32` | | the robot the incident belongs to. Whose incident it is, never who was hit |
| `robot_name` | `String` | | that robot's name |
| `seq` | `u64` | | that robot's event counter: 0, 1, 2 and so on |
| `t_ns` | `i64` | ns since the unix epoch | when the simulator reported it, on the clock of `State::t_ns` |
| `elapsed` | `f64` | s | since the robot's first state sample; 0 on the swarm-wide stream |
| `tier` | `SafetyTier` | | how bad it was, below |
| `other_name` | `String` | | the other party's name in the scene; for another robot, that robot's name |
| `other_kind` | `SafetyOtherKind` | | `person`, `robot` or `object`; `unspecified` for a kind this SDK does not know |
| `closing_speed` | `f64` | m/s | the robot's speed toward the other party at the contact point, or at its nearest point for a near miss. Positive is closing. The peak over the incident; a person walking into a parked robot reads about 0 |
| `impulse` | `f64` | N s | contact impulse. 0 for a near miss, and whenever the robot part involved is an arm link |
| `robot_at_fault` | `bool` | | the robot was the one moving in. A person walking into a stationary robot is not the robot's fault |
| `position` | three of `f64` | m | where it happened: the contact point, or the robot's nearest point for a near miss. A world position, expressed in the robot's frame |
| `coord_frame_id` | `String` | | the frame `position` is expressed in: the robot's own, so on the swarm-wide stream it can differ from one event to the next |
| `axis_convention` | `Axes` | | the enum tag beside the id; the id is authoritative |

`position` follows the convention of the robot's own position in its state: a place in the
world, with its axes named by the robot's frame. It is not an offset from the robot.

| `tier` | Meaning |
|---|---|
| `near_miss` | passed close while moving toward the other party; never touched |
| `contact` | touched, below both strike thresholds |
| `strike` | hit: a closing speed above 1 m/s, or with a person an impulse above 150 N s |
| `unspecified` | no tier was set, or a tier newer than this SDK |

**The tiers are deliberately not ordered.** Compare for equality, never with `>=`:
`unspecified` also stands for a tier this SDK has not heard of, and an ordered comparison
would rank it below a near miss. Read it as unknown severity, never as harmless. Rust and
C++ expose the tier and the kind as enums with a name, and Python as integer codes
(`SafetyTier.STRIKE`) with `tier_name` and `other_kind_name` beside them.

## On the Community plan

On the Community plan only the focused robot is on the wire at all, so only the focused
robot's events arrive, on either form of subscription. An incident involving another robot
is discarded inside the simulator. It is not queued, and it is not replayed when the focus
moves to that robot. Safety events themselves are not a Pro feature: the limit is the one
that applies to every topic.

**Next:** [Services and configuration](../ch06-services/00-intro.md)

**See also:** [Forklift](../ch07-robots/08-forklift.md), [More than one robot](../ch08-tooling/06-multi-robot.md), [Stream health](../ch03-reading-state/08-health.md)
