# Arm configuration

One service for both arms: the protective stop, the contact thresholds and the tool each arm
carries.

## One request, two arms

`configure_arm` sends `srv/arm`, and both the rail arm and the table arm serve it. Any other
robot type has no responder for it, which returns
[`VrError::NoResponder`](../appendix-c-errors.md) after the service timeout.

The request has three groups of fields. Both arms read the shared group. The rail arm
carries the vacuum and reads the `suction_*` fields; the table arm carries the gripper and
reads the `gripper_*` fields.

> **Gotcha.** Each arm **ignores the other arm's tool fields**. The simulator notes it in its
> own log, applies the rest of the request and acks `ok`, so `gripper_speed` sent to the rail
> arm changes nothing and looks like success. The SDK logs a warning when the handle's robot
> type tells it the fields will be ignored, and still sends the request.

The service follows the
[rules every config service shares](00-intro.md#rules-every-config-service-shares): the ack
is a receipt, out-of-range values are healed and acked `ok`, and nothing reads a value back.
Only the fields you set are written. Everything set here survives `reset()`, and nothing
survives a simulator restart.

## Shared fields

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `protective_stop` | bool | | on | refuse joint steps that would drive the arm into anything; off lets the links pass through the scene |
| `self_collision` | bool | | off on the rail arm, on on the table arm | also refuse steps that would drive the arm into itself; acts only with the protective stop on |
| `contact_tolerance` | float | m | 0.005 | how far a link may sink into something before the step is refused; the tool stops at first touch; `< 0` becomes 0 |
| `strike_speed` | float | m/s | 1 | link closing speed above which a collision event is a strike and not a contact; `<= 0` reverts to the default |
| `collision_rearm` | float | s | 1 | time clear of an obstacle before meeting it again is a new collision event; `< 0` becomes 0 |
| `fallback_speed_limits` | list of float | m/s for the rail, rad/s for the other joints | see below | one entry per joint in actuator order, 7 on the rail arm and 6 on the table arm; an entry `<= 0` means 1 m/s or 90 deg/s |

The protective stop and the tools read their settings on every step, so a change takes
effect from the next one. What the stop does, and how the CONTACT entry reports it, is in
[Arm joints and tools](../ch04-commands/13-arms.md#the-protective-stop-and-the-contact-reading).
`strike_speed` and `collision_rearm` shape the
[safety events](../ch10-events/00-safety-events.md) an arm publishes when it meets an object
or another robot.

## The speed fallbacks have no effect on the shipped arms

A joint runs at its own speed limit. A fallback backs only a joint that reports none, and
every joint of both shipped arms reports one. So `fallback_speed_limits` changes no speed on
either arm today, whatever you send, and the ack is `ok` regardless.

The defaults repeat the speed limits the joints already report:

| Arm | Default `fallback_speed_limits` |
|---|---|
| Rail arm | 1.2 m/s, then 105 / 94 / 100 / 136 / 129 / 206 deg/s |
| Table arm | 120 / 120 / 180 / 180 / 180 / 180 deg/s |

The field is all or nothing. It must hold exactly one finite entry per joint, or the
simulator refuses the whole field while the rest of the request still applies. The SDK
checks the count against the handle's arm type before sending.

> **Not yet.** `fallback_speed_limits` is on the wire and accepted, and no joint of the rail
> arm or the table arm uses it. The joint speeds listed on the
> [rail arm](../ch07-robots/10-rail-arm.md) and [table arm](../ch07-robots/11-table-arm.md)
> pages cannot be changed through this service.

## Vacuum fields, rail arm only

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `suction_capacity` | float | N | 400 | suction force; a held load that needs more, mass times the magnitude of its acceleration minus gravity, drops; `< 0` becomes 0 |
| `suction_reach` | float | m | 0.01 | the gap between the cup and a surface within which it seals; `< 0` becomes 0 |
| `suction_max_tilt` | float | rad | 0.262 | the largest angle between the cup axis and the surface normal that still seals, 15 degrees; clamped to [0, pi/2] |
| `suction_accel_window` | float | s | 0.5 | the span the held load's acceleration is averaged over; `<= 0` reverts to the default; capped at 5 |

A capacity of 400 N holds about 40 kg at rest. The joints have no acceleration ramp, so a
fast start can tear a heavy load off even when the pad holds it at rest.

## Gripper fields, table arm only

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `gripper_speed` | float | m/s of opening | 0.15 | finger speed; `<= 0` reverts to the default; clamped to [0.02, 0.15] |
| `gripper_max_payload` | float | kg | 5 | the heaviest body the fingers hold; a body already held stays held; `<= 0` reverts to the default |

At the default speed the whole 0.085 m stroke takes under 0.6 seconds. At the slowest
setting it takes a little over 4 seconds, so widen any timeout you put around a grip.

## Making the call

No example program calls `configure_arm`. The call takes the same shape as every other
configuration service: name the fields you want and leave the rest out.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

Written against the `vrobots-sdk` crate:

```rust
// Table arm: hold heavier objects, close the fingers more slowly.
table_arm.configure_arm(
    &ArmConfig::default()
        .with_gripper_max_payload(8.0) // kg
        .with_gripper_speed(0.05), // m/s of opening
)?;

// Rail arm: tolerate 2 mm of contact, hold up to about 60 kg.
rail_arm.configure_arm(
    &ArmConfig::default()
        .with_contact_tolerance(0.002) // m
        .with_suction_capacity(600.0), // N
)?;
```

{{#endtab }}
{{#tab name="C++" }}

Written from `include/vrobots_sdk.hpp`:

```cpp
// Table arm: hold heavier objects, close the fingers more slowly.
auto grip = vrsdk::arm_config();
grip.has_gripper_max_payload = true;
grip.gripper_max_payload = 8.0;  // kg
grip.has_gripper_speed = true;
grip.gripper_speed = 0.05;       // m/s of opening
table_arm.configure_arm(grip);

// Rail arm: tolerate 2 mm of contact, hold up to about 60 kg.
auto pad = vrsdk::arm_config();
pad.has_contact_tolerance = true;
pad.contact_tolerance = 0.002;   // m
pad.has_suction_capacity = true;
pad.suction_capacity = 600.0;    // N
rail_arm.configure_arm(pad);
```

{{#endtab }}
{{#tab name="Python" }}

Written from `vrsdk/_vrsdk.pyi`:

```python
# Table arm: hold heavier objects, close the fingers more slowly.
table_arm.configure_arm(gripper_max_payload=8.0, gripper_speed=0.05)

# Rail arm: tolerate 2 mm of contact, hold up to about 60 kg.
rail_arm.configure_arm(contact_tolerance=0.002, suction_capacity=600.0)
```

{{#endtab }}
{{#endtabs }}

`table_arm` and `rail_arm` are two connected `VirtualRobot` handles, one per arm. Neither
call prints anything. In C++ each value needs its `has_*` flag beside it, and
`vrsdk::set_fallback_speed_limits(config, limits)` fills the per-joint list together with
its flag and count. In Rust the same list goes through
`ArmConfig::with_fallback_speed_limits(&[f64])`.

## Confirming a change

The ack says the arm heard the request. Confirm with the behaviour the field controls:

| Changed | Measure |
|---|---|
| `protective_stop` off | a setpoint into an obstacle now arrives instead of stopping short |
| `gripper_speed` | the time `gripper_opening_m` takes to cover a known distance |
| `gripper_max_payload` | whether `holding` turns true on an object of known mass |
| `suction_capacity` | whether `held_mass_kg` stays non-zero through a move that used to drop the load |

Turning the protective stop off is for experiments that need the arm to pass through
geometry. With it off, nothing stops a link from being driven through the scene.

## What the SDK refuses

`configure_arm` refuses these before anything reaches the wire, with
`VrError::InvalidArgument`: a request with nothing set, a value that is not finite, and a
`fallback_speed_limits` list that is empty or, on a handle connected as a rail arm or a
table arm, not that arm's joint count.

**Next:** [Turret configuration](11-turret-config.md)

**See also:** [Arm joints and tools](../ch04-commands/13-arms.md), [Rail arm](../ch07-robots/10-rail-arm.md), [Table arm](../ch07-robots/11-table-arm.md), [Safety events](../ch10-events/00-safety-events.md)
