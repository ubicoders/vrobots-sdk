# Truth, measured and believed

The three epistemic categories a snapshot keeps apart, and the two differences between them that are worth measuring.

## Why the schema separates them

A simulator can tell you exactly where a robot is. A robot cannot know that about
itself. If both numbers live in the same struct under similar names, a control loop
that accidentally reads the exact one works beautifully in simulation and fails on
hardware, and nothing in the code looks wrong.

The message schema prevents that by construction, and the SDK's `State` mirrors it:
truth, measurement and belief sit in separate blocks, and no field appears in two of
them. Choosing a block is therefore a deliberate act. If your controller reads
`kin.lin_pos`, you have decided to use ground truth, and that decision is visible in
the source.

| Category | Blocks | Available on a real robot | What it is |
|---|---|---|---|
| **truth** | `kin`, `wrench`, `env` | no | simulator-exact values, the physics engine's own numbers |
| **measured** | `sensors` | yes | the noisy, robot-observable view of the same instant |
| **believed** | `estimate` | yes | what the robot's own filter has concluded |
| neither | `actuator` | yes | command in, realised motion out |

`actuator` is listed as none of the three on purpose. It is not a measurement of the
world and not a belief about it: it is the echo of what you commanded beside what the
device did. It has its own page, [Actuators](05-actuator.md).

## The two differences are the experiment

Because the blocks are published from the same instant, characterising a sensor or an
estimator is a subtraction between two fields of one snapshot. Nothing has to be
inferred, and no separate ground-truth log has to be aligned in time.

The SDK source states two of these outright:

| Difference | What it is |
|---|---|
| `estimate.kin - kin` | the estimator error |
| `sensors.barometer.pressure - env.air_pressure` | the barometer's error |

The same construction extends to every other device. `sensors.gyroscope.angular_velocity - kin.ang_vel`
is the gyro's noise realisation on that sample, because both are body-frame angular
rates in rad/s. `sensors.gnss.geo_point` against `env.geo_point` is the receiver's
position error.

Two cautions apply to every such diff. Both sides must be in the same frame, which is
not automatic when a device overrides the robot's `axis_convention` (see
[Sensors](03-sensors.md)). And the measured side must be fresh: a sensor slower than
the state stream republishes its previous reading, so a diff taken every sample
measures the same noise realisation several times over. Compare timestamps first, as
[Timestamps and sequence numbers](06-timestamps.md) describes.

## Reading the believed block

`estimate` has the same `Kinematics` shape as the truth block, plus its own clock, its
own frame, and a validity flag.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex10_sensors_tour.rs`:

```rust
// -- believed -------------------------------------------------------
let e = &s.estimate;
println!(
    "BELIEVED  estimate  {}  frame={:?}",
    stamp(e.valid, e.timestamp),
    e.coord_frame_id
);
println!(
    "  lin_pos   {} m       (estimate.kin - kin IS the error)",
    v3(e.kin.lin_pos)
);
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex10_sensors_tour.cpp`:

```cpp
// -- believed -----------------------------------------------------
std::printf("BELIEVED  estimate  ");
stamp(r.estimate.valid, r.estimate.timestamp);
// The fixed C char arrays are NUL-terminated; the precision bounds
// the read even if a future field ever fills the buffer exactly.
std::printf("  frame=\"%.*s\"\n",
            static_cast<int>(sizeof r.estimate.coord_frame_id - 1),
            r.estimate.coord_frame_id);
v3("lin_pos", r.estimate.kin.lin_pos, "m", "(estimate.kin - kin IS the error)");
v3("lin_vel", r.estimate.kin.lin_vel, "m/s", "");
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex10_sensors_tour.py`:

```python
# -- believed -------------------------------------------------------
e = s.estimate
print(
    f"BELIEVED  estimate  {stamp(e.valid, e.timestamp)}  "
    f"frame={e.coord_frame_id!r}"
)
print(f"  lin_pos   {v3(e.kin.lin_pos)} m       (estimate.kin - kin IS the error)")
print(f"  lin_vel   {v3(e.kin.lin_vel)} m/s")
```

{{#endtab }}
{{#endtabs }}

C++ reaches the block through `s.raw`, the copied C struct, where the other two have named
fields on the snapshot: `r.estimate.kin.lin_pos` against `s.estimate.kin.lin_pos`. The
fields, the units and the subtraction that gives the estimator error are identical.

Today that always prints an invalid stamp, zeroed vectors and an empty frame id:

<!-- VERIFY: printout reconstructed from the format strings; the values are illustrative, not captured from a run. -->

```text
BELIEVED  estimate  [INVALID t=0.000]  frame=""
  lin_pos   (  +0.000,  +0.000,  +0.000) m       (estimate.kin - kin IS the error)
```

It is not a decode failure and not a dropped block: a missing nested table decodes to its
`Default`, which is all zeros, `false` and empty strings. The simulator runs no estimator
and omits the field on purpose, for the same reason this page opens with. An estimate that
silently mirrors truth would make `estimate.kin - kin` a tautology, so the block is absent
rather than filled from the truth block.

## Publishing the belief yourself

The believed block is therefore yours to supply, and it travels on its own topic rather
than in the snapshot. `publish_estimate` puts a `swarmbotix.states.EstimateState` on
`vrobots/{sys_id}/z/estimate`, where the fixed wing reads it under `FW_EST_OBSERVER`.
Read `sensors` off `z/state`, run your filter, publish the result, and the subtraction
against `kin` measures a real estimator error. [Publishing
estimates](../ch04-commands/09-publishing-estimates.md) is that loop end to end.

> **Gotcha.** `estimate.valid` is false until the filter converges, and an unconverged
> estimate that silently mirrors truth is the classic trap. A filter initialised from
> the simulator's own state reads as a perfect estimator for as long as nothing
> disturbs it, and your error metric reads zero because you are subtracting a number
> from itself. Gate every use of `estimate` on `estimate.valid`, and treat an error of
> exactly zero as evidence of a bug rather than of quality.

## Which block should your code read

| You are writing | Read | Because |
|---|---|---|
| a controller you intend to port to hardware | `sensors`, or `estimate` when valid | these are the only blocks a physical robot has |
| an estimator or filter under test | `sensors` in, `kin` only to score it | reading `kin` inside the filter invalidates the test |
| a sensor characterisation | both sides of one of the diffs above | the pair is published from the same instant |
| a plotting or debugging tool | anything | there is no port to fail |

> **Note.** Nothing in the SDK stops a controller reading `kin`. Early on that is
> often the right choice, because it separates a controller bug from a sensing
> problem. Make it a decision you can find later, not a default.

**Next:** [Kinematics](02-kinematics.md)

**See also:** [Publishing estimates](../ch04-commands/09-publishing-estimates.md), [Frames, axes and units](../ch02-concepts/07-frames-and-units.md), [A tour of the whole snapshot](09-sensors-tour.md), [Sensor noise](../ch06-services/03-sensor-config.md)
