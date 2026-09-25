# A tour of the whole snapshot

One program that prints truth, measurement and belief side by side once a second, and what to read in the differences between them.

```sh
cargo run -p vrobots-examples --bin ex10_sensors_tour
./target/cpp-build/ex10_sensors_tour
python examples/python/ex10_sensors_tour.py
```

## What this example is for

Every other example in the book reads two or three fields. This one walks the entire
`State`, block by block, and prints it at 1 Hz because each iteration is a page of text.
It is the capstone of the chapter for a specific reason: the value of the snapshot is
not in any single field but in the fact that truth, measurement and belief for the same
instant are published together, so characterising anything is a subtraction rather than
an inference.

Read the output once with the simulator idle and once with the robot moving, and most of
this chapter becomes concrete.

## The header, and the two helpers

Each iteration opens with the snapshot's identity: name, id, sequence number, elapsed
time, schema version, and the frame every vector below it is expressed in.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex10_sensors_tour.rs`:

```rust
println!(
    "\n=== {} sys_id={} seq={} t={:.3}s schema={} frame={:?} ({}) ===",
    s.name,
    s.sys_id,
    s.seq,
    s.elapsed,
    s.schema_version,
    s.coord_frame_id,
    s.axis_convention.name()
);
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex10_sensors_tour.cpp`:

```cpp
const vrsdk::State s = robot.states();
const vrsdk_state_t& r = s.raw;

std::printf("\n=== %s sys_id=%u seq=%llu t=%.3fs schema=%u frame=\"%s\" ===\n",
            s.name.c_str(), s.sys_id, static_cast<unsigned long long>(s.seq),
            s.elapsed, r.schema_version, s.coord_frame_id.c_str());
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex10_sensors_tour.py`:

```python
s = mr.states

print(
    f"\n=== {s.name} sys_id={s.sys_id} seq={s.seq} t={s.elapsed:.3f}s "
    f"schema={s.schema_version} frame={s.coord_frame_id!r} "
    f"({s.axis_convention_name}) ==="
)
```

{{#endtab }}
{{#endtabs }}

The C++ header line omits the convention name, because the C surface exposes
`axis_convention` as a bare `int32_t` with no name lookup beside it. Rust spells the lookup
`axis_convention.name()` and Python spells it `axis_convention_name`.

Printing `coord_frame_id` beside `axis_convention.name()` is not redundant. The string
is authoritative and is the only way to name a frame registered at runtime; the
convention is the enum tag beside it, and returns `""` for anything outside the three
built-in conventions.

Two small helpers do all the formatting, and the second one is the interesting one.
Also from `examples/rust/src/bin/ex10_sensors_tour.rs`:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
/// A 3-vector, aligned so a column of them reads as a column.
fn v3(v: [f64; 3]) -> String {
    format!("({:+8.3},{:+8.3},{:+8.3})", v[0], v[1], v[2])
}

/// A sensor's own validity and clock -- the two fields that reveal its rate.
fn stamp(valid: bool, timestamp: f64) -> String {
    format!(
        "[{} t={timestamp:.3}]",
        if valid { "valid" } else { "INVALID" }
    )
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex10_sensors_tour.cpp`:

```cpp
/// A 3-vector, aligned so a column of them reads as a column.
static void v3(const char* label, const double* v, const char* unit, const char* note) {
    std::printf("  %-9s (%+8.3f,%+8.3f,%+8.3f) %-8s %s\n", label, v[0], v[1], v[2], unit, note);
}

/// A sensor's own validity and clock -- the two fields that reveal its rate.
static void stamp(bool valid, double timestamp) {
    std::printf("[%s t=%.3f]", valid ? "valid" : "INVALID", timestamp);
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex10_sensors_tour.py`:

```python
def v3(v: Sequence[float]) -> str:
    """A 3-vector, aligned so a column of them reads as a column."""
    return "(" + ",".join(f"{c:+8.3f}" for c in v) + ")"


def stamp(valid: bool, timestamp: float) -> str:
    """A sensor's own validity and clock -- the two fields that reveal its rate."""
    return f"[{'valid' if valid else 'INVALID'} t={timestamp:.3f}]"
```

{{#endtab }}
{{#endtabs }}

The C++ helpers print rather than return, because building strings with `printf` formatting
would mean a scratch buffer per call. That is why the C++ output puts the label and unit
inside `v3` where the other two paste them at the call site.

Neither helper prints anything on its own: `v3` renders one vector as
`(  +0.031,  +0.852,  -1.204)` and `stamp` renders one device's validity and clock as
`[valid t=1770000000.960]`. `stamp` is applied to every device, which is what makes the
differing sensor rates visible in a static printout: the bracketed time next to the GNSS
row sits still for about five iterations of a 25 Hz loop while the header's `t` advances
on every one.

## One iteration

The loop body prints five labelled sections in a fixed order: `TRUTH`, `MEASURED`,
`BELIEVED`, `WORLD` and `ACTUATORS`. Each section's code is quoted on its own page of
this chapter, and the whole body is one `println!` per line with no logic between them.

<!-- VERIFY: printout reconstructed from the example's format strings; the values are illustrative, not captured from a run. -->

```text
=== multirotor sys_id=1 seq=725 t=29.000s schema=1 frame="frd" (frd) ===
TRUTH  kinematics
  lin_pos   (  +0.031,  +0.852,  -1.204) m       (world)
  quat      [+0.000,+0.000,+0.000,+1.000] (world, xyzw)
  lin_vel   (  +0.000,  +0.000,  +0.000) m/s     (body)
  ang_vel   (  +0.000,  +0.000,  +0.000) rad/s   (body -- what a gyro measures)
  lin_acc   (  +0.000,  +0.000,  +0.000) m/s^2   (body)
  ang_acc   (  +0.000,  +0.000,  +0.000) rad/s^2 (body)
  wrench    F=(  +0.000,  +0.000,  +0.000) N  T=(  +0.000,  +0.000,  +0.000) N.m
MEASURED  sensors
  accel     (  +0.012,  -0.004,  -9.803) m/s^2  [valid t=1770000000.960]   [specific force: +1 g at rest]
  gyro      (  +0.001,  -0.002,  +0.000) rad/s  [valid t=1770000000.960]
  mag       ( +22.100,  +1.400, +42.300) gauss  [valid t=1770000000.960]
  baro      101318.4 Pa  alt=0.58 m (qnh 101325.0 hPa)  [valid t=1770000000.940]
  gnss      lat=37.400000 lon=-122.100000 alt=12.34 m  vel=(  +0.000,  +0.000,  +0.000) m/s (NED)
            fix=3 eph=1.20 epv=1.80 m  [valid t=1770000000.800]   [slowest device, ~5 Hz]
  flow      (  +0.000,  +0.000,  +0.000) m/s  [INVALID t=0.000]   [optional; mount it via srv/sensors]
BELIEVED  estimate  [INVALID t=0.000]  frame=""
  lin_pos   (  +0.000,  +0.000,  +0.000) m       (estimate.kin - kin IS the error)
  lin_vel   (  +0.000,  +0.000,  +0.000) m/s
WORLD  environment
  gravity   (  +0.000,  +0.000,  +9.807) m/s^2   air 101325.0 Pa 1.225 kg/m^3 15.0 C
  agl       0.00 m    home lat=37.400000 lon=-122.100000   [agl is hard-coded 0 in sim v3.0.0 -- use -lin_pos[2]]
ACTUATORS  command in, motion out
  pwm        [] us      (echo of the last command)
  normalized []
  measured   []   (rotor rad/s -- what the devices did)
```

## Reading the differences

The printout is arranged so that the pairs worth subtracting sit near each other.

| Compare | Against | Gives |
|---|---|---|
| `MEASURED gyro` | `TRUTH ang_vel` | the gyro's noise realisation on that sample |
| `MEASURED baro` pressure | `WORLD` air pressure | the barometer's error in Pa |
| `MEASURED gnss` position | `WORLD` home position | the receiver's position error |
| `BELIEVED lin_pos` | `TRUTH lin_pos` | the estimator error, once `valid` is true |
| `ACTUATORS measured` | `ACTUATORS normalized` | how far the device is from its command |

Four things the printout makes visible that a field table cannot.

**The accelerometer disagrees with `lin_acc` on purpose.** It reads specific force, so a
robot at rest reports about 1 g and a robot in free fall reports zero, while
`kin.lin_acc` does the opposite. Subtracting gravity is your job and it needs an
attitude.

**A sensor's stamp moves at the sensor's rate.** Watch the GNSS row's bracketed time sit
still while the header's `t` advances. Nothing else in the snapshot reveals that, and
code that treats every sample's GNSS reading as new will differentiate a constant four
times out of five.

**An invalid block is not a failure.** The test scene runs no estimator, so `estimate`
arrives `valid=false`, zero-filled, with an empty frame id. Optical flow is optional and
arrives the same way when it is not mounted. That is what "not present" looks like on
the wire; a missing nested table decodes to its `Default` rather than raising.

**The frame is on every line for a reason.** `coord_frame_id` is the robot's, not yours.
In a scene where the multirotor publishes `frd`, `lin_pos[2]` counts downwards and
altitude is its negation, which is the substitute the example uses for `env.agl`.

> **Gotcha.** Everything in `TRUTH` and `WORLD` is unavailable on a physical vehicle.
> The tour is a debugging and characterisation tool, and reading it is the fastest way
> to understand a robot; taking a control decision from those two sections is how a
> program that works in simulation stops working anywhere else.

**Next:** [Sending commands](../ch04-commands/00-intro.md)

**See also:** [Truth, measured and believed](01-truth-measured-believed.md), [Sensors](03-sensors.md), [Supported virtual robots](../ch07-robots/00-intro.md)
