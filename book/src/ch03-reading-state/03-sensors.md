# Sensors

Every device in the measured block, its fields, its units, and the two common fields that reveal the rate it actually runs at.

## What is in the block, and what is deliberately not

`State::sensors` is everything the robot can observe about itself. There are six
devices: accelerometer, gyroscope, magnetometer, barometer, GNSS receiver and optical
flow. Each is a separate struct with its own fields.

Two absences are deliberate. There is no `imu` grouping, because the accelerometer and
the gyroscope are separate devices with separate clocks and separate noise models, and
bundling them invites code that assumes they updated together. There is no attitude
field anywhere in `sensors`, because attitude is never measured, only fused: if you
want the robot's belief about its orientation, that is `estimate`, and if you want the
simulator's, that is `kin.quat`.

Every device carries the same two fields, and they matter more than the readings do.

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `timestamp` | `f64` | s since the unix epoch | `0.0` | the **sensor's own** capture clock, not the snapshot header's |
| `valid` | `bool` | | `false` | false until the device has produced a usable reading |

`valid = false` has three distinct causes that look identical from the client: the
device is not mounted on this robot, it is mounted but has not produced a first
reading, or it has lost its fix. Treat it as "do not use this number" and check the
robot's configured sensor set if you expected otherwise.

## The devices

### Accelerometer

| Field | Type | Units | Frame | Default | Notes |
|---|---|---|---|---|---|
| `linear_acceleration` | `[f64; 3]` | m/s² | body | `[0.0; 3]` | **specific force**, so +1 g at rest and 0 in free fall |
| `axis_convention` | `Axes` | | | `UNSPECIFIED` | mounting convention, when it differs from the robot's |
| `coord_frame_id` | `String` | | | `""` | mounting frame id, when it differs from the robot's |

Specific force is the field people misread. It is not `kin.lin_acc`: subtracting
gravity is your job, and doing it needs an attitude you do not have from this device
alone.

### Gyroscope

| Field | Type | Units | Frame | Default | Notes |
|---|---|---|---|---|---|
| `angular_velocity` | `[f64; 3]` | rad/s | body | `[0.0; 3]` | body rates, directly comparable to `kin.ang_vel` |
| `axis_convention` | `Axes` | | | `UNSPECIFIED` | mounting override |
| `coord_frame_id` | `String` | | | `""` | mounting override |

### Magnetometer

| Field | Type | Units | Frame | Default | Notes |
|---|---|---|---|---|---|
| `magnetic_field` | `[f64; 3]` | see below | body | `[0.0; 3]` | the sources disagree on the unit |
| `axis_convention` | `Axes` | | | `UNSPECIFIED` | mounting override |
| `coord_frame_id` | `String` | | | `""` | mounting override |

<!-- VERIFY: magnetometer unit. `state.rs` documents no unit for `magnetic_field`. `ex10_sensors_tour.rs` prints the field labelled "gauss" and states that magnetic field is in gauss, not tesla. `config.rs` documents the magnetometer *noise* parameter in tesla. The two are only consistent if the noise setting and the reading use different units, so this needs a live check. -->

> **Note.** The unit of `magnetic_field` is not documented in `state.rs`. The tour
> example labels its printout gauss and says so explicitly; the sensor-noise service
> documents the magnetometer's noise parameter in tesla. Until this is checked against
> a running simulator, do not assume the reading and the noise setting share a unit
> (1 T is 10 000 G).

### Barometer

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `pressure` | `f64` | Pa | `0.0` | static pressure; diff against `env.air_pressure` for the error |
| `altitude` | `f64` | m | `0.0` | pressure altitude computed against `qnh`, so it drifts with the weather |
| `qnh` | `f64` | Pa | `0.0` | reference sea-level pressure used for `altitude` |

The barometer is the one device with no `axis_convention` and no `coord_frame_id`,
because it is a scalar sensor with nothing to orient.

### GNSS

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `geo_point` | `GeoPoint` | deg, deg, m | zeroed | reported geodetic position |
| `velocity` | `[f64; 3]` | m/s | `[0.0; 3]` | reported velocity |
| `eph` | `f64` | m | `0.0` | horizontal position accuracy estimate |
| `epv` | `f64` | m | `0.0` | vertical position accuracy estimate |
| `fix_type` | `u32` | | `0` | fix quality, receiver-defined |
| `axis_convention` | `Axes` | | `UNSPECIFIED` | mounting override |
| `coord_frame_id` | `String` | | `""` | mounting override |

`GeoPoint` is `latitude` and `longitude` in degrees and `altitude` in metres, and the
same struct appears in `env.geo_point` as the true position.

> **Gotcha.** The receiver runs at roughly 5 Hz against a 25 Hz state stream, so the
> same fix is republished in about five consecutive snapshots. Presence is not
> freshness. To detect an update boundary, compare `gnss.timestamp` with the value you
> saw last, and only then treat the reading as new. Code that differentiates GNSS
> position once per state sample instead of once per fix produces a velocity that is
> zero four samples out of five and then spikes.

### Optical flow

| Field | Type | Units | Frame | Default | Notes |
|---|---|---|---|---|---|
| `velocity` | `[f64; 3]` | m/s | body | `[0.0; 3]` | estimated velocity from a downward-looking sensor |
| `axis_convention` | `Axes` | | | `UNSPECIFIED` | mounting override |
| `coord_frame_id` | `String` | | | `""` | mounting override |

Optical flow is optional and deliberately a poor sensor: `valid` goes false over
featureless ground. Robots mount it only when asked, so `valid = false` on a default
robot usually means the device is not fitted rather than that it failed.

## The per-device frame override

Five of the six devices carry their own `axis_convention` and `coord_frame_id`. They
exist for one reason: a device whose mounting frame differs from the robot's, for
example an IMU rotated in its bracket or a receiver quoting velocity in NED while the
body publishes `frd`. When they are set, they win for that device's vectors only.

The practical rule is that a diff between a sensor vector and a truth vector is only
meaningful once both are in the same frame. Read the device's `coord_frame_id` first;
when it is empty, the device is in the robot's frame from the snapshot header.

## Reading the measured block

The tour example prints each device with its reading, its validity and its own clock
side by side, which is what makes the differing rates visible.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex10_sensors_tour.rs`:

```rust
println!(
    "  gnss      lat={:.6} lon={:.6} alt={:.2} m  vel={} m/s (NED)",
    n.gnss.geo_point.latitude,
    n.gnss.geo_point.longitude,
    n.gnss.geo_point.altitude,
    v3(n.gnss.velocity)
);
println!(
    "            fix={} eph={:.2} epv={:.2} m  {}   [slowest device, ~5 Hz]",
    n.gnss.fix_type,
    n.gnss.eph,
    n.gnss.epv,
    stamp(n.gnss.valid, n.gnss.timestamp)
);
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex10_sensors_tour.cpp`:

```cpp
std::printf("  gnss      lat=%.6f lon=%.6f alt=%.2f m  vel=(%+.3f,%+.3f,%+.3f) m/s (NED)\n",
            n.gnss.geo_point.latitude, n.gnss.geo_point.longitude,
            n.gnss.geo_point.altitude, n.gnss.velocity[0], n.gnss.velocity[1],
            n.gnss.velocity[2]);
std::printf("            fix=%u eph=%.2f epv=%.2f m  ", n.gnss.fix_type, n.gnss.eph,
            n.gnss.epv);
stamp(n.gnss.valid, n.gnss.timestamp);
std::printf("   [slowest device, ~5 Hz]\n");
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex10_sensors_tour.py`:

```python
g = n.gnss.geo_point
print(
    f"  gnss      lat={g.latitude:.6f} lon={g.longitude:.6f} alt={g.altitude:.2f} m  "
    f"vel={v3(n.gnss.velocity)} m/s (NED)"
)
print(
    f"            fix={n.gnss.fix_type} eph={n.gnss.eph:.2f} epv={n.gnss.epv:.2f} m  "
    f"{stamp(n.gnss.valid, n.gnss.timestamp)}   [slowest device, ~5 Hz]"
)
```

{{#endtab }}
{{#endtabs }}

The device path is the same in all three: `sensors.gnss.geo_point.latitude`, reached through
`s.raw.sensors` in C++ and `s.sensors` in the other two. So is the `valid` and `timestamp`
pair every device carries, which is what the shared `stamp` helper prints.

Run the tour at 1 Hz and the GNSS stamp advances once per printed block; run it at the
state rate and the same stamp repeats:

<!-- VERIFY: printout reconstructed from the format strings; the values are illustrative, not captured from a run. -->

```text
  gnss      lat=37.400000 lon=-122.100000 alt=12.34 m  vel=(  +0.000,  +0.000,  +0.000) m/s (NED)
            fix=3 eph=1.20 epv=1.80 m  [valid t=1770000000.200]   [slowest device, ~5 Hz]
```

The helper `stamp` that formats the bracketed field is two lines long and is quoted in
[A tour of the whole snapshot](09-sensors-tour.md).

**Next:** [The environment block](04-environment.md)

**See also:** [Sensor noise](../ch06-services/03-sensor-config.md), [Coordinate frames](../ch06-services/04-frames-config.md), [Timestamps and sequence numbers](06-timestamps.md)
