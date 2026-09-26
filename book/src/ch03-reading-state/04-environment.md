# The environment block

The true world the sensor readings were noised from, field by field, including the one field the sources disagree about.

## Truth about the world, not about the robot

`State::env` is a truth block, like `kin` and `wrench`. It holds the atmosphere and the
geodetic position the simulator used when it generated that sample's measurements. Its
value to you is almost entirely as the reference side of a subtraction: the barometer
reading minus `env.air_pressure` is the barometer's error, and the GNSS position minus
`env.geo_point` is the receiver's.

The block is small and every field is scalar or a fixed-size array, so reading it costs
nothing.

| Field | Type | Units | Frame | Default | Notes |
|---|---|---|---|---|---|
| `gravity` | `[f64; 3]` | m/s² | world | `[0.0; 3]` | the acceleration the physics engine applied |
| `air_pressure` | `f64` | Pa | | `0.0` | true static pressure; the barometer's reference |
| `air_density` | `f64` | kg/m³ | | `0.0` | |
| `temperature` | `f64` | °C | | `0.0` | true air temperature |
| `geo_point` | `GeoPoint` | deg, deg, m | | zeroed | the robot's true geodetic position |
| `agl` | `f64` | m | | `0.0` | placeholder, published as 0; see below |

`gravity` is a world-frame vector, not a scalar, so its sign tells you which way the
frame's third axis points: positive where that axis counts downwards, negative where it
counts upwards. That makes it a cheap runtime check that you have understood the frame
the rest of the snapshot is in.

## The two diffs this block exists for

| Difference | Gives you |
|---|---|
| `sensors.barometer.pressure - env.air_pressure` | the barometer's error in Pa on that sample |
| `sensors.gnss.geo_point - env.geo_point` | the receiver's position error |

Both are only meaningful when the measured side is fresh. The barometer and the GNSS
receiver run at their own rates, so compare their timestamps before differencing. See
[Sensors](03-sensors.md).

> **Note.** `air_density` and `temperature` are published as truth and are not derived
> from any sensor in the block: there is no thermometer and no air-data device in
> `sensors`. If your model needs density, this is where it comes from, and a real
> vehicle would have to estimate it.

## The `agl` field

`agl` is a placeholder. Checked against a running simulator (v3.0.1, 2026-09-26): a
multirotor hovering with `kin.lin_pos[2]` at -0.90 m reported `agl` as exactly `0.0` in
every snapshot. The C header, the `vrobots-sdk-sys` bindings and the Rust crate's field
documentation now all say the same thing: the simulator publishes 0 here, because the
downward raycast that would fill it is not run.

Use the vertical component of `kin.lin_pos` instead, with the sign that the snapshot's
`coord_frame_id` implies: on a robot publishing `frd` (NED), height above the origin
plane is `-lin_pos[2]`. The examples treat the zero as a placeholder rather than a
measurement, on the reasoning that `env` is the truth block and an invented height above
ground would be worse than a visibly missing one. If a later simulator build starts
filling `agl`, code that falls back only when `agl` is exactly zero keeps working.

## Reading the world block

The tour example prints the environment in two lines, and labels the `agl` line with
the example's own claim about it.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex10_sensors_tour.rs`:

```rust
let env = &s.env;
println!("WORLD  environment");
println!(
    "  gravity   {} m/s^2   air {:.1} Pa {:.3} kg/m^3 {:.1} C",
    v3(env.gravity),
    env.air_pressure,
    env.air_density,
    env.temperature
);
println!(
    "  agl       {:.2} m    home lat={:.6} lon={:.6}   [agl is hard-coded 0 in sim v3.0.0 -- use -lin_pos[2]]",
    env.agl, env.geo_point.latitude, env.geo_point.longitude
);
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex10_sensors_tour.cpp`:

```cpp
std::printf("WORLD  environment\n");
std::printf("  gravity   (%+.3f,%+.3f,%+.3f) m/s^2   air %.1f Pa %.3f kg/m^3 %.1f C\n",
            r.env.gravity[0], r.env.gravity[1], r.env.gravity[2], r.env.air_pressure,
            r.env.air_density, r.env.temperature);
std::printf(
    "  agl       %.2f m    home lat=%.6f lon=%.6f   [agl is hard-coded 0 in sim "
    "v3.0.0 -- use -lin_pos[2]]\n",
    r.env.agl, r.env.geo_point.latitude, r.env.geo_point.longitude);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex10_sensors_tour.py`:

```python
env = s.env
print("WORLD  environment")
print(
    f"  gravity   {v3(env.gravity)} m/s^2   air {env.air_pressure:.1f} Pa "
    f"{env.air_density:.3f} kg/m^3 {env.temperature:.1f} C"
)
print(
    f"  agl       {env.agl:.2f} m    home lat={env.geo_point.latitude:.6f} "
    f"lon={env.geo_point.longitude:.6f}   "
    f"[agl is hard-coded 0 in sim v3.0.0 -- use -lin_pos[2]]"
)
```

{{#endtab }}
{{#endtabs }}

All three carry the same warning in the same place, because all three read the same field
from the same message: `agl` is the one entry in this block you cannot use as it stands.

The printed `agl` value is the first thing to look at when you check this page's
conflict, because a non-zero reading on a robot in the air settles it:

<!-- VERIFY: printout reconstructed from the format strings; the values are illustrative, not captured from a run. -->

```text
WORLD  environment
  gravity   (  +0.000,  +0.000,  +9.807) m/s^2   air 101325.0 Pa 1.225 kg/m^3 15.0 C
  agl       0.00 m    home lat=37.400000 lon=-122.100000   [agl is hard-coded 0 in sim v3.0.0 -- use -lin_pos[2]]
```

**Next:** [Actuators](05-actuator.md)

**See also:** [Sensors](03-sensors.md), [Known simulator issues](../ch07-robots/07-known-issues.md), [Kinematics](02-kinematics.md)
