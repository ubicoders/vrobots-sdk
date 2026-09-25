# Sensor noise

The noise model the simulator applies, and the eight blocks you can set independently.

```sh
cargo run -p vrobots-examples --bin ex24_sensor_config
./target/cpp-build/ex24_sensor_config
python examples/python/ex24_sensor_config.py
```

## The model

Every IMU channel is corrupted the same way:

```text
measured = scale_factor * true + bias(t) + white(t)
bias(t)  = Gauss-Markov process + random walk
```

The Gauss-Markov part has a steady-state standard deviation (`bias_instability`) and a
correlation time (`bias_tau_s`); the random walk adds an unbounded drift on top of it, and a
fixed offset is drawn once at power-on (`turn_on_bias_std`). This is the one service whose
effect you can see directly in the state stream: degrade the gyro and the measured rates
roughen while the truth in `kin` stays perfectly smooth.

## `ImuNoise`

The same block serves the accelerometer, the gyroscope and the magnetometer.

| Field | Type | Units | Default (`ideal()`) | Notes |
|---|---|---|---|---|
| `scale_factor` | `[f64;3]` | gain | `[1.0; 3]` | `1.0` is ideal; **`0.0` is a dead channel** that reads 0.000 forever |
| `white_std` | `[f64;3]` | sensor units | `[0.0; 3]` | |
| `bias_instability` | `[f64;3]` | sensor units | `[0.0; 3]` | steady-state standard deviation of the Gauss-Markov bias |
| `bias_tau_s` | `f64` | s | `0.0` | **`<= 0` keeps the current value**; the simulator's own default is 100 |
| `random_walk_std` | `[f64;3]` | sensor units per root-second | `[0.0; 3]` | |
| `turn_on_bias_std` | `[f64;3]` | sensor units | `[0.0; 3]` | fixed bias drawn at power-on |

Sensor units are m/s² for the accelerometer, rad/s for the gyroscope and tesla for the
magnetometer. The vectors are per body axis in your header's convention and are permuted into
the sensor's own axes, which may differ from the robot's.

## Start from `ideal()`, not from zero

`SensorConfig` gates each block independently, so `None` means the simulator does not touch
that setting. **Inside an `ImuNoise` block there are no flags at all.** The schema has none and
the simulator writes the whole block unconditionally, so a field you leave at zero is not
"keep the current value", it is zero.

`ImuNoise::default()` is therefore `ImuNoise::ideal()`, a working sensor with unit gain and no
noise, rather than an all-zero struct. From the `examples/rust/src/bin/ex24_sensor_config.rs`
header:

```text
ImuNoise::ideal().with_white_std([0.02; 3])            // a realistic gyro
ImuNoise::ideal().with_scale_factor([0.0; 3])          // a DEAD channel
```

Start from a whole sensor and spoil exactly what you mean to spoil. `bias_tau_s` is the block's
one exception, which is why `ideal()` leaves it at 0: that is the keep-current value.

## The eight gated blocks

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `accel_noise` | `Option<ImuNoise>` | m/s² | `None` | |
| `gyro_noise` | `Option<ImuNoise>` | rad/s | `None` | |
| `mag_noise` | `Option<ImuNoise>` | tesla | `None` | |
| `gps_quality` | `Option<GpsQuality>` | | `None` | reported values, not a model |
| `gps_noise` | `Option<GpsNoise>` | | `None` | the error actually applied |
| `baro_pressure_noise_std` | `Option<f64>` | Pa | `None` | |
| `optical_flow_noise_std` | `Option<[f64;3]>` | m/s, body axes | `None` | |
| `optical_flow_mounted` | `Option<bool>` | | `None` | idempotent: asking for the state it is already in does nothing |

Configuring the gyro leaves the accelerometer exactly as it was. `configure_sensors` refuses a
config where every block is `None`, and refuses any non-finite value.

## Reported quality is not applied noise

The two GNSS blocks are independent, and the split is deliberate.

| `GpsQuality` field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `eph` | `f64` | m | 1.5 | reported horizontal accuracy |
| `epv` | `f64` | m | 3.0 | reported vertical accuracy |
| `fix_type` | `u32` | | 3 | 0 none, 1 dead reckoning, 2 two-dimensional, 3 three-dimensional, 4 RTK |

| `GpsNoise` field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `position_std` | `[f64;3]` | m | `[0.0; 3]` | NED, frame-invariant |
| `velocity_std` | `[f64;3]` | m/s | `[0.0; 3]` | NED, frame-invariant |

`GpsQuality` is what the receiver **claims**. The simulator always has a perfect fix, so
setting `fix_type` to 0 does not invalidate the GNSS and raising `eph` does not scatter the
position: it feeds your filter's covariance and nothing else. `GpsNoise` is the error actually
applied, and unlike the IMU blocks it is not re-expressed from your header frame, because a
geodetic receiver's error ellipsoid is not a body quantity.

## One call, seven blocks

`ex24` rests the robot on the ground and commands nothing, so truth is constant and the spread
of each reading is that sensor's noise realisation. It measures a window, configures, and
measures again.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex24_sensor_config.rs`:

```rust
    let config = SensorConfig::default()
        .with_gyro_noise(
            ImuNoise::ideal()
                .with_white_std(GYRO_WHITE)
                .with_bias_instability([0.002; 3])
                .with_bias_tau_s(60.0),
        )
        .with_accel_noise(ImuNoise::ideal().with_white_std(ACCEL_WHITE))
        .with_baro_pressure_noise_std(BARO_WHITE_PA)
        .with_gps_quality(GpsQuality::default().with_eph(REPORTED_EPH).with_epv(9.0))
        .with_gps_noise(
            GpsNoise::default()
                .with_position_std([2.0, 2.0, 3.0]) // NED metres
                .with_velocity_std([0.2, 0.2, 0.3]), // NED m/s
        )
        .with_optical_flow_mounted(true)
        .with_optical_flow_noise_std([0.05; 3]);
    robot.configure_sensors(&config)?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex24_sensor_config.cpp`:

```cpp
auto config = vrsdk::sensor_config();

config.has_gyro_noise = true;
config.gyro_noise = vrsdk::imu_noise();
for (int i = 0; i < 3; ++i) {
    config.gyro_noise.white_std[i] = GYRO_WHITE;
    config.gyro_noise.bias_instability[i] = 0.002;
}
config.gyro_noise.bias_tau_s = 60.0;

config.has_accel_noise = true;
config.accel_noise = vrsdk::imu_noise();
for (int i = 0; i < 3; ++i) {
    config.accel_noise.white_std[i] = ACCEL_WHITE;
}

config.has_baro_pressure_noise_std = true;
config.baro_pressure_noise_std = BARO_WHITE_PA;

config.has_gps_quality = true;
config.gps_quality = vrsdk::gps_quality();
config.gps_quality.eph = REPORTED_EPH;
config.gps_quality.epv = 9.0;

config.has_gps_noise = true;
config.gps_noise = vrsdk::gps_noise();
config.gps_noise.position_std[0] = 2.0;  // NED metres
config.gps_noise.position_std[1] = 2.0;
config.gps_noise.position_std[2] = 3.0;
config.gps_noise.velocity_std[0] = 0.2;  // NED m/s
config.gps_noise.velocity_std[1] = 0.2;
config.gps_noise.velocity_std[2] = 0.3;

config.has_optical_flow_mounted = true;
config.optical_flow_mounted = true;
config.has_optical_flow_noise_std = true;
for (int i = 0; i < 3; ++i) {
    config.optical_flow_noise_std[i] = 0.05;
}

robot.configure_sensors(config);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex24_sensor_config.py`:

```python
robot.configure_sensors(
    gyro_noise=vrsdk.ImuNoise(
        white_std=GYRO_WHITE,
        bias_instability=(0.002,) * 3,
        bias_tau_s=60.0,
    ),
    accel_noise=vrsdk.ImuNoise(white_std=ACCEL_WHITE),
    baro_pressure_noise_std=BARO_WHITE_PA,
    gps_quality=vrsdk.GpsQuality(eph=REPORTED_EPH, epv=9.0),
    gps_noise=vrsdk.GpsNoise(
        position_std=(2.0, 2.0, 3.0),  # NED metres
        velocity_std=(0.2, 0.2, 0.3),  # NED m/s
    ),
    optical_flow_mounted=True,
    optical_flow_noise_std=(0.05,) * 3,
)
```

{{#endtab }}
{{#endtabs }}

Only C++ gates the blocks by hand: it builds the plain C struct from `vrsdk::sensor_config()`
and sets a `has_*` flag beside each one, where Rust chains `with_*` setters and Python passes
keyword arguments. The "start from `ideal()`, not from zero" rule is `vrsdk::imu_noise()` in
C++ and a bare `vrsdk.ImuNoise()` in Python, and writing `{}` in C++ instead is what produces
a dead channel.

The change is live from the next sensor sample. The measured standard deviations move toward
what was asked for, the reported `eph` echoes back exactly, and the optical flow announces
itself:

```text
as spawned  gyro sigma=[...] rad/s  accel sigma=[...] m/s^2  baro sigma=<Pa>  eph=1.50 m  flow_valid=false
configured  gyro sigma=[...] rad/s  accel sigma=[...] m/s^2  baro sigma=<Pa>  eph=4.50 m  flow_valid=true
```

> **Note.** Mounting the optical flow is the one part of this service that reports itself:
> `sensors.optical_flow.valid` goes from false to true. Everything else you have to measure.

## What this service does not cover

GNSS home coordinates, the magnetic field vector and sea-level pressure are not here, and the
omission is deliberate: they are scene truths rather than per-robot configuration, and two
robots in one world must not disagree about them.

A block naming a sensor the robot does not carry is skipped with a log line inside the
simulator and acked `ok` like every other silent refusal. Nothing in the repository enumerates
which sensors each robot type carries, so the only way to find out is to read
`sensors.<name>.valid` on a live robot.

**Next:** [Coordinate frames](04-frames-config.md)

**See also:** [Sensors](../ch03-reading-state/03-sensors.md), [Truth, measured and believed](../ch03-reading-state/01-truth-measured-believed.md), [The environment block](../ch03-reading-state/04-environment.md)
