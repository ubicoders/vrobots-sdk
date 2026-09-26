//! ex24 -- sensor_config: give the sensors a noise model worth filtering.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex24_sensor_config
//! ```
//!
//! ex10 read the sensor block and told you to mount and tune it through
//! `srv/sensors`. This is that call. It is also the one service whose effect you
//! can *see* in the state stream rather than infer: degrade the gyro and the
//! measured rates roughen while the truth in `kin` stays perfectly smooth.
//!
//! The robot sits still on the ground for the whole run -- nothing is commanded
//! -- so truth is constant and the spread of each sensor reading **is** its noise
//! realisation.
//!
//! # The one thing that will break a sensor by accident
//!
//! [`SensorConfig`] gates each block independently: `None` means the simulator
//! does not touch that setting. **Inside an [`ImuNoise`] block there are no flags
//! at all.** The schema has none, and the simulator writes all six vectors
//! unconditionally -- so a field you leave at zero is not "keep the current
//! value", it is **zero**. A block built from zeros has `scale_factor = [0,0,0]`,
//! which is not a noisy sensor, it is a **dead** one that reads 0.000 forever.
//!
//! Which is why [`ImuNoise::ideal`] exists and is what `default()` returns: unit
//! gain, no noise, no bias. Start from a whole sensor and spoil exactly what you
//! mean to spoil:
//!
//! ```text
//! ImuNoise::ideal().with_white_std([0.02; 3])            // a realistic gyro
//! ImuNoise::ideal().with_scale_factor([0.0; 3])          // a DEAD channel
//! ```
//!
//! `bias_tau_s` is the block's one exception: `<= 0` keeps the simulator's
//! current time constant (100 s), which is why `ideal()` leaves it at 0.
//!
//! # Reported quality is not applied noise
//!
//! The two GNSS blocks are independent, and the split is deliberate:
//!
//! - [`GpsQuality`] (`eph`, `epv`, `fix_type`) is what the receiver **claims**.
//!   The simulator always has a perfect fix, so setting `fix_type = 0` does not
//!   invalidate the GNSS and raising `eph` does not scatter the position. It
//!   feeds your filter's covariance, nothing else -- and it echoes back
//!   *exactly*, which makes it the crispest confirmation in this whole example.
//! - [`GpsNoise`] is the error actually applied, in **NED**, and unlike the IMU
//!   blocks it is not re-expressed from your header frame: a geodetic receiver's
//!   error ellipsoid is not a body quantity.
//!
//! # What is deliberately not here
//!
//! GNSS home coordinates, the magnetic field vector and sea-level pressure are
//! **scene** truths, not per-robot config -- two robots in one world must not
//! disagree about them. And a block naming a sensor this robot does not carry is
//! dropped with a log line and an `ok` ack, like every other silent refusal.
//!
//! Mounting the optical flow is the exception that reports itself:
//! `sensors.optical_flow.valid` goes from false (ex10's "usually means not
//! mounted") to true.
//!
//! [`SensorConfig`]: vrobots_sdk::SensorConfig
//! [`ImuNoise`]: vrobots_sdk::ImuNoise
//! [`ImuNoise::ideal`]: vrobots_sdk::ImuNoise::ideal
//! [`GpsQuality`]: vrobots_sdk::GpsQuality
//! [`GpsNoise`]: vrobots_sdk::GpsNoise

use std::time::Duration;

use vrobots_sdk::{
    GpsNoise, GpsQuality, ImuNoise, RobotType, SensorConfig, State, VirtualRobot, VrError,
};

const SAMPLES: u32 = 100; // ~4 s at the 25 Hz state rate
const SETTLE_SAMPLES: u32 = 50; // ~2 s to land and stop moving before measuring
const SAMPLE_TIMEOUT: Duration = Duration::from_millis(500);

const GYRO_WHITE: [f64; 3] = [0.02, 0.02, 0.02]; // rad/s
const ACCEL_WHITE: [f64; 3] = [0.4, 0.4, 0.4]; // m/s^2
const BARO_WHITE_PA: f64 = 25.0; // pascals
const REPORTED_EPH: f64 = 4.5; // metres -- echoes back exactly

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, None)?;
    println!(
        "created sys_id={}, service key = {}",
        robot.sys_id(),
        vrobots_sdk::topics::srv_sensors(robot.sys_id())
    );
    println!("Nothing is commanded: the robot rests, so the spread IS the noise.\n");

    // A fresh spawn latches 1100 us -- idle, which is not enough to hold it up --
    // so give it a moment to arrive and stop moving before measuring. A transient
    // in the window would be counted as sensor noise.
    for _ in 0..SETTLE_SAMPLES {
        robot.rate(25.0);
    }

    // ===== before =====
    let before = measure(&robot)?;
    before.print("as spawned");

    // ===== configure =====
    // Every block starts from `ideal()` -- see the header. The gyro block below
    // also carries a slow bias, which is what makes an estimator earn its keep.
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
    println!("\nconfigure_sensors acked -- live from the next sensor sample.\n");

    // ===== after =====
    let after = measure(&robot)?;
    after.print("configured");

    println!(
        "\ngyro  white-noise floor {:?} -> {:?} rad/s (asked for {GYRO_WHITE:?})",
        rounded(before.gyro.std()),
        rounded(after.gyro.std())
    );
    println!(
        "accel white-noise floor {:?} -> {:?} m/s^2 (asked for {ACCEL_WHITE:?})",
        rounded(before.accel.std()),
        rounded(after.accel.std())
    );
    println!(
        "baro  pressure spread {:.2} -> {:.2} Pa (asked for {BARO_WHITE_PA})",
        before.baro.std(),
        after.baro.std()
    );
    println!(
        "gnss  reported eph {:.2} -> {:.2} m -- an exact echo, because reported \
         quality is a claim, not a model",
        before.eph, after.eph
    );
    println!(
        "flow  valid {} -> {} -- mounting the sensor is the one part of this \
         service that announces itself",
        before.flow_valid, after.flow_valid
    );

    // ===== and the request that changes nothing =====
    match robot.configure_sensors(&SensorConfig::default()) {
        Ok(()) => println!("\nUNEXPECTED: an empty config was accepted"),
        Err(e) => println!("\nempty config -> [{}] {}", e.code(), e.detail()),
    }

    robot.delete()?;
    println!("deleted sys_id={}", robot.sys_id());
    Ok(())
}

/// Per-axis spread of one vector channel, accumulated sample by sample.
#[derive(Default)]
struct Spread {
    n: f64,
    sum: [f64; 3],
    sum_sq: [f64; 3],
}

impl Spread {
    fn push(&mut self, v: [f64; 3]) {
        self.n += 1.0;
        for ((sum, sum_sq), value) in self.sum.iter_mut().zip(&mut self.sum_sq).zip(v) {
            *sum += value;
            *sum_sq += value * value;
        }
    }

    /// Population standard deviation per axis. With the robot at rest, truth is
    /// constant, so this is the sensor's own noise.
    fn std(&self) -> [f64; 3] {
        let mut out = [0.0; 3];
        if self.n < 2.0 {
            return out;
        }
        for ((slot, sum), sum_sq) in out.iter_mut().zip(self.sum).zip(self.sum_sq) {
            let mean = sum / self.n;
            *slot = (sum_sq / self.n - mean * mean).max(0.0).sqrt();
        }
        out
    }
}

/// The same, for a scalar channel.
#[derive(Default)]
struct ScalarSpread {
    n: f64,
    sum: f64,
    sum_sq: f64,
}

impl ScalarSpread {
    fn push(&mut self, v: f64) {
        self.n += 1.0;
        self.sum += v;
        self.sum_sq += v * v;
    }

    fn std(&self) -> f64 {
        if self.n < 2.0 {
            return 0.0;
        }
        let mean = self.sum / self.n;
        (self.sum_sq / self.n - mean * mean).max(0.0).sqrt()
    }
}

/// One measurement window.
struct Window {
    gyro: Spread,
    accel: Spread,
    baro: ScalarSpread,
    eph: f64,
    flow_valid: bool,
}

impl Window {
    fn print(&self, label: &str) {
        println!(
            "{label:<11} gyro sigma={:?} rad/s  accel sigma={:?} m/s^2  baro \
             sigma={:.2} Pa  eph={:.2} m  flow_valid={}",
            rounded(self.gyro.std()),
            rounded(self.accel.std()),
            self.baro.std(),
            self.eph,
            self.flow_valid
        );
    }
}

/// Collect one window of samples, one control iteration per state message.
fn measure(robot: &VirtualRobot) -> Result<Window, VrError> {
    let mut gyro = Spread::default();
    let mut accel = Spread::default();
    let mut baro = ScalarSpread::default();
    let mut last: Option<State> = None;

    for _ in 0..SAMPLES {
        // One sample per iteration: states() alone would count the same snapshot
        // several times and understate the spread.
        robot.wait_new_state(SAMPLE_TIMEOUT)?;
        let s = robot.states();
        gyro.push(s.sensors.gyroscope.angular_velocity);
        accel.push(s.sensors.accelerometer.linear_acceleration);
        baro.push(s.sensors.barometer.pressure);
        last = Some(s);
    }

    let s = last.expect("SAMPLES is non-zero");
    Ok(Window {
        gyro,
        accel,
        baro,
        eph: s.sensors.gnss.eph,
        flow_valid: s.sensors.optical_flow.valid,
    })
}

/// Three decimals is plenty, and it keeps two windows comparable at a glance.
fn rounded(v: [f64; 3]) -> [f64; 3] {
    [
        (v[0] * 1000.0).round() / 1000.0,
        (v[1] * 1000.0).round() / 1000.0,
        (v[2] * 1000.0).round() / 1000.0,
    ]
}
