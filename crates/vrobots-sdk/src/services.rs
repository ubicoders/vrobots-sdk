//! Services: reset, activate, and the configuration requests.
//!
//! Each service is a query the robot answers with an acknowledgement, and
//! **the ack is a receipt, not a result**: the simulator replies the instant the
//! request lands and applies it on the next physics step. Only the tier gates
//! of [`set_skin`](VirtualRobot::set_skin) and
//! [`configure_lidars`](VirtualRobot::configure_lidars) ever answer "no"; an
//! unknown frame id, a wrong rotor count or a nonsense drive mode are all
//! acknowledged and then refused by a log line inside the simulator, and the
//! forklift, pallet-robot and arm services heal an out-of-range value
//! silently. So the SDK refuses what it can check before sending
//! ([`VrError::InvalidArgument`](crate::VrError::InvalidArgument)), and the
//! confirmation for the rest is always the state stream.
//!
//! The request payloads mirror their schemas: an `Option` field is one "set
//! this" flag on the wire, and `None` leaves that setting untouched. Build them
//! with `default()` and the `with_*` setters. A type-specific service
//! registers only on its own robot type, so asking the wrong robot fails with
//! [`VrError::NoResponder`](crate::VrError::NoResponder) after the service
//! timeout: that is the capability probe.

use std::ptr;

use vrobots_sdk_sys as sys;

use crate::error::VrResult;
use crate::ffi::{self, check, fixed_str};
use crate::robot::VirtualRobot;
use crate::state::{Axes, FrameDef};

// ---------------------------------------------------------------------------
// srv/params
// ---------------------------------------------------------------------------

/// Mass and inertia: the [`set_physical_params`](VirtualRobot::set_physical_params)
/// request.
///
/// The only channel for either figure; neither appears in the state message.
///
/// ```
/// use vrobots_sdk::PhysicalParams;
///
/// let params = PhysicalParams::default().with_mass(1.6).with_moi([0.03, 0.03, 0.05]);
/// assert!(!params.is_empty());
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct PhysicalParams {
    /// Body mass, kg. Must be strictly positive.
    pub mass: Option<f64>,
    /// Principal moments of inertia, kg.m^2, in **your** header frame; the robot
    /// permutes them into its own. All three must be strictly positive.
    pub moi: Option<[f64; 3]>,
}

impl PhysicalParams {
    /// See [`mass`](Self::mass).
    #[must_use]
    pub fn with_mass(mut self, kg: f64) -> Self {
        self.mass = Some(kg);
        self
    }

    /// See [`moi`](Self::moi).
    #[must_use]
    pub fn with_moi(mut self, moi: [f64; 3]) -> Self {
        self.moi = Some(moi);
        self
    }

    /// Whether this request would change nothing.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.mass.is_none() && self.moi.is_none()
    }

    fn to_raw(self) -> sys::vrsdk_physical_params_t {
        let mut raw = sys::vrsdk_physical_params_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_physical_params_t`.
        unsafe { sys::vrsdk_physical_params_default(&mut raw) };
        if let Some(mass) = self.mass {
            raw.has_mass = true;
            raw.mass = mass;
        }
        if let Some(moi) = self.moi {
            raw.has_moi = true;
            raw.moi = moi;
        }
        raw
    }
}

// ---------------------------------------------------------------------------
// srv/sensors
// ---------------------------------------------------------------------------

/// One IMU channel's noise model:
/// `measured = scale_factor * true + bias(t) + white(t)`.
///
/// **The whole block is written**, including the fields you leave alone: there
/// are no per-field flags, so a `scale_factor` of zero means the channel now
/// reads zero. That is why [`ImuNoise::default`] is [`ideal`](Self::ideal)
/// (unit gain, no noise), the base to change from. `bias_tau_s <= 0` is the one
/// "keep the current value" field. Units are SI per channel: m/s^2 for the
/// accelerometer, rad/s for the gyroscope, tesla for the magnetometer.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct ImuNoise {
    /// Per-axis gain. `[1.0; 3]` is ideal; `[0.0; 3]` is a dead sensor.
    pub scale_factor: [f64; 3],
    /// White-noise standard deviation.
    pub white_std: [f64; 3],
    /// Steady-state standard deviation of the Gauss-Markov bias.
    pub bias_instability: [f64; 3],
    /// Bias correlation time, seconds. `<= 0` keeps the current value.
    pub bias_tau_s: f64,
    /// Bias random-walk standard deviation, per root-second.
    pub random_walk_std: [f64; 3],
    /// Standard deviation of the fixed bias drawn at power-on.
    pub turn_on_bias_std: [f64; 3],
}

impl Default for ImuNoise {
    /// [`ImuNoise::ideal`], not all zeros.
    fn default() -> ImuNoise {
        ImuNoise::ideal()
    }
}

impl ImuNoise {
    /// A perfect sensor: unit gain, no noise, current time constant kept. The
    /// library's own `vrsdk_imu_noise_default`.
    #[must_use]
    pub fn ideal() -> ImuNoise {
        let mut raw = sys::vrsdk_imu_noise_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_imu_noise_t`.
        unsafe { sys::vrsdk_imu_noise_default(&mut raw) };
        ImuNoise::from_raw(&raw)
    }

    /// See [`scale_factor`](Self::scale_factor).
    #[must_use]
    pub fn with_scale_factor(mut self, scale: [f64; 3]) -> Self {
        self.scale_factor = scale;
        self
    }

    /// See [`white_std`](Self::white_std).
    #[must_use]
    pub fn with_white_std(mut self, std: [f64; 3]) -> Self {
        self.white_std = std;
        self
    }

    /// See [`bias_instability`](Self::bias_instability).
    #[must_use]
    pub fn with_bias_instability(mut self, std: [f64; 3]) -> Self {
        self.bias_instability = std;
        self
    }

    /// See [`bias_tau_s`](Self::bias_tau_s).
    #[must_use]
    pub fn with_bias_tau_s(mut self, seconds: f64) -> Self {
        self.bias_tau_s = seconds;
        self
    }

    /// See [`random_walk_std`](Self::random_walk_std).
    #[must_use]
    pub fn with_random_walk_std(mut self, std: [f64; 3]) -> Self {
        self.random_walk_std = std;
        self
    }

    /// See [`turn_on_bias_std`](Self::turn_on_bias_std).
    #[must_use]
    pub fn with_turn_on_bias_std(mut self, std: [f64; 3]) -> Self {
        self.turn_on_bias_std = std;
        self
    }

    fn from_raw(raw: &sys::vrsdk_imu_noise_t) -> ImuNoise {
        ImuNoise {
            scale_factor: raw.scale_factor,
            white_std: raw.white_std,
            bias_instability: raw.bias_instability,
            bias_tau_s: raw.bias_tau_s,
            random_walk_std: raw.random_walk_std,
            turn_on_bias_std: raw.turn_on_bias_std,
        }
    }

    fn to_raw(self) -> sys::vrsdk_imu_noise_t {
        sys::vrsdk_imu_noise_t {
            scale_factor: self.scale_factor,
            white_std: self.white_std,
            bias_instability: self.bias_instability,
            bias_tau_s: self.bias_tau_s,
            random_walk_std: self.random_walk_std,
            turn_on_bias_std: self.turn_on_bias_std,
        }
    }
}

/// The GNSS quality the receiver **reports**: claims, not a model. The
/// simulator always has a perfect fix, so `fix_type = 0` does not invalidate it
/// and a larger `eph` does not scatter it; use [`GpsNoise`] for that.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct GpsQuality {
    /// Reported horizontal position accuracy, metres. Default 1.5.
    pub eph: f64,
    /// Reported vertical position accuracy, metres. Default 3.0.
    pub epv: f64,
    /// Reported fix type: 0 none, 1 dead reckoning, 2 two-dimensional,
    /// 3 three-dimensional, 4 RTK. Default 3.
    pub fix_type: u32,
}

impl Default for GpsQuality {
    /// The simulator's own reported values, 1.5 m, 3.0 m and a 3D fix: the
    /// library's `vrsdk_gps_quality_default`.
    fn default() -> GpsQuality {
        let mut raw = sys::vrsdk_gps_quality_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_gps_quality_t`.
        unsafe { sys::vrsdk_gps_quality_default(&mut raw) };
        GpsQuality {
            eph: raw.eph,
            epv: raw.epv,
            fix_type: raw.fix_type,
        }
    }
}

impl GpsQuality {
    /// See [`eph`](Self::eph).
    #[must_use]
    pub fn with_eph(mut self, metres: f64) -> Self {
        self.eph = metres;
        self
    }

    /// See [`epv`](Self::epv).
    #[must_use]
    pub fn with_epv(mut self, metres: f64) -> Self {
        self.epv = metres;
        self
    }

    /// See [`fix_type`](Self::fix_type).
    #[must_use]
    pub fn with_fix_type(mut self, fix_type: u32) -> Self {
        self.fix_type = fix_type;
        self
    }
}

/// The GNSS noise the receiver actually applies, per NED axis. Frame-invariant:
/// not re-expressed from your header frame.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct GpsNoise {
    /// Position standard deviation per NED axis, metres.
    pub position_std: [f64; 3],
    /// Velocity standard deviation per NED axis, m/s.
    pub velocity_std: [f64; 3],
}

impl Default for GpsNoise {
    /// A noiseless receiver: the library's `vrsdk_gps_noise_default`.
    fn default() -> GpsNoise {
        let mut raw = sys::vrsdk_gps_noise_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_gps_noise_t`.
        unsafe { sys::vrsdk_gps_noise_default(&mut raw) };
        GpsNoise {
            position_std: raw.position_std,
            velocity_std: raw.velocity_std,
        }
    }
}

impl GpsNoise {
    /// See [`position_std`](Self::position_std).
    #[must_use]
    pub fn with_position_std(mut self, std: [f64; 3]) -> Self {
        self.position_std = std;
        self
    }

    /// See [`velocity_std`](Self::velocity_std).
    #[must_use]
    pub fn with_velocity_std(mut self, std: [f64; 3]) -> Self {
        self.velocity_std = std;
        self
    }
}

/// The [`configure_sensors`](VirtualRobot::configure_sensors) request: blocks
/// gated independently, so configuring the gyro leaves the accelerometer alone.
///
/// ```
/// use vrobots_sdk::{ImuNoise, SensorConfig};
///
/// let config = SensorConfig::default()
///     .with_gyro_noise(ImuNoise::ideal().with_white_std([0.004; 3]))
///     .with_baro_pressure_noise_std(12.0);
/// assert!(config.accel_noise.is_none());
/// ```
///
/// A block naming a sensor the robot does not carry is skipped by the
/// simulator, and the ack still says OK.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct SensorConfig {
    /// Accelerometer noise, m/s^2.
    pub accel_noise: Option<ImuNoise>,
    /// Gyroscope noise, rad/s.
    pub gyro_noise: Option<ImuNoise>,
    /// Magnetometer noise, tesla.
    pub mag_noise: Option<ImuNoise>,
    /// Reported GNSS accuracy and fix type.
    pub gps_quality: Option<GpsQuality>,
    /// Actual GNSS position and velocity noise, NED.
    pub gps_noise: Option<GpsNoise>,
    /// Barometer pressure noise standard deviation, pascals.
    pub baro_pressure_noise_std: Option<f64>,
    /// Optical-flow velocity noise standard deviation, m/s, body axes.
    pub optical_flow_noise_std: Option<[f64; 3]>,
    /// Mount (`true`) or unmount (`false`) the optical-flow sensor. Idempotent.
    pub optical_flow_mounted: Option<bool>,
}

impl SensorConfig {
    /// See [`accel_noise`](Self::accel_noise).
    #[must_use]
    pub fn with_accel_noise(mut self, noise: ImuNoise) -> Self {
        self.accel_noise = Some(noise);
        self
    }

    /// See [`gyro_noise`](Self::gyro_noise).
    #[must_use]
    pub fn with_gyro_noise(mut self, noise: ImuNoise) -> Self {
        self.gyro_noise = Some(noise);
        self
    }

    /// See [`mag_noise`](Self::mag_noise).
    #[must_use]
    pub fn with_mag_noise(mut self, noise: ImuNoise) -> Self {
        self.mag_noise = Some(noise);
        self
    }

    /// See [`gps_quality`](Self::gps_quality).
    #[must_use]
    pub fn with_gps_quality(mut self, quality: GpsQuality) -> Self {
        self.gps_quality = Some(quality);
        self
    }

    /// See [`gps_noise`](Self::gps_noise).
    #[must_use]
    pub fn with_gps_noise(mut self, noise: GpsNoise) -> Self {
        self.gps_noise = Some(noise);
        self
    }

    /// See [`baro_pressure_noise_std`](Self::baro_pressure_noise_std).
    #[must_use]
    pub fn with_baro_pressure_noise_std(mut self, pascals: f64) -> Self {
        self.baro_pressure_noise_std = Some(pascals);
        self
    }

    /// See [`optical_flow_noise_std`](Self::optical_flow_noise_std).
    #[must_use]
    pub fn with_optical_flow_noise_std(mut self, std: [f64; 3]) -> Self {
        self.optical_flow_noise_std = Some(std);
        self
    }

    /// See [`optical_flow_mounted`](Self::optical_flow_mounted).
    #[must_use]
    pub fn with_optical_flow_mounted(mut self, mounted: bool) -> Self {
        self.optical_flow_mounted = Some(mounted);
        self
    }

    /// Whether this request would change nothing.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.accel_noise.is_none()
            && self.gyro_noise.is_none()
            && self.mag_noise.is_none()
            && self.gps_quality.is_none()
            && self.gps_noise.is_none()
            && self.baro_pressure_noise_std.is_none()
            && self.optical_flow_noise_std.is_none()
            && self.optical_flow_mounted.is_none()
    }

    fn to_raw(self) -> sys::vrsdk_sensor_config_t {
        let mut raw = sys::vrsdk_sensor_config_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_sensor_config_t`. The
        // initializer pre-fills every block with a working default, so an unset
        // block is never a zeroed, dead one.
        unsafe { sys::vrsdk_sensor_config_default(&mut raw) };
        if let Some(noise) = self.accel_noise {
            raw.has_accel_noise = true;
            raw.accel_noise = noise.to_raw();
        }
        if let Some(noise) = self.gyro_noise {
            raw.has_gyro_noise = true;
            raw.gyro_noise = noise.to_raw();
        }
        if let Some(noise) = self.mag_noise {
            raw.has_mag_noise = true;
            raw.mag_noise = noise.to_raw();
        }
        if let Some(quality) = self.gps_quality {
            raw.has_gps_quality = true;
            raw.gps_quality = sys::vrsdk_gps_quality_t {
                eph: quality.eph,
                epv: quality.epv,
                fix_type: quality.fix_type,
            };
        }
        if let Some(noise) = self.gps_noise {
            raw.has_gps_noise = true;
            raw.gps_noise = sys::vrsdk_gps_noise_t {
                position_std: noise.position_std,
                velocity_std: noise.velocity_std,
            };
        }
        if let Some(std) = self.baro_pressure_noise_std {
            raw.has_baro_pressure_noise_std = true;
            raw.baro_pressure_noise_std = std;
        }
        if let Some(std) = self.optical_flow_noise_std {
            raw.has_optical_flow_noise_std = true;
            raw.optical_flow_noise_std = std;
        }
        if let Some(mounted) = self.optical_flow_mounted {
            raw.has_optical_flow_mounted = true;
            raw.optical_flow_mounted = mounted;
        }
        raw
    }
}

// ---------------------------------------------------------------------------
// srv/frames
// ---------------------------------------------------------------------------

/// The frame id that **clears** an override, letting the level below win: a
/// device falls back to its robot's frame, a robot to the scene's. Distinct
/// from `""`, which the SDK refuses because the simulator would skip it.
pub const INHERIT_FRAME: &str = "inherit";

/// The device names [`set_frames`](VirtualRobot::set_frames) knows, matched
/// exactly and case-sensitively. An unrecognised name is skipped with an OK
/// ack, so a typo looks like success; use these instead of literals.
pub mod device {
    /// The three-axis accelerometer.
    pub const ACCELEROMETER: &str = "accelerometer";
    /// The three-axis rate gyro.
    pub const GYROSCOPE: &str = "gyroscope";
    /// The three-axis magnetometer.
    pub const MAGNETOMETER: &str = "magnetometer";
    /// The barometric pressure sensor.
    pub const BAROMETER: &str = "barometer";
    /// The GNSS receiver.
    pub const GPS: &str = "gps";
    /// The optical-flow sensor.
    pub const OPTICAL_FLOW: &str = "optical_flow";

    /// One camera, by the name it was mounted under: `"camera/<name>"`.
    #[must_use]
    pub fn camera(name: &str) -> String {
        format!("camera/{name}")
    }

    /// One lidar, by its unit name: `"lidar/<name>"`. A lidar starts in
    /// `"flu"` rather than inheriting the robot's frame, and an override
    /// re-labels its points, never its ranges.
    #[must_use]
    pub fn lidar(name: &str) -> String {
        format!("lidar/{name}")
    }
}

/// One `device -> frame` override, an entry of
/// [`set_frames`](VirtualRobot::set_frames).
///
/// ```
/// use vrobots_sdk::{device, DeviceFrame, INHERIT_FRAME};
///
/// let gyro = DeviceFrame::new(device::GYROSCOPE, "frd");
/// let camera = DeviceFrame::new(device::camera("front"), "cv");
/// let clear = DeviceFrame::new(device::GPS, INHERIT_FRAME);
/// # let _ = (gyro, camera, clear);
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct DeviceFrame {
    /// The device, exactly as [`device`] spells it.
    pub device: String,
    /// A registered frame id (`"unity"`, `"frd"`, `"fru"`, `"flu"`, `"cv"` or
    /// one the scene registered), or [`INHERIT_FRAME`] to clear the override.
    pub coord_frame_id: String,
}

impl DeviceFrame {
    /// One override entry.
    #[must_use]
    pub fn new(device: impl Into<String>, coord_frame_id: impl Into<String>) -> DeviceFrame {
        DeviceFrame {
            device: device.into(),
            coord_frame_id: coord_frame_id.into(),
        }
    }
}

/// The scene's coordinate frame, from [`scene_frame`](VirtualRobot::scene_frame).
///
/// Level 1 of the frame chain: a robot with no override reports in this frame,
/// and so does a device whose robot has none.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct SceneFrame {
    /// The registry id: `"unity"`, `"frd"`, `"fru"`, `"cv"`, or one the scene
    /// registered at run time. Authoritative when it and `axis_convention`
    /// disagree.
    pub coord_frame_id: String,
    /// The tag beside it; [`Axes::UNSPECIFIED`] for a frame with no tag, such as
    /// the default `"fru"`.
    pub axis_convention: Axes,
}

// ---------------------------------------------------------------------------
// srv/drive (Truck)
// ---------------------------------------------------------------------------

/// A pulse-width band, microseconds: the truck's, the forklift's or the
/// pallet robot's. One flag covers all four numbers.
///
/// The factory band of all three is 1100 / 1500 / 1900. The simulator replaces
/// the whole band with the factory values unless `min_us < neutral_us <
/// max_us`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct PwmBand {
    /// Full reverse or full left.
    pub min_us: u32,
    /// Centre stick.
    pub neutral_us: u32,
    /// Full forward or full right.
    pub max_us: u32,
    /// Half-width of the neutral deadband, where the throttle idles and the
    /// idle brake holds the truck.
    pub deadband_us: u32,
}

impl PwmBand {
    /// A band.
    #[must_use]
    pub fn new(min_us: u32, neutral_us: u32, max_us: u32, deadband_us: u32) -> PwmBand {
        PwmBand {
            min_us,
            neutral_us,
            max_us,
            deadband_us,
        }
    }

    fn to_raw(self) -> sys::vrsdk_pwm_band_t {
        sys::vrsdk_pwm_band_t {
            min_us: self.min_us,
            neutral_us: self.neutral_us,
            max_us: self.max_us,
            deadband_us: self.deadband_us,
        }
    }
}

/// The [`configure_drive`](VirtualRobot::configure_drive) request: a truck's
/// drivetrain. **Truck only.**
///
/// Every value is read live, so a change bites from the next physics step.
/// Watch for the simulator's silent substitutions: `max_steer_deg` is clamped
/// to 0 to 60, a `no_load_wheel_rpm` of zero or less becomes 200, and a band
/// that is not strictly increasing is replaced by the factory one.
///
/// ```
/// use vrobots_sdk::DriveConfig;
///
/// let config = DriveConfig::default()
///     .with_drive_mode(2)
///     .with_steer_rate_dps(120.0)
///     .with_max_motor_torque_nm(60.0);
/// # let _ = config;
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct DriveConfig {
    /// Driven axles: **2** (rear only) or **4** (all wheels); nothing else is
    /// accepted.
    pub drive_mode: Option<u32>,
    /// Wheel angle at full steering stick, degrees. Clamped to 0 to 60.
    pub max_steer_deg: Option<f64>,
    /// Steering servo sweep rate, degrees per second. `0` is an ideal servo.
    pub steer_rate_dps: Option<f64>,
    /// Peak motor torque per driven wheel, N.m.
    pub max_motor_torque_nm: Option<f64>,
    /// Wheel speed at full throttle with no load, rpm: this sets top speed.
    pub no_load_wheel_rpm: Option<f64>,
    /// Brake torque per wheel while the throttle sits in the deadband, N.m.
    pub idle_brake_torque_nm: Option<f64>,
    /// Brake torque per wheel at full brake, N.m.
    pub max_brake_torque_nm: Option<f64>,
    /// The pulse-width band the three channels are read on.
    pub pwm_band: Option<PwmBand>,
}

impl DriveConfig {
    /// See [`drive_mode`](Self::drive_mode).
    #[must_use]
    pub fn with_drive_mode(mut self, mode: u32) -> Self {
        self.drive_mode = Some(mode);
        self
    }

    /// See [`max_steer_deg`](Self::max_steer_deg).
    #[must_use]
    pub fn with_max_steer_deg(mut self, degrees: f64) -> Self {
        self.max_steer_deg = Some(degrees);
        self
    }

    /// See [`steer_rate_dps`](Self::steer_rate_dps).
    #[must_use]
    pub fn with_steer_rate_dps(mut self, dps: f64) -> Self {
        self.steer_rate_dps = Some(dps);
        self
    }

    /// See [`max_motor_torque_nm`](Self::max_motor_torque_nm).
    #[must_use]
    pub fn with_max_motor_torque_nm(mut self, nm: f64) -> Self {
        self.max_motor_torque_nm = Some(nm);
        self
    }

    /// See [`no_load_wheel_rpm`](Self::no_load_wheel_rpm).
    #[must_use]
    pub fn with_no_load_wheel_rpm(mut self, rpm: f64) -> Self {
        self.no_load_wheel_rpm = Some(rpm);
        self
    }

    /// See [`idle_brake_torque_nm`](Self::idle_brake_torque_nm).
    #[must_use]
    pub fn with_idle_brake_torque_nm(mut self, nm: f64) -> Self {
        self.idle_brake_torque_nm = Some(nm);
        self
    }

    /// See [`max_brake_torque_nm`](Self::max_brake_torque_nm).
    #[must_use]
    pub fn with_max_brake_torque_nm(mut self, nm: f64) -> Self {
        self.max_brake_torque_nm = Some(nm);
        self
    }

    /// See [`pwm_band`](Self::pwm_band).
    #[must_use]
    pub fn with_pwm_band(mut self, band: PwmBand) -> Self {
        self.pwm_band = Some(band);
        self
    }

    /// Whether this request would change nothing.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.drive_mode.is_none()
            && self.max_steer_deg.is_none()
            && self.steer_rate_dps.is_none()
            && self.max_motor_torque_nm.is_none()
            && self.no_load_wheel_rpm.is_none()
            && self.idle_brake_torque_nm.is_none()
            && self.max_brake_torque_nm.is_none()
            && self.pwm_band.is_none()
    }

    fn to_raw(self) -> sys::vrsdk_drive_config_t {
        let mut raw = sys::vrsdk_drive_config_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_drive_config_t`.
        unsafe { sys::vrsdk_drive_config_default(&mut raw) };
        if let Some(mode) = self.drive_mode {
            raw.has_drive_mode = true;
            raw.drive_mode = mode;
        }
        if let Some(value) = self.max_steer_deg {
            raw.has_max_steer_deg = true;
            raw.max_steer_deg = value;
        }
        if let Some(value) = self.steer_rate_dps {
            raw.has_steer_rate_dps = true;
            raw.steer_rate_dps = value;
        }
        if let Some(value) = self.max_motor_torque_nm {
            raw.has_max_motor_torque_nm = true;
            raw.max_motor_torque_nm = value;
        }
        if let Some(value) = self.no_load_wheel_rpm {
            raw.has_no_load_wheel_rpm = true;
            raw.no_load_wheel_rpm = value;
        }
        if let Some(value) = self.idle_brake_torque_nm {
            raw.has_idle_brake_torque_nm = true;
            raw.idle_brake_torque_nm = value;
        }
        if let Some(value) = self.max_brake_torque_nm {
            raw.has_max_brake_torque_nm = true;
            raw.max_brake_torque_nm = value;
        }
        if let Some(band) = self.pwm_band {
            raw.has_pwm_band = true;
            raw.pwm_band = band.to_raw();
        }
        raw
    }
}

// ---------------------------------------------------------------------------
// srv/rotors (Multirotor)
// ---------------------------------------------------------------------------

/// One rotor of a multirotor, an entry of
/// [`configure_rotors`](VirtualRobot::configure_rotors).
///
/// No per-field flags: the simulator rebuilds the rotor from every value, so
/// start from [`RotorSpec::default`], the simulator's reference rotor, and
/// change what you mean to change. With `pwm` in microseconds and `g` the
/// scene's gravity:
///
/// ```text
/// thrust = (thrust_a*pwm^2 + thrust_b*pwm + thrust_c) * g               [N]
/// torque = spin_dir * (torque_a*pwm^2 + torque_b*pwm + torque_c) * g    [N.m]
/// omega  = ang_vel_slope*pwm + ang_vel_intercept                        [rad/s]
/// ```
///
/// Roll, pitch and yaw fall out of the rotor positions: there is no mixing
/// matrix in the simulator.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct RotorSpec {
    /// Hub position in metres from the robot's origin (not its centre of mass),
    /// in your header frame.
    pub position: [f64; 3],
    /// `+1` clockwise, `-1` counter-clockwise; `0` lets the simulator alternate
    /// by index, which is how the airframes are built.
    pub spin_dir: f64,
    /// Quadratic term of the thrust curve.
    pub thrust_a: f64,
    /// Linear term of the thrust curve.
    pub thrust_b: f64,
    /// Constant term of the thrust curve.
    pub thrust_c: f64,
    /// Quadratic term of the yaw-torque curve.
    pub torque_a: f64,
    /// Linear term of the yaw-torque curve.
    pub torque_b: f64,
    /// Constant term of the yaw-torque curve.
    pub torque_c: f64,
    /// Slope of the reported propeller speed, rad/s per microsecond.
    pub ang_vel_slope: f64,
    /// Intercept of the reported propeller speed, rad/s.
    pub ang_vel_intercept: f64,
    /// Bottom of this rotor's pulse-width band, microseconds.
    pub pwm_min_us: u32,
    /// Top of this rotor's pulse-width band, microseconds.
    pub pwm_max_us: u32,
}

impl Default for RotorSpec {
    /// The simulator's reference rotor, at the origin with its spin direction
    /// left to the index: the library's `vrsdk_rotor_spec_default`.
    fn default() -> RotorSpec {
        let mut raw = sys::vrsdk_rotor_spec_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_rotor_spec_t`.
        unsafe { sys::vrsdk_rotor_spec_default(&mut raw) };
        RotorSpec {
            position: raw.position,
            spin_dir: raw.spin_dir,
            thrust_a: raw.thrust_a,
            thrust_b: raw.thrust_b,
            thrust_c: raw.thrust_c,
            torque_a: raw.torque_a,
            torque_b: raw.torque_b,
            torque_c: raw.torque_c,
            ang_vel_slope: raw.ang_vel_slope,
            ang_vel_intercept: raw.ang_vel_intercept,
            pwm_min_us: raw.pwm_min_us,
            pwm_max_us: raw.pwm_max_us,
        }
    }
}

impl RotorSpec {
    /// See [`position`](Self::position).
    #[must_use]
    pub fn with_position(mut self, position: [f64; 3]) -> Self {
        self.position = position;
        self
    }

    /// See [`spin_dir`](Self::spin_dir).
    #[must_use]
    pub fn with_spin_dir(mut self, spin_dir: f64) -> Self {
        self.spin_dir = spin_dir;
        self
    }

    /// The thrust curve's three coefficients.
    #[must_use]
    pub fn with_thrust_curve(mut self, a: f64, b: f64, c: f64) -> Self {
        self.thrust_a = a;
        self.thrust_b = b;
        self.thrust_c = c;
        self
    }

    /// The yaw-torque curve's three coefficients.
    #[must_use]
    pub fn with_torque_curve(mut self, a: f64, b: f64, c: f64) -> Self {
        self.torque_a = a;
        self.torque_b = b;
        self.torque_c = c;
        self
    }

    /// The reported propeller-speed line.
    #[must_use]
    pub fn with_ang_vel_curve(mut self, slope: f64, intercept: f64) -> Self {
        self.ang_vel_slope = slope;
        self.ang_vel_intercept = intercept;
        self
    }

    /// The pulse-width band, microseconds.
    #[must_use]
    pub fn with_pwm_band(mut self, min_us: u32, max_us: u32) -> Self {
        self.pwm_min_us = min_us;
        self.pwm_max_us = max_us;
        self
    }

    fn to_raw(self) -> sys::vrsdk_rotor_spec_t {
        sys::vrsdk_rotor_spec_t {
            position: self.position,
            spin_dir: self.spin_dir,
            thrust_a: self.thrust_a,
            thrust_b: self.thrust_b,
            thrust_c: self.thrust_c,
            torque_a: self.torque_a,
            torque_b: self.torque_b,
            torque_c: self.torque_c,
            ang_vel_slope: self.ang_vel_slope,
            ang_vel_intercept: self.ang_vel_intercept,
            pwm_min_us: self.pwm_min_us,
            pwm_max_us: self.pwm_max_us,
        }
    }
}

// ---------------------------------------------------------------------------
// srv/msd (Msd) and srv/cartpole (CartPole)
// ---------------------------------------------------------------------------

/// The [`configure_msd`](VirtualRobot::configure_msd) request: the `k` and `c`
/// of `m*x'' + c*x' + k*x = F`. **Msd only.** With the mass from
/// [`set_physical_params`](VirtualRobot::set_physical_params) they fix the
/// natural frequency and damping ratio.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct MsdConfig {
    /// Spring constant `k`, N/m. Must not be negative.
    pub spring_k: Option<f64>,
    /// Damping constant `c`, N.s/m. Must not be negative.
    pub damping_c: Option<f64>,
}

impl MsdConfig {
    /// See [`spring_k`](Self::spring_k).
    #[must_use]
    pub fn with_spring_k(mut self, n_per_m: f64) -> Self {
        self.spring_k = Some(n_per_m);
        self
    }

    /// See [`damping_c`](Self::damping_c).
    #[must_use]
    pub fn with_damping_c(mut self, ns_per_m: f64) -> Self {
        self.damping_c = Some(ns_per_m);
        self
    }

    /// Whether this request would change nothing.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.spring_k.is_none() && self.damping_c.is_none()
    }

    fn to_raw(self) -> sys::vrsdk_msd_config_t {
        let mut raw = sys::vrsdk_msd_config_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_msd_config_t`.
        unsafe { sys::vrsdk_msd_config_default(&mut raw) };
        if let Some(k) = self.spring_k {
            raw.has_spring_k = true;
            raw.spring_k = k;
        }
        if let Some(c) = self.damping_c {
            raw.has_damping_c = true;
            raw.damping_c = c;
        }
        raw
    }
}

/// The [`configure_cartpole`](VirtualRobot::configure_cartpole) request: every
/// mass, length and limit of a cart-pole, **including the cart's mass**.
/// **CartPole only.**
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct CartPoleConfig {
    /// Cart body mass, kg. Owned here, not by `srv/params`.
    pub cart_mass: Option<f64>,
    /// Rail travel each side of the spawn point, metres.
    pub travel_half_range: Option<f64>,
    /// Mass of the uniform rod between hinge and bob, kg.
    pub pole_rod_mass: Option<f64>,
    /// Point mass at the tip of the pole, kg.
    pub bob_mass: Option<f64>,
    /// Hinge-to-bob distance, metres: the pendulum's length.
    pub pole_length: Option<f64>,
    /// Angular damping at the hinge, dimensionless. `0` is frictionless.
    pub pole_angular_damping: Option<f64>,
    /// Clamp on the commanded force, N.
    pub max_force: Option<f64>,
    /// The pole's home angle in **degrees**, 0 upright and +/-180 hanging, while
    /// the state topic reports the same angle in radians. Changing it re-seats
    /// the pole at rest immediately, not on the next reset.
    pub initial_pole_angle_deg: Option<f64>,
}

impl CartPoleConfig {
    /// See [`cart_mass`](Self::cart_mass).
    #[must_use]
    pub fn with_cart_mass(mut self, kg: f64) -> Self {
        self.cart_mass = Some(kg);
        self
    }

    /// See [`travel_half_range`](Self::travel_half_range).
    #[must_use]
    pub fn with_travel_half_range(mut self, metres: f64) -> Self {
        self.travel_half_range = Some(metres);
        self
    }

    /// See [`pole_rod_mass`](Self::pole_rod_mass).
    #[must_use]
    pub fn with_pole_rod_mass(mut self, kg: f64) -> Self {
        self.pole_rod_mass = Some(kg);
        self
    }

    /// See [`bob_mass`](Self::bob_mass).
    #[must_use]
    pub fn with_bob_mass(mut self, kg: f64) -> Self {
        self.bob_mass = Some(kg);
        self
    }

    /// See [`pole_length`](Self::pole_length).
    #[must_use]
    pub fn with_pole_length(mut self, metres: f64) -> Self {
        self.pole_length = Some(metres);
        self
    }

    /// See [`pole_angular_damping`](Self::pole_angular_damping).
    #[must_use]
    pub fn with_pole_angular_damping(mut self, damping: f64) -> Self {
        self.pole_angular_damping = Some(damping);
        self
    }

    /// See [`max_force`](Self::max_force).
    #[must_use]
    pub fn with_max_force(mut self, newtons: f64) -> Self {
        self.max_force = Some(newtons);
        self
    }

    /// See [`initial_pole_angle_deg`](Self::initial_pole_angle_deg).
    #[must_use]
    pub fn with_initial_pole_angle_deg(mut self, degrees: f64) -> Self {
        self.initial_pole_angle_deg = Some(degrees);
        self
    }

    /// Whether this request would change nothing.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.cart_mass.is_none()
            && self.travel_half_range.is_none()
            && self.pole_rod_mass.is_none()
            && self.bob_mass.is_none()
            && self.pole_length.is_none()
            && self.pole_angular_damping.is_none()
            && self.max_force.is_none()
            && self.initial_pole_angle_deg.is_none()
    }

    fn to_raw(self) -> sys::vrsdk_cartpole_config_t {
        let mut raw = sys::vrsdk_cartpole_config_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_cartpole_config_t`.
        unsafe { sys::vrsdk_cartpole_config_default(&mut raw) };
        let fields = [
            (self.cart_mass, &mut raw.has_cart_mass, &mut raw.cart_mass),
            (
                self.travel_half_range,
                &mut raw.has_travel_half_range,
                &mut raw.travel_half_range,
            ),
            (
                self.pole_rod_mass,
                &mut raw.has_pole_rod_mass,
                &mut raw.pole_rod_mass,
            ),
            (self.bob_mass, &mut raw.has_bob_mass, &mut raw.bob_mass),
            (
                self.pole_length,
                &mut raw.has_pole_length,
                &mut raw.pole_length,
            ),
            (
                self.pole_angular_damping,
                &mut raw.has_pole_angular_damping,
                &mut raw.pole_angular_damping,
            ),
            (self.max_force, &mut raw.has_max_force, &mut raw.max_force),
            (
                self.initial_pole_angle_deg,
                &mut raw.has_initial_pole_angle_deg,
                &mut raw.initial_pole_angle_deg,
            ),
        ];
        for (value, has, slot) in fields {
            if let Some(value) = value {
                *has = true;
                *slot = value;
            }
        }
        raw
    }
}

// ---------------------------------------------------------------------------
// srv/turret (Turret)
// ---------------------------------------------------------------------------

/// The [`configure_turret`](VirtualRobot::configure_turret) request: a
/// turret's joint limits and max rates, its laser's range and its stereo
/// pair. **Turret only.**
///
/// Angles are radians, rates rad/s and lengths metres, signed as in
/// [`set_turret_joints`](VirtualRobot::set_turret_joints).
///
/// | Field | Default | Out of range, in the simulator |
/// |---|---|---|
/// | `max_heading_rate` | pi/2 rad/s | `<= 0` reverts to the default |
/// | `max_pitch_rate` | pi/3 rad/s | `<= 0` reverts to the default |
/// | `heading_min`, `heading_max` | -pi/2, +pi/2 rad | clamped to [-pi, pi] |
/// | `pitch_min`, `pitch_max` | -0.262, +1.178 rad | clamped to [-pi/2, pi/2] |
/// | `stereo_baseline` | 0.80 m | clamped to [0.05, 0.94] m |
/// | `stereo_focal_length` | 600 px | clamped to [100, 5000] px |
/// | `laser_max_range` | 200 m | `<= 0`: refused, the range is kept |
/// | `stereo_height` | 0.35 m | clamped to [0.25, 1.0] m |
/// | `stereo_forward` | 0.40 m | clamped to [-0.79, 0.45] m |
///
/// The stereo fields act on the cameras named `left` and `right` together, so
/// the pair stays rectified; each frame carries the pose and focal length it
/// was rendered with, in [`Frame::mount`](crate::Frame::mount) and
/// [`Frame::intrinsics`](crate::Frame::intrinsics), which are the ones to
/// trust.
///
/// ```
/// use vrobots_sdk::TurretConfig;
///
/// // Let the barrel dip further, and tilt twice as fast as the default.
/// let config = TurretConfig::default()
///     .with_pitch_min(f64::to_radians(-30.0))
///     .with_max_pitch_rate(f64::to_radians(120.0));
/// assert!(!config.is_empty());
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct TurretConfig {
    /// How fast the pylon turns, rad/s.
    pub max_heading_rate: Option<f64>,
    /// How fast the head tilts, rad/s.
    pub max_pitch_rate: Option<f64>,
    /// Lower heading limit, rad: how far the pylon turns to the left.
    pub heading_min: Option<f64>,
    /// Upper heading limit, rad: how far the pylon turns to the right.
    pub heading_max: Option<f64>,
    /// Lower pitch limit, rad: how far the barrel goes down.
    pub pitch_min: Option<f64>,
    /// Upper pitch limit, rad: how far the barrel goes up.
    pub pitch_max: Option<f64>,
    /// Distance between the two optical centres, m; each camera sits half of it
    /// to one side of the barrel.
    pub stereo_baseline: Option<f64>,
    /// Focal length of both cameras, px at the current resolution, fx = fy.
    pub stereo_focal_length: Option<f64>,
    /// How far the laser measures, m. Beyond it the range reads 0, as for no
    /// hit.
    pub laser_max_range: Option<f64>,
    /// Height of both optical centres above the pivot, m.
    pub stereo_height: Option<f64>,
    /// Distance of both optical centres ahead of the pivot along the barrel, m;
    /// negative is behind it.
    pub stereo_forward: Option<f64>,
}

impl TurretConfig {
    /// See [`max_heading_rate`](Self::max_heading_rate).
    #[must_use]
    pub fn with_max_heading_rate(mut self, rad_per_s: f64) -> Self {
        self.max_heading_rate = Some(rad_per_s);
        self
    }

    /// See [`max_pitch_rate`](Self::max_pitch_rate).
    #[must_use]
    pub fn with_max_pitch_rate(mut self, rad_per_s: f64) -> Self {
        self.max_pitch_rate = Some(rad_per_s);
        self
    }

    /// See [`heading_min`](Self::heading_min).
    #[must_use]
    pub fn with_heading_min(mut self, rad: f64) -> Self {
        self.heading_min = Some(rad);
        self
    }

    /// See [`heading_max`](Self::heading_max).
    #[must_use]
    pub fn with_heading_max(mut self, rad: f64) -> Self {
        self.heading_max = Some(rad);
        self
    }

    /// See [`pitch_min`](Self::pitch_min).
    #[must_use]
    pub fn with_pitch_min(mut self, rad: f64) -> Self {
        self.pitch_min = Some(rad);
        self
    }

    /// See [`pitch_max`](Self::pitch_max).
    #[must_use]
    pub fn with_pitch_max(mut self, rad: f64) -> Self {
        self.pitch_max = Some(rad);
        self
    }

    /// See [`stereo_baseline`](Self::stereo_baseline).
    #[must_use]
    pub fn with_stereo_baseline(mut self, metres: f64) -> Self {
        self.stereo_baseline = Some(metres);
        self
    }

    /// See [`stereo_focal_length`](Self::stereo_focal_length).
    #[must_use]
    pub fn with_stereo_focal_length(mut self, px: f64) -> Self {
        self.stereo_focal_length = Some(px);
        self
    }

    /// See [`laser_max_range`](Self::laser_max_range).
    #[must_use]
    pub fn with_laser_max_range(mut self, metres: f64) -> Self {
        self.laser_max_range = Some(metres);
        self
    }

    /// See [`stereo_height`](Self::stereo_height).
    #[must_use]
    pub fn with_stereo_height(mut self, metres: f64) -> Self {
        self.stereo_height = Some(metres);
        self
    }

    /// See [`stereo_forward`](Self::stereo_forward).
    #[must_use]
    pub fn with_stereo_forward(mut self, metres: f64) -> Self {
        self.stereo_forward = Some(metres);
        self
    }

    /// Whether this request would change nothing.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.max_heading_rate.is_none()
            && self.max_pitch_rate.is_none()
            && self.heading_min.is_none()
            && self.heading_max.is_none()
            && self.pitch_min.is_none()
            && self.pitch_max.is_none()
            && self.stereo_baseline.is_none()
            && self.stereo_focal_length.is_none()
            && self.laser_max_range.is_none()
            && self.stereo_height.is_none()
            && self.stereo_forward.is_none()
    }

    fn to_raw(self) -> sys::vrsdk_turret_config_t {
        let mut raw = sys::vrsdk_turret_config_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_turret_config_t`.
        unsafe { sys::vrsdk_turret_config_default(&mut raw) };
        let fields = [
            (
                self.max_heading_rate,
                &mut raw.has_max_heading_rate,
                &mut raw.max_heading_rate,
            ),
            (
                self.max_pitch_rate,
                &mut raw.has_max_pitch_rate,
                &mut raw.max_pitch_rate,
            ),
            (
                self.heading_min,
                &mut raw.has_heading_min,
                &mut raw.heading_min,
            ),
            (
                self.heading_max,
                &mut raw.has_heading_max,
                &mut raw.heading_max,
            ),
            (self.pitch_min, &mut raw.has_pitch_min, &mut raw.pitch_min),
            (self.pitch_max, &mut raw.has_pitch_max, &mut raw.pitch_max),
            (
                self.stereo_baseline,
                &mut raw.has_stereo_baseline,
                &mut raw.stereo_baseline,
            ),
            (
                self.stereo_focal_length,
                &mut raw.has_stereo_focal_length,
                &mut raw.stereo_focal_length,
            ),
            (
                self.laser_max_range,
                &mut raw.has_laser_max_range,
                &mut raw.laser_max_range,
            ),
            (
                self.stereo_height,
                &mut raw.has_stereo_height,
                &mut raw.stereo_height,
            ),
            (
                self.stereo_forward,
                &mut raw.has_stereo_forward,
                &mut raw.stereo_forward,
            ),
        ];
        for (value, has, slot) in fields {
            if let Some(value) = value {
                *has = true;
                *slot = value;
            }
        }
        raw
    }
}

// ---------------------------------------------------------------------------
// srv/forklift (Forklift) and srv/palletrobot (PalletRobot)
// ---------------------------------------------------------------------------

/// The [`configure_forklift`](VirtualRobot::configure_forklift) request: a
/// forklift's drive, rear steering and mast. **Forklift only.**
///
/// SI where the simulator stores degrees or rpm (angles in rad, rates in
/// rad/s) and pulse widths in microseconds, the unit of
/// [`set_forklift`](VirtualRobot::set_forklift). The simulator heals an
/// out-of-range value and acks it, so only a value that is not finite is
/// refused here.
///
/// | Field | Default |
/// |---|---|
/// | `max_motor_torque` | 1200 N.m per front wheel |
/// | `no_load_wheel_speed` | 13.93 rad/s (about 5 m/s) |
/// | `idle_brake_torque`, `max_brake_torque` | 800 N.m, 2000 N.m |
/// | `max_steer_angle`, `steer_slew_rate` | 0.785 rad, 1.047 rad/s |
/// | `lift_min`, `lift_max`, `lift_speed` | -0.215 m, 1.11 m, 0.5 m/s |
/// | `fork_width_min`, `fork_width_max`, `fork_width_speed` | 0.158 m, 1.334 m, 0.1 m/s |
/// | `pwm_band` | 1100 / 1500 / 1900 us, deadband 20 us |
/// | `initial_*_us` | 1500, 1500, 1100, 1301 us |
///
/// A range is judged one request at a time, so send both ends to move one
/// past its other end. The `initial_*_us` pulses are the frame
/// [`reset`](VirtualRobot::reset) re-latches, so they take effect at the next
/// reset.
///
/// ```
/// use vrobots_sdk::{ForkliftConfig, PwmBand};
///
/// // A slower forklift on a wider pulse band.
/// let config = ForkliftConfig::default()
///     .with_no_load_wheel_speed(7.0)
///     .with_lift_speed(0.25)
///     .with_pwm_band(PwmBand::new(1000, 1500, 2000, 25));
/// assert!(!config.is_empty());
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct ForkliftConfig {
    /// Peak motor torque per front wheel, N.m.
    pub max_motor_torque: Option<f64>,
    /// Front-wheel speed at full throttle with no load, rad/s: this sets the
    /// top speed.
    pub no_load_wheel_speed: Option<f64>,
    /// Brake torque per wheel while the throttle sits in the deadband, N.m.
    pub idle_brake_torque: Option<f64>,
    /// Brake torque per wheel at a full brake command, N.m.
    pub max_brake_torque: Option<f64>,
    /// Rear-wheel angle at full stick, rad.
    pub max_steer_angle: Option<f64>,
    /// Steering servo sweep rate, rad/s; 0 is an instant servo.
    pub steer_slew_rate: Option<f64>,
    /// Carriage height at the bottom of the pulse band, m; negative is allowed.
    pub lift_min: Option<f64>,
    /// Carriage height at the top of the pulse band, m.
    pub lift_max: Option<f64>,
    /// Carriage speed, m/s.
    pub lift_speed: Option<f64>,
    /// Fork width, centre to centre, at the bottom of the pulse band, m.
    pub fork_width_min: Option<f64>,
    /// Fork width, centre to centre, at the top of the pulse band, m.
    pub fork_width_max: Option<f64>,
    /// Fork-width change speed, m/s.
    pub fork_width_speed: Option<f64>,
    /// The pulse-width band of all five `SET_FORKLIFT` channels.
    pub pwm_band: Option<PwmBand>,
    /// The steer pulse a reset re-latches, us.
    pub initial_steer_us: Option<u32>,
    /// The throttle pulse a reset re-latches, us.
    pub initial_throttle_us: Option<u32>,
    /// The lift pulse a reset re-latches and snaps the carriage to, us.
    pub initial_lift_us: Option<u32>,
    /// The fork-width pulse a reset re-latches and snaps the forks to, us.
    pub initial_fork_width_us: Option<u32>,
}

impl ForkliftConfig {
    /// See [`max_motor_torque`](Self::max_motor_torque).
    #[must_use]
    pub fn with_max_motor_torque(mut self, newton_metres: f64) -> Self {
        self.max_motor_torque = Some(newton_metres);
        self
    }

    /// See [`no_load_wheel_speed`](Self::no_load_wheel_speed).
    #[must_use]
    pub fn with_no_load_wheel_speed(mut self, rad_per_s: f64) -> Self {
        self.no_load_wheel_speed = Some(rad_per_s);
        self
    }

    /// See [`idle_brake_torque`](Self::idle_brake_torque).
    #[must_use]
    pub fn with_idle_brake_torque(mut self, newton_metres: f64) -> Self {
        self.idle_brake_torque = Some(newton_metres);
        self
    }

    /// See [`max_brake_torque`](Self::max_brake_torque).
    #[must_use]
    pub fn with_max_brake_torque(mut self, newton_metres: f64) -> Self {
        self.max_brake_torque = Some(newton_metres);
        self
    }

    /// See [`max_steer_angle`](Self::max_steer_angle).
    #[must_use]
    pub fn with_max_steer_angle(mut self, rad: f64) -> Self {
        self.max_steer_angle = Some(rad);
        self
    }

    /// See [`steer_slew_rate`](Self::steer_slew_rate).
    #[must_use]
    pub fn with_steer_slew_rate(mut self, rad_per_s: f64) -> Self {
        self.steer_slew_rate = Some(rad_per_s);
        self
    }

    /// See [`lift_min`](Self::lift_min).
    #[must_use]
    pub fn with_lift_min(mut self, metres: f64) -> Self {
        self.lift_min = Some(metres);
        self
    }

    /// See [`lift_max`](Self::lift_max).
    #[must_use]
    pub fn with_lift_max(mut self, metres: f64) -> Self {
        self.lift_max = Some(metres);
        self
    }

    /// See [`lift_speed`](Self::lift_speed).
    #[must_use]
    pub fn with_lift_speed(mut self, metres_per_s: f64) -> Self {
        self.lift_speed = Some(metres_per_s);
        self
    }

    /// See [`fork_width_min`](Self::fork_width_min).
    #[must_use]
    pub fn with_fork_width_min(mut self, metres: f64) -> Self {
        self.fork_width_min = Some(metres);
        self
    }

    /// See [`fork_width_max`](Self::fork_width_max).
    #[must_use]
    pub fn with_fork_width_max(mut self, metres: f64) -> Self {
        self.fork_width_max = Some(metres);
        self
    }

    /// See [`fork_width_speed`](Self::fork_width_speed).
    #[must_use]
    pub fn with_fork_width_speed(mut self, metres_per_s: f64) -> Self {
        self.fork_width_speed = Some(metres_per_s);
        self
    }

    /// See [`pwm_band`](Self::pwm_band).
    #[must_use]
    pub fn with_pwm_band(mut self, band: PwmBand) -> Self {
        self.pwm_band = Some(band);
        self
    }

    /// See [`initial_steer_us`](Self::initial_steer_us).
    #[must_use]
    pub fn with_initial_steer_us(mut self, us: u32) -> Self {
        self.initial_steer_us = Some(us);
        self
    }

    /// See [`initial_throttle_us`](Self::initial_throttle_us).
    #[must_use]
    pub fn with_initial_throttle_us(mut self, us: u32) -> Self {
        self.initial_throttle_us = Some(us);
        self
    }

    /// See [`initial_lift_us`](Self::initial_lift_us).
    #[must_use]
    pub fn with_initial_lift_us(mut self, us: u32) -> Self {
        self.initial_lift_us = Some(us);
        self
    }

    /// See [`initial_fork_width_us`](Self::initial_fork_width_us).
    #[must_use]
    pub fn with_initial_fork_width_us(mut self, us: u32) -> Self {
        self.initial_fork_width_us = Some(us);
        self
    }

    /// Whether this request would change nothing.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.max_motor_torque.is_none()
            && self.no_load_wheel_speed.is_none()
            && self.idle_brake_torque.is_none()
            && self.max_brake_torque.is_none()
            && self.max_steer_angle.is_none()
            && self.steer_slew_rate.is_none()
            && self.lift_min.is_none()
            && self.lift_max.is_none()
            && self.lift_speed.is_none()
            && self.fork_width_min.is_none()
            && self.fork_width_max.is_none()
            && self.fork_width_speed.is_none()
            && self.pwm_band.is_none()
            && self.initial_steer_us.is_none()
            && self.initial_throttle_us.is_none()
            && self.initial_lift_us.is_none()
            && self.initial_fork_width_us.is_none()
    }

    fn to_raw(self) -> sys::vrsdk_forklift_config_t {
        let mut raw = sys::vrsdk_forklift_config_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_forklift_config_t`.
        unsafe { sys::vrsdk_forklift_config_default(&mut raw) };
        let floats = [
            (
                self.max_motor_torque,
                &mut raw.has_max_motor_torque,
                &mut raw.max_motor_torque,
            ),
            (
                self.no_load_wheel_speed,
                &mut raw.has_no_load_wheel_speed,
                &mut raw.no_load_wheel_speed,
            ),
            (
                self.idle_brake_torque,
                &mut raw.has_idle_brake_torque,
                &mut raw.idle_brake_torque,
            ),
            (
                self.max_brake_torque,
                &mut raw.has_max_brake_torque,
                &mut raw.max_brake_torque,
            ),
            (
                self.max_steer_angle,
                &mut raw.has_max_steer_angle,
                &mut raw.max_steer_angle,
            ),
            (
                self.steer_slew_rate,
                &mut raw.has_steer_slew_rate,
                &mut raw.steer_slew_rate,
            ),
            (self.lift_min, &mut raw.has_lift_min, &mut raw.lift_min),
            (self.lift_max, &mut raw.has_lift_max, &mut raw.lift_max),
            (
                self.lift_speed,
                &mut raw.has_lift_speed,
                &mut raw.lift_speed,
            ),
            (
                self.fork_width_min,
                &mut raw.has_fork_width_min,
                &mut raw.fork_width_min,
            ),
            (
                self.fork_width_max,
                &mut raw.has_fork_width_max,
                &mut raw.fork_width_max,
            ),
            (
                self.fork_width_speed,
                &mut raw.has_fork_width_speed,
                &mut raw.fork_width_speed,
            ),
        ];
        for (value, has, slot) in floats {
            if let Some(value) = value {
                *has = true;
                *slot = value;
            }
        }
        let pulses = [
            (
                self.initial_steer_us,
                &mut raw.has_initial_steer_us,
                &mut raw.initial_steer_us,
            ),
            (
                self.initial_throttle_us,
                &mut raw.has_initial_throttle_us,
                &mut raw.initial_throttle_us,
            ),
            (
                self.initial_lift_us,
                &mut raw.has_initial_lift_us,
                &mut raw.initial_lift_us,
            ),
            (
                self.initial_fork_width_us,
                &mut raw.has_initial_fork_width_us,
                &mut raw.initial_fork_width_us,
            ),
        ];
        for (value, has, slot) in pulses {
            if let Some(value) = value {
                *has = true;
                *slot = value;
            }
        }
        if let Some(band) = self.pwm_band {
            raw.has_pwm_band = true;
            raw.pwm_band = band.to_raw();
        }
        raw
    }
}

/// The [`configure_palletrobot`](VirtualRobot::configure_palletrobot) request:
/// a pallet robot's pulse band, virtual mecanum geometry, drive and pad.
/// **PalletRobot only.**
///
/// SI (m, m/s, rad/s) and pulse widths in microseconds, the unit of
/// [`set_palletrobot`](VirtualRobot::set_palletrobot). The simulator heals an
/// out-of-range value and acks it, so only a value that is not finite is
/// refused here.
///
/// | Field | Default |
/// |---|---|
/// | `pwm_band` | 1100 / 1500 / 1900 us, deadband 20 us |
/// | `wheel_radius`, `half_wheelbase`, `half_track` | 0.054 m, 0.35 m, 0.30 m |
/// | `max_wheel_speed`, `wheel_accel` | 28 rad/s, 60 rad/s^2 |
/// | `max_traction_accel`, `parked_hold_gain` | 6 m/s^2, 4 /s |
/// | `pad_lift_max`, `pad_lift_speed`, `pad_turn_max_rate` | 0.06 m, 0.03 m/s, 1.571 rad/s |
///
/// The latched command is re-mapped onto a new band on the next step, so send
/// a fresh frame, or [`reset`](VirtualRobot::reset), after changing it. Nothing
/// on the wire reads the geometry back: keep the values you send.
///
/// ```
/// use vrobots_sdk::PalletRobotConfig;
///
/// // Slower wheels for a heavy load: 14 rad/s at full stick.
/// let config = PalletRobotConfig::default().with_max_wheel_speed(14.0);
/// assert!(!config.is_empty());
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct PalletRobotConfig {
    /// The pulse-width band of all six `SET_PALLETROBOT` channels.
    pub pwm_band: Option<PwmBand>,
    /// Virtual wheel radius R, m.
    pub wheel_radius: Option<f64>,
    /// Centre to the front and rear axles, lx, m.
    pub half_wheelbase: Option<f64>,
    /// Centre to the left and right wheels, ly, m.
    pub half_track: Option<f64>,
    /// Wheel speed at full stick, rad/s.
    pub max_wheel_speed: Option<f64>,
    /// Wheel motor acceleration limit, rad/s^2; 0 jumps to the command.
    pub wheel_accel: Option<f64>,
    /// Ceiling on the body acceleration the drive applies, m/s^2.
    pub max_traction_accel: Option<f64>,
    /// Parked hold stiffness, 1/s: with all four wheels stopped, the rate the
    /// body is pulled back to where it parked. 0 is no hold.
    pub parked_hold_gain: Option<f64>,
    /// Pad height at the top of the band, m.
    pub pad_lift_max: Option<f64>,
    /// Pad lift speed, m/s; 0 is instant.
    pub pad_lift_speed: Option<f64>,
    /// Pad turn rate at full stick, rad/s.
    pub pad_turn_max_rate: Option<f64>,
}

impl PalletRobotConfig {
    /// See [`pwm_band`](Self::pwm_band).
    #[must_use]
    pub fn with_pwm_band(mut self, band: PwmBand) -> Self {
        self.pwm_band = Some(band);
        self
    }

    /// See [`wheel_radius`](Self::wheel_radius).
    #[must_use]
    pub fn with_wheel_radius(mut self, metres: f64) -> Self {
        self.wheel_radius = Some(metres);
        self
    }

    /// See [`half_wheelbase`](Self::half_wheelbase).
    #[must_use]
    pub fn with_half_wheelbase(mut self, metres: f64) -> Self {
        self.half_wheelbase = Some(metres);
        self
    }

    /// See [`half_track`](Self::half_track).
    #[must_use]
    pub fn with_half_track(mut self, metres: f64) -> Self {
        self.half_track = Some(metres);
        self
    }

    /// See [`max_wheel_speed`](Self::max_wheel_speed).
    #[must_use]
    pub fn with_max_wheel_speed(mut self, rad_per_s: f64) -> Self {
        self.max_wheel_speed = Some(rad_per_s);
        self
    }

    /// See [`wheel_accel`](Self::wheel_accel).
    #[must_use]
    pub fn with_wheel_accel(mut self, rad_per_s2: f64) -> Self {
        self.wheel_accel = Some(rad_per_s2);
        self
    }

    /// See [`max_traction_accel`](Self::max_traction_accel).
    #[must_use]
    pub fn with_max_traction_accel(mut self, metres_per_s2: f64) -> Self {
        self.max_traction_accel = Some(metres_per_s2);
        self
    }

    /// See [`parked_hold_gain`](Self::parked_hold_gain).
    #[must_use]
    pub fn with_parked_hold_gain(mut self, per_s: f64) -> Self {
        self.parked_hold_gain = Some(per_s);
        self
    }

    /// See [`pad_lift_max`](Self::pad_lift_max).
    #[must_use]
    pub fn with_pad_lift_max(mut self, metres: f64) -> Self {
        self.pad_lift_max = Some(metres);
        self
    }

    /// See [`pad_lift_speed`](Self::pad_lift_speed).
    #[must_use]
    pub fn with_pad_lift_speed(mut self, metres_per_s: f64) -> Self {
        self.pad_lift_speed = Some(metres_per_s);
        self
    }

    /// See [`pad_turn_max_rate`](Self::pad_turn_max_rate).
    #[must_use]
    pub fn with_pad_turn_max_rate(mut self, rad_per_s: f64) -> Self {
        self.pad_turn_max_rate = Some(rad_per_s);
        self
    }

    /// Whether this request would change nothing.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.pwm_band.is_none()
            && self.wheel_radius.is_none()
            && self.half_wheelbase.is_none()
            && self.half_track.is_none()
            && self.max_wheel_speed.is_none()
            && self.wheel_accel.is_none()
            && self.max_traction_accel.is_none()
            && self.parked_hold_gain.is_none()
            && self.pad_lift_max.is_none()
            && self.pad_lift_speed.is_none()
            && self.pad_turn_max_rate.is_none()
    }

    fn to_raw(self) -> sys::vrsdk_palletrobot_config_t {
        let mut raw = sys::vrsdk_palletrobot_config_t::default();
        // SAFETY: `raw` is writable storage for one
        // `vrsdk_palletrobot_config_t`.
        unsafe { sys::vrsdk_palletrobot_config_default(&mut raw) };
        let fields = [
            (
                self.wheel_radius,
                &mut raw.has_wheel_radius,
                &mut raw.wheel_radius,
            ),
            (
                self.half_wheelbase,
                &mut raw.has_half_wheelbase,
                &mut raw.half_wheelbase,
            ),
            (
                self.half_track,
                &mut raw.has_half_track,
                &mut raw.half_track,
            ),
            (
                self.max_wheel_speed,
                &mut raw.has_max_wheel_speed,
                &mut raw.max_wheel_speed,
            ),
            (
                self.wheel_accel,
                &mut raw.has_wheel_accel,
                &mut raw.wheel_accel,
            ),
            (
                self.max_traction_accel,
                &mut raw.has_max_traction_accel,
                &mut raw.max_traction_accel,
            ),
            (
                self.parked_hold_gain,
                &mut raw.has_parked_hold_gain,
                &mut raw.parked_hold_gain,
            ),
            (
                self.pad_lift_max,
                &mut raw.has_pad_lift_max,
                &mut raw.pad_lift_max,
            ),
            (
                self.pad_lift_speed,
                &mut raw.has_pad_lift_speed,
                &mut raw.pad_lift_speed,
            ),
            (
                self.pad_turn_max_rate,
                &mut raw.has_pad_turn_max_rate,
                &mut raw.pad_turn_max_rate,
            ),
        ];
        for (value, has, slot) in fields {
            if let Some(value) = value {
                *has = true;
                *slot = value;
            }
        }
        if let Some(band) = self.pwm_band {
            raw.has_pwm_band = true;
            raw.pwm_band = band.to_raw();
        }
        raw
    }
}

// ---------------------------------------------------------------------------
// srv/arm (RailArm, TableArm)
// ---------------------------------------------------------------------------

/// The [`configure_arm`](VirtualRobot::configure_arm) request: an arm's joint
/// speed fallbacks, its protective stop, and its tool. **Rail arm and table
/// arm only.**
///
/// One request for both arms: the rail arm reads the `suction_*` fields and
/// the table arm the `gripper_*` fields, and each **ignores the other arm's
/// tool fields**; the rest of the request still applies. SI, angles in rad.
/// The simulator heals an out-of-range value and acks it.
///
/// | Field | Default |
/// |---|---|
/// | `protective_stop` | on |
/// | `self_collision` | off on the rail arm, on on the table arm |
/// | `contact_tolerance`, `strike_speed`, `collision_rearm` | 0.005 m, 1 m/s, 1 s |
/// | `suction_capacity`, `suction_reach`, `suction_max_tilt`, `suction_accel_window` | 400 N, 0.01 m, 0.262 rad, 0.5 s |
/// | `gripper_speed`, `gripper_max_payload` | 0.15 m/s, 5 kg |
///
/// The speed fallbacks back only a joint whose driver reports no speed limit,
/// and every joint of both shipped arms reports one: on those rigs
/// `fallback_speed_limits` changes no speed.
///
/// ```
/// use vrobots_sdk::ArmConfig;
///
/// // A table arm that holds heavier objects and closes more slowly.
/// let config = ArmConfig::default()
///     .with_gripper_max_payload(8.0)
///     .with_gripper_speed(0.05);
/// assert!(!config.is_empty());
/// ```
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct ArmConfig {
    /// One speed limit per joint, in actuator order: m/s for the rail, rad/s
    /// for a revolute joint. Seven on the rail arm, six on the table arm; an
    /// entry `<= 0` means the simulator's generic 1 m/s or 90 deg/s. The C API
    /// carries at most 16.
    pub fallback_speed_limits: Option<Vec<f64>>,
    /// Refuse joint steps that would drive the arm into anything. Off lets the
    /// links pass through the scene.
    pub protective_stop: Option<bool>,
    /// Also refuse steps that would drive the arm into itself. Needs the
    /// protective stop.
    pub self_collision: Option<bool>,
    /// How far a link may sink into something before the step is refused, m.
    pub contact_tolerance: Option<f64>,
    /// Link closing speed above which a collision is a strike rather than a
    /// contact, m/s.
    pub strike_speed: Option<f64>,
    /// Time clear of an obstacle before meeting it again is a new collision
    /// event, s.
    pub collision_rearm: Option<f64>,
    /// Vacuum (rail arm): suction force, N.
    pub suction_capacity: Option<f64>,
    /// Vacuum (rail arm): the cup-to-surface gap within which it seals, m.
    pub suction_reach: Option<f64>,
    /// Vacuum (rail arm): the largest tilt that still seals, rad.
    pub suction_max_tilt: Option<f64>,
    /// Vacuum (rail arm): the span the held load's acceleration is averaged
    /// over, s.
    pub suction_accel_window: Option<f64>,
    /// Gripper (table arm): finger speed, m/s of opening.
    pub gripper_speed: Option<f64>,
    /// Gripper (table arm): the heaviest body the fingers hold, kg.
    pub gripper_max_payload: Option<f64>,
}

impl ArmConfig {
    /// See [`fallback_speed_limits`](Self::fallback_speed_limits).
    #[must_use]
    pub fn with_fallback_speed_limits(mut self, limits: &[f64]) -> Self {
        self.fallback_speed_limits = Some(limits.to_vec());
        self
    }

    /// See [`protective_stop`](Self::protective_stop).
    #[must_use]
    pub fn with_protective_stop(mut self, on: bool) -> Self {
        self.protective_stop = Some(on);
        self
    }

    /// See [`self_collision`](Self::self_collision).
    #[must_use]
    pub fn with_self_collision(mut self, on: bool) -> Self {
        self.self_collision = Some(on);
        self
    }

    /// See [`contact_tolerance`](Self::contact_tolerance).
    #[must_use]
    pub fn with_contact_tolerance(mut self, metres: f64) -> Self {
        self.contact_tolerance = Some(metres);
        self
    }

    /// See [`strike_speed`](Self::strike_speed).
    #[must_use]
    pub fn with_strike_speed(mut self, metres_per_s: f64) -> Self {
        self.strike_speed = Some(metres_per_s);
        self
    }

    /// See [`collision_rearm`](Self::collision_rearm).
    #[must_use]
    pub fn with_collision_rearm(mut self, seconds: f64) -> Self {
        self.collision_rearm = Some(seconds);
        self
    }

    /// See [`suction_capacity`](Self::suction_capacity).
    #[must_use]
    pub fn with_suction_capacity(mut self, newtons: f64) -> Self {
        self.suction_capacity = Some(newtons);
        self
    }

    /// See [`suction_reach`](Self::suction_reach).
    #[must_use]
    pub fn with_suction_reach(mut self, metres: f64) -> Self {
        self.suction_reach = Some(metres);
        self
    }

    /// See [`suction_max_tilt`](Self::suction_max_tilt).
    #[must_use]
    pub fn with_suction_max_tilt(mut self, rad: f64) -> Self {
        self.suction_max_tilt = Some(rad);
        self
    }

    /// See [`suction_accel_window`](Self::suction_accel_window).
    #[must_use]
    pub fn with_suction_accel_window(mut self, seconds: f64) -> Self {
        self.suction_accel_window = Some(seconds);
        self
    }

    /// See [`gripper_speed`](Self::gripper_speed).
    #[must_use]
    pub fn with_gripper_speed(mut self, metres_per_s: f64) -> Self {
        self.gripper_speed = Some(metres_per_s);
        self
    }

    /// See [`gripper_max_payload`](Self::gripper_max_payload).
    #[must_use]
    pub fn with_gripper_max_payload(mut self, kg: f64) -> Self {
        self.gripper_max_payload = Some(kg);
        self
    }

    /// Whether this request would change nothing.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.fallback_speed_limits.is_none()
            && self.protective_stop.is_none()
            && self.self_collision.is_none()
            && self.contact_tolerance.is_none()
            && self.strike_speed.is_none()
            && self.collision_rearm.is_none()
            && self.suction_capacity.is_none()
            && self.suction_reach.is_none()
            && self.suction_max_tilt.is_none()
            && self.suction_accel_window.is_none()
            && self.gripper_speed.is_none()
            && self.gripper_max_payload.is_none()
    }

    fn to_raw(&self) -> sys::vrsdk_arm_config_t {
        let mut raw = sys::vrsdk_arm_config_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_arm_config_t`.
        unsafe { sys::vrsdk_arm_config_default(&mut raw) };
        if let Some(limits) = &self.fallback_speed_limits {
            raw.has_fallback_speed_limits = true;
            // The true count, even past the array: the library refuses a count
            // beyond its 16 by name before it reads an entry.
            raw.fallback_speed_limit_count = u32::try_from(limits.len()).unwrap_or(u32::MAX);
            let used = limits.len().min(raw.fallback_speed_limits.len());
            raw.fallback_speed_limits[..used].copy_from_slice(&limits[..used]);
        }
        if let Some(on) = self.protective_stop {
            raw.has_protective_stop = true;
            raw.protective_stop = on;
        }
        if let Some(on) = self.self_collision {
            raw.has_self_collision = true;
            raw.self_collision = on;
        }
        let fields = [
            (
                self.contact_tolerance,
                &mut raw.has_contact_tolerance,
                &mut raw.contact_tolerance,
            ),
            (
                self.strike_speed,
                &mut raw.has_strike_speed,
                &mut raw.strike_speed,
            ),
            (
                self.collision_rearm,
                &mut raw.has_collision_rearm,
                &mut raw.collision_rearm,
            ),
            (
                self.suction_capacity,
                &mut raw.has_suction_capacity,
                &mut raw.suction_capacity,
            ),
            (
                self.suction_reach,
                &mut raw.has_suction_reach,
                &mut raw.suction_reach,
            ),
            (
                self.suction_max_tilt,
                &mut raw.has_suction_max_tilt,
                &mut raw.suction_max_tilt,
            ),
            (
                self.suction_accel_window,
                &mut raw.has_suction_accel_window,
                &mut raw.suction_accel_window,
            ),
            (
                self.gripper_speed,
                &mut raw.has_gripper_speed,
                &mut raw.gripper_speed,
            ),
            (
                self.gripper_max_payload,
                &mut raw.has_gripper_max_payload,
                &mut raw.gripper_max_payload,
            ),
        ];
        for (value, has, slot) in fields {
            if let Some(value) = value {
                *has = true;
                *slot = value;
            }
        }
        raw
    }
}

// ---------------------------------------------------------------------------
// the service calls
// ---------------------------------------------------------------------------

/// Services. Each is a query with a receipt for an answer; see the
/// [module documentation](crate::services).
impl VirtualRobot {
    /// Release a dormant robot's dynamics hold. `srv/activate`.
    ///
    /// Half of the deterministic experiment start: connect with
    /// [`ConnectOptions::with_start_active`](crate::ConnectOptions::with_start_active)
    /// and [`with_activate_after_create`](crate::ConnectOptions::with_activate_after_create)
    /// both `false`, configure the held robot, then call this. Idempotent: an
    /// already-active robot acks and does nothing.
    ///
    /// # Errors
    ///
    /// [`VrError::NoResponder`](crate::VrError::NoResponder) or
    /// [`VrError::Timeout`](crate::VrError::Timeout) if nothing answers,
    /// [`VrError::Service`](crate::VrError::Service) if the robot refuses,
    /// [`VrError::Deleted`](crate::VrError::Deleted).
    pub fn activate(&self) -> VrResult<()> {
        // SAFETY: the robot handle is live.
        check(unsafe { sys::vrsdk_robot_activate(self.raw()) })
    }

    /// Return the robot to its home pose. `srv/reset`.
    ///
    /// Teleports it to the pose captured at its first physics step, zeroes the
    /// velocities, rests the actuators and re-latches the initial command. On a
    /// fixed wing it also reverts the control mode to onboard and the estimate
    /// source to truth. Configuration sent through the other services survives:
    /// it is a state reset, not a factory reset.
    ///
    /// # Errors
    ///
    /// As [`activate`](Self::activate).
    pub fn reset(&self) -> VrResult<()> {
        // SAFETY: the robot handle is live.
        check(unsafe { sys::vrsdk_robot_reset(self.raw()) })
    }

    /// Set the robot's mass and inertia. `srv/params`.
    ///
    /// Works mid-flight. The confirmation is the robot's changed response, as
    /// neither figure appears in the state message.
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) if nothing
    /// is set, or if the mass or a moment of inertia is not strictly positive
    /// and finite (the simulator would silently keep its own); the errors of
    /// [`activate`](Self::activate) otherwise.
    pub fn set_physical_params(&self, params: &PhysicalParams) -> VrResult<()> {
        let raw = params.to_raw();
        // SAFETY: the robot handle is live and `raw` is an initialised request.
        check(unsafe { sys::vrsdk_robot_set_physical_params(self.raw(), &raw) })
    }

    /// Change the robot's skin. `srv/skin`.
    ///
    /// Catalog keys, case-insensitive: a multirotor wears `blue`, `desert`,
    /// `gold`, `green`, `mono`, `pink`, `snow` or `white`; a truck wears
    /// `black`, `blue`, `camouflage`, `gray` or `red`. **One of the two
    /// services that say no** (the other is
    /// [`configure_lidars`](Self::configure_lidars)): skins are tier-gated, and
    /// a refusal is final, so do not retry. An unknown key, by contrast, is
    /// acked and ignored.
    ///
    /// # Errors
    ///
    /// [`VrError::Service`](crate::VrError::Service) carrying the simulator's
    /// reason for a refusal; [`VrError::InvalidArgument`](crate::VrError::InvalidArgument)
    /// for an empty key; the errors of [`activate`](Self::activate).
    pub fn set_skin(&self, skin: &str) -> VrResult<()> {
        let skin = ffi::c_string(skin, "skin")?;
        // SAFETY: the robot handle is live and `skin` is a NUL-terminated string
        // alive for the call.
        check(unsafe { sys::vrsdk_robot_set_skin(self.raw(), skin.as_ptr()) })
    }

    /// Configure the robot's sensor noise models. `srv/sensors`.
    ///
    /// Live from the next sensor sample, and visible: the noise floor in
    /// `states().sensors` is the confirmation.
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) if nothing
    /// is set or a value is not finite; the errors of
    /// [`activate`](Self::activate).
    pub fn configure_sensors(&self, config: &SensorConfig) -> VrResult<()> {
        let raw = config.to_raw();
        // SAFETY: the robot handle is live and `raw` is an initialised request.
        check(unsafe { sys::vrsdk_robot_configure_sensors(self.raw(), &raw) })
    }

    /// Set which coordinate frames the robot and its devices report in.
    /// `srv/frames`.
    ///
    /// A presentation change, not a physical one: the motion is identical and
    /// the numbers describing it are permuted. Frames resolve device override
    /// first, then robot, then scene. `None` (or `""`) leaves the robot's level
    /// alone; [`INHERIT_FRAME`] clears it. The `z/frames` topic and the frame on
    /// every later state are the confirmation. A lidar is device
    /// [`device::lidar`], and its override re-labels the points, never the
    /// ranges.
    ///
    /// ```no_run
    /// # use vrobots_sdk::{device, DeviceFrame, RobotType, VirtualRobot};
    /// # let robot = VirtualRobot::connect(RobotType::Truck, Some(0))?;
    /// // The whole robot in FRD, but the gyro left in FRU.
    /// robot.set_frames(Some("frd"), &[DeviceFrame::new(device::GYROSCOPE, "fru")])?;
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) if nothing
    /// is set, or an entry has an empty device or frame id; the errors of
    /// [`activate`](Self::activate).
    pub fn set_frames(
        &self,
        robot_frame_id: Option<&str>,
        devices: &[DeviceFrame],
    ) -> VrResult<()> {
        let robot_frame = robot_frame_id
            .map(|id| ffi::c_string(id, "robot frame id"))
            .transpose()?;
        let strings = devices
            .iter()
            .map(|entry| {
                Ok((
                    ffi::c_string(&entry.device, "device")?,
                    ffi::c_string(&entry.coord_frame_id, "coord_frame_id")?,
                ))
            })
            .collect::<VrResult<Vec<_>>>()?;
        let entries: Vec<sys::vrsdk_device_frame_t> = strings
            .iter()
            .map(|(device, frame)| sys::vrsdk_device_frame_t {
                device: device.as_ptr(),
                coord_frame_id: frame.as_ptr(),
            })
            .collect();
        let robot_frame_ptr = robot_frame.as_ref().map_or(ptr::null(), |s| s.as_ptr());
        let entries_ptr = if entries.is_empty() {
            ptr::null()
        } else {
            entries.as_ptr()
        };
        // SAFETY: the robot handle is live; `robot_frame_ptr` is NULL or a
        // NUL-terminated string, and `entries_ptr` is NULL or readable for
        // `entries.len()` entries whose strings live in `strings`; everything
        // outlives the call.
        check(unsafe {
            sys::vrsdk_robot_set_frames(self.raw(), robot_frame_ptr, entries_ptr, entries.len())
        })
    }

    /// Read the scene's active coordinate frame. Scene scope: the same answer
    /// for every robot loaded, read and changing nothing.
    ///
    /// # Errors
    ///
    /// [`VrError::NoResponder`](crate::VrError::NoResponder) or
    /// [`VrError::Timeout`](crate::VrError::Timeout) when no scene is loaded or
    /// the simulator is not in Play mode.
    pub fn scene_frame(&self) -> VrResult<SceneFrame> {
        let mut raw = sys::vrsdk_scene_frame_t::default();
        // SAFETY: the robot handle is live and `raw` is writable storage for one
        // `vrsdk_scene_frame_t`.
        check(unsafe { sys::vrsdk_robot_scene_frame(self.raw(), &mut raw) })?;
        Ok(SceneFrame {
            coord_frame_id: fixed_str(&raw.coord_frame_id),
            axis_convention: Axes(raw.axis_convention),
        })
    }

    /// Read one coordinate-frame definition off the robot's `z/frames` topic.
    ///
    /// A one-shot read: it takes the first definition that arrives. On a robot
    /// whose devices report in frames of their own, which one arrives first is
    /// not something to rely on.
    ///
    /// # Errors
    ///
    /// [`VrError::Timeout`](crate::VrError::Timeout) if none arrives within
    /// [`ConnectOptions::probe_timeout`](crate::ConnectOptions::probe_timeout);
    /// [`VrError::Decode`](crate::VrError::Decode) for a malformed payload.
    pub fn frame_def(&self) -> VrResult<FrameDef> {
        let mut raw = sys::vrsdk_frame_def_t::default();
        // SAFETY: the robot handle is live and `raw` is writable storage for one
        // `vrsdk_frame_def_t`.
        check(unsafe { sys::vrsdk_robot_frame_def(self.raw(), &mut raw) })?;
        Ok(FrameDef::from_raw(&raw))
    }

    /// Configure a truck's drivetrain. `srv/drive`. **Truck only.**
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) if nothing
    /// is set, `drive_mode` is neither 2 nor 4, or a value is not finite;
    /// [`VrError::NoResponder`](crate::VrError::NoResponder) if the robot is not
    /// a truck.
    pub fn configure_drive(&self, config: &DriveConfig) -> VrResult<()> {
        let raw = config.to_raw();
        // SAFETY: the robot handle is live and `raw` is an initialised request.
        check(unsafe { sys::vrsdk_robot_configure_drive(self.raw(), &raw) })
    }

    /// Retune a multirotor's rotors. `srv/rotors`. **Multirotor only.**
    ///
    /// The list **replaces** the whole rotor list, so it must describe every
    /// rotor in index order; the count (`states().actuator.pwm.len()`) is fixed
    /// at spawn, and a list of the wrong length is dropped whole by the
    /// simulator and still acknowledged.
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) for an
    /// empty list; [`VrError::NoResponder`](crate::VrError::NoResponder) if the
    /// robot is not a multirotor.
    pub fn configure_rotors(&self, rotors: &[RotorSpec]) -> VrResult<()> {
        let raw: Vec<sys::vrsdk_rotor_spec_t> = rotors.iter().map(|r| r.to_raw()).collect();
        // SAFETY: the robot handle is live and `raw` is readable for
        // `raw.len()` entries; an empty list passes a dangling but aligned,
        // non-null pointer with a zero count.
        check(unsafe { sys::vrsdk_robot_configure_rotors(self.raw(), raw.as_ptr(), raw.len()) })
    }

    /// Set a mass-spring-damper's spring and damping constants. `srv/msd`.
    /// **Msd only.**
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) if nothing
    /// is set or a constant is negative or not finite (the simulator would
    /// commit a negative as zero); [`VrError::NoResponder`](crate::VrError::NoResponder)
    /// if the robot is not an MSD.
    pub fn configure_msd(&self, config: &MsdConfig) -> VrResult<()> {
        let raw = config.to_raw();
        // SAFETY: the robot handle is live and `raw` is an initialised request.
        check(unsafe { sys::vrsdk_robot_configure_msd(self.raw(), &raw) })
    }

    /// Set a cart-pole's masses, lengths and limits. `srv/cartpole`.
    /// **CartPole only.**
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) if nothing
    /// is set or a mass, length or force limit is not strictly positive;
    /// [`VrError::NoResponder`](crate::VrError::NoResponder) if the robot is not
    /// a cart-pole.
    pub fn configure_cartpole(&self, config: &CartPoleConfig) -> VrResult<()> {
        let raw = config.to_raw();
        // SAFETY: the robot handle is live and `raw` is an initialised request.
        check(unsafe { sys::vrsdk_robot_configure_cartpole(self.raw(), &raw) })
    }

    /// Set a turret's joint limits and max rates, its laser's range and its
    /// stereo pair. `srv/turret`. **Turret only.**
    ///
    /// ```no_run
    /// # use vrobots_sdk::{RobotType, TurretConfig, VirtualRobot};
    /// # let turret_id = 3;
    /// let turret = VirtualRobot::connect(RobotType::Turret, Some(turret_id))?;
    /// // 30 degrees each side, and a laser that reaches 50 m.
    /// turret.configure_turret(
    ///     &TurretConfig::default()
    ///         .with_heading_min(-0.5236)
    ///         .with_heading_max(0.5236)
    ///         .with_laser_max_range(50.0),
    /// )?;
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    ///
    /// Limits that exclude the current pose do not snap it: the joint walks
    /// back to the nearest limit. Everything set here survives
    /// [`reset`](Self::reset).
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) if nothing
    /// is set, a value is not finite, a rate, baseline, focal length or laser
    /// range is not strictly positive, or a min and max sent together are
    /// reversed: the simulator acks whatever it does with those.
    /// [`VrError::NoResponder`](crate::VrError::NoResponder) if the robot is not
    /// a turret; the errors of [`activate`](Self::activate) otherwise.
    pub fn configure_turret(&self, config: &TurretConfig) -> VrResult<()> {
        let raw = config.to_raw();
        // SAFETY: the robot handle is live and `raw` is an initialised request.
        check(unsafe { sys::vrsdk_robot_configure_turret(self.raw(), &raw) })
    }

    /// Tune a forklift's drive, rear steering and mast. `srv/forklift`.
    /// **Forklift only.**
    ///
    /// ```no_run
    /// # use vrobots_sdk::{ForkliftConfig, RobotType, VirtualRobot};
    /// let forklift = VirtualRobot::connect(RobotType::Forklift, Some(0))?;
    /// // A gentler forklift: half the top speed, a slower lift.
    /// forklift.configure_forklift(
    ///     &ForkliftConfig::default()
    ///         .with_no_load_wheel_speed(7.0)
    ///         .with_lift_speed(0.25),
    /// )?;
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    ///
    /// The drive, the steering and the mast read their settings live; the
    /// `initial_*_us` pulses wait for the next [`reset`](Self::reset).
    /// Everything set here survives a reset.
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) if nothing
    /// is set or a value is not finite, also once converted to degrees or rpm
    /// as the simulator does; [`VrError::NoResponder`](crate::VrError::NoResponder)
    /// if the robot is not a forklift; the errors of
    /// [`activate`](Self::activate) otherwise.
    pub fn configure_forklift(&self, config: &ForkliftConfig) -> VrResult<()> {
        let raw = config.to_raw();
        // SAFETY: the robot handle is live and `raw` is an initialised request.
        check(unsafe { sys::vrsdk_robot_configure_forklift(self.raw(), &raw) })
    }

    /// Tune a pallet robot's pulse band, virtual mecanum geometry, drive and
    /// pad. `srv/palletrobot`. **PalletRobot only.**
    ///
    /// The drive reads its settings live. Everything set here survives
    /// [`reset`](Self::reset).
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) if nothing
    /// is set or a value is not finite, the pad rate also once converted to
    /// degrees; [`VrError::NoResponder`](crate::VrError::NoResponder) if the
    /// robot is not a pallet robot; the errors of [`activate`](Self::activate)
    /// otherwise.
    pub fn configure_palletrobot(&self, config: &PalletRobotConfig) -> VrResult<()> {
        let raw = config.to_raw();
        // SAFETY: the robot handle is live and `raw` is an initialised request.
        check(unsafe { sys::vrsdk_robot_configure_palletrobot(self.raw(), &raw) })
    }

    /// Tune an arm's joint speed fallbacks, its protective stop and its tool.
    /// `srv/arm`. **Rail arm and table arm only.**
    ///
    /// ```no_run
    /// # use vrobots_sdk::{ArmConfig, RobotType, VirtualRobot};
    /// let arm = VirtualRobot::connect(RobotType::TableArm, Some(0))?;
    /// arm.configure_arm(&ArmConfig::default().with_gripper_max_payload(8.0))?;
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    ///
    /// The protective stop and the tools read their settings every step.
    /// Everything set here survives [`reset`](Self::reset).
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) if nothing
    /// is set, a value is not finite, or the speed fallbacks are empty, more
    /// than 16, or (on a handle connected as a rail or table arm) not that
    /// arm's joint count; [`VrError::NoResponder`](crate::VrError::NoResponder)
    /// if the robot is not an arm; the errors of [`activate`](Self::activate)
    /// otherwise.
    pub fn configure_arm(&self, config: &ArmConfig) -> VrResult<()> {
        let raw = config.to_raw();
        // SAFETY: the robot handle is live and `raw` is an initialised request.
        check(unsafe { sys::vrsdk_robot_configure_arm(self.raw(), &raw) })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_unset_request_is_empty_and_sets_no_flag() {
        assert!(PhysicalParams::default().is_empty());
        let raw = PhysicalParams::default().to_raw();
        assert!(!raw.has_mass && !raw.has_moi);

        let raw = SensorConfig::default().to_raw();
        assert!(!raw.has_gyro_noise && !raw.has_gps_quality && !raw.has_optical_flow_mounted);
        // Unset blocks are still pre-filled with a working sensor.
        assert_eq!(raw.gyro_noise.scale_factor, [1.0; 3]);

        let raw = CartPoleConfig::default().to_raw();
        assert!(!raw.has_cart_mass && !raw.has_initial_pole_angle_deg);
    }

    #[test]
    fn a_set_field_raises_exactly_its_flag() {
        let raw = PhysicalParams::default().with_mass(1.6).to_raw();
        assert!(raw.has_mass && !raw.has_moi);
        assert_eq!(raw.mass, 1.6);

        let raw = DriveConfig::default()
            .with_drive_mode(2)
            .with_pwm_band(PwmBand::new(1100, 1500, 1900, 30))
            .to_raw();
        assert!(raw.has_drive_mode && raw.has_pwm_band && !raw.has_max_steer_deg);
        assert_eq!(raw.pwm_band.deadband_us, 30);

        let raw = CartPoleConfig::default()
            .with_pole_length(1.2)
            .with_initial_pole_angle_deg(-3.0)
            .to_raw();
        assert!(raw.has_pole_length && raw.has_initial_pole_angle_deg && !raw.has_bob_mass);
        assert_eq!((raw.pole_length, raw.initial_pole_angle_deg), (1.2, -3.0));

        let raw = MsdConfig::default().with_damping_c(16.0).to_raw();
        assert!(raw.has_damping_c && !raw.has_spring_k);
    }

    #[test]
    fn defaults_come_from_the_library() {
        let imu = ImuNoise::ideal();
        assert_eq!(imu.scale_factor, [1.0; 3]);
        assert_eq!(imu.white_std, [0.0; 3]);
        assert_eq!(ImuNoise::default(), imu);

        let q = GpsQuality::default();
        assert_eq!((q.eph, q.epv, q.fix_type), (1.5, 3.0, 3));
        assert_eq!(GpsNoise::default().position_std, [0.0; 3]);

        let rotor = RotorSpec::default();
        assert_eq!((rotor.pwm_min_us, rotor.pwm_max_us), (1100, 2000));
        assert_eq!(rotor.spin_dir, 0.0);
        assert_eq!(RotorSpec::from_raw_for_test(rotor.to_raw()), rotor);
    }

    impl RotorSpec {
        fn from_raw_for_test(raw: sys::vrsdk_rotor_spec_t) -> RotorSpec {
            RotorSpec {
                position: raw.position,
                spin_dir: raw.spin_dir,
                thrust_a: raw.thrust_a,
                thrust_b: raw.thrust_b,
                thrust_c: raw.thrust_c,
                torque_a: raw.torque_a,
                torque_b: raw.torque_b,
                torque_c: raw.torque_c,
                ang_vel_slope: raw.ang_vel_slope,
                ang_vel_intercept: raw.ang_vel_intercept,
                pwm_min_us: raw.pwm_min_us,
                pwm_max_us: raw.pwm_max_us,
            }
        }
    }

    #[test]
    fn device_names_are_the_exact_strings_the_simulator_matches() {
        assert_eq!(device::OPTICAL_FLOW, "optical_flow");
        assert_eq!(device::camera("front_left"), "camera/front_left");
        assert_eq!(device::lidar("lidar_front_left"), "lidar/lidar_front_left");
        assert_eq!(INHERIT_FRAME, "inherit");
    }

    #[test]
    fn the_robot_requests_raise_exactly_their_flags() {
        let raw = TurretConfig::default().to_raw();
        assert!(!raw.has_heading_min && !raw.has_laser_max_range && !raw.has_stereo_forward);
        let raw = TurretConfig::default()
            .with_heading_min(-0.5)
            .with_stereo_forward(0.3)
            .to_raw();
        assert!(raw.has_heading_min && raw.has_stereo_forward && !raw.has_heading_max);
        assert_eq!((raw.heading_min, raw.stereo_forward), (-0.5, 0.3));

        let raw = ForkliftConfig::default()
            .with_lift_speed(0.25)
            .with_initial_fork_width_us(1301)
            .with_pwm_band(PwmBand::new(1000, 1500, 2000, 25))
            .to_raw();
        assert!(raw.has_lift_speed && raw.has_initial_fork_width_us && raw.has_pwm_band);
        assert!(!raw.has_lift_min && !raw.has_initial_steer_us);
        assert_eq!(
            (
                raw.lift_speed,
                raw.initial_fork_width_us,
                raw.pwm_band.min_us
            ),
            (0.25, 1301, 1000)
        );

        let raw = PalletRobotConfig::default()
            .with_pad_turn_max_rate(1.0)
            .to_raw();
        assert!(raw.has_pad_turn_max_rate && !raw.has_pwm_band && !raw.has_wheel_radius);

        let raw = ArmConfig::default()
            .with_protective_stop(false)
            .with_gripper_max_payload(8.0)
            .to_raw();
        assert!(raw.has_protective_stop && !raw.protective_stop);
        assert!(raw.has_gripper_max_payload && !raw.has_suction_capacity);
        assert!(!raw.has_fallback_speed_limits);
    }

    #[test]
    fn arm_speed_fallbacks_carry_their_true_count() {
        let raw = ArmConfig::default()
            .with_fallback_speed_limits(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
            .to_raw();
        assert!(raw.has_fallback_speed_limits);
        assert_eq!(raw.fallback_speed_limit_count, 6);
        assert_eq!(
            raw.fallback_speed_limits[..6],
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        );

        // More than the C API carries: the count stays true so the library
        // refuses it by name; nothing is written past the array.
        let raw = ArmConfig::default()
            .with_fallback_speed_limits(&[1.0; 20])
            .to_raw();
        assert_eq!(raw.fallback_speed_limit_count, 20);
    }
}
