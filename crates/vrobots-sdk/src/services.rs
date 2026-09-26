//! Services: reset, activate, and the configuration requests.
//!
//! Each service is a query the robot answers with an acknowledgement, and
//! **the ack is a receipt, not a result**: the simulator replies the instant the
//! request lands and applies it on the next physics step. Only
//! [`set_skin`](VirtualRobot::set_skin) ever answers "no"; an unknown frame id,
//! a wrong rotor count or a nonsense drive mode are all acknowledged and then
//! refused by a log line inside the simulator. So the SDK refuses what it can
//! check before sending ([`VrError::InvalidArgument`](crate::VrError::InvalidArgument)),
//! and the confirmation for the rest is always the state stream.
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
    /// A registered frame id (`"unity"`, `"frd"`, `"fru"`, `"cv"` or one the
    /// scene registered), or [`INHERIT_FRAME`] to clear the override.
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

/// A truck's pulse-width band, microseconds. One flag covers all four numbers.
///
/// The factory band is 1100 / 1500 / 1900. The simulator replaces the whole
/// band with the factory values unless `min_us < neutral_us < max_us`.
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
            raw.pwm_band = sys::vrsdk_pwm_band_t {
                min_us: band.min_us,
                neutral_us: band.neutral_us,
                max_us: band.max_us,
                deadband_us: band.deadband_us,
            };
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
    /// `black`, `blue`, `camouflage`, `gray` or `red`. **The one service that
    /// says no**: skins are tier-gated, and a refusal is final, so do not retry.
    /// An unknown key, by contrast, is acked and ignored.
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
    /// every later state are the confirmation.
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
        assert_eq!(INHERIT_FRAME, "inherit");
    }
}
