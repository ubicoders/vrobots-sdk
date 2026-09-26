//! State snapshots: what a robot reports about itself, and the frame tags that
//! say which convention every vector is in.
//!
//! A [`State`] keeps three kinds of knowledge apart on purpose: `kin`, `wrench`
//! and `env` are simulator **truth**, `sensors` is the noisy **measured** view
//! of the same instant, and `estimate` is what the robot's own filter
//! **believes**, so `estimate.kin - kin` is the estimator error. Everything is
//! SI and expressed in the frame named by `coord_frame_id`, the robot's frame
//! and not yours; pose is a world quantity, twist and acceleration are body
//! quantities.

use vrobots_sdk_sys as sys;

use crate::ffi::{self, fixed_str, unflatten};

/// The axis convention a vector is expressed in: the enum tag beside a
/// `coord_frame_id`.
///
/// A transparent `i32` rather than a closed enum, because a frame the scene
/// registers at run time has no tag value and an unknown value must survive a
/// round trip. `coord_frame_id` is the authoritative name; this is the
/// convenience beside it. Note that the scene default `"fru"` has no tag at all
/// and reports [`Axes::UNSPECIFIED`].
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Axes(pub i32);

impl Axes {
    /// No convention declared by the sender.
    pub const UNSPECIFIED: Axes = Axes(sys::VRSDK_AXES_UNSPECIFIED);
    /// Unity's left-handed x-right, y-up, z-forward.
    pub const UNITY: Axes = Axes(sys::VRSDK_AXES_UNITY);
    /// Aerospace forward-right-down.
    pub const FRD: Axes = Axes(sys::VRSDK_AXES_FRD);
    /// Computer-vision right-down-forward.
    pub const CV: Axes = Axes(sys::VRSDK_AXES_CV);

    /// The registry id for this tag, `"unity"`, `"frd"` or `"cv"`, or `""` for
    /// [`UNSPECIFIED`](Self::UNSPECIFIED) and anything unrecognised. The
    /// library's own `vrsdk_axes_name`.
    #[must_use]
    pub fn name(self) -> &'static str {
        // SAFETY: `vrsdk_axes_name` accepts any value and returns a static,
        // NUL-terminated literal, never NULL.
        unsafe { ffi::static_str(sys::vrsdk_axes_name(self.0)) }
    }
}

/// The order a frame reports Euler angles in, as the wire tags it.
///
/// A transparent `i32` for the same reason [`Axes`] is: a value that names no
/// order must survive a round trip. The rotation functions take the closed
/// [`rotations::EulerOrder`](crate::rotations::EulerOrder) instead; bridge the
/// two with `rotations::EulerOrder::from_wire(tag.0)` and handle the `None`.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct EulerOrder(pub i32);

impl EulerOrder {
    /// No order declared by the sender.
    pub const UNSPECIFIED: EulerOrder = EulerOrder(sys::VRSDK_EULER_UNSPECIFIED);
    /// About x, then the new y, then the newest z.
    pub const XYZ: EulerOrder = EulerOrder(sys::VRSDK_EULER_XYZ);
    /// About x, then the new z, then the newest y.
    pub const XZY: EulerOrder = EulerOrder(sys::VRSDK_EULER_XZY);
    /// About y, then the new x, then the newest z.
    pub const YXZ: EulerOrder = EulerOrder(sys::VRSDK_EULER_YXZ);
    /// About y, then the new z, then the newest x.
    pub const YZX: EulerOrder = EulerOrder(sys::VRSDK_EULER_YZX);
    /// About z, then the new x, then the newest y. Unity's native order.
    pub const ZXY: EulerOrder = EulerOrder(sys::VRSDK_EULER_ZXY);
    /// About z, then the new y, then the newest x. The aerospace order.
    pub const ZYX: EulerOrder = EulerOrder(sys::VRSDK_EULER_ZYX);

    /// The lowercase name, `"zyx"` and so on, or `""` when the value names no
    /// order. The library's own `vrsdk_euler_order_name`.
    #[must_use]
    pub fn name(self) -> &'static str {
        // SAFETY: `vrsdk_euler_order_name` accepts any value and returns a
        // static, NUL-terminated literal, never NULL.
        unsafe { ffi::static_str(sys::vrsdk_euler_order_name(self.0)) }
    }
}

/// What a coordinate frame is, rather than a reference to one: the nine numbers
/// read off a robot's `z/frames` topic by
/// [`frame_def`](crate::VirtualRobot::frame_def).
///
/// Every other message only names its frame. The simulator's frame registry is
/// string-keyed, so a scene can register a convention at run time that has no
/// [`Axes`] value; [`rows`](Self::rows) still describes it, and
/// [`AxisBasis::from_frame_def`](crate::AxisBasis::from_frame_def) turns it into
/// something the rotation functions can use.
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct FrameDef {
    /// Registry id, the join key: matches the `coord_frame_id` on every message
    /// this frame applies to.
    pub id: String,
    /// Human-readable label, e.g. `"FRD (aero body)"`. Display only.
    pub name: String,
    /// The built-in this frame corresponds to, or [`Axes::UNSPECIFIED`] for one
    /// registered at run time.
    pub axis_convention: Axes,
    /// The angle order this frame reports in, or [`EulerOrder::UNSPECIFIED`]
    /// when the publisher left it out.
    pub euler_order: EulerOrder,
    /// `R`, the signed-permutation matrix from Unity into this frame, row-major:
    /// `R * v` re-expresses a Unity vector in frame components.
    pub rows: [[f64; 3]; 3],
}

impl FrameDef {
    pub(crate) fn from_raw(raw: &sys::vrsdk_frame_def_t) -> FrameDef {
        FrameDef {
            id: fixed_str(&raw.id),
            name: fixed_str(&raw.name),
            axis_convention: Axes(raw.axis_convention),
            euler_order: EulerOrder(raw.euler_order),
            rows: unflatten(raw.rows),
        }
    }

    pub(crate) fn to_raw(&self) -> sys::vrsdk_frame_def_t {
        let mut raw = sys::vrsdk_frame_def_t::default();
        ffi::write_fixed_str(&mut raw.id, &self.id);
        ffi::write_fixed_str(&mut raw.name, &self.name);
        raw.axis_convention = self.axis_convention.0;
        raw.euler_order = self.euler_order.0;
        raw.rows = ffi::flatten(self.rows);
        raw
    }
}

/// Position, attitude and their derivatives.
///
/// `lin_pos` and `quat` are **world** quantities; the rest are **body**
/// quantities.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct Kinematics {
    /// World position, metres.
    pub lin_pos: [f64; 3],
    /// Attitude relative to world as a unit quaternion, ordered `[x, y, z, w]`.
    pub quat: [f64; 4],
    /// Body-frame linear velocity, m/s.
    pub lin_vel: [f64; 3],
    /// Body-frame angular velocity, rad/s: what a gyro measures.
    pub ang_vel: [f64; 3],
    /// Body-frame linear acceleration, m/s^2.
    pub lin_acc: [f64; 3],
    /// Body-frame angular acceleration, rad/s^2.
    pub ang_acc: [f64; 3],
}

impl Kinematics {
    fn from_raw(raw: &sys::vrsdk_kinematics_t) -> Kinematics {
        Kinematics {
            lin_pos: raw.lin_pos,
            quat: raw.quat,
            lin_vel: raw.lin_vel,
            ang_vel: raw.ang_vel,
            lin_acc: raw.lin_acc,
            ang_acc: raw.ang_acc,
        }
    }
}

/// Net force and torque actually applied to the body this step (truth).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct Wrench {
    /// Net force, N.
    pub force: [f64; 3],
    /// Net torque, N.m.
    pub torque: [f64; 3],
}

/// What the actuators were told and what they did.
///
/// The three vectors are index-parallel: entry `i` is the same device in each.
/// Each has its own length because the simulator does not have to report all
/// three for every device.
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct Actuator {
    /// Commanded pulse widths, microseconds: **your last command echoed back**,
    /// the round-trip check that a command landed.
    pub pwm: Vec<u32>,
    /// The same command mapped to `[-1, 1]`.
    pub normalized: Vec<f64>,
    /// What the device actually did: rotor rad/s, wheel rad/s, servo rad,
    /// panel rad or engine newtons, depending on the robot.
    pub measured: Vec<f64>,
    /// True when the robot reported more devices than the C API carries (16)
    /// and the vectors above were cut short.
    pub truncated: bool,
}

impl Actuator {
    fn from_raw(raw: &sys::vrsdk_actuator_t) -> Actuator {
        fn prefix<T: Copy>(values: &[T], count: u32) -> Vec<T> {
            let count = usize::try_from(count).unwrap_or(usize::MAX);
            values[..count.min(values.len())].to_vec()
        }
        Actuator {
            pwm: prefix(&raw.pwm, raw.pwm_count),
            normalized: prefix(&raw.normalized, raw.normalized_count),
            measured: prefix(&raw.measured, raw.measured_count),
            truncated: raw.truncated,
        }
    }
}

/// A geodetic position.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct GeoPoint {
    /// Degrees.
    pub latitude: f64,
    /// Degrees.
    pub longitude: f64,
    /// Metres.
    pub altitude: f64,
}

impl GeoPoint {
    fn from_raw(raw: &sys::vrsdk_geo_point_t) -> GeoPoint {
        GeoPoint {
            latitude: raw.latitude,
            longitude: raw.longitude,
            altitude: raw.altitude,
        }
    }
}

/// Three-axis accelerometer (measured).
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct Accelerometer {
    /// Sensor capture time, seconds since the unix epoch: the sensor's own
    /// clock, not the header's.
    pub timestamp: f64,
    /// False until the device has produced a usable reading.
    pub valid: bool,
    /// Specific force, m/s^2: +1 g upward at rest.
    pub linear_acceleration: [f64; 3],
    /// Mounting convention, when it differs from the robot's.
    pub axis_convention: Axes,
    /// Mounting frame id, when it differs from the robot's.
    pub coord_frame_id: String,
}

/// Three-axis rate gyro (measured).
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct Gyroscope {
    /// Sensor capture time, seconds since the unix epoch.
    pub timestamp: f64,
    /// False until the device has produced a usable reading.
    pub valid: bool,
    /// Body rates, rad/s.
    pub angular_velocity: [f64; 3],
    /// Mounting convention, when it differs from the robot's.
    pub axis_convention: Axes,
    /// Mounting frame id, when it differs from the robot's.
    pub coord_frame_id: String,
}

/// Three-axis magnetometer (measured).
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct Magnetometer {
    /// Sensor capture time, seconds since the unix epoch.
    pub timestamp: f64,
    /// False until the device has produced a usable reading.
    pub valid: bool,
    /// Magnetic field vector.
    pub magnetic_field: [f64; 3],
    /// Mounting convention, when it differs from the robot's.
    pub axis_convention: Axes,
    /// Mounting frame id, when it differs from the robot's.
    pub coord_frame_id: String,
}

/// Barometric pressure sensor (measured).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct Barometer {
    /// Sensor capture time, seconds since the unix epoch.
    pub timestamp: f64,
    /// False until the device has produced a usable reading.
    pub valid: bool,
    /// Static pressure, Pa. Compare against `env.air_pressure` for the error.
    pub pressure: f64,
    /// Pressure altitude, m.
    pub altitude: f64,
    /// Reference sea-level pressure used for `altitude`, Pa.
    pub qnh: f64,
}

/// Satellite navigation receiver (measured). Runs at about 5 Hz, so the same
/// reading repeats across several state samples: check `timestamp`.
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct Gnss {
    /// Sensor capture time, seconds since the unix epoch.
    pub timestamp: f64,
    /// False until the receiver has a fix.
    pub valid: bool,
    /// Reported geodetic position.
    pub geo_point: GeoPoint,
    /// Reported velocity, m/s.
    pub velocity: [f64; 3],
    /// Horizontal position accuracy estimate, m.
    pub eph: f64,
    /// Vertical position accuracy estimate, m.
    pub epv: f64,
    /// Fix quality: 0 none, 1 dead reckoning, 2 two-dimensional,
    /// 3 three-dimensional, 4 RTK.
    pub fix_type: u32,
    /// Mounting convention, when it differs from the robot's.
    pub axis_convention: Axes,
    /// Mounting frame id, when it differs from the robot's.
    pub coord_frame_id: String,
}

/// Downward optical-flow sensor (measured). Optional; mount it with
/// [`SensorConfig::with_optical_flow_mounted`](crate::SensorConfig::with_optical_flow_mounted).
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct OpticalFlow {
    /// Sensor capture time, seconds since the unix epoch.
    pub timestamp: f64,
    /// False until the device has produced a usable reading.
    pub valid: bool,
    /// Estimated velocity, m/s.
    pub velocity: [f64; 3],
    /// Mounting convention, when it differs from the robot's.
    pub axis_convention: Axes,
    /// Mounting frame id, when it differs from the robot's.
    pub coord_frame_id: String,
}

/// Everything the robot can observe about itself.
///
/// Deliberately no attitude: attitude is never measured, only fused. See
/// [`Estimate`].
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct Sensors {
    /// Three-axis accelerometer.
    pub accelerometer: Accelerometer,
    /// Three-axis rate gyro.
    pub gyroscope: Gyroscope,
    /// Three-axis magnetometer.
    pub magnetometer: Magnetometer,
    /// Barometric pressure sensor.
    pub barometer: Barometer,
    /// Satellite navigation receiver.
    pub gnss: Gnss,
    /// Downward optical flow.
    pub optical_flow: OpticalFlow,
}

impl Sensors {
    fn from_raw(raw: &sys::vrsdk_sensors_t) -> Sensors {
        let a = &raw.accelerometer;
        let g = &raw.gyroscope;
        let m = &raw.magnetometer;
        let b = &raw.barometer;
        let n = &raw.gnss;
        let o = &raw.optical_flow;
        Sensors {
            accelerometer: Accelerometer {
                timestamp: a.timestamp,
                valid: a.valid,
                linear_acceleration: a.linear_acceleration,
                axis_convention: Axes(a.axis_convention),
                coord_frame_id: fixed_str(&a.coord_frame_id),
            },
            gyroscope: Gyroscope {
                timestamp: g.timestamp,
                valid: g.valid,
                angular_velocity: g.angular_velocity,
                axis_convention: Axes(g.axis_convention),
                coord_frame_id: fixed_str(&g.coord_frame_id),
            },
            magnetometer: Magnetometer {
                timestamp: m.timestamp,
                valid: m.valid,
                magnetic_field: m.magnetic_field,
                axis_convention: Axes(m.axis_convention),
                coord_frame_id: fixed_str(&m.coord_frame_id),
            },
            barometer: Barometer {
                timestamp: b.timestamp,
                valid: b.valid,
                pressure: b.pressure,
                altitude: b.altitude,
                qnh: b.qnh,
            },
            gnss: Gnss {
                timestamp: n.timestamp,
                valid: n.valid,
                geo_point: GeoPoint::from_raw(&n.geo_point),
                velocity: n.velocity,
                eph: n.eph,
                epv: n.epv,
                fix_type: n.fix_type,
                axis_convention: Axes(n.axis_convention),
                coord_frame_id: fixed_str(&n.coord_frame_id),
            },
            optical_flow: OpticalFlow {
                timestamp: o.timestamp,
                valid: o.valid,
                velocity: o.velocity,
                axis_convention: Axes(o.axis_convention),
                coord_frame_id: fixed_str(&o.coord_frame_id),
            },
        }
    }
}

/// The world the sensors were noised from (truth).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct Environment {
    /// World-frame gravity, m/s^2.
    pub gravity: [f64; 3],
    /// True static pressure, Pa.
    pub air_pressure: f64,
    /// True air density, kg/m^3.
    pub air_density: f64,
    /// True air temperature, degrees C.
    pub temperature: f64,
    /// The robot's true geodetic position.
    pub geo_point: GeoPoint,
    /// True height above ground, m. A placeholder the simulator publishes as 0;
    /// use the vertical component of `kin.lin_pos` instead.
    pub agl: f64,
}

/// What the robot's own filter believes.
///
/// **Check `valid`.** The simulator runs no filter of its own and leaves this
/// block empty; see [`publish_estimate`](crate::VirtualRobot::publish_estimate)
/// for the channel that carries yours.
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct Estimate {
    /// Filter output time, seconds since the unix epoch.
    pub timestamp: f64,
    /// False until the filter converges.
    pub valid: bool,
    /// Believed kinematics, in the same shape as the truth block.
    pub kin: Kinematics,
    /// The estimator's convention; it may legitimately differ from the robot's.
    pub axis_convention: Axes,
    /// The estimator's frame id.
    pub coord_frame_id: String,
}

/// One complete state sample, copied out of the SDK's latest snapshot by
/// [`VirtualRobot::states`](crate::VirtualRobot::states).
///
/// A plain owned value: keep it, clone it, send it to another thread. See the
/// [module documentation](self) for what the blocks mean.
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct State {
    /// Sim capture time, nanoseconds since the unix epoch. Signed, because the
    /// useful operation is a difference against a frame's `t_ns`.
    pub t_ns: i64,
    /// Seconds since this robot's first state sample: one epoch shared with its
    /// camera frames.
    pub elapsed: f64,
    /// Per-topic sequence number. A gap means a dropped sample.
    pub seq: u64,
    /// Which robot this is.
    pub sys_id: u32,
    /// Who published it (the simulator).
    pub src_id: u32,
    /// The robot's wire name.
    pub name: String,
    /// Schema version stamped by the publisher.
    pub schema_version: u32,
    /// The convention every vector below is expressed in.
    pub axis_convention: Axes,
    /// The frame id every vector below is expressed in. Authoritative when it
    /// and `axis_convention` disagree.
    pub coord_frame_id: String,
    /// Truth: pose, twist, acceleration.
    pub kin: Kinematics,
    /// Truth: the net wrench on the body this step.
    pub wrench: Wrench,
    /// Command in, realised motion out.
    pub actuator: Actuator,
    /// Measured: the robot-observable view.
    pub sensors: Sensors,
    /// Truth: the world the sensors were noised from.
    pub env: Environment,
    /// Believed: what the robot's own filter says.
    pub estimate: Estimate,
}

impl State {
    pub(crate) fn from_raw(raw: &sys::vrsdk_state_t) -> State {
        State {
            t_ns: raw.t_ns,
            elapsed: raw.elapsed,
            seq: raw.seq,
            sys_id: raw.sys_id,
            src_id: raw.src_id,
            name: fixed_str(&raw.name),
            schema_version: raw.schema_version,
            axis_convention: Axes(raw.axis_convention),
            coord_frame_id: fixed_str(&raw.coord_frame_id),
            kin: Kinematics::from_raw(&raw.kin),
            wrench: Wrench {
                force: raw.wrench.force,
                torque: raw.wrench.torque,
            },
            actuator: Actuator::from_raw(&raw.actuator),
            sensors: Sensors::from_raw(&raw.sensors),
            env: Environment {
                gravity: raw.env.gravity,
                air_pressure: raw.env.air_pressure,
                air_density: raw.env.air_density,
                temperature: raw.env.temperature,
                geo_point: GeoPoint::from_raw(&raw.env.geo_point),
                agl: raw.env.agl,
            },
            estimate: Estimate {
                timestamp: raw.estimate.timestamp,
                valid: raw.estimate.valid,
                kin: Kinematics::from_raw(&raw.estimate.kin),
                axis_convention: Axes(raw.estimate.axis_convention),
                coord_frame_id: fixed_str(&raw.estimate.coord_frame_id),
            },
        }
    }
}

/// Subscriber counters for the state stream, from
/// [`VirtualRobot::stats`](crate::VirtualRobot::stats).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub struct StateStats {
    /// Samples that arrived and decoded.
    pub received: u64,
    /// Samples that arrived and did not decode. Counted, never fatal.
    pub decode_errors: u64,
    /// How many times `seq` jumped forward by more than one.
    pub seq_gaps: u64,
    /// Total samples implied missing by those jumps.
    pub missed_samples: u64,
    /// The `seq` of the most recent decoded sample.
    pub last_seq: u64,
}

impl StateStats {
    pub(crate) fn from_raw(raw: &sys::vrsdk_state_stats_t) -> StateStats {
        StateStats {
            received: raw.received,
            decode_errors: raw.decode_errors,
            seq_gaps: raw.seq_gaps,
            missed_samples: raw.missed_samples,
            last_seq: raw.last_seq,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_state_converts_field_for_field() {
        let mut raw = sys::vrsdk_state_t {
            t_ns: 42,
            seq: 7,
            sys_id: 3,
            axis_convention: sys::VRSDK_AXES_FRD,
            ..Default::default()
        };
        ffi::write_fixed_str(&mut raw.name, "multirotor");
        ffi::write_fixed_str(&mut raw.coord_frame_id, "frd");
        raw.kin.lin_pos = [1.0, 2.0, 3.0];
        raw.kin.quat = [0.0, 0.0, 0.0, 1.0];
        raw.actuator.pwm_count = 4;
        raw.actuator.pwm[..4].copy_from_slice(&[1500, 1501, 1502, 1503]);
        raw.actuator.measured_count = 2;
        raw.actuator.measured[..2].copy_from_slice(&[10.0, 20.0]);
        ffi::write_fixed_str(&mut raw.sensors.gyroscope.coord_frame_id, "fru");

        let s = State::from_raw(&raw);
        assert_eq!((s.t_ns, s.seq, s.sys_id), (42, 7, 3));
        assert_eq!(s.name, "multirotor");
        assert_eq!(s.coord_frame_id, "frd");
        assert_eq!(s.axis_convention, Axes::FRD);
        assert_eq!(s.kin.lin_pos, [1.0, 2.0, 3.0]);
        assert_eq!(s.actuator.pwm, vec![1500, 1501, 1502, 1503]);
        assert!(s.actuator.normalized.is_empty());
        assert_eq!(s.actuator.measured, vec![10.0, 20.0]);
        assert!(!s.actuator.truncated);
        assert_eq!(s.sensors.gyroscope.coord_frame_id, "fru");
    }

    #[test]
    fn an_actuator_count_beyond_the_array_is_clamped() {
        let raw = sys::vrsdk_actuator_t {
            pwm_count: 1000,
            truncated: true,
            ..Default::default()
        };
        let a = Actuator::from_raw(&raw);
        assert_eq!(a.pwm.len(), raw.pwm.len());
        assert!(a.truncated);
    }

    #[test]
    fn a_frame_def_round_trips_through_the_c_struct() {
        let def = FrameDef {
            id: "frd".to_string(),
            name: "FRD (aero body)".to_string(),
            axis_convention: Axes::FRD,
            euler_order: EulerOrder::ZYX,
            rows: [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        };
        assert_eq!(FrameDef::from_raw(&def.to_raw()), def);
    }
}
