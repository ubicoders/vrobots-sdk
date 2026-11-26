//! What a robot is: mass, centre of gravity, inertia, and the kind of each
//! actuator entry, read off its `z/properties` topic.
//!
//! These are the values the simulation runs *with*, not per-tick output, so
//! they ride a topic of their own: published when the robot's wire opens, on
//! any parameter change, and as a 1 Hz keepalive.
//! [`properties`](VirtualRobot::properties) takes one sample and keeps the
//! actuator layout on the handle, which the typed actuator readers in
//! [`actuators`](crate::actuators) then check against.
//! [`read_properties`] takes one with no handle at all.

use std::ptr;

use vrobots_sdk_sys as sys;

use crate::error::VrResult;
use crate::ffi::{self, check, fixed_str, unflatten};
use crate::options::ConnectOptions;
use crate::robot::VirtualRobot;
use crate::state::Axes;

/// What one actuator entry is, as `z/properties` publishes it, and so the unit
/// its `measured` value is in.
///
/// Index-parallel to [`Actuator`](crate::Actuator): the kind at index `i`
/// names the device whose command and readings sit at index `i` of the
/// state's actuator arrays. The discriminants are the wire values (the C API's
/// `VRSDK_ACTUATOR_KIND_*`), so [`code`](Self::code) is a plain cast.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
pub enum ActuatorKind {
    /// No kind fits, such as the force on a mass-spring-damper or a cart-pole.
    #[default]
    Unspecified = sys::VRSDK_ACTUATOR_KIND_UNSPECIFIED,
    /// A propeller: `measured` in rad/s.
    Rotor = sys::VRSDK_ACTUATOR_KIND_ROTOR,
    /// A revolute joint or a steering servo: `measured` in rad.
    Servo = sys::VRSDK_ACTUATOR_KIND_SERVO,
    /// A driven wheel: `measured` in rad/s.
    Wheel = sys::VRSDK_ACTUATOR_KIND_WHEEL,
    /// An aerodynamic control surface.
    Surface = sys::VRSDK_ACTUATOR_KIND_SURFACE,
    /// A prismatic joint (a lift, a fork width, a linear rail): `measured` in
    /// m.
    Linear = sys::VRSDK_ACTUATOR_KIND_LINEAR,
    /// A suction tool: `normalized` the commanded on/off, `measured` the held
    /// mass in kg.
    Vacuum = sys::VRSDK_ACTUATOR_KIND_VACUUM,
    /// A laser: `normalized` the commanded on/off, `measured` the range to the
    /// hit in m, 0 for none.
    Laser = sys::VRSDK_ACTUATOR_KIND_LASER,
    /// A two-finger gripper: `normalized` the commanded opening over the
    /// stroke, `measured` the gap between the pads in m.
    Gripper = sys::VRSDK_ACTUATOR_KIND_GRIPPER,
    /// An arm's stop report, not a device: `measured` is an
    /// [`ArmContact`](crate::ArmContact) code.
    Contact = sys::VRSDK_ACTUATOR_KIND_CONTACT,
    /// A tool's load: `normalized` 1 while holding, `measured` the held mass in
    /// kg.
    Hold = sys::VRSDK_ACTUATOR_KIND_HOLD,
    /// A kind this SDK does not know, from a newer simulator.
    Unknown = sys::VRSDK_ACTUATOR_KIND_UNKNOWN,
}

impl ActuatorKind {
    /// The kind for a wire value; one this SDK does not know is
    /// [`Unknown`](Self::Unknown).
    #[must_use]
    pub fn from_code(code: i32) -> ActuatorKind {
        match code {
            sys::VRSDK_ACTUATOR_KIND_UNSPECIFIED => ActuatorKind::Unspecified,
            sys::VRSDK_ACTUATOR_KIND_ROTOR => ActuatorKind::Rotor,
            sys::VRSDK_ACTUATOR_KIND_SERVO => ActuatorKind::Servo,
            sys::VRSDK_ACTUATOR_KIND_WHEEL => ActuatorKind::Wheel,
            sys::VRSDK_ACTUATOR_KIND_SURFACE => ActuatorKind::Surface,
            sys::VRSDK_ACTUATOR_KIND_LINEAR => ActuatorKind::Linear,
            sys::VRSDK_ACTUATOR_KIND_VACUUM => ActuatorKind::Vacuum,
            sys::VRSDK_ACTUATOR_KIND_LASER => ActuatorKind::Laser,
            sys::VRSDK_ACTUATOR_KIND_GRIPPER => ActuatorKind::Gripper,
            sys::VRSDK_ACTUATOR_KIND_CONTACT => ActuatorKind::Contact,
            sys::VRSDK_ACTUATOR_KIND_HOLD => ActuatorKind::Hold,
            _ => ActuatorKind::Unknown,
        }
    }

    /// The wire value, 0 to 10, and -1 for [`Unknown`](Self::Unknown).
    #[must_use]
    pub const fn code(self) -> i32 {
        self as i32
    }

    /// A lowercase name for logs and displays: `"unspecified"`, `"rotor"`,
    /// `"servo"`, `"wheel"`, `"surface"`, `"linear"`, `"vacuum"`, `"laser"`,
    /// `"gripper"`, `"contact"`, `"hold"` or `"unknown"`. The library's own
    /// `vrsdk_actuator_kind_name`.
    #[must_use]
    pub fn name(self) -> &'static str {
        // SAFETY: `vrsdk_actuator_kind_name` accepts any value and returns a
        // static, NUL-terminated literal, never NULL.
        unsafe { ffi::static_str(sys::vrsdk_actuator_kind_name(self.code())) }
    }
}

/// A robot's mass properties and actuator layout, as
/// [`properties`](VirtualRobot::properties) reads them.
///
/// `cg` and `inertia` are on body axes in the frame `coord_frame_id` names:
/// the robot's own frame, the one its state reports in.
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct RobotProperties {
    /// Mass, kg. The one frame-invariant field.
    pub mass: f64,
    /// Centre of gravity from the robot origin, m.
    pub cg: [f64; 3],
    /// Inertia tensor about the centre of gravity, kg.m^2, row-major. A tensor
    /// converts by conjugation (`R * I * R^T`), never with the vector rule.
    pub inertia: [[f64; 3]; 3],
    /// Each actuator's moment arm from the centre of gravity, m,
    /// index-parallel to the state's actuator arrays. Empty for a robot type
    /// that publishes none.
    pub moment_arms: Vec<f64>,
    /// What each actuator entry is, index-parallel to the state's actuator
    /// arrays. Empty for a robot type that publishes no layout.
    pub actuator_types: Vec<ActuatorKind>,
    /// True when the robot published more entries than the C API carries (16)
    /// and the two lists above were cut short.
    pub truncated: bool,
    /// The convention `cg` and `inertia` are expressed in.
    pub axis_convention: Axes,
    /// The frame id `cg` and `inertia` are expressed in. Authoritative when it
    /// and `axis_convention` disagree.
    pub coord_frame_id: String,
}

impl RobotProperties {
    pub(crate) fn from_raw(raw: &sys::vrsdk_robot_properties_t) -> RobotProperties {
        fn prefix<T: Copy>(values: &[T], count: u32) -> &[T] {
            let count = usize::try_from(count).unwrap_or(usize::MAX);
            &values[..count.min(values.len())]
        }
        RobotProperties {
            mass: raw.mass,
            cg: raw.cg,
            inertia: unflatten(raw.inertia),
            moment_arms: prefix(&raw.moment_arms, raw.moment_arm_count).to_vec(),
            actuator_types: prefix(&raw.actuator_types, raw.actuator_type_count)
                .iter()
                .map(|&code| ActuatorKind::from_code(code))
                .collect(),
            truncated: raw.truncated,
            axis_convention: Axes(raw.axis_convention),
            coord_frame_id: fixed_str(&raw.coord_frame_id),
        }
    }
}

/// What the robot is. See the [module documentation](crate::properties).
impl VirtualRobot {
    /// Read what the robot is off `z/properties`: mass, centre of gravity,
    /// inertia, and the kind of each actuator entry.
    ///
    /// ```no_run
    /// # use vrobots_sdk::{RobotType, VirtualRobot};
    /// let robot = VirtualRobot::connect(RobotType::Forklift, Some(0))?;
    /// let p = robot.properties()?;
    /// let kinds: Vec<&str> = p.actuator_types.iter().map(|k| k.name()).collect();
    /// println!("{} kg, cg {:?} in {}, actuators {kinds:?}", p.mass, p.cg, p.coord_frame_id);
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    ///
    /// A one-shot read, as [`frame_def`](Self::frame_def) is: subscribe, take
    /// the first sample, undeclare. The mass properties come off the rigid
    /// body, so a skin swap or a moving mast shows on the next keepalive.
    ///
    /// **It also arms a check.** The handle keeps the layout read here, and
    /// from then on [`forklift_actuators`](Self::forklift_actuators) and its
    /// siblings refuse a report whose published layout contradicts their own:
    /// the one way to tell a rail arm from a table arm, whose reports are both
    /// nine entries long. Without a read they decode as before.
    ///
    /// # Errors
    ///
    /// [`VrError::Timeout`](crate::VrError::Timeout) if no sample arrives
    /// within [`ConnectOptions::probe_timeout`](crate::ConnectOptions::probe_timeout),
    /// which is also what a simulator from before this topic existed gives;
    /// [`VrError::Decode`](crate::VrError::Decode) if the payload is malformed;
    /// [`VrError::Session`](crate::VrError::Session) if the subscriber cannot
    /// be declared.
    pub fn properties(&self) -> VrResult<RobotProperties> {
        let mut raw = sys::vrsdk_robot_properties_t::default();
        // SAFETY: the robot handle is live and `raw` is writable storage for
        // one `vrsdk_robot_properties_t`.
        check(unsafe { sys::vrsdk_robot_properties(self.raw(), &mut raw) })?;
        Ok(RobotProperties::from_raw(&raw))
    }
}

/// Read what robot `sys_id` is off its `z/properties`, with no handle to it and
/// default connection options.
///
/// The cheap way to find a robot by type: [`list_topics`](crate::list_topics)
/// says which ids are on the wire, and this reads each one's actuator layout
/// without connecting to it:
///
/// ```no_run
/// use vrobots_sdk::{read_properties, turret_actuator};
///
/// if read_properties(3)?.actuator_types == turret_actuator::KINDS {
///     println!("sys_id 3 is a turret");
/// }
/// # Ok::<(), vrobots_sdk::VrError>(())
/// ```
///
/// A one-shot read like [`VirtualRobot::properties`], on a zenoh session of
/// its own that is closed before this returns. A fresh session needs a moment
/// of zenoh discovery and the topic is a 1 Hz keepalive, so a read takes about
/// a second. Nothing is kept: a handle connected afterwards arms its layout
/// check with its own [`properties`](VirtualRobot::properties) read.
///
/// # Errors
///
/// [`VrError::Timeout`](crate::VrError::Timeout) if no sample arrives within
/// [`ConnectOptions::probe_timeout`] (no robot at that id, or a simulator from
/// before this topic existed); [`VrError::Session`](crate::VrError::Session)
/// if zenoh will not open; [`VrError::Decode`](crate::VrError::Decode) if the
/// payload is malformed.
pub fn read_properties(sys_id: u32) -> VrResult<RobotProperties> {
    read(sys_id, None)
}

/// [`read_properties`] with explicit options, e.g. a router endpoint or a
/// shorter [`probe_timeout`](ConnectOptions::probe_timeout). Only the
/// connection settings and the probe timeout are used.
///
/// # Errors
///
/// As [`read_properties`], plus [`VrError::Config`](crate::VrError::Config)
/// for an unusable router endpoint.
pub fn read_properties_with(sys_id: u32, options: &ConnectOptions) -> VrResult<RobotProperties> {
    read(sys_id, Some(options))
}

fn read(sys_id: u32, options: Option<&ConnectOptions>) -> VrResult<RobotProperties> {
    let options = options.map(ConnectOptions::to_c).transpose()?;
    let options_ptr = options.as_ref().map_or(ptr::null(), |o| o.as_ptr());
    let mut raw = sys::vrsdk_robot_properties_t::default();
    // SAFETY: `options_ptr` is NULL or an initialised struct alive for the
    // call, and `raw` is writable storage for one `vrsdk_robot_properties_t`.
    check(unsafe { sys::vrsdk_read_properties(sys_id, options_ptr, &mut raw) })?;
    Ok(RobotProperties::from_raw(&raw))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kinds_round_trip_their_wire_codes() {
        for code in -1..=10 {
            assert_eq!(ActuatorKind::from_code(code).code(), code);
        }
        assert_eq!(ActuatorKind::from_code(11), ActuatorKind::Unknown);
        assert_eq!(ActuatorKind::default(), ActuatorKind::Unspecified);
    }

    #[test]
    fn properties_copy_only_the_used_entries() {
        let mut raw = sys::vrsdk_robot_properties_t {
            mass: 2500.0,
            cg: [0.0, 0.5, 0.2],
            moment_arm_count: 2,
            actuator_type_count: 3,
            axis_convention: sys::VRSDK_AXES_UNSPECIFIED,
            ..Default::default()
        };
        raw.inertia[0] = 1.0;
        raw.inertia[4] = 2.0;
        raw.inertia[8] = 3.0;
        raw.moment_arms[..3].copy_from_slice(&[0.6, 0.7, 99.0]);
        raw.actuator_types[..4].copy_from_slice(&[3, 2, 5, 99]);
        ffi::write_fixed_str(&mut raw.coord_frame_id, "fru");

        let p = RobotProperties::from_raw(&raw);
        assert_eq!(p.mass, 2500.0);
        assert_eq!(
            p.inertia,
            [[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]]
        );
        assert_eq!(p.moment_arms, vec![0.6, 0.7]);
        assert_eq!(
            p.actuator_types,
            vec![
                ActuatorKind::Wheel,
                ActuatorKind::Servo,
                ActuatorKind::Linear
            ]
        );
        assert_eq!(p.coord_frame_id, "fru");
        assert!(!p.truncated);

        // A count past the array is clamped, never read past.
        raw.actuator_type_count = 1000;
        assert_eq!(RobotProperties::from_raw(&raw).actuator_types.len(), 16);
    }
}
