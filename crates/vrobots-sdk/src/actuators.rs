//! Actuator reports decoded per robot type: the forklift, the pallet robot,
//! the rail arm, the table arm and the turret.
//!
//! [`State::actuator`](crate::State::actuator) is three index-parallel arrays
//! whose meaning depends on the robot type. The layout modules here
//! ([`forklift_actuator`], [`palletrobot_channel`], [`rail_arm`],
//! [`table_arm`], [`turret_actuator`]) name the indices, and the decoded
//! structs read one robot type's report through the C library's decoders.
//! These reports are the proof of every command those robots take, since no
//! command has a reply.
//!
//! **Decode by robot type, never by count.** A decoder reads by index alone
//! and cannot tell robot types apart: both arms report nine entries, and a
//! pallet robot's six decode as a forklift's. So
//! [`forklift_actuators`](VirtualRobot::forklift_actuators) and its siblings
//! check the type the handle was connected as, and, once
//! [`properties`](VirtualRobot::properties) has been read, the layout the
//! robot published; `from_actuator` on a snapshot already in hand trusts the
//! caller.

use vrobots_sdk_sys as sys;

use crate::error::VrResult;
use crate::ffi::{self, check, expect_ok};
use crate::robot::VirtualRobot;
use crate::state::Actuator;

/// Positions in a `SET_FORKLIFT` frame's `int_arr`: the order
/// [`set_forklift`](crate::VirtualRobot::set_forklift) sends its arguments in.
///
/// **Not** the actuator report's order, which leads with the drive: see
/// [`forklift_actuator`]. Only a frame built by hand for
/// [`send_cmd`](crate::VirtualRobot::send_cmd) needs these.
pub mod forklift_channel {
    use vrobots_sdk_sys as sys;

    /// `steer_us`: 1100 full left, 1500 straight, 1900 full right.
    pub const STEER: usize = sys::VRSDK_FORKLIFT_CH_STEER as usize;
    /// `throttle_us`: 1100 full reverse, 1500 stop, 1900 full forward.
    pub const THROTTLE: usize = sys::VRSDK_FORKLIFT_CH_THROTTLE as usize;
    /// `lift_us`: the lift position servo, 1100 lowest, 1900 highest.
    pub const LIFT: usize = sys::VRSDK_FORKLIFT_CH_LIFT as usize;
    /// `fork_width_us`: the fork-width position servo, 1100 narrowest, 1900
    /// widest.
    pub const FORK_WIDTH: usize = sys::VRSDK_FORKLIFT_CH_FORK_WIDTH as usize;
    /// `brake_us`, only in a five-entry frame: 1100 released, 1900 full.
    pub const BRAKE: usize = sys::VRSDK_FORKLIFT_CH_BRAKE as usize;
}

/// Indices into a forklift's actuator report.
///
/// **Not** the command's order: the report leads with the drive, so
/// `pwm[DRIVE]` echoes the throttle and `pwm[STEERING]` the steer, and there is
/// no brake entry. [`ForkliftActuators`] decodes
/// `measured` by these.
pub mod forklift_actuator {
    use vrobots_sdk_sys as sys;

    use crate::ActuatorKind;

    /// Entries in the report.
    pub const COUNT: usize = sys::VRSDK_FORKLIFT_ACT_COUNT as usize;
    /// What each entry is, as `z/properties` publishes it: the drive wheels,
    /// the steering servo, then the two linear axes of the mast.
    pub const KINDS: [ActuatorKind; COUNT] = [
        ActuatorKind::Wheel,
        ActuatorKind::Servo,
        ActuatorKind::Linear,
        ActuatorKind::Linear,
    ];
    /// Front-axle drive: `pwm` the throttle, `normalized` the demand in
    /// [-1, 1], `measured` the axle speed in rad/s.
    pub const DRIVE: usize = sys::VRSDK_FORKLIFT_ACT_DRIVE as usize;
    /// Rear-axle steering: `pwm` the steer, `normalized` in [-1, 1],
    /// `measured` the angle in rad, positive turning right.
    pub const STEERING: usize = sys::VRSDK_FORKLIFT_ACT_STEERING as usize;
    /// Lift: `normalized` the commanded fraction of the range in [0, 1],
    /// `measured` the carriage height in m.
    pub const LIFT: usize = sys::VRSDK_FORKLIFT_ACT_LIFT as usize;
    /// Fork width: `normalized` in [0, 1] of the range, `measured` centre to
    /// centre in m.
    pub const FORK_WIDTH: usize = sys::VRSDK_FORKLIFT_ACT_FORK_WIDTH as usize;
}

/// Pallet-robot channel indices: one order for the `SET_PALLETROBOT` frame and
/// the actuator report alike.
///
/// In the report, `pwm` echoes the clamped command at every index,
/// `normalized` is the demand in [-1, 1] for the wheels and the pad turn and
/// the commanded fraction of travel in [0, 1] for the pad lift, and `measured`
/// is wheel rad/s, pad height in m and pad angle in rad, which
/// [`PalletRobotActuators`] decodes.
pub mod palletrobot_channel {
    use vrobots_sdk_sys as sys;

    use crate::ActuatorKind;

    /// Front-left wheel ESC.
    pub const FL: usize = sys::VRSDK_PALLETROBOT_CH_FL as usize;
    /// Front-right wheel ESC.
    pub const FR: usize = sys::VRSDK_PALLETROBOT_CH_FR as usize;
    /// Rear-left wheel ESC.
    pub const RL: usize = sys::VRSDK_PALLETROBOT_CH_RL as usize;
    /// Rear-right wheel ESC.
    pub const RR: usize = sys::VRSDK_PALLETROBOT_CH_RR as usize;
    /// Pad lift, a position servo: 1100 down, 1900 top.
    pub const PAD_LIFT: usize = sys::VRSDK_PALLETROBOT_CH_PAD_LIFT as usize;
    /// Pad turn, a continuous-rotation servo: 1500 holds, the offset sets the
    /// rate.
    pub const PAD_TURN: usize = sys::VRSDK_PALLETROBOT_CH_PAD_TURN as usize;
    /// Channels in a frame. Exactly this many, or the robot ignores the frame.
    pub const COUNT: usize = sys::VRSDK_PALLETROBOT_CH_COUNT as usize;
    /// What each report entry is, as `z/properties` publishes it: four wheels,
    /// the pad lift, the pad turn.
    pub const KINDS: [ActuatorKind; COUNT] = [
        ActuatorKind::Wheel,
        ActuatorKind::Wheel,
        ActuatorKind::Wheel,
        ActuatorKind::Wheel,
        ActuatorKind::Linear,
        ActuatorKind::Servo,
    ];
}

/// The rail arm's numbers: joint count and actuator-report indices.
///
/// A KUKA KR210 on an 8 m linear rail with a vacuum pad on its flange,
/// [`RobotType::RailArm`](crate::RobotType::RailArm). The indices hold in
/// `normalized` and `measured` alike;
/// [`RailArmActuators`] decodes the whole report.
///
/// | index | entry | `normalized` | `measured` |
/// |---|---|---|---|
/// | 0 | [`RAIL`](crate::rail_arm::RAIL) | position across its range, [-1, 1] | m |
/// | 1 to 6 | [`A1`](crate::rail_arm::A1) to [`A6`](crate::rail_arm::A6) | position across its range, [-1, 1] | rad |
/// | 7 | [`VACUUM`](crate::rail_arm::VACUUM) | the latch: 0 off, 1 on | held mass, kg |
/// | 8 | [`CONTACT`](crate::rail_arm::CONTACT) | 0 | an [`ArmContact`] code |
///
/// `pwm` is 0 throughout. The joint angles are the KUKA controller's axis
/// values; there is no tool pose on the wire, so forward kinematics is yours.
pub mod rail_arm {
    use vrobots_sdk_sys as sys;

    use crate::ActuatorKind;

    /// Joints, in actuator order: the rail, then A1 to A6.
    pub const JOINT_COUNT: usize = sys::VRSDK_RAIL_ARM_JOINT_COUNT as usize;
    /// Entries in the actuator report: the joints, [`VACUUM`], [`CONTACT`].
    pub const ACTUATOR_COUNT: usize = sys::VRSDK_RAIL_ARM_ACTUATOR_COUNT as usize;
    /// What each report entry is, as `z/properties` publishes it.
    pub const ACTUATOR_KINDS: [ActuatorKind; ACTUATOR_COUNT] = [
        ActuatorKind::Linear,
        ActuatorKind::Servo,
        ActuatorKind::Servo,
        ActuatorKind::Servo,
        ActuatorKind::Servo,
        ActuatorKind::Servo,
        ActuatorKind::Servo,
        ActuatorKind::Vacuum,
        ActuatorKind::Contact,
    ];
    /// The rail: the carriage's position along the track, m.
    pub const RAIL: usize = sys::VRSDK_RAIL_ARM_RAIL as usize;
    /// A1, the swivel about the vertical axis, rad.
    pub const A1: usize = sys::VRSDK_RAIL_ARM_A1 as usize;
    /// A2, the shoulder, rad.
    pub const A2: usize = sys::VRSDK_RAIL_ARM_A2 as usize;
    /// A3, the elbow, rad.
    pub const A3: usize = sys::VRSDK_RAIL_ARM_A3 as usize;
    /// A4, the forearm roll, rad.
    pub const A4: usize = sys::VRSDK_RAIL_ARM_A4 as usize;
    /// A5, the wrist bend, rad.
    pub const A5: usize = sys::VRSDK_RAIL_ARM_A5 as usize;
    /// A6, the flange turn, rad.
    pub const A6: usize = sys::VRSDK_RAIL_ARM_A6 as usize;
    /// The vacuum: `normalized` the `SET_ARM_VACUUM` latch, `measured` the held
    /// mass in kg (0 when nothing is held).
    pub const VACUUM: usize = sys::VRSDK_RAIL_ARM_VACUUM as usize;
    /// What cut the last joint step short: `measured` is an
    /// [`ArmContact`](crate::ArmContact) code.
    pub const CONTACT: usize = sys::VRSDK_RAIL_ARM_CONTACT as usize;
}

/// The table arm's numbers: joint count, actuator-report indices, gripper
/// stroke.
///
/// A UR10 with a two-finger gripper, [`RobotType::TableArm`](crate::RobotType::TableArm).
/// [`TableArmActuators`] decodes the whole report.
///
/// | index | entry | `normalized` | `measured` |
/// |---|---|---|---|
/// | 0 to 5 | [`A1`](crate::table_arm::A1) to [`A6`](crate::table_arm::A6) | position across its range, [-1, 1] | rad |
/// | 6 | [`GRIPPER`](crate::table_arm::GRIPPER) | the latch over the stroke | the gap between the pads, m |
/// | 7 | [`CONTACT`](crate::table_arm::CONTACT) | 0 | an [`ArmContact`] code |
/// | 8 | [`HOLD`](crate::table_arm::HOLD) | 1 holding, 0 not | held mass, kg |
///
/// `pwm` is 0 throughout. Home is the pose the arm loads in: read
/// [`TableArmActuators::joints`](crate::TableArmActuators::joints) once after
/// connecting to learn it.
pub mod table_arm {
    use vrobots_sdk_sys as sys;

    use crate::ActuatorKind;

    /// Joints, in actuator order: A1 to A6.
    pub const JOINT_COUNT: usize = sys::VRSDK_TABLE_ARM_JOINT_COUNT as usize;
    /// Entries in the actuator report: the joints, [`GRIPPER`], [`CONTACT`],
    /// [`HOLD`].
    pub const ACTUATOR_COUNT: usize = sys::VRSDK_TABLE_ARM_ACTUATOR_COUNT as usize;
    /// What each report entry is, as `z/properties` publishes it.
    pub const ACTUATOR_KINDS: [ActuatorKind; ACTUATOR_COUNT] = [
        ActuatorKind::Servo,
        ActuatorKind::Servo,
        ActuatorKind::Servo,
        ActuatorKind::Servo,
        ActuatorKind::Servo,
        ActuatorKind::Servo,
        ActuatorKind::Gripper,
        ActuatorKind::Contact,
        ActuatorKind::Hold,
    ];
    /// A1, the base, rad.
    pub const A1: usize = sys::VRSDK_TABLE_ARM_A1 as usize;
    /// A2, the shoulder, rad.
    pub const A2: usize = sys::VRSDK_TABLE_ARM_A2 as usize;
    /// A3, the elbow, rad.
    pub const A3: usize = sys::VRSDK_TABLE_ARM_A3 as usize;
    /// A4, wrist 1, rad.
    pub const A4: usize = sys::VRSDK_TABLE_ARM_A4 as usize;
    /// A5, wrist 2, rad.
    pub const A5: usize = sys::VRSDK_TABLE_ARM_A5 as usize;
    /// A6, wrist 3, rad.
    pub const A6: usize = sys::VRSDK_TABLE_ARM_A6 as usize;
    /// The gripper: `normalized` the `SET_ARM_GRIPPER` latch over
    /// [`GRIPPER_STROKE_M`], `measured` the gap between the pads now, m.
    pub const GRIPPER: usize = sys::VRSDK_TABLE_ARM_GRIPPER as usize;
    /// What cut the last joint step short: `measured` is an
    /// [`ArmContact`](crate::ArmContact) code.
    pub const CONTACT: usize = sys::VRSDK_TABLE_ARM_CONTACT as usize;
    /// Whether a body is held: `normalized` 1 or 0, `measured` its mass in kg.
    pub const HOLD: usize = sys::VRSDK_TABLE_ARM_HOLD as usize;
    /// The gripper's stroke, m: the widest gap between its pads, and the top of
    /// the range `SET_ARM_GRIPPER` is clamped to.
    pub const GRIPPER_STROKE_M: f64 = sys::VRSDK_TABLE_ARM_GRIPPER_STROKE_M;
}

/// Where each turret entry sits in the index-parallel arrays of
/// [`Actuator`].
///
/// | index | entry | `normalized` | `measured` |
/// |---|---|---|---|
/// | 0 | [`HEADING`](crate::turret_actuator::HEADING) | rate over max rate, [-1, 1] | angle, rad |
/// | 1 | [`PITCH`](crate::turret_actuator::PITCH) | rate over max rate, [-1, 1] | angle, rad |
/// | 2 | [`LASER`](crate::turret_actuator::LASER) | 1 on, 0 off | range, m; 0 is off or no hit |
pub mod turret_actuator {
    use vrobots_sdk_sys as sys;

    use crate::ActuatorKind;

    /// The pylon, the heading joint.
    pub const HEADING: usize = sys::VRSDK_TURRET_ACT_HEADING as usize;
    /// The head, the pitch joint.
    pub const PITCH: usize = sys::VRSDK_TURRET_ACT_PITCH as usize;
    /// The laser.
    pub const LASER: usize = sys::VRSDK_TURRET_ACT_LASER as usize;
    /// Entries in the report.
    pub const COUNT: usize = sys::VRSDK_TURRET_ACT_COUNT as usize;
    /// What each entry is, as `z/properties` publishes it: two servos, then
    /// the laser.
    pub const KINDS: [ActuatorKind; COUNT] = [
        ActuatorKind::Servo,
        ActuatorKind::Servo,
        ActuatorKind::Laser,
    ];
}

/// What cut an arm's last joint step short: the contact entry of its actuator
/// report.
///
/// Every joint step is tested against the scene before it is taken, and a step
/// that would drive the arm into something is **refused**: the joints stay and
/// the setpoint stays latched. This names the **last** step only, so it reads
/// [`NoContact`](Self::NoContact) again after the first step that goes
/// through. Moving away is never refused, so a setpoint that backs off always
/// moves. The discriminants are the wire codes (the C API's
/// `VRSDK_ARM_CONTACT_*`).
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
pub enum ArmContact {
    /// The step went through, or nothing moved.
    #[default]
    NoContact = sys::VRSDK_ARM_CONTACT_NONE,
    /// The tool or its load met something: a guarded stop, and the normal end
    /// of "descend until contact"; no safety event.
    ToolContact = sys::VRSDK_ARM_CONTACT_TOOL,
    /// Another part of the arm met something: a protective stop. Meeting an
    /// object or another robot also publishes a safety event.
    CollisionStop = sys::VRSDK_ARM_CONTACT_COLLISION_STOP,
    /// A code this SDK does not know, from a newer simulator.
    Unknown = sys::VRSDK_ARM_CONTACT_UNKNOWN,
}

impl ArmContact {
    /// The variant for a wire code; one this SDK does not know is
    /// [`Unknown`](Self::Unknown).
    #[must_use]
    pub fn from_code(code: i32) -> ArmContact {
        match code {
            sys::VRSDK_ARM_CONTACT_NONE => ArmContact::NoContact,
            sys::VRSDK_ARM_CONTACT_TOOL => ArmContact::ToolContact,
            sys::VRSDK_ARM_CONTACT_COLLISION_STOP => ArmContact::CollisionStop,
            _ => ArmContact::Unknown,
        }
    }

    /// The wire code: 0, 1 or 2, and -1 for [`Unknown`](Self::Unknown).
    #[must_use]
    pub const fn code(self) -> i32 {
        self as i32
    }

    /// A short, stable name: `"no_contact"`, `"tool_contact"`,
    /// `"collision_stop"` or `"unknown"`. The library's own
    /// `vrsdk_arm_contact_name`.
    #[must_use]
    pub fn name(self) -> &'static str {
        // SAFETY: `vrsdk_arm_contact_name` accepts any value and returns a
        // static, NUL-terminated literal, never NULL.
        unsafe { ffi::static_str(sys::vrsdk_arm_contact_name(self.code())) }
    }
}

/// Run one of the library's `*_from_actuator` decoders on a report.
///
/// `None` is the library's `VRSDK_ERR_DECODE`: the report is shorter than the
/// layout. Any other failure is impossible with the arguments passed here.
fn decode<C: Default>(
    actuator: &Actuator,
    decoder: unsafe extern "C" fn(*const sys::vrsdk_actuator_t, *mut C) -> sys::vrsdk_err_t,
    what: &str,
) -> Option<C> {
    let report = actuator.to_raw();
    let mut out = C::default();
    // SAFETY: every decoder passed here takes one initialised
    // `vrsdk_actuator_t` and writable storage for its own output struct.
    let code = unsafe { decoder(&report, &mut out) };
    match code {
        sys::VRSDK_OK => Some(out),
        sys::VRSDK_ERR_DECODE => None,
        other => {
            expect_ok(other, what);
            None
        }
    }
}

/// What a forklift's drive, steering and mast are doing: its actuator report's
/// `measured` values, decoded by [`forklift_actuator`].
///
/// The confirmation channel for
/// [`set_forklift`](crate::VirtualRobot::set_forklift). The lift and the fork
/// width slew, so a move is done when these arrive, not when `actuator.pwm`
/// echoes the command.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct ForkliftActuators {
    /// Front-axle speed, rad/s, positive rolling forward.
    pub drive_speed: f64,
    /// Steering angle, rad, positive turning right. The rear wheels steer the
    /// tail, so they sit at minus this.
    pub steering_angle: f64,
    /// Carriage height on the mast, m. Negative near the bottom of the band by
    /// design: -0.215 m is the forks at a floor pallet's pocket height.
    pub lift_height: f64,
    /// Distance between the forks, centre to centre, m.
    pub fork_width: f64,
}

impl ForkliftActuators {
    /// Decode a report already in hand, `State::actuator` say, as a
    /// forklift's.
    ///
    /// `None` when `measured` holds fewer than [`forklift_actuator::COUNT`]
    /// entries. **The robot type is not checked**: another robot's report
    /// decodes too, into wrong numbers, which is why
    /// [`VirtualRobot::forklift_actuators`] checks the handle first.
    ///
    /// ```
    /// use vrobots_sdk::{forklift_actuator, Actuator, ForkliftActuators};
    ///
    /// let mut report = Actuator::default();
    /// assert!(ForkliftActuators::from_actuator(&report).is_none());
    /// report.measured = vec![2.5, -0.1, 0.448, 0.454];
    /// let a = ForkliftActuators::from_actuator(&report).expect("four entries");
    /// assert_eq!(a.lift_height, report.measured[forklift_actuator::LIFT]);
    /// ```
    #[must_use]
    pub fn from_actuator(actuator: &Actuator) -> Option<ForkliftActuators> {
        decode(
            actuator,
            sys::vrsdk_forklift_actuators_from_actuator,
            "vrsdk_forklift_actuators_from_actuator",
        )
        .map(|raw| ForkliftActuators::from_raw(&raw))
    }

    fn from_raw(raw: &sys::vrsdk_forklift_actuators_t) -> ForkliftActuators {
        ForkliftActuators {
            drive_speed: raw.drive_speed,
            steering_angle: raw.steering_angle,
            lift_height: raw.lift_height,
            fork_width: raw.fork_width,
        }
    }
}

/// What a pallet robot's wheels and pad are doing: its actuator report's
/// `measured` values, decoded by [`palletrobot_channel`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct PalletRobotActuators {
    /// Wheel motor speeds, rad/s, positive rolling forward, ordered FL, FR,
    /// RL, RR, so [`palletrobot_channel`] indexes them. The motors' own
    /// encoders: a blocked robot still reports them, and the body's real
    /// motion is in `kin`.
    pub wheel_speeds: [f64; 4],
    /// Pad height above its rest, m: 0 to 0.06.
    pub pad_height: f64,
    /// Pad angle relative to the body, rad, in [-pi, pi), clockwise seen from
    /// above. The pad's own angle: a load on it may not have followed.
    pub pad_angle: f64,
}

impl PalletRobotActuators {
    /// Decode a report already in hand as a pallet robot's.
    ///
    /// `None` when `measured` holds fewer than [`palletrobot_channel::COUNT`]
    /// entries. **The robot type is not checked**, so
    /// [`VirtualRobot::palletrobot_actuators`] checks the handle first.
    #[must_use]
    pub fn from_actuator(actuator: &Actuator) -> Option<PalletRobotActuators> {
        decode(
            actuator,
            sys::vrsdk_palletrobot_actuators_from_actuator,
            "vrsdk_palletrobot_actuators_from_actuator",
        )
        .map(|raw| PalletRobotActuators::from_raw(&raw))
    }

    fn from_raw(raw: &sys::vrsdk_palletrobot_actuators_t) -> PalletRobotActuators {
        PalletRobotActuators {
            wheel_speeds: raw.wheel_speeds,
            pad_height: raw.pad_height,
            pad_angle: raw.pad_angle,
        }
    }
}

/// The rail arm's actuator report, decoded. [`rail_arm`] has the layout.
///
/// **Decode by robot type, never by count**: the table arm reports nine
/// entries too, and read as this one they decode without complaint into wrong
/// numbers.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct RailArmActuators {
    /// Joint positions in actuator order: the rail in m, then A1 to A6 in rad.
    /// Index with the [`rail_arm`] joint constants.
    pub joints: [f64; rail_arm::JOINT_COUNT],
    /// Each joint's position across its range, [-1, 1]. -1 or 1 is a joint at
    /// a limit, where a setpoint past its range ends up.
    pub joints_normalized: [f64; rail_arm::JOINT_COUNT],
    /// The `SET_ARM_VACUUM` latch: your last command, echoed back.
    pub vacuum_on: bool,
    /// Whether a load is on the pad: [`held_mass_kg`](Self::held_mass_kg)
    /// above zero.
    pub holding: bool,
    /// The held load's mass, kg; 0 when nothing is held.
    pub held_mass_kg: f64,
    /// What cut the last joint step short.
    pub contact: ArmContact,
}

impl RailArmActuators {
    /// Decode a report already in hand as the rail arm's, which is also the way
    /// to read a rail arm through a handle connected as another type.
    ///
    /// `None` when `normalized` or `measured` holds fewer than
    /// [`rail_arm::ACTUATOR_COUNT`] entries. **The robot type is not checked.**
    #[must_use]
    pub fn from_actuator(actuator: &Actuator) -> Option<RailArmActuators> {
        decode(
            actuator,
            sys::vrsdk_rail_arm_actuators_from_actuator,
            "vrsdk_rail_arm_actuators_from_actuator",
        )
        .map(|raw| RailArmActuators::from_raw(&raw))
    }

    fn from_raw(raw: &sys::vrsdk_rail_arm_actuators_t) -> RailArmActuators {
        RailArmActuators {
            joints: raw.joints,
            joints_normalized: raw.joints_normalized,
            vacuum_on: raw.vacuum_on,
            holding: raw.holding,
            held_mass_kg: raw.held_mass_kg,
            contact: ArmContact::from_code(raw.contact),
        }
    }
}

/// The table arm's actuator report, decoded. [`table_arm`] has the layout.
///
/// **Decode by robot type, never by count**: the rail arm reports nine entries
/// too.
///
/// ```
/// use vrobots_sdk::{Actuator, ArmContact, RailArmActuators, TableArmActuators};
///
/// // A table arm at rest, commanded shut on a 50 g sphere 59.7 mm wide.
/// let mut report = Actuator::default();
/// report.normalized = vec![0.0, -0.1, 0.25, 0.2, 0.25, 0.1, 0.0, 0.0, 1.0];
/// report.measured = vec![0.0, -0.6, 1.6, 1.3, 1.6, 0.8, 0.0597, 0.0, 0.05];
///
/// let arm = TableArmActuators::from_actuator(&report).expect("nine entries");
/// assert!(arm.holding);
/// assert_eq!(arm.gripper_opening_m, 0.0597);
/// assert_eq!(arm.contact, ArmContact::NoContact);
///
/// // The same nine entries read as a rail arm's: no error, wrong answers.
/// let wrong = RailArmActuators::from_actuator(&report).expect("nine entries");
/// assert_eq!(wrong.joints[6], 0.0597); // the pad gap, as A6
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct TableArmActuators {
    /// Joint positions, A1 to A6, rad. Index with the [`table_arm`] joint
    /// constants.
    pub joints: [f64; table_arm::JOINT_COUNT],
    /// Each joint's position across its range, [-1, 1]. -1 or 1 is a joint at
    /// a limit.
    pub joints_normalized: [f64; table_arm::JOINT_COUNT],
    /// The `SET_ARM_GRIPPER` latch, m: your last command echoed back, after
    /// the clamp to the stroke. It crossed the wire as an `f32` fraction of the
    /// stroke, so compare with a tolerance.
    pub gripper_command_m: f64,
    /// The gap between the pads now, m: the held object's width while holding.
    pub gripper_opening_m: f64,
    /// Whether a body is held between the pads.
    pub holding: bool,
    /// The held body's mass, kg; 0 when nothing is held.
    pub held_mass_kg: f64,
    /// What cut the last joint step short. Fingers stopping on an object are
    /// not a contact: that is how the gripper feels it.
    pub contact: ArmContact,
}

impl TableArmActuators {
    /// Decode a report already in hand as the table arm's, which is also the
    /// way to read a table arm through a handle connected as another type.
    ///
    /// `None` when `normalized` or `measured` holds fewer than
    /// [`table_arm::ACTUATOR_COUNT`] entries. **The robot type is not
    /// checked.**
    #[must_use]
    pub fn from_actuator(actuator: &Actuator) -> Option<TableArmActuators> {
        decode(
            actuator,
            sys::vrsdk_table_arm_actuators_from_actuator,
            "vrsdk_table_arm_actuators_from_actuator",
        )
        .map(|raw| TableArmActuators::from_raw(&raw))
    }

    fn from_raw(raw: &sys::vrsdk_table_arm_actuators_t) -> TableArmActuators {
        TableArmActuators {
            joints: raw.joints,
            joints_normalized: raw.joints_normalized,
            gripper_command_m: raw.gripper_command_m,
            gripper_opening_m: raw.gripper_opening_m,
            holding: raw.holding,
            held_mass_kg: raw.held_mass_kg,
            contact: ArmContact::from_code(raw.contact),
        }
    }
}

/// A turret's joints and laser, decoded from its actuator report: the proof of
/// every turret command, since none of them has a reply.
///
/// The joints are the robot's attitude too: `kin.quat` is the head relative to
/// its base, in the robot's frame (CV unless
/// [`set_frames`](crate::VirtualRobot::set_frames) changed it), not a world
/// attitude. These fields are the same two angles with no frame to unpick.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct TurretActuators {
    /// Pylon angle, rad: positive to the right of rest, seen from above.
    pub heading: f64,
    /// Head angle, rad: positive raises the barrel.
    pub pitch: f64,
    /// The pylon's rate over the last physics step as a fraction of its max
    /// rate, [-1, 1]; 0 while it holds.
    pub heading_rate_fraction: f64,
    /// The head's rate as a fraction of its max pitch rate, [-1, 1].
    pub pitch_rate_fraction: f64,
    /// Whether the laser is switched on.
    pub laser_on: bool,
    /// Muzzle to the first thing the beam hits, m; 0 while the laser is off or
    /// nothing is within its range.
    pub laser_range: f64,
}

impl TurretActuators {
    /// Decode a report already in hand as a turret's.
    ///
    /// `None` when `normalized` or `measured` holds fewer than
    /// [`turret_actuator::COUNT`] entries. **The robot type is not checked**,
    /// so [`VirtualRobot::turret_actuators`] checks the handle first.
    #[must_use]
    pub fn from_actuator(actuator: &Actuator) -> Option<TurretActuators> {
        decode(
            actuator,
            sys::vrsdk_turret_actuators_from_actuator,
            "vrsdk_turret_actuators_from_actuator",
        )
        .map(|raw| TurretActuators::from_raw(&raw))
    }

    fn from_raw(raw: &sys::vrsdk_turret_actuators_t) -> TurretActuators {
        TurretActuators {
            heading: raw.heading,
            pitch: raw.pitch,
            heading_rate_fraction: raw.heading_rate_fraction,
            pitch_rate_fraction: raw.pitch_rate_fraction,
            laser_on: raw.laser_on,
            laser_range: raw.laser_range,
        }
    }
}

/// Typed actuator reports. Each reads [`states`](VirtualRobot::states), so it
/// never blocks; pair it with [`wait_new_state`](VirtualRobot::wait_new_state)
/// to see each sample once. See the [module documentation](crate::actuators).
impl VirtualRobot {
    /// The forklift's drive, steering and mast, from the latest snapshot.
    /// **Forklift only.**
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) if this
    /// handle was not connected as a [`Forklift`](crate::RobotType::Forklift):
    /// the decode is by index, so another robot's report would read as wrong
    /// numbers. [`VrError::Decode`](crate::VrError::Decode) if the report is
    /// shorter than a forklift's, or, once [`properties`](Self::properties)
    /// has been read, the layout the robot published says otherwise: another
    /// kind of robot at this id.
    pub fn forklift_actuators(&self) -> VrResult<ForkliftActuators> {
        let mut raw = sys::vrsdk_forklift_actuators_t::default();
        // SAFETY: the robot handle is live and `raw` is writable storage for
        // one `vrsdk_forklift_actuators_t`.
        check(unsafe { sys::vrsdk_robot_forklift_actuators(self.raw(), &mut raw) })?;
        Ok(ForkliftActuators::from_raw(&raw))
    }

    /// The pallet robot's wheels and pad, from the latest snapshot.
    /// **PalletRobot only.**
    ///
    /// # Errors
    ///
    /// As [`forklift_actuators`](Self::forklift_actuators), for a
    /// [`PalletRobot`](crate::RobotType::PalletRobot).
    pub fn palletrobot_actuators(&self) -> VrResult<PalletRobotActuators> {
        let mut raw = sys::vrsdk_palletrobot_actuators_t::default();
        // SAFETY: the robot handle is live and `raw` is writable storage for
        // one `vrsdk_palletrobot_actuators_t`.
        check(unsafe { sys::vrsdk_robot_palletrobot_actuators(self.raw(), &mut raw) })?;
        Ok(PalletRobotActuators::from_raw(&raw))
    }

    /// The rail arm's joints, vacuum and contact, from the latest snapshot.
    /// **RailArm only.**
    ///
    /// # Errors
    ///
    /// As [`forklift_actuators`](Self::forklift_actuators), for a
    /// [`RailArm`](crate::RobotType::RailArm).
    pub fn rail_arm_actuators(&self) -> VrResult<RailArmActuators> {
        let mut raw = sys::vrsdk_rail_arm_actuators_t::default();
        // SAFETY: the robot handle is live and `raw` is writable storage for
        // one `vrsdk_rail_arm_actuators_t`.
        check(unsafe { sys::vrsdk_robot_rail_arm_actuators(self.raw(), &mut raw) })?;
        Ok(RailArmActuators::from_raw(&raw))
    }

    /// The table arm's joints, gripper and contact, from the latest snapshot.
    /// **TableArm only.**
    ///
    /// # Errors
    ///
    /// As [`forklift_actuators`](Self::forklift_actuators), for a
    /// [`TableArm`](crate::RobotType::TableArm).
    pub fn table_arm_actuators(&self) -> VrResult<TableArmActuators> {
        let mut raw = sys::vrsdk_table_arm_actuators_t::default();
        // SAFETY: the robot handle is live and `raw` is writable storage for
        // one `vrsdk_table_arm_actuators_t`.
        check(unsafe { sys::vrsdk_robot_table_arm_actuators(self.raw(), &mut raw) })?;
        Ok(TableArmActuators::from_raw(&raw))
    }

    /// The turret's joints and laser, from the latest snapshot. **Turret
    /// only.**
    ///
    /// # Errors
    ///
    /// As [`forklift_actuators`](Self::forklift_actuators), for a
    /// [`Turret`](crate::RobotType::Turret).
    pub fn turret_actuators(&self) -> VrResult<TurretActuators> {
        let mut raw = sys::vrsdk_turret_actuators_t::default();
        // SAFETY: the robot handle is live and `raw` is writable storage for
        // one `vrsdk_turret_actuators_t`.
        check(unsafe { sys::vrsdk_robot_turret_actuators(self.raw(), &mut raw) })?;
        Ok(TurretActuators::from_raw(&raw))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ActuatorKind;

    #[test]
    fn contacts_round_trip_their_wire_codes() {
        for contact in [
            ArmContact::NoContact,
            ArmContact::ToolContact,
            ArmContact::CollisionStop,
            ArmContact::Unknown,
        ] {
            assert_eq!(ArmContact::from_code(contact.code()), contact);
        }
        assert_eq!(ArmContact::from_code(7), ArmContact::Unknown);
    }

    #[test]
    fn every_arm_puts_its_tool_at_the_joint_count() {
        assert_eq!(rail_arm::VACUUM, rail_arm::JOINT_COUNT);
        assert_eq!(rail_arm::CONTACT, rail_arm::JOINT_COUNT + 1);
        assert_eq!(table_arm::GRIPPER, table_arm::JOINT_COUNT);
        assert_eq!(table_arm::CONTACT, table_arm::JOINT_COUNT + 1);
        assert_eq!(table_arm::HOLD, table_arm::JOINT_COUNT + 2);
        assert_eq!(
            rail_arm::ACTUATOR_KINDS[rail_arm::RAIL],
            ActuatorKind::Linear
        );
        assert_eq!(
            table_arm::ACTUATOR_KINDS[table_arm::GRIPPER],
            ActuatorKind::Gripper
        );
        assert_eq!(
            turret_actuator::KINDS[turret_actuator::LASER],
            ActuatorKind::Laser
        );
    }
}
