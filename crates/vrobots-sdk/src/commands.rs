//! Commands: the setpoints a robot acts on, and the generic escape hatch.
//!
//! A command is a **setpoint that latches**: the last one received stays in
//! effect until the next arrives, so 5 Hz and 50 Hz senders are both fine and a
//! controller that stops publishing is still commanding its last value. There
//! is no reply: proof that a command landed is the state stream, usually
//! `actuator.pwm` echoing it back. A robot silently ignores an id it does not
//! implement, so a wrong id, a wrong robot and a wrong array length all look
//! the same: nothing changes. Vectors are expressed in your header frame
//! ([`ConnectOptions::with_frame`](crate::ConnectOptions::with_frame)) and the
//! robot converts them into its own.

use std::ptr;

use vrobots_sdk_sys as sys;

use crate::error::VrResult;
use crate::ffi::check;
use crate::robot::VirtualRobot;
use crate::rotations;

/// The command ids, as `Command.cmd_id` wants them, plus the fixed-wing mode
/// values.
///
/// The ones marked **Live** drive a robot today. The rest are defined on the
/// wire but no robot type acts on them yet, and sending one is silently
/// ignored. The id space is shared across robot types, so `SET_CAR` sent to a
/// multirotor is ignored exactly like an unimplemented id.
pub mod cmd {
    use vrobots_sdk_sys as sys;

    /// Commanded linear acceleration. *Not yet acted on.*
    pub const SET_ACC: u32 = sys::VRSDK_CMD_SET_ACC as u32;
    /// Commanded linear velocity. *Not yet acted on.*
    pub const SET_VEL: u32 = sys::VRSDK_CMD_SET_VEL as u32;
    /// Commanded position. *Not yet acted on.*
    pub const SET_POS: u32 = sys::VRSDK_CMD_SET_POS as u32;
    /// Commanded angular acceleration. *Not yet acted on.*
    pub const SET_ANGACC: u32 = sys::VRSDK_CMD_SET_ANGACC as u32;
    /// **Live on the fixed wing.** Commanded body angular velocity, rad/s, in
    /// `vec3`. Also what the simulator's in-game IMU panel publishes; see
    /// [`subscribe_setpoint`](crate::VirtualRobot::subscribe_setpoint).
    pub const SET_ANGVEL: u32 = sys::VRSDK_CMD_SET_ANGVEL as u32;
    /// Commanded Euler angles. *Not yet acted on.*
    pub const SET_EULER: u32 = sys::VRSDK_CMD_SET_EULER as u32;
    /// Commanded Euler rates. *Not yet acted on.*
    pub const SET_EULER_DOT: u32 = sys::VRSDK_CMD_SET_EULER_DOT as u32;
    /// Commanded attitude quaternion. *Not yet acted on.*
    pub const SET_QUAT: u32 = sys::VRSDK_CMD_SET_QUAT as u32;
    /// Set mass. *Live today as
    /// [`set_physical_params`](crate::VirtualRobot::set_physical_params) instead.*
    pub const SET_MASS: u32 = sys::VRSDK_CMD_SET_MASS as u32;
    /// Set principal moments of inertia. *Live today as `set_physical_params`.*
    pub const SET_MOI_3X1: u32 = sys::VRSDK_CMD_SET_MOI_3X1 as u32;
    /// Set the full inertia tensor. *Live today as `set_physical_params`.*
    pub const SET_MOI_3X3: u32 = sys::VRSDK_CMD_SET_MOI_3X3 as u32;
    /// Set an external body force, `vec3`. *Not yet acted on.*
    pub const SET_BODY_FORCE: u32 = sys::VRSDK_CMD_SET_BODY_FORCE as u32;
    /// Set an external body torque, `vec3`. *Not yet acted on.*
    pub const SET_BODY_TORQUE: u32 = sys::VRSDK_CMD_SET_BODY_TORQUE as u32;
    /// Set an external body wrench: force in `vec3`, torque in `vec3_arr[0]`.
    /// *Not yet acted on.*
    pub const SET_BODY_FT: u32 = sys::VRSDK_CMD_SET_BODY_FT as u32;
    /// Add to the external body force. *Not yet acted on.*
    pub const ADD_BODY_FORCE: u32 = sys::VRSDK_CMD_ADD_BODY_FORCE as u32;
    /// Add to the external body torque. *Not yet acted on.*
    pub const ADD_BODY_TORQUE: u32 = sys::VRSDK_CMD_ADD_BODY_TORQUE as u32;
    /// Add to the external body wrench. *Not yet acted on.*
    pub const ADD_BODY_FT: u32 = sys::VRSDK_CMD_ADD_BODY_FT as u32;
    /// **Live.** Multirotor pulse widths, one per rotor, in `int_arr`.
    pub const SET_MR_PWM: u32 = sys::VRSDK_CMD_SET_MR_PWM as u32;
    /// Multirotor normalised throttle. *Not yet acted on.*
    pub const SET_MR_THROTTLE: u32 = sys::VRSDK_CMD_SET_MR_THROTTLE as u32;
    /// Omnidirectional rover. *Robot type not in the simulator yet.*
    pub const SET_OMROVER: u32 = sys::VRSDK_CMD_SET_OMROVER as u32;
    /// Helicopter. *Robot type not in the simulator yet.*
    pub const SET_HELI: u32 = sys::VRSDK_CMD_SET_HELI as u32;
    /// **Live.** Truck `[steer_us, throttle_us(, brake_us)]` in `int_arr`.
    pub const SET_CAR: u32 = sys::VRSDK_CMD_SET_CAR as u32;
    /// **Live.** Mass-spring-damper drive force, newtons, in `float_val`.
    pub const SET_MSD: u32 = sys::VRSDK_CMD_SET_MSD as u32;
    /// **Live.** Cart-pole cart force, newtons along the rail, in `float_val`.
    pub const SET_INVPEN: u32 = sys::VRSDK_CMD_SET_INVPEN as u32;
    /// **Live (fixed wing).** Per-panel deflection, radians, in `float_arr`.
    pub const SET_FW_SURFACES: u32 = sys::VRSDK_CMD_SET_FW_SURFACES as u32;
    /// **Live (fixed wing).** Engine thrust, newtons, in `float_val`.
    pub const SET_FW_THRUST: u32 = sys::VRSDK_CMD_SET_FW_THRUST as u32;
    /// **Live (fixed wing).** Signed thrust trim, newtons, in `float_val`.
    pub const SET_FW_THRUST_BIAS: u32 = sys::VRSDK_CMD_SET_FW_THRUST_BIAS as u32;
    /// **Live (fixed wing).** Control mode in `int_val`: [`FW_ONBOARD_RATE`] or
    /// [`FW_DIRECT_SURFACE`].
    pub const SET_FW_CTRL_MODE: u32 = sys::VRSDK_CMD_SET_FW_CTRL_MODE as u32;
    /// **Live (fixed wing).** Attitude source in `int_val`: [`FW_EST_TRUTH`] or
    /// [`FW_EST_OBSERVER`].
    pub const SET_FW_EST_SOURCE: u32 = sys::VRSDK_CMD_SET_FW_EST_SOURCE as u32;

    /// [`SET_FW_CTRL_MODE`]: the aircraft flies itself, tracking `SET_ANGVEL` on
    /// its onboard rate loop with airspeed hold. The default, and what a reset
    /// returns it to.
    pub const FW_ONBOARD_RATE: i32 = sys::VRSDK_FW_ONBOARD_RATE;
    /// [`SET_FW_CTRL_MODE`]: the rate loop is bypassed and the panels take
    /// [`SET_FW_SURFACES`] verbatim. **You are the autopilot.**
    pub const FW_DIRECT_SURFACE: i32 = sys::VRSDK_FW_DIRECT_SURFACE;
    /// [`SET_FW_EST_SOURCE`]: the onboard loop uses the simulator's true
    /// attitude. The default, and what a reset returns it to.
    pub const FW_EST_TRUTH: i32 = sys::VRSDK_FW_EST_TRUTH;
    /// [`SET_FW_EST_SOURCE`]: the onboard loop uses the attitude published on
    /// the robot's `z/estimate` topic, yours from
    /// [`publish_estimate`](crate::VirtualRobot::publish_estimate).
    pub const FW_EST_OBSERVER: i32 = sys::VRSDK_FW_EST_OBSERVER;

    /// The schema name of a command id, e.g. `"SET_CAR"`, or `""` for an id the
    /// schema does not define. For log lines.
    #[must_use]
    pub fn name(cmd_id: u32) -> &'static str {
        match cmd_id {
            SET_ACC => "SET_ACC",
            SET_VEL => "SET_VEL",
            SET_POS => "SET_POS",
            SET_ANGACC => "SET_ANGACC",
            SET_ANGVEL => "SET_ANGVEL",
            SET_EULER => "SET_EULER",
            SET_EULER_DOT => "SET_EULER_DOT",
            SET_QUAT => "SET_QUAT",
            SET_MASS => "SET_MASS",
            SET_MOI_3X1 => "SET_MOI_3X1",
            SET_MOI_3X3 => "SET_MOI_3X3",
            SET_BODY_FORCE => "SET_BODY_FORCE",
            SET_BODY_TORQUE => "SET_BODY_TORQUE",
            SET_BODY_FT => "SET_BODY_FT",
            ADD_BODY_FORCE => "ADD_BODY_FORCE",
            ADD_BODY_TORQUE => "ADD_BODY_TORQUE",
            ADD_BODY_FT => "ADD_BODY_FT",
            SET_MR_PWM => "SET_MR_PWM",
            SET_MR_THROTTLE => "SET_MR_THROTTLE",
            SET_OMROVER => "SET_OMROVER",
            SET_HELI => "SET_HELI",
            SET_CAR => "SET_CAR",
            SET_MSD => "SET_MSD",
            SET_INVPEN => "SET_INVPEN",
            SET_FW_SURFACES => "SET_FW_SURFACES",
            SET_FW_THRUST => "SET_FW_THRUST",
            SET_FW_THRUST_BIAS => "SET_FW_THRUST_BIAS",
            SET_FW_CTRL_MODE => "SET_FW_CTRL_MODE",
            SET_FW_EST_SOURCE => "SET_FW_EST_SOURCE",
            _ => "",
        }
    }
}

/// The payload of a generic command, for
/// [`send_cmd`](crate::VirtualRobot::send_cmd).
///
/// `cmd_id` decides which fields mean anything: `SET_MR_PWM` reads `int_arr`,
/// `SET_BODY_FORCE` reads `vec3`, and the rest is ignored. Empty vectors and
/// `None` are left off the wire. Floats narrow to 32 bits on the wire.
///
/// ```
/// use vrobots_sdk::{cmd, CmdArgs};
///
/// // What set_car sends, built by hand.
/// let drive = CmdArgs::ints(&[1500, 1600, 1100]);
/// // SET_BODY_FT is asymmetric: force in vec3, torque in vec3_arr.
/// let wrench = CmdArgs::default()
///     .with_vec3([1.0, 0.0, 0.0])
///     .with_vec3_arr(&[[0.0, 0.0, 0.5]]);
/// # let _ = (cmd::SET_CAR, drive, cmd::SET_BODY_FT, wrench);
/// ```
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct CmdArgs {
    /// Scalar integer payload.
    pub int_val: i32,
    /// Scalar float payload.
    pub float_val: f64,
    /// Integer array payload: pulse widths live here.
    pub int_arr: Vec<i32>,
    /// Float array payload.
    pub float_arr: Vec<f64>,
    /// Single vector payload, in your header frame.
    pub vec3: Option<[f64; 3]>,
    /// Single 4-vector payload, ordered `[x, y, z, w]`.
    pub vec4: Option<[f64; 4]>,
    /// Vector-of-vectors payload. `SET_BODY_FT` puts the torque here.
    pub vec3_arr: Vec<[f64; 3]>,
    /// Vector-of-4-vectors payload.
    pub vec4_arr: Vec<[f64; 4]>,
}

impl CmdArgs {
    /// Just an `int_arr`: the shape `SET_MR_PWM` and `SET_CAR` use.
    #[must_use]
    pub fn ints(values: &[i32]) -> CmdArgs {
        CmdArgs::default().with_int_arr(values)
    }

    /// Just a `float_arr`.
    #[must_use]
    pub fn floats(values: &[f64]) -> CmdArgs {
        CmdArgs::default().with_float_arr(values)
    }

    /// Just a `vec3`.
    #[must_use]
    pub fn vector(v: [f64; 3]) -> CmdArgs {
        CmdArgs::default().with_vec3(v)
    }

    /// See [`int_val`](Self::int_val).
    #[must_use]
    pub fn with_int_val(mut self, value: i32) -> Self {
        self.int_val = value;
        self
    }

    /// See [`float_val`](Self::float_val).
    #[must_use]
    pub fn with_float_val(mut self, value: f64) -> Self {
        self.float_val = value;
        self
    }

    /// See [`int_arr`](Self::int_arr).
    #[must_use]
    pub fn with_int_arr(mut self, values: &[i32]) -> Self {
        self.int_arr = values.to_vec();
        self
    }

    /// See [`float_arr`](Self::float_arr).
    #[must_use]
    pub fn with_float_arr(mut self, values: &[f64]) -> Self {
        self.float_arr = values.to_vec();
        self
    }

    /// See [`vec3`](Self::vec3).
    #[must_use]
    pub fn with_vec3(mut self, v: [f64; 3]) -> Self {
        self.vec3 = Some(v);
        self
    }

    /// See [`vec4`](Self::vec4).
    #[must_use]
    pub fn with_vec4(mut self, v: [f64; 4]) -> Self {
        self.vec4 = Some(v);
        self
    }

    /// See [`vec3_arr`](Self::vec3_arr).
    #[must_use]
    pub fn with_vec3_arr(mut self, values: &[[f64; 3]]) -> Self {
        self.vec3_arr = values.to_vec();
        self
    }

    /// See [`vec4_arr`](Self::vec4_arr).
    #[must_use]
    pub fn with_vec4_arr(mut self, values: &[[f64; 4]]) -> Self {
        self.vec4_arr = values.to_vec();
        self
    }

    /// The C struct for this payload. Its pointers borrow from `self`, so it is
    /// valid only while `self` is borrowed and unchanged.
    fn to_raw(&self) -> sys::vrsdk_cmd_args_t {
        fn slice_ptr<T>(values: &[T]) -> *const T {
            if values.is_empty() {
                ptr::null()
            } else {
                values.as_ptr()
            }
        }
        let mut raw = sys::vrsdk_cmd_args_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_cmd_args_t`.
        unsafe { sys::vrsdk_cmd_args_default(&mut raw) };
        raw.int_val = self.int_val;
        raw.float_val = self.float_val;
        raw.int_arr = slice_ptr(&self.int_arr);
        raw.int_arr_len = self.int_arr.len();
        raw.float_arr = slice_ptr(&self.float_arr);
        raw.float_arr_len = self.float_arr.len();
        raw.vec3 = self.vec3.as_ref().map_or(ptr::null(), |v| v.as_ptr());
        raw.vec4 = self.vec4.as_ref().map_or(ptr::null(), |v| v.as_ptr());
        // `[[f64; 3]]` is laid out as 3 * len contiguous doubles, which is the
        // flat layout the C struct describes.
        raw.vec3_arr = slice_ptr(self.vec3_arr.as_flattened());
        raw.vec3_arr_len = self.vec3_arr.len();
        raw.vec4_arr = slice_ptr(self.vec4_arr.as_flattened());
        raw.vec4_arr_len = self.vec4_arr.len();
        raw
    }
}

/// Commands. Every one is a publish with no reply; see the
/// [module documentation](crate::commands).
impl VirtualRobot {
    /// Publish any command by id: the escape hatch for the whole command space,
    /// including ids no robot type acts on yet.
    ///
    /// ```no_run
    /// # use vrobots_sdk::{cmd, CmdArgs, RobotType, VirtualRobot};
    /// # let robot = VirtualRobot::connect(RobotType::Truck, Some(0))?;
    /// robot.send_cmd(cmd::SET_CAR, &CmdArgs::ints(&[1500, 1600, 1100]))?;
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    ///
    /// # Errors
    ///
    /// [`VrError::Deleted`](crate::VrError::Deleted) after
    /// [`delete`](Self::delete); [`VrError::Publish`](crate::VrError::Publish)
    /// if zenoh refuses the put.
    pub fn send_cmd(&self, cmd_id: u32, args: &CmdArgs) -> VrResult<()> {
        let raw = args.to_raw();
        // SAFETY: the robot handle is live; `raw` is an initialised
        // `vrsdk_cmd_args_t` whose arrays point into `args`, which is borrowed
        // and unchanged for the whole call, with lengths that match.
        check(unsafe { sys::vrsdk_robot_send_cmd(self.raw(), cmd_id, &raw) })
    }

    /// `SET_MR_PWM` for a quadrotor: four pulse widths, microseconds.
    ///
    /// The lowest actuation level there is: **you are the flight controller**.
    /// 1100 is idle, 2000 is full, and hover is wherever total thrust crosses
    /// weight. Proof it landed is `states().actuator.pwm` echoing it back. For
    /// any other rotor count use [`set_mr_pwm_n`](Self::set_mr_pwm_n).
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) for a value
    /// outside 1100 to 2000, refused rather than clamped because a normalised
    /// `0.7` is a unit mistake; [`VrError::Deleted`](crate::VrError::Deleted);
    /// [`VrError::Publish`](crate::VrError::Publish).
    pub fn set_mr_pwm(&self, pwm: [f64; 4]) -> VrResult<()> {
        self.set_mr_pwm_n(&pwm)
    }

    /// `SET_MR_PWM` with one pulse width per rotor, however many rotors the
    /// airframe has: two for the half-drone, `states().actuator.pwm.len()` in
    /// general. A wrong length is silently ignored by the robot.
    ///
    /// # Errors
    ///
    /// As [`set_mr_pwm`](Self::set_mr_pwm), plus
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) for an empty
    /// slice.
    pub fn set_mr_pwm_n(&self, pwm: &[f64]) -> VrResult<()> {
        // SAFETY: the robot handle is live and `pwm` is readable for `pwm.len()`
        // doubles; an empty slice passes a dangling but aligned, non-null
        // pointer with a zero count, which the library reads as empty.
        check(unsafe { sys::vrsdk_robot_set_mr_pwm(self.raw(), pwm.as_ptr(), pwm.len()) })
    }

    /// `SET_MR_THROTTLE`: normalised per-rotor throttle, four values. **No
    /// robot type acts on this yet**; it goes on the wire and is ignored.
    ///
    /// # Errors
    ///
    /// [`VrError::Deleted`](crate::VrError::Deleted),
    /// [`VrError::Publish`](crate::VrError::Publish).
    pub fn set_mr_throttle(&self, throttle: [f64; 4]) -> VrResult<()> {
        // SAFETY: the robot handle is live and `throttle` is readable for 4
        // doubles.
        check(unsafe { sys::vrsdk_robot_set_mr_throttle(self.raw(), throttle.as_ptr()) })
    }

    /// `SET_CAR`: steer, throttle and optional brake for the truck, in
    /// microseconds on the 1100 to 2000 band.
    ///
    /// Steer 1100 is full left, 1500 centre, 1900 full right; throttle 1100 is
    /// full reverse, 1500 stop, 1900 full forward. The brake is
    /// **bottom-anchored**: 1100 is released. `None` sends the two-channel form,
    /// which brakes nothing.
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) for a value
    /// outside the band; [`VrError::Deleted`](crate::VrError::Deleted);
    /// [`VrError::Publish`](crate::VrError::Publish).
    pub fn set_car(&self, steer: f64, throttle: f64, brake: Option<f64>) -> VrResult<()> {
        let brake_ptr = brake.as_ref().map_or(ptr::null(), ptr::from_ref);
        // SAFETY: the robot handle is live; `brake_ptr` is NULL or points at the
        // `f64` inside `brake`, alive for the call.
        check(unsafe { sys::vrsdk_robot_set_car(self.raw(), steer, throttle, brake_ptr) })
    }

    /// `SET_BODY_FORCE`: an external force on the body, newtons, in your header
    /// frame. **No robot type acts on this yet.**
    ///
    /// # Errors
    ///
    /// [`VrError::Deleted`](crate::VrError::Deleted),
    /// [`VrError::Publish`](crate::VrError::Publish).
    pub fn set_body_force(&self, force: [f64; 3]) -> VrResult<()> {
        // SAFETY: the robot handle is live and `force` is readable for 3 doubles.
        check(unsafe { sys::vrsdk_robot_set_body_force(self.raw(), force.as_ptr()) })
    }

    /// `SET_BODY_TORQUE`: an external torque on the body, N.m. A pseudovector,
    /// so it converts between frames differently from a force. **No robot type
    /// acts on this yet.**
    ///
    /// # Errors
    ///
    /// [`VrError::Deleted`](crate::VrError::Deleted),
    /// [`VrError::Publish`](crate::VrError::Publish).
    pub fn set_body_torque(&self, torque: [f64; 3]) -> VrResult<()> {
        // SAFETY: the robot handle is live and `torque` is readable for 3
        // doubles.
        check(unsafe { sys::vrsdk_robot_set_body_torque(self.raw(), torque.as_ptr()) })
    }

    /// `SET_BODY_FT`: force and torque together. **No robot type acts on this
    /// yet.**
    ///
    /// # Errors
    ///
    /// [`VrError::Deleted`](crate::VrError::Deleted),
    /// [`VrError::Publish`](crate::VrError::Publish).
    pub fn set_body_ft(&self, force: [f64; 3], torque: [f64; 3]) -> VrResult<()> {
        // SAFETY: the robot handle is live and both arrays are readable for 3
        // doubles.
        check(unsafe { sys::vrsdk_robot_set_body_ft(self.raw(), force.as_ptr(), torque.as_ptr()) })
    }

    /// `SET_ANGVEL`: commanded body angular velocity, rad/s,
    /// `[about x, about y, about z]` in your header frame.
    ///
    /// **Live on the fixed wing only**: its onboard rate loop tracks it in
    /// [`cmd::FW_ONBOARD_RATE`] mode, where in FRD it reads `[p, q, r]`. The
    /// robot re-expresses it as an axial vector, so stamping the right header
    /// frame matters.
    ///
    /// # Errors
    ///
    /// [`VrError::Deleted`](crate::VrError::Deleted),
    /// [`VrError::Publish`](crate::VrError::Publish).
    pub fn set_angvel(&self, rates: [f64; 3]) -> VrResult<()> {
        // SAFETY: the robot handle is live and `rates` is readable for 3 doubles.
        check(unsafe { sys::vrsdk_robot_set_angvel(self.raw(), rates.as_ptr()) })
    }

    /// `SET_MSD`: the drive force on a mass-spring-damper, newtons, the `F` of
    /// `m*x'' + c*x' + k*x = F`.
    ///
    /// Clamped silently by the simulator to the plant's `max_force`, and it
    /// **latches**: the force stays applied until the next one replaces it.
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) for a value
    /// that is not finite; [`VrError::Deleted`](crate::VrError::Deleted);
    /// [`VrError::Publish`](crate::VrError::Publish).
    pub fn set_msd_force(&self, newtons: f64) -> VrResult<()> {
        // SAFETY: the robot handle is live.
        check(unsafe { sys::vrsdk_robot_set_msd_force(self.raw(), newtons) })
    }

    /// `SET_INVPEN`: the drive force on a cart-pole's cart, newtons along the
    /// rail's +x. The only actuator a cart-pole has.
    ///
    /// Clamped silently to
    /// [`CartPoleConfig::max_force`](crate::CartPoleConfig::max_force), and it
    /// latches, so a balance loop must keep publishing.
    ///
    /// # Errors
    ///
    /// As [`set_msd_force`](Self::set_msd_force).
    pub fn set_cartpole_force(&self, newtons: f64) -> VrResult<()> {
        // SAFETY: the robot handle is live.
        check(unsafe { sys::vrsdk_robot_set_cartpole_force(self.raw(), newtons) })
    }

    /// `SET_FW_CTRL_MODE`: who flies the fixed wing, [`cmd::FW_ONBOARD_RATE`]
    /// or [`cmd::FW_DIRECT_SURFACE`].
    ///
    /// Direct-surface mode bypasses the simulator's rate loop and hands the six
    /// panels to [`set_fw_surfaces`](Self::set_fw_surfaces): you become the
    /// autopilot, mixing included. Two things catch people out:
    /// [`reset`](Self::reset) **reverts the mode to onboard**, so a
    /// direct-surface client must re-assert it; and transitions are bumpless,
    /// seeded from the plant, so send [`set_fw_thrust`](Self::set_fw_thrust)
    /// **after** every mode entry. Nothing acknowledges the switch.
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) for any
    /// other value, which the simulator would ignore;
    /// [`VrError::Deleted`](crate::VrError::Deleted);
    /// [`VrError::Publish`](crate::VrError::Publish).
    pub fn set_fw_ctrl_mode(&self, mode: i32) -> VrResult<()> {
        // SAFETY: the robot handle is live.
        check(unsafe { sys::vrsdk_robot_set_fw_ctrl_mode(self.raw(), mode) })
    }

    /// `SET_FW_SURFACES`: one deflection per aero panel, radians. Only acted on
    /// in [`cmd::FW_DIRECT_SURFACE`] mode.
    ///
    /// **There is no mixer**: each entry drives its own panel. The RQ-4B has
    /// six, in this order: left outboard flap, right outboard flap, left inner
    /// flap, right inner flap, rear left ruddervator, rear right ruddervator.
    /// The length must equal the panel count exactly or the simulator drops the
    /// whole command. Each deflection is clamped to the airframe's limit and
    /// echoed on `actuator.measured` at the same index.
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) for an
    /// empty slice or a value that is not finite;
    /// [`VrError::Deleted`](crate::VrError::Deleted);
    /// [`VrError::Publish`](crate::VrError::Publish).
    pub fn set_fw_surfaces(&self, radians: &[f64]) -> VrResult<()> {
        // SAFETY: the robot handle is live and `radians` is readable for
        // `radians.len()` doubles (a zero count for an empty slice).
        check(unsafe {
            sys::vrsdk_robot_set_fw_surfaces(self.raw(), radians.as_ptr(), radians.len())
        })
    }

    /// `SET_FW_THRUST`: engine thrust, newtons, clamped by the simulator to
    /// `[0, max_thrust]` (20 kN on the RQ-4B) and echoed on the last entry of
    /// `actuator.measured`.
    ///
    /// In direct-surface mode it is the thrust. In onboard mode it takes the
    /// engine off airspeed hold and pins it here, clearing any
    /// [`set_fw_thrust_bias`](Self::set_fw_thrust_bias).
    ///
    /// # Errors
    ///
    /// As [`set_msd_force`](Self::set_msd_force).
    pub fn set_fw_thrust(&self, newtons: f64) -> VrResult<()> {
        // SAFETY: the robot handle is live.
        check(unsafe { sys::vrsdk_robot_set_fw_thrust(self.raw(), newtons) })
    }

    /// `SET_FW_THRUST_BIAS`: a signed trim on the onboard airspeed loop,
    /// newtons. `0` is neutral and also releases a
    /// [`set_fw_thrust`](Self::set_fw_thrust) override. Ignored in
    /// direct-surface mode.
    ///
    /// # Errors
    ///
    /// As [`set_msd_force`](Self::set_msd_force).
    pub fn set_fw_thrust_bias(&self, newtons: f64) -> VrResult<()> {
        // SAFETY: the robot handle is live.
        check(unsafe { sys::vrsdk_robot_set_fw_thrust_bias(self.raw(), newtons) })
    }

    /// `SET_FW_EST_SOURCE`: which attitude the onboard loop believes,
    /// [`cmd::FW_EST_TRUTH`] or [`cmd::FW_EST_OBSERVER`].
    ///
    /// Observer feeds the loop whatever is published on the robot's
    /// `z/estimate` topic, which is how an estimator you wrote flies the
    /// aircraft; see [`publish_estimate`](Self::publish_estimate). An estimate
    /// older than half a second is stale and the loop falls back to truth, and
    /// [`reset`](Self::reset) returns the source to truth.
    ///
    /// # Errors
    ///
    /// As [`set_fw_ctrl_mode`](Self::set_fw_ctrl_mode).
    pub fn set_fw_est_source(&self, source: i32) -> VrResult<()> {
        // SAFETY: the robot handle is live.
        check(unsafe { sys::vrsdk_robot_set_fw_est_source(self.raw(), source) })
    }

    /// Publish **your** attitude estimate on the robot's `z/estimate` topic.
    ///
    /// The simulator runs no filter and leaves `State::estimate` empty; this is
    /// the other half of that split. `quat` is `[x, y, z, w]` in your header
    /// frame and is sent exactly as given, not normalised. `angular_rates` is
    /// the believed body rates in rad/s, or `None` for a filter that does not
    /// estimate them, which is different from claiming zero. `valid` is a gate:
    /// the simulator drops an invalid estimate.
    ///
    /// Unlike a command, an estimate **does not latch**: the fixed wing ages it
    /// from arrival and falls back to truth after half a second, so publish it
    /// every control iteration, at 20 Hz or better.
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) for a
    /// component that is not finite or a degenerate quaternion;
    /// [`VrError::Deleted`](crate::VrError::Deleted);
    /// [`VrError::Publish`](crate::VrError::Publish).
    pub fn publish_estimate(
        &self,
        quat: [f64; 4],
        angular_rates: Option<[f64; 3]>,
        valid: bool,
    ) -> VrResult<()> {
        let rates = angular_rates.as_ref().map_or(ptr::null(), |r| r.as_ptr());
        // SAFETY: the robot handle is live; `quat` is readable for 4 doubles and
        // `rates` is NULL or points at the 3 doubles inside `angular_rates`,
        // alive for the call.
        check(unsafe { sys::vrsdk_robot_publish_estimate(self.raw(), quat.as_ptr(), rates, valid) })
    }

    /// [`publish_estimate`](Self::publish_estimate), from Euler angles.
    ///
    /// `euler` is stored `[about x, about y, about z]` in radians and `order` is
    /// the application order; an FRD publisher wants
    /// [`EulerOrder::Zyx`](rotations::EulerOrder::Zyx). Everything after the
    /// conversion is exactly `publish_estimate`.
    ///
    /// # Errors
    ///
    /// As [`publish_estimate`](Self::publish_estimate).
    pub fn publish_estimate_euler(
        &self,
        euler: [f64; 3],
        order: rotations::EulerOrder,
        angular_rates: Option<[f64; 3]>,
        valid: bool,
    ) -> VrResult<()> {
        let rates = angular_rates.as_ref().map_or(ptr::null(), |r| r.as_ptr());
        // SAFETY: the robot handle is live; `euler` is readable for 3 doubles,
        // `order` names a defined sequence, and `rates` is NULL or points at the
        // 3 doubles inside `angular_rates`, alive for the call.
        check(unsafe {
            sys::vrsdk_robot_publish_estimate_euler(
                self.raw(),
                euler.as_ptr(),
                order.to_wire(),
                rates,
                valid,
            )
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_id_has_its_schema_name() {
        assert_eq!(cmd::name(cmd::SET_CAR), "SET_CAR");
        assert_eq!(cmd::name(cmd::SET_FW_EST_SOURCE), "SET_FW_EST_SOURCE");
        assert_eq!(cmd::name(cmd::SET_ANGVEL), "SET_ANGVEL");
        assert_eq!(cmd::name(9999), "");
        assert_eq!(
            (cmd::SET_MR_PWM, cmd::SET_CAR, cmd::SET_INVPEN),
            (300, 304, 306)
        );
    }

    #[test]
    fn the_c_payload_points_into_the_rust_one() {
        let args = CmdArgs::ints(&[1500, 1600])
            .with_vec3([1.0, 2.0, 3.0])
            .with_vec3_arr(&[[4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        let raw = args.to_raw();
        assert_eq!(raw.int_arr, args.int_arr.as_ptr());
        assert_eq!(raw.int_arr_len, 2);
        assert!(raw.float_arr.is_null());
        assert_eq!(raw.float_arr_len, 0);
        assert!(raw.vec4.is_null());
        assert_eq!(raw.vec3_arr_len, 2);
        // SAFETY: `vec3_arr` points at the 6 doubles owned by `args`, alive here.
        let flat = unsafe { std::slice::from_raw_parts(raw.vec3_arr, 6) };
        assert_eq!(flat, [4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
    }
}
