//! The robot handle: connect, read state, pace a loop, delete.
//!
//! A [`VirtualRobot`] is one robot in the simulator, attached by `sys_id` or
//! created on connect. **Robots outlive the process**: dropping the handle
//! closes the session and leaves the robot running, and
//! [`delete`](VirtualRobot::delete) is the only verb that removes one. Reading
//! is never a wait: [`states`](VirtualRobot::states) returns the latest
//! snapshot an SDK-owned subscriber keeps fresh, and you own the loop and its
//! rate. Commands, services, cameras and setpoints are methods on the same
//! handle, documented in [`commands`](crate::commands),
//! [`services`](crate::services), [`camera`](crate::camera) and
//! [`setpoint`](crate::setpoint).

use std::ffi::c_char;
use std::fmt;
use std::ptr::{self, NonNull};
use std::time::Duration;

use vrobots_sdk_sys as sys;

use crate::error::{VrError, VrResult};
use crate::ffi::{self, check, expect_ok};
use crate::options::{ConnectOptions, RobotType};
use crate::state::{State, StateStats};

/// Capacity for the decode-failure detail read back by `last_error`.
pub(crate) const MESSAGE_CAPACITY: usize = 1024;

/// One robot in the simulator.
///
/// Obtained connected, from [`connect`](Self::connect) or
/// [`connect_with`](Self::connect_with), which block until the first state
/// snapshot has arrived, so [`states`](Self::states) is real data the moment
/// they return.
///
/// ```no_run
/// use vrobots_sdk::{RobotType, VirtualRobot, VrError};
///
/// fn main() -> Result<(), VrError> {
///     // ===== setup =====
///     vrobots_sdk::init_logging("info");
///     let robot = VirtualRobot::connect(RobotType::Multirotor, Some(1))?;
///
///     // ===== loop =====
///     loop {
///         let s = robot.states(); // latest snapshot, never torn
///         let [x, y, z] = s.kin.lin_pos;
///         println!("t={:.3} pos=({x:.2},{y:.2},{z:.2})", s.elapsed);
///         robot.set_mr_pwm([1501.0; 4])?; // your controller's output
///         robot.rate(100.0); // drift-compensated pacing, Hz
///     }
/// }
/// ```
///
/// # Threading
///
/// `VirtualRobot` is `Send` and `Sync`: share it between threads with `&` or an
/// `Arc`, for instance one thread in a control loop and another reading
/// [`stats`](Self::stats). The C library documents its robot handle as safe to
/// use from several threads (the C++ header shipped with it says so, every
/// call takes the handle as `const`, and the error detail of a failing call is
/// kept per thread). The one rule it leaves to the caller, never to free a
/// handle another thread is using, is enforced by ownership: the handle is
/// freed only in `Drop`. See also the crate-level section on threads.
///
/// # Calls that cannot fail
///
/// [`states`](Self::states), [`stats`](Self::stats), [`rate`](Self::rate) and
/// [`last_error`](Self::last_error) have no failure on a connected handle. They
/// panic only if the C library reports an internal error (`VRSDK_ERR_PANIC`),
/// which is a bug in the SDK.
pub struct VirtualRobot {
    raw: RawRobot,
    sys_id: u32,
    robot_type: RobotType,
    options: ConnectOptions,
}

impl fmt::Debug for VirtualRobot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("VirtualRobot")
            .field("sys_id", &self.sys_id)
            .field("robot_type", &self.robot_type)
            .field("deleted", &self.is_deleted())
            .finish_non_exhaustive()
    }
}

impl VirtualRobot {
    /// Connect with default [`ConnectOptions`].
    ///
    /// `Some(sys_id)` attaches to an existing robot; `None` creates a new one,
    /// and [`sys_id`](Self::sys_id) then reports the id the manager assigned.
    /// Either way this blocks until the first state snapshot arrives.
    ///
    /// Attaching never touches the create service, so it works for any robot
    /// the scene contains, including the scene-authored kinds the spawn catalog
    /// refuses to create. `vrobots topic list` shows the ids that exist.
    ///
    /// # Errors
    ///
    /// [`VrError::Session`] if zenoh will not open; [`VrError::Timeout`] if no
    /// state arrives within [`ConnectOptions::probe_timeout`], which is what
    /// attaching to an id nobody publishes looks like;
    /// [`VrError::NoResponder`] if the manager is not there;
    /// [`VrError::Service`] if the simulator refuses a create, carrying its own
    /// message, which lists the catalog.
    pub fn connect(robot_type: RobotType, sys_id: Option<u32>) -> VrResult<VirtualRobot> {
        VirtualRobot::connect_with(robot_type, sys_id, ConnectOptions::default())
    }

    /// Connect with explicit options. See [`connect`](Self::connect).
    ///
    /// ```no_run
    /// use std::time::Duration;
    /// use vrobots_sdk::{Axes, ConnectOptions, RobotType, VirtualRobot};
    ///
    /// let robot = VirtualRobot::connect_with(
    ///     RobotType::GlobalHawk,
    ///     Some(15),
    ///     ConnectOptions::default()
    ///         .with_frame("frd", Axes::FRD)
    ///         .with_service_timeout(Duration::from_secs(3)),
    /// )?;
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    ///
    /// # Errors
    ///
    /// As [`connect`](Self::connect), plus [`VrError::InvalidArgument`] for a
    /// zero `src_id`, a zero timeout or a string containing a NUL byte, and
    /// [`VrError::Config`] for an unusable router endpoint.
    pub fn connect_with(
        robot_type: RobotType,
        sys_id: Option<u32>,
        options: ConnectOptions,
    ) -> VrResult<VirtualRobot> {
        let mut raw = RawRobot::new(robot_type, sys_id, &options)?;
        // SAFETY: `raw` is a live handle from `vrsdk_robot_new`, owned here and
        // not shared with any other thread, as `vrsdk_robot_connect` requires.
        // On failure `raw` drops and frees the never-connected handle.
        check(unsafe { sys::vrsdk_robot_connect(raw.as_mut_ptr()) })?;

        let mut id = 0u32;
        // SAFETY: `raw` is live and connected; `id` is writable.
        check(unsafe { sys::vrsdk_robot_sys_id(raw.as_ptr(), &mut id) })?;
        Ok(VirtualRobot {
            raw,
            sys_id: id,
            robot_type,
            options,
        })
    }

    /// This robot's id: the address of every topic and service it owns.
    #[must_use]
    pub fn sys_id(&self) -> u32 {
        self.sys_id
    }

    /// The kind this handle was connected as.
    ///
    /// On an attach this is what the caller said, not something read off the
    /// wire: the state message carries no catalog key, and a mismatch presents
    /// the usual way, as a robot that silently ignores commands it does not
    /// implement.
    #[must_use]
    pub fn robot_type(&self) -> RobotType {
        let mut code = self.robot_type.code();
        // SAFETY: `self.raw` is a live handle and `code` is one writable
        // `vrsdk_robot_type_t`, which is all `vrsdk_robot_type` requires.
        let status = unsafe { sys::vrsdk_robot_type(self.raw.as_ptr(), &mut code) };
        expect_ok(status, "vrsdk_robot_type");
        RobotType::from_code(code).unwrap_or(self.robot_type)
    }

    /// The options this robot connected with.
    #[must_use]
    pub fn options(&self) -> ConnectOptions {
        self.options.clone()
    }

    /// Whether the session is open. True for every handle
    /// [`connect`](Self::connect) returned, including after
    /// [`delete`](Self::delete), which spends the robot rather than the session.
    #[must_use]
    pub fn is_connected(&self) -> bool {
        // SAFETY: `self.raw` is a live handle for as long as `self` exists.
        unsafe { sys::vrsdk_robot_is_connected(self.raw.as_ptr()) }
    }

    /// The latest state snapshot, copied out.
    ///
    /// Never blocks and never returns a torn value. **If the simulator stops,
    /// this keeps returning the last snapshot** rather than failing: that is the
    /// observer contract. Watch [`State::elapsed`], or use
    /// [`wait_new_state`](Self::wait_new_state), to detect a stall.
    #[must_use]
    pub fn states(&self) -> State {
        let mut raw = sys::vrsdk_state_t::default();
        // SAFETY: `self.raw` is live and connected; `raw` is writable storage
        // for one `vrsdk_state_t`.
        let code = unsafe { sys::vrsdk_robot_states(self.raw.as_ptr(), &mut raw) };
        expect_ok(code, "vrsdk_robot_states");
        State::from_raw(&raw)
    }

    /// Block until a state snapshot newer than the current one arrives.
    ///
    /// For state-paced loops that run exactly once per sample instead of
    /// polling.
    ///
    /// # Errors
    ///
    /// [`VrError::Timeout`] if no new snapshot arrives in time. That is how a
    /// stalled or paused simulator is detected; it never means the session is
    /// broken. [`VrError::InvalidArgument`] for a zero timeout.
    pub fn wait_new_state(&self, timeout: Duration) -> VrResult<()> {
        // SAFETY: `self.raw` is live and connected.
        check(unsafe { sys::vrsdk_robot_wait_new_state(self.raw.as_ptr(), ffi::seconds(timeout)) })
    }

    /// Sleep so that the calling loop runs at `hz`.
    ///
    /// Drift-compensated: deadlines are absolute multiples of the period from
    /// the first call, so the work in the loop body does not accumulate as
    /// drift, and an iteration that overruns re-anchors instead of bursting. A
    /// non-positive or non-finite `hz` returns immediately.
    pub fn rate(&self, hz: f64) {
        // SAFETY: `self.raw` is live and connected.
        let code = unsafe { sys::vrsdk_robot_rate(self.raw.as_ptr(), hz) };
        expect_ok(code, "vrsdk_robot_rate");
    }

    /// Subscriber counters: samples, decode failures, sequence gaps.
    #[must_use]
    pub fn stats(&self) -> StateStats {
        let mut raw = sys::vrsdk_state_stats_t::default();
        // SAFETY: `self.raw` is live and connected; `raw` is writable.
        let code = unsafe { sys::vrsdk_robot_stats(self.raw.as_ptr(), &mut raw) };
        expect_ok(code, "vrsdk_robot_stats");
        StateStats::from_raw(&raw)
    }

    /// The most recent decode failure on the state stream, if any.
    ///
    /// Decode failures are counted in [`stats`](Self::stats) and stored here
    /// rather than returned: they never tear the session down, and the loop
    /// keeps running.
    #[must_use]
    pub fn last_error(&self) -> Option<VrError> {
        let mut code = sys::VRSDK_OK;
        let mut message: [c_char; MESSAGE_CAPACITY] = [0; MESSAGE_CAPACITY];
        // SAFETY: `self.raw` is live and connected; `code` is writable and
        // `message` is writable for its full length, which is the capacity
        // passed.
        let status = unsafe {
            sys::vrsdk_robot_last_error(
                self.raw.as_ptr(),
                &mut code,
                message.as_mut_ptr(),
                message.len(),
            )
        };
        expect_ok(status, "vrsdk_robot_last_error");
        VrError::from_code(code, ffi::fixed_str(&message))
    }

    /// Remove this robot from the simulator.
    ///
    /// Explicit and never implicit: dropping the handle only closes the
    /// session. The manager's ack is only a receipt, so this also waits for the
    /// robot's state topic to fall silent, which is the real confirmation.
    /// Afterwards the handle is spent: commands and services fail with
    /// [`VrError::Deleted`], while [`states`](Self::states) keeps returning the
    /// last snapshot.
    ///
    /// # Errors
    ///
    /// [`VrError::Service`] if the manager refuses, [`VrError::NoResponder`] or
    /// [`VrError::Timeout`] if it does not answer, [`VrError::Deleted`] if this
    /// handle already deleted the robot.
    pub fn delete(&self) -> VrResult<()> {
        // SAFETY: `self.raw` is live and connected.
        check(unsafe { sys::vrsdk_robot_delete(self.raw.as_ptr()) })
    }

    /// Whether [`delete`](Self::delete) has been called on this handle.
    #[must_use]
    pub fn is_deleted(&self) -> bool {
        // SAFETY: `self.raw` is a live handle.
        unsafe { sys::vrsdk_robot_is_deleted(self.raw.as_ptr()) }
    }

    /// The C handle, for calling a function of [`crate::sys`] directly.
    ///
    /// Still owned by this value: do not free it, and do not use it after this
    /// value is dropped.
    #[must_use]
    pub fn as_raw(&self) -> *const sys::vrsdk_robot_t {
        self.raw.as_ptr()
    }

    pub(crate) fn raw(&self) -> *const sys::vrsdk_robot_t {
        self.raw.as_ptr()
    }
}

/// Owner of one `vrsdk_robot_t`, connected or not; frees it on drop.
pub(crate) struct RawRobot(NonNull<sys::vrsdk_robot_t>);

impl RawRobot {
    /// Build a handle. Touches nothing: no session, no network.
    pub(crate) fn new(
        robot_type: RobotType,
        sys_id: Option<u32>,
        options: &ConnectOptions,
    ) -> VrResult<RawRobot> {
        let options = options.to_c()?;
        let mut out: *mut sys::vrsdk_robot_t = ptr::null_mut();
        // SAFETY: `options` is an initialised struct whose strings live until the
        // end of this function, longer than the call, and `out` is a writable
        // pointer slot. A negative id asks for a create.
        let code = unsafe {
            sys::vrsdk_robot_new(
                robot_type.code(),
                sys_id.map_or(-1, i64::from),
                options.as_ptr(),
                &mut out,
            )
        };
        check(code)?;
        NonNull::new(out).map(RawRobot).ok_or_else(|| {
            VrError::InvalidHandle(
                "vrsdk_robot_new reported success but returned no handle; this is a bug in the \
                 VRobots SDK"
                    .to_string(),
            )
        })
    }

    fn as_ptr(&self) -> *const sys::vrsdk_robot_t {
        self.0.as_ptr()
    }

    fn as_mut_ptr(&mut self) -> *mut sys::vrsdk_robot_t {
        self.0.as_ptr()
    }
}

impl Drop for RawRobot {
    /// Closes the session. **Never deletes the robot.** Camera and setpoint
    /// streams opened from this handle keep running until they are dropped.
    fn drop(&mut self) {
        // SAFETY: the handle came from `vrsdk_robot_new`, is freed exactly once,
        // here, and `&mut self` proves no other reference to it is in use.
        unsafe { sys::vrsdk_robot_free(self.0.as_ptr()) };
    }
}

// SAFETY: the C library documents its robot handle as usable from several
// threads (the THREADING section of the C++ header shipped beside
// `vrobots_sdk.h`); moving the owner between threads moves nothing but the
// pointer, and `vrsdk_robot_free` runs only in `Drop`, with exclusive access,
// so no other thread can be using the handle when it is freed.
unsafe impl Send for RawRobot {}
// SAFETY: every call reachable through `&RawRobot` passes the handle as
// `const vrsdk_robot_t *`, the form the library allows from several threads
// at once; the one call that takes it mutably, `vrsdk_robot_connect`, is made
// through `&mut` before the handle is shared.
unsafe impl Sync for RawRobot {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_handle_that_never_connects_frees_cleanly() {
        let options = ConnectOptions::default();
        for (kind, id) in [
            (RobotType::Multirotor, Some(1)),
            (RobotType::Truck, None),
            (RobotType::GlobalHawk, Some(u32::MAX)),
        ] {
            let raw = RawRobot::new(kind, id, &options).expect("construction touches nothing");
            // SAFETY: `raw` is live for this block.
            assert!(!unsafe { sys::vrsdk_robot_is_connected(raw.as_ptr()) });
            let mut code = -1;
            // SAFETY: `raw` is live and `code` writable.
            let status = unsafe { sys::vrsdk_robot_type(raw.as_ptr(), &mut code) };
            assert_eq!(status, sys::VRSDK_OK);
            assert_eq!(RobotType::from_code(code), Some(kind));
            drop(raw);
        }
    }

    #[test]
    fn a_handle_that_will_create_has_no_id_until_it_connects() {
        let raw = RawRobot::new(RobotType::Msd, None, &ConnectOptions::default()).expect("builds");
        let mut id = 0u32;
        // SAFETY: `raw` is live and `id` writable.
        let err = check(unsafe { sys::vrsdk_robot_sys_id(raw.as_ptr(), &mut id) })
            .expect_err("no id before a create");
        assert!(matches!(err, VrError::Session(_)), "{err:?}");
    }

    #[test]
    fn a_zero_timeout_is_refused_when_the_handle_is_built() {
        let options = ConnectOptions::default().with_probe_timeout(Duration::ZERO);
        let err = RawRobot::new(RobotType::Multirotor, Some(1), &options)
            .err()
            .expect("a zero timeout is refused");
        assert!(matches!(err, VrError::InvalidArgument(_)), "{err:?}");
        assert!(err.detail().contains("probe_timeout_s"), "{err}");
    }
}
