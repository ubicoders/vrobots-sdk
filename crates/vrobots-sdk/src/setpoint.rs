//! Setpoints: reading somebody else's commands off a robot's command topic.
//!
//! Commands are write-only everywhere else, but zenoh is many-to-many, so a
//! robot's `z/cmd` topic is a shared bus: the simulator's in-game IMU panel
//! publishes `SET_ANGVEL` on it at 50 Hz, and a client can read that stick as
//! an input to its own controller. **Everything anyone sends passes through,
//! this process's own commands included**; compare [`Setpoint::src_id`] with
//! your [`ConnectOptions::src_id`](crate::ConnectOptions::src_id) to ignore your
//! own. The vector arrives in the sender's frame, unconverted.

use std::ffi::c_char;
use std::fmt;
use std::ptr::{self, NonNull};
use std::time::Duration;

use vrobots_sdk_sys as sys;

use crate::error::{VrError, VrResult};
use crate::ffi::{self, check, expect_ok, fixed_str};
use crate::robot::{VirtualRobot, MESSAGE_CAPACITY};
use crate::state::Axes;

/// One command seen on a robot's `z/cmd` topic.
///
/// The vector is taken exactly as the sender stamped it: `axis_convention` and
/// `coord_frame_id` say which frame that is, and converting into yours is your
/// business. For `SET_ANGVEL` from the IMU panel that is the target robot's own
/// frame, `[p, q, r]` in rad/s for a fixed wing in FRD.
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct Setpoint {
    /// The command id this was filtered on.
    pub cmd_id: u32,
    /// The `vec3` payload, in the sender's frame. rad/s for `SET_ANGVEL`.
    pub value: [f64; 3],
    /// The convention `value` is expressed in.
    pub axis_convention: Axes,
    /// The frame id `value` is expressed in. Authoritative when it and
    /// `axis_convention` disagree.
    pub coord_frame_id: String,
    /// Who sent it: the IMU panel is 108, and this client is its
    /// [`ConnectOptions::src_id`](crate::ConnectOptions::src_id).
    pub src_id: u32,
    /// Which robot it was addressed to.
    pub sys_id: u32,
    /// The sender's per-topic sequence number.
    pub seq: u64,
    /// The sender's capture time, nanoseconds since the unix epoch.
    pub t_ns: i64,
    /// Seconds since the robot's first state sample: the same clock as
    /// [`State::elapsed`](crate::State::elapsed), so the two subtract directly.
    pub elapsed: f64,
}

impl Setpoint {
    fn from_raw(raw: &sys::vrsdk_setpoint_t) -> Setpoint {
        Setpoint {
            cmd_id: raw.cmd_id,
            value: raw.value,
            axis_convention: Axes(raw.axis_convention),
            coord_frame_id: fixed_str(&raw.coord_frame_id),
            src_id: raw.src_id,
            sys_id: raw.sys_id,
            seq: raw.seq,
            t_ns: raw.t_ns,
            elapsed: raw.elapsed,
        }
    }
}

/// Subscriber counters for one setpoint stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub struct SetpointStats {
    /// Matching setpoints received.
    pub received: u64,
    /// Samples that were some **other** command. Expected to be large: it is
    /// every other peer's traffic to the same robot.
    pub filtered: u64,
    /// Payloads that did not decode as a command. Counted, never fatal.
    pub decode_errors: u64,
    /// Times a matching sequence number jumped forward by more than one. Only
    /// meaningful with a single publisher.
    pub seq_gaps: u64,
    /// The last matching sequence number seen.
    pub last_seq: u64,
}

/// A live view of one command id on a robot's `z/cmd` topic, from
/// [`subscribe_setpoint`](VirtualRobot::subscribe_setpoint) or
/// [`subscribe_command`](VirtualRobot::subscribe_command).
///
/// Dropping it undeclares the subscriber; it does not stop anyone publishing.
/// `Send` and `Sync`: each value is handed out once by
/// [`fresh`](Self::fresh), so two threads racing on one stream never both
/// receive it.
pub struct SetpointStream {
    raw: NonNull<sys::vrsdk_setpoint_stream_t>,
    cmd_id: u32,
    key: String,
}

impl fmt::Debug for SetpointStream {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SetpointStream")
            .field("cmd_id", &self.cmd_id)
            .field("key", &self.key)
            .finish_non_exhaustive()
    }
}

impl SetpointStream {
    /// Take ownership of a stream handle and read its identity once.
    fn adopt(out: *mut sys::vrsdk_setpoint_stream_t) -> VrResult<SetpointStream> {
        let raw = NonNull::new(out).ok_or_else(|| {
            VrError::InvalidHandle(
                "the library reported a setpoint stream but returned no handle; this is a bug in \
                 the VRobots SDK"
                    .to_string(),
            )
        })?;
        let mut stream = SetpointStream {
            raw,
            cmd_id: 0,
            key: String::new(),
        };
        // SAFETY: the stream handle is live and `cmd_id` is writable.
        check(unsafe { sys::vrsdk_setpoint_cmd_id(raw.as_ptr(), &mut stream.cmd_id) })?;
        let mut key: [c_char; 512] = [0; 512];
        // SAFETY: the stream handle is live and `key` is writable for its full
        // length, the capacity passed.
        check(unsafe { sys::vrsdk_setpoint_key(raw.as_ptr(), key.as_mut_ptr(), key.len()) })?;
        stream.key = fixed_str(&key);
        Ok(stream)
    }

    /// The latest setpoint **if it is new since the last call**, otherwise
    /// `None`: the read for a loop that must not act twice on one input.
    ///
    /// `None` does not mean the setpoint went away. Commands latch, and a
    /// publisher that stops has not commanded zero; use
    /// [`latest`](Self::latest) for that reading.
    #[must_use]
    pub fn fresh(&self) -> Option<Setpoint> {
        self.read(sys::vrsdk_setpoint_fresh, "vrsdk_setpoint_fresh")
    }

    /// The latest setpoint, new or not, or `None` before the first one. Does
    /// not consume freshness. The read for a rate loop, since a setpoint
    /// latches.
    #[must_use]
    pub fn latest(&self) -> Option<Setpoint> {
        self.read(sys::vrsdk_setpoint_latest, "vrsdk_setpoint_latest")
    }

    /// Block until a setpoint newer than the last stored one arrives.
    ///
    /// # Errors
    ///
    /// [`VrError::Timeout`] if none arrives in time, which means nobody is
    /// publishing, not that the stream is broken; [`VrError::InvalidArgument`]
    /// for a zero timeout.
    pub fn wait_new_setpoint(&self, timeout: Duration) -> VrResult<()> {
        // SAFETY: the stream handle is live.
        check(unsafe {
            sys::vrsdk_setpoint_wait_new_setpoint(self.raw.as_ptr(), ffi::seconds(timeout))
        })
    }

    /// The command id this stream filters for.
    #[must_use]
    pub fn cmd_id(&self) -> u32 {
        self.cmd_id
    }

    /// The key this stream reads, as `vrobots topic list` prints it, e.g.
    /// `vrobots/4/z/cmd`.
    #[must_use]
    pub fn key(&self) -> &str {
        &self.key
    }

    /// Subscriber counters.
    #[must_use]
    pub fn stats(&self) -> SetpointStats {
        let mut raw = sys::vrsdk_setpoint_stats_t::default();
        // SAFETY: the stream handle is live and `raw` is writable.
        let code = unsafe { sys::vrsdk_setpoint_stats(self.raw.as_ptr(), &mut raw) };
        expect_ok(code, "vrsdk_setpoint_stats");
        SetpointStats {
            received: raw.received,
            filtered: raw.filtered,
            decode_errors: raw.decode_errors,
            seq_gaps: raw.seq_gaps,
            last_seq: raw.last_seq,
        }
    }

    /// The most recent decode failure on this stream, if any.
    #[must_use]
    pub fn last_error(&self) -> Option<VrError> {
        let mut code = sys::VRSDK_OK;
        let mut message: [c_char; MESSAGE_CAPACITY] = [0; MESSAGE_CAPACITY];
        // SAFETY: the stream handle is live; `code` is writable and `message`
        // is writable for its full length, the capacity passed.
        let status = unsafe {
            sys::vrsdk_setpoint_last_error(
                self.raw.as_ptr(),
                &mut code,
                message.as_mut_ptr(),
                message.len(),
            )
        };
        expect_ok(status, "vrsdk_setpoint_last_error");
        VrError::from_code(code, fixed_str(&message))
    }

    /// Whether the subscription is still declared.
    #[must_use]
    pub fn is_running(&self) -> bool {
        // SAFETY: the stream handle is live.
        unsafe { sys::vrsdk_setpoint_is_running(self.raw.as_ptr()) }
    }

    /// Stop reading. Idempotent, and dropping the stream does it too. The
    /// publisher is untouched.
    pub fn stop(&self) {
        // SAFETY: the stream handle is live.
        let code = unsafe { sys::vrsdk_setpoint_stop(self.raw.as_ptr()) };
        expect_ok(code, "vrsdk_setpoint_stop");
    }

    /// The C handle, for calling a function of [`crate::sys`] directly. Still
    /// owned by this value.
    #[must_use]
    pub fn as_raw(&self) -> *const sys::vrsdk_setpoint_stream_t {
        self.raw.as_ptr()
    }

    fn read(
        &self,
        reader: unsafe extern "C" fn(
            *const sys::vrsdk_setpoint_stream_t,
            *mut sys::vrsdk_setpoint_t,
            *mut bool,
        ) -> sys::vrsdk_err_t,
        what: &str,
    ) -> Option<Setpoint> {
        let mut raw = sys::vrsdk_setpoint_t::default();
        let mut valid = false;
        // SAFETY: `reader` is `vrsdk_setpoint_fresh` or `vrsdk_setpoint_latest`,
        // which both take a live stream handle and two writable out-parameters.
        let code = unsafe { reader(self.raw.as_ptr(), &mut raw, &mut valid) };
        expect_ok(code, what);
        valid.then(|| Setpoint::from_raw(&raw))
    }
}

impl Drop for SetpointStream {
    fn drop(&mut self) {
        // SAFETY: the handle came from the library, is owned only by this value
        // and freed exactly once, here.
        unsafe { sys::vrsdk_setpoint_free(self.raw.as_ptr()) };
    }
}

// SAFETY: the C API documents setpoint streams as safe to use from several
// threads ("two threads racing on one stream cannot both receive it"); the
// handle is freed only in `Drop`, with exclusive access.
unsafe impl Send for SetpointStream {}
// SAFETY: every method through `&SetpointStream` passes the handle as `const`.
unsafe impl Sync for SetpointStream {}

/// Reading commands. See the [module documentation](crate::setpoint).
impl VirtualRobot {
    /// Watch this robot's `SET_ANGVEL` setpoint: **somebody else's command**, as
    /// an input to your controller.
    ///
    /// The experiment the fixed wing is built for: put it in
    /// [`cmd::FW_DIRECT_SURFACE`](crate::cmd::FW_DIRECT_SURFACE), take the
    /// operator's stick from here and the gyro from
    /// [`states`](Self::states), and close the rate loop with your gains.
    ///
    /// ```no_run
    /// # use vrobots_sdk::{RobotType, VirtualRobot};
    /// # let robot = VirtualRobot::connect(RobotType::GlobalHawk, Some(15))?;
    /// let own = robot.options().src_id;
    /// let setpoints = robot.subscribe_setpoint()?;
    /// let demand = match setpoints.latest() {
    ///     Some(sp) if sp.src_id != own => sp.value,
    ///     _ => [0.0; 3], // nobody else is flying it
    /// };
    /// # let _ = demand;
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    ///
    /// # Errors
    ///
    /// [`VrError::Session`] if the subscriber cannot be declared,
    /// [`VrError::Deleted`].
    pub fn subscribe_setpoint(&self) -> VrResult<SetpointStream> {
        let mut out: *mut sys::vrsdk_setpoint_stream_t = ptr::null_mut();
        // SAFETY: the robot handle is live and `out` is a writable slot.
        check(unsafe { sys::vrsdk_robot_subscribe_setpoint(self.raw(), &mut out) })?;
        SetpointStream::adopt(out)
    }

    /// [`subscribe_setpoint`](Self::subscribe_setpoint) for any command id, for
    /// watching a peer drive a robot. Only commands carrying a `vec3` yield a
    /// setpoint; an id whose payload lives elsewhere is counted as filtered.
    ///
    /// # Errors
    ///
    /// As [`subscribe_setpoint`](Self::subscribe_setpoint).
    pub fn subscribe_command(&self, cmd_id: u32) -> VrResult<SetpointStream> {
        let mut out: *mut sys::vrsdk_setpoint_stream_t = ptr::null_mut();
        // SAFETY: the robot handle is live and `out` is a writable slot.
        check(unsafe { sys::vrsdk_robot_subscribe_command(self.raw(), cmd_id, &mut out) })?;
        SetpointStream::adopt(out)
    }
}
