//! Safety events: one robot's `z/event/safety`, and every robot's at once.
//!
//! One sample per incident and nothing in between, which sets this stream
//! apart from every other one in the SDK:
//!
//! - **Events queue.** A state snapshot is superseded by the next one; an
//!   incident is superseded by nothing, so a [`SafetyEventStream`] hands out
//!   every event once, oldest first.
//! - **Silence is the normal state.** Nothing arrives until something happens,
//!   so a quiet stream is not a broken one, and
//!   [`next_timeout`](SafetyEventStream::next_timeout) coming back empty is an
//!   answer, not an error.
//!
//! People are detected on the person's side, so every robot type reports them
//! alike; objects and other robots come from an arm's protective stop. The
//! simulator picks the [`SafetyTier`] with thresholds of its own and sends the
//! measured values beside it. On the Community tier only the focused robot is
//! on the wire at all, and another robot's incident is dropped inside the
//! simulator.

use std::ffi::c_char;
use std::fmt;
use std::ptr::{self, NonNull};
use std::time::Duration;

use vrobots_sdk_sys as sys;

use crate::error::{VrError, VrResult};
use crate::ffi::{self, check, expect_ok, fixed_str};
use crate::options::ConnectOptions;
use crate::robot::{VirtualRobot, MESSAGE_CAPACITY};
use crate::state::Axes;

/// How many unread events a [`SafetyEventStream`] holds. When the queue is
/// full the **oldest** unread event is evicted and counted in
/// [`SafetyEventStats::dropped`]; a reader that drains once per loop iteration
/// never comes near it.
pub const SAFETY_EVENT_QUEUE_CAPACITY: usize = sys::VRSDK_SAFETY_EVENT_QUEUE_CAPACITY as usize;

/// How bad an incident was.
///
/// **Deliberately not ordered**: [`Unspecified`](Self::Unspecified) also
/// stands for a tier newer than this SDK, which a `>=` comparison would rank
/// below a near miss.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
pub enum SafetyTier {
    /// No tier, or one this SDK does not know. Read it as "unknown severity",
    /// never as "harmless".
    #[default]
    Unspecified = sys::VRSDK_SAFETY_TIER_UNSPECIFIED,
    /// Passed close while moving toward the other party; never touched.
    NearMiss = sys::VRSDK_SAFETY_TIER_NEAR_MISS,
    /// Touched, below the strike thresholds.
    Contact = sys::VRSDK_SAFETY_TIER_CONTACT,
    /// Hit: a fast closing speed or a large impulse.
    Strike = sys::VRSDK_SAFETY_TIER_STRIKE,
}

impl SafetyTier {
    /// The tier for a wire value; an unknown one is
    /// [`Unspecified`](Self::Unspecified).
    pub(crate) fn from_code(code: i32) -> SafetyTier {
        match code {
            sys::VRSDK_SAFETY_TIER_NEAR_MISS => SafetyTier::NearMiss,
            sys::VRSDK_SAFETY_TIER_CONTACT => SafetyTier::Contact,
            sys::VRSDK_SAFETY_TIER_STRIKE => SafetyTier::Strike,
            _ => SafetyTier::Unspecified,
        }
    }

    /// The wire value: 0 to 3, the discriminant.
    #[must_use]
    pub const fn code(self) -> i32 {
        self as i32
    }

    /// A short, stable name: `"near_miss"`, `"contact"`, `"strike"` or
    /// `"unspecified"`. The library's own `vrsdk_safety_tier_name`.
    #[must_use]
    pub fn name(self) -> &'static str {
        // SAFETY: `vrsdk_safety_tier_name` accepts any value and returns a
        // static, NUL-terminated literal, never NULL.
        unsafe { ffi::static_str(sys::vrsdk_safety_tier_name(self.code())) }
    }
}

/// What the robot met in a safety incident.
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
pub enum SafetyOtherKind {
    /// No kind, or one this SDK does not know.
    #[default]
    Unspecified = sys::VRSDK_SAFETY_OTHER_UNSPECIFIED,
    /// A person, such as a warehouse pedestrian.
    Person = sys::VRSDK_SAFETY_OTHER_PERSON,
    /// A part of another registered robot.
    Robot = sys::VRSDK_SAFETY_OTHER_ROBOT,
    /// Anything else: a box, a pallet, a rack, the floor.
    Object = sys::VRSDK_SAFETY_OTHER_OBJECT,
}

impl SafetyOtherKind {
    /// The kind for a wire value; an unknown one is
    /// [`Unspecified`](Self::Unspecified).
    pub(crate) fn from_code(code: i32) -> SafetyOtherKind {
        match code {
            sys::VRSDK_SAFETY_OTHER_PERSON => SafetyOtherKind::Person,
            sys::VRSDK_SAFETY_OTHER_ROBOT => SafetyOtherKind::Robot,
            sys::VRSDK_SAFETY_OTHER_OBJECT => SafetyOtherKind::Object,
            _ => SafetyOtherKind::Unspecified,
        }
    }

    /// The wire value: 0 to 3, the discriminant.
    #[must_use]
    pub const fn code(self) -> i32 {
        self as i32
    }

    /// A short, stable name: `"person"`, `"robot"`, `"object"` or
    /// `"unspecified"`. The library's own `vrsdk_safety_other_kind_name`.
    #[must_use]
    pub fn name(self) -> &'static str {
        // SAFETY: `vrsdk_safety_other_kind_name` accepts any value and returns
        // a static, NUL-terminated literal, never NULL.
        unsafe { ffi::static_str(sys::vrsdk_safety_other_kind_name(self.code())) }
    }
}

/// One incident, as the robot reported it.
///
/// Owned and plain, like [`State`](crate::State). The fields are the
/// **robot's**: `sys_id` and `robot_name` say whose incident it is, never who
/// was hit.
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct SafetyEvent {
    /// The robot the incident belongs to: on the swarm-wide stream, the way
    /// events from different robots are told apart.
    pub sys_id: u32,
    /// The robot's wire name.
    pub robot_name: String,
    /// The robot's event counter, `0, 1, 2, ...` per robot. A jump is an event
    /// lost on the way, counted in [`SafetyEventStats::seq_gaps`].
    pub seq: u64,
    /// When the simulator reported it, nanoseconds since the unix epoch.
    pub t_ns: i64,
    /// Seconds since the robot's first state sample, the clock of
    /// [`State::elapsed`](crate::State::elapsed); 0 on the swarm-wide stream,
    /// which has no robot epoch.
    pub elapsed: f64,
    /// How bad it was.
    pub tier: SafetyTier,
    /// The other party's scene name, e.g. `"Pedestrian_03"`; for
    /// [`SafetyOtherKind::Robot`], the other robot's wire name.
    pub other_name: String,
    /// What the robot met.
    pub other_kind: SafetyOtherKind,
    /// The robot's peak speed toward the other party, m/s, `+` closing. A
    /// person walking into a parked robot reads about 0.
    pub closing_speed: f64,
    /// Contact impulse, N.s; 0 for a near miss, and whenever the robot part
    /// involved is an arm link, which is posed rather than pushed.
    pub impulse: f64,
    /// The robot was the one moving in.
    pub robot_at_fault: bool,
    /// Where it happened: a **world** position in `coord_frame_id`, the
    /// convention of [`Kinematics::lin_pos`](crate::Kinematics::lin_pos).
    pub position: [f64; 3],
    /// The convention `position` is expressed in.
    pub axis_convention: Axes,
    /// The frame id `position` is expressed in: the robot's own frame,
    /// authoritative, and on the swarm-wide stream possibly different from one
    /// robot to the next.
    pub coord_frame_id: String,
}

impl SafetyEvent {
    pub(crate) fn from_raw(raw: &sys::vrsdk_safety_event_t) -> SafetyEvent {
        SafetyEvent {
            sys_id: raw.sys_id,
            robot_name: fixed_str(&raw.robot_name),
            seq: raw.seq,
            t_ns: raw.t_ns,
            elapsed: raw.elapsed,
            tier: SafetyTier::from_code(raw.tier),
            other_name: fixed_str(&raw.other_name),
            other_kind: SafetyOtherKind::from_code(raw.other_kind),
            closing_speed: raw.closing_speed,
            impulse: raw.impulse,
            robot_at_fault: raw.robot_at_fault,
            position: raw.position,
            axis_convention: Axes(raw.axis_convention),
            coord_frame_id: fixed_str(&raw.coord_frame_id),
        }
    }
}

/// Counters for a [`SafetyEventStream`].
///
/// On an aperiodic topic a lost event looks exactly like no event, so these
/// are the only way to know the record is complete: `dropped` counts what this
/// process discarded, `missed_events` what never reached it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub struct SafetyEventStats {
    /// Events that arrived and decoded, read or not.
    pub received: u64,
    /// Events evicted unread from a full queue
    /// ([`SAFETY_EVENT_QUEUE_CAPACITY`]).
    pub dropped: u64,
    /// Payloads that did not decode. Counted, never fatal.
    pub decode_errors: u64,
    /// Times a robot's `seq` jumped forward by more than one: events lost
    /// between the simulator and this process. Tracked per robot, so robots
    /// interleaving on the swarm-wide stream are not gaps.
    pub seq_gaps: u64,
    /// Total events those jumps imply missing.
    pub missed_events: u64,
}

/// Safety events from one robot or from every robot, queued until read.
///
/// From [`subscribe_safety_events`](VirtualRobot::subscribe_safety_events) or
/// [`subscribe_all_safety_events`]. Each event is handed out **once**, oldest
/// first, by whichever of [`try_next`](Self::try_next),
/// [`next_timeout`](Self::next_timeout) and [`drain`](Self::drain) reaches it
/// first: read from one thread, or accept that two readers split the events.
/// Only incidents after the subscriber is live arrive; zenoh replays nothing.
///
/// Dropping it undeclares the subscriber, and closes the session if the stream
/// opened its own (the swarm-wide one). The per-robot stream rides its robot's
/// session and goes quiet once that [`VirtualRobot`] is dropped. `Send` and
/// `Sync`.
pub struct SafetyEventStream {
    raw: NonNull<sys::vrsdk_safety_stream_t>,
    key: String,
}

impl fmt::Debug for SafetyEventStream {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SafetyEventStream")
            .field("key", &self.key)
            .finish_non_exhaustive()
    }
}

impl SafetyEventStream {
    /// Take ownership of a stream handle a successful subscribe returned, and
    /// read its key once.
    fn adopt(out: *mut sys::vrsdk_safety_stream_t) -> VrResult<SafetyEventStream> {
        let raw = NonNull::new(out).ok_or_else(|| {
            VrError::InvalidHandle(
                "the library reported a safety-event stream but returned no handle; this is a \
                 bug in the VRobots SDK"
                    .to_string(),
            )
        })?;
        let mut stream = SafetyEventStream {
            raw,
            key: String::new(),
        };
        let mut key: [c_char; 512] = [0; 512];
        // SAFETY: the stream handle is live and `key` is writable for its full
        // length, the capacity passed.
        check(unsafe { sys::vrsdk_safety_key(raw.as_ptr(), key.as_mut_ptr(), key.len()) })?;
        stream.key = fixed_str(&key);
        Ok(stream)
    }

    /// The oldest unread event, or `None` when there is none. Never blocks.
    #[must_use]
    pub fn try_next(&self) -> Option<SafetyEvent> {
        let mut raw = sys::vrsdk_safety_event_t::default();
        let mut valid = false;
        // SAFETY: the stream handle is live and both out-parameters are
        // writable.
        let code = unsafe { sys::vrsdk_safety_try_next(self.raw.as_ptr(), &mut raw, &mut valid) };
        expect_ok(code, "vrsdk_safety_try_next");
        valid.then(|| SafetyEvent::from_raw(&raw))
    }

    /// The oldest unread event, waiting up to `timeout` for one to arrive.
    ///
    /// `None` means nothing happened in that time, which on this topic is the
    /// normal answer rather than a failure. It also comes back at once when the
    /// stream is stopped and empty. A zero `timeout` does not wait: it is
    /// [`try_next`](Self::try_next).
    #[must_use]
    pub fn next_timeout(&self, timeout: Duration) -> Option<SafetyEvent> {
        if timeout.is_zero() {
            return self.try_next();
        }
        let mut raw = sys::vrsdk_safety_event_t::default();
        let mut valid = false;
        // SAFETY: the stream handle is live, the timeout is positive and
        // finite, and both out-parameters are writable.
        let code = unsafe {
            sys::vrsdk_safety_next_timeout(
                self.raw.as_ptr(),
                ffi::seconds(timeout),
                &mut raw,
                &mut valid,
            )
        };
        expect_ok(code, "vrsdk_safety_next_timeout");
        valid.then(|| SafetyEvent::from_raw(&raw))
    }

    /// Every event unread when the call begins, oldest first. Never blocks.
    ///
    /// The read for a control loop: once per iteration, then act on all of it.
    /// Events that arrive during the call stay queued for the next one.
    ///
    /// ```no_run
    /// # use vrobots_sdk::{RobotType, SafetyTier, VirtualRobot};
    /// // The warehouse forklift. On the Community tier it must be the focused robot.
    /// let robot = VirtualRobot::connect(RobotType::Forklift, Some(0))?;
    /// let events = robot.subscribe_safety_events()?;
    /// loop {
    ///     for event in events.drain() {
    ///         println!(
    ///             "{} with {} '{}' at {:.2} m/s",
    ///             event.tier.name(),
    ///             event.other_kind.name(),
    ///             event.other_name,
    ///             event.closing_speed
    ///         );
    ///         if event.tier == SafetyTier::Strike {
    ///             robot.set_forklift_with_brake(1500.0, 1500.0, 1100.0, 1301.0, 1900.0)?;
    ///         }
    ///     }
    ///     robot.rate(50.0);
    /// }
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    #[must_use]
    pub fn drain(&self) -> Vec<SafetyEvent> {
        let pending = self.pending();
        let mut events = Vec::with_capacity(pending);
        while events.len() < pending {
            match self.try_next() {
                Some(event) => events.push(event),
                // Another reader took the rest.
                None => break,
            }
        }
        events
    }

    /// How many events are waiting to be read.
    #[must_use]
    pub fn pending(&self) -> usize {
        let mut count = 0usize;
        // SAFETY: the stream handle is live and `count` is writable.
        let code = unsafe { sys::vrsdk_safety_pending(self.raw.as_ptr(), &mut count) };
        expect_ok(code, "vrsdk_safety_pending");
        count
    }

    /// The key expression this stream reads: one robot's
    /// `vrobots/<id>/z/event/safety`, or the swarm-wide
    /// [`ALL_SAFETY_EVENTS`](crate::topics::ALL_SAFETY_EVENTS).
    #[must_use]
    pub fn key(&self) -> &str {
        &self.key
    }

    /// Subscriber counters: received, dropped unread, decode failures and
    /// sequence gaps.
    #[must_use]
    pub fn stats(&self) -> SafetyEventStats {
        let mut raw = sys::vrsdk_safety_stats_t::default();
        // SAFETY: the stream handle is live and `raw` is writable.
        let code = unsafe { sys::vrsdk_safety_stats(self.raw.as_ptr(), &mut raw) };
        expect_ok(code, "vrsdk_safety_stats");
        SafetyEventStats {
            received: raw.received,
            dropped: raw.dropped,
            decode_errors: raw.decode_errors,
            seq_gaps: raw.seq_gaps,
            missed_events: raw.missed_events,
        }
    }

    /// The most recent decode failure on this stream, if any. Counted and
    /// stored rather than returned, as on every other stream.
    #[must_use]
    pub fn last_error(&self) -> Option<VrError> {
        let mut code = sys::VRSDK_OK;
        let mut message: [c_char; MESSAGE_CAPACITY] = [0; MESSAGE_CAPACITY];
        // SAFETY: the stream handle is live; `code` is writable and `message`
        // is writable for its full length, the capacity passed.
        let status = unsafe {
            sys::vrsdk_safety_last_error(
                self.raw.as_ptr(),
                &mut code,
                message.as_mut_ptr(),
                message.len(),
            )
        };
        expect_ok(status, "vrsdk_safety_last_error");
        VrError::from_code(code, fixed_str(&message))
    }

    /// Whether the subscription is still declared.
    #[must_use]
    pub fn is_running(&self) -> bool {
        // SAFETY: the stream handle is live.
        unsafe { sys::vrsdk_safety_is_running(self.raw.as_ptr()) }
    }

    /// Stop listening. Idempotent, and dropping the stream does it too. Events
    /// already queued stay readable, and a reader blocked in
    /// [`next_timeout`](Self::next_timeout) wakes at once.
    pub fn stop(&self) {
        // SAFETY: the stream handle is live.
        let code = unsafe { sys::vrsdk_safety_stop(self.raw.as_ptr()) };
        expect_ok(code, "vrsdk_safety_stop");
    }

    /// The C handle, for calling a function of [`crate::sys`] directly. Still
    /// owned by this value.
    #[must_use]
    pub fn as_raw(&self) -> *const sys::vrsdk_safety_stream_t {
        self.raw.as_ptr()
    }
}

impl Drop for SafetyEventStream {
    fn drop(&mut self) {
        // SAFETY: the handle came from the library, is owned only by this value
        // and freed exactly once, here.
        unsafe { sys::vrsdk_safety_free(self.raw.as_ptr()) };
    }
}

// SAFETY: the C API documents safety streams as safe to use from several
// threads (each event is handed out once, to whichever reader reaches it
// first); the handle is freed only in `Drop`, with exclusive access.
unsafe impl Send for SafetyEventStream {}
// SAFETY: every method through `&SafetyEventStream` passes the handle as
// `const`.
unsafe impl Sync for SafetyEventStream {}

/// Safety events. See the [module documentation](crate::safety).
impl VirtualRobot {
    /// Subscribe to this robot's safety events: every near miss, contact and
    /// strike it is involved in, queued until read.
    ///
    /// Nothing arrives until an incident happens, so an empty stream says
    /// nothing about the subscription; [`SafetyEventStream::stats`] says
    /// whether anything was lost. The stream rides this robot's session and
    /// goes quiet once this handle is dropped; [`subscribe_all_safety_events`]
    /// is the swarm-wide form.
    ///
    /// # Errors
    ///
    /// [`VrError::Session`] if the subscriber cannot be declared,
    /// [`VrError::Deleted`].
    pub fn subscribe_safety_events(&self) -> VrResult<SafetyEventStream> {
        let mut out: *mut sys::vrsdk_safety_stream_t = ptr::null_mut();
        // SAFETY: the robot handle is live and `out` is a writable slot.
        check(unsafe { sys::vrsdk_robot_subscribe_safety_events(self.raw(), &mut out) })?;
        SafetyEventStream::adopt(out)
    }
}

/// Subscribe to **every** robot's safety events, with default connection
/// options.
///
/// Needs no robot: the stream opens a zenoh session of its own on
/// [`ALL_SAFETY_EVENTS`](crate::topics::ALL_SAFETY_EVENTS) and closes it when
/// dropped, and every event carries its robot's
/// [`sys_id`](SafetyEvent::sys_id). A fresh session needs a moment of zenoh
/// discovery, and an incident inside that window is not seen. On the Community
/// tier only the focused robot's incidents arrive.
///
/// ```no_run
/// use std::time::Duration;
///
/// let events = vrobots_sdk::subscribe_all_safety_events()?;
/// loop {
///     // `None` is a quiet minute, not an error.
///     let Some(event) = events.next_timeout(Duration::from_secs(60)) else {
///         continue;
///     };
///     println!("robot {} ({}): {}", event.sys_id, event.robot_name, event.tier.name());
/// }
/// # Ok::<(), vrobots_sdk::VrError>(())
/// ```
///
/// # Errors
///
/// [`VrError::Session`] if zenoh will not open, [`VrError::Timeout`] if it does
/// not open within [`ConnectOptions::connect_timeout`].
pub fn subscribe_all_safety_events() -> VrResult<SafetyEventStream> {
    subscribe_all(None)
}

/// [`subscribe_all_safety_events`] with explicit options, for a router
/// endpoint on a cross-machine setup. Only the connection settings are used.
///
/// # Errors
///
/// As [`subscribe_all_safety_events`], plus [`VrError::Config`] for an unusable
/// router endpoint and [`VrError::InvalidArgument`] for a string containing a
/// NUL byte.
pub fn subscribe_all_safety_events_with(options: &ConnectOptions) -> VrResult<SafetyEventStream> {
    subscribe_all(Some(options))
}

fn subscribe_all(options: Option<&ConnectOptions>) -> VrResult<SafetyEventStream> {
    let options = options.map(ConnectOptions::to_c).transpose()?;
    let options_ptr = options.as_ref().map_or(ptr::null(), |o| o.as_ptr());
    let mut out: *mut sys::vrsdk_safety_stream_t = ptr::null_mut();
    // SAFETY: `options_ptr` is NULL or an initialised struct whose strings live
    // in `options` for the call, and `out` is a writable slot.
    check(unsafe { sys::vrsdk_subscribe_all_safety_events(options_ptr, &mut out) })?;
    SafetyEventStream::adopt(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tiers_and_kinds_round_trip_their_wire_codes() {
        for tier in [
            SafetyTier::Unspecified,
            SafetyTier::NearMiss,
            SafetyTier::Contact,
            SafetyTier::Strike,
        ] {
            assert_eq!(SafetyTier::from_code(tier.code()), tier);
        }
        assert_eq!(SafetyTier::from_code(9), SafetyTier::Unspecified);
        for kind in [
            SafetyOtherKind::Unspecified,
            SafetyOtherKind::Person,
            SafetyOtherKind::Robot,
            SafetyOtherKind::Object,
        ] {
            assert_eq!(SafetyOtherKind::from_code(kind.code()), kind);
        }
        assert_eq!(SafetyOtherKind::from_code(-4), SafetyOtherKind::Unspecified);
    }

    #[test]
    fn an_event_converts_field_for_field() {
        let mut raw = sys::vrsdk_safety_event_t {
            sys_id: 1,
            seq: 4,
            t_ns: 99,
            tier: sys::VRSDK_SAFETY_TIER_STRIKE,
            other_kind: sys::VRSDK_SAFETY_OTHER_PERSON,
            closing_speed: 1.5,
            robot_at_fault: true,
            position: [1.0, 2.0, 3.0],
            ..Default::default()
        };
        ffi::write_fixed_str(&mut raw.robot_name, "palletrobot");
        ffi::write_fixed_str(&mut raw.other_name, "Pedestrian_03");
        ffi::write_fixed_str(&mut raw.coord_frame_id, "fru");
        let e = SafetyEvent::from_raw(&raw);
        assert_eq!((e.sys_id, e.seq, e.t_ns), (1, 4, 99));
        assert_eq!(e.tier, SafetyTier::Strike);
        assert_eq!(e.other_kind, SafetyOtherKind::Person);
        assert_eq!(e.robot_name, "palletrobot");
        assert_eq!(e.other_name, "Pedestrian_03");
        assert_eq!(e.coord_frame_id, "fru");
        assert!(e.robot_at_fault);
    }
}
