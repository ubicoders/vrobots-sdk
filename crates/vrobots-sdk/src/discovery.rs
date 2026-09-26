//! Discovery: what is on the wire right now, and how well it is running.
//!
//! [`list_topics`] answers "is the simulator publishing at all, and under which
//! ids?" without any other tool, and [`measure_rate`] watches one topic and
//! reports its rate, jitter, drops and latency. The two transports differ:
//! zenoh has no registry, so listing it means **listening** for a window and a
//! quiet topic does not appear; iceoryx2 camera streams come from a registry,
//! unmeasured and same host only.

use std::ptr::{self, NonNull};
use std::time::Duration;

use vrobots_sdk_sys as sys;

use crate::error::{VrError, VrResult};
use crate::ffi::{self, check, fixed_str};
use crate::options::ConnectOptions;

/// Which wire a topic lives on: the segment after the id in its name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum Transport {
    /// zenoh: FlatBuffers, works across a network.
    Zenoh,
    /// iceoryx2: raw bytes in shared memory, **same host only**.
    Iceoryx2,
}

impl Transport {
    /// The one-letter form used in topic names: `"z"` or `"i"`.
    #[must_use]
    pub fn tag(self) -> &'static str {
        match self {
            Transport::Zenoh => "z",
            Transport::Iceoryx2 => "i",
        }
    }

    fn from_code(code: sys::vrsdk_transport_t) -> Transport {
        if code == sys::VRSDK_TRANSPORT_ICEORYX2 {
            Transport::Iceoryx2
        } else {
            Transport::Zenoh
        }
    }
}

/// One topic found by [`list_topics`].
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct TopicInfo {
    /// The full key, e.g. `vrobots/1/z/state`. For iceoryx2 also the service
    /// name.
    pub key: String,
    /// Which wire it lives on.
    pub transport: Transport,
    /// The owning robot, or `None` for the `manager` and `scene` keys.
    pub sys_id: Option<u32>,
    /// True when `samples`, `bytes` and `hz` were **measured** by watching
    /// traffic (zenoh); false when the entry came from a registry (iceoryx2).
    pub observed: bool,
    /// Whether anything holds the topic open. False marks a stale iceoryx2
    /// record whose owning process is gone.
    pub live: bool,
    /// Samples seen during the window; zero when not observed.
    pub samples: u64,
    /// Payload bytes seen during the window; zero when not observed.
    pub bytes: u64,
    /// Observed rate, Hz; zero when not observed.
    pub hz: f64,
}

/// List the vrobots topics that are live right now.
///
/// Use a window of about a second or more: much shorter mostly measures zenoh's
/// discovery latency. An empty list is a legitimate answer, not an error.
///
/// ```no_run
/// use std::time::Duration;
///
/// for t in vrobots_sdk::list_topics(Duration::from_millis(1500))? {
///     println!("[{}] {:>7.1} Hz  {}", t.transport.tag(), t.hz, t.key);
/// }
/// # Ok::<(), vrobots_sdk::VrError>(())
/// ```
///
/// # Errors
///
/// [`VrError::Session`] if zenoh will not open; [`VrError::InvalidArgument`]
/// for a zero window.
pub fn list_topics(timeout: Duration) -> VrResult<Vec<TopicInfo>> {
    list(timeout, None)
}

/// [`list_topics`] with explicit options, e.g. a router endpoint. Only the
/// zenoh half honours the endpoint; iceoryx2 is always local.
///
/// # Errors
///
/// As [`list_topics`], plus [`VrError::Config`] for an unusable endpoint.
pub fn list_topics_with(timeout: Duration, options: &ConnectOptions) -> VrResult<Vec<TopicInfo>> {
    list(timeout, Some(options))
}

fn list(timeout: Duration, options: Option<&ConnectOptions>) -> VrResult<Vec<TopicInfo>> {
    let options = options.map(ConnectOptions::to_c).transpose()?;
    let options_ptr = options.as_ref().map_or(ptr::null(), |o| o.as_ptr());
    let mut out: *mut sys::vrsdk_topic_list_t = ptr::null_mut();
    // SAFETY: `options_ptr` is NULL or an initialised struct alive for the call,
    // and `out` is a writable slot.
    check(unsafe { sys::vrsdk_list_topics(ffi::seconds(timeout), options_ptr, &mut out) })?;
    let list = TopicList(NonNull::new(out).ok_or_else(|| {
        VrError::InvalidHandle(
            "vrsdk_list_topics reported success but returned no list; this is a bug in the \
             VRobots SDK"
                .to_string(),
        )
    })?);

    // SAFETY: `list` holds a live list.
    let count = unsafe { sys::vrsdk_topic_list_len(list.0.as_ptr()) };
    let mut topics = Vec::with_capacity(count);
    for index in 0..count {
        let mut raw = sys::vrsdk_topic_info_t::default();
        // SAFETY: `list` holds a live list, `index` is in range and `raw` is
        // writable storage for one `vrsdk_topic_info_t`.
        check(unsafe { sys::vrsdk_topic_list_get(list.0.as_ptr(), index, &mut raw) })?;
        topics.push(TopicInfo {
            key: fixed_str(&raw.key),
            transport: Transport::from_code(raw.transport),
            sys_id: raw.has_sys_id.then_some(raw.sys_id),
            observed: raw.observed,
            live: raw.live,
            samples: raw.samples,
            bytes: raw.bytes,
            hz: raw.hz,
        });
    }
    Ok(topics)
}

/// Owner of one topic list; frees it on drop, including on an early return.
struct TopicList(NonNull<sys::vrsdk_topic_list_t>);

impl Drop for TopicList {
    fn drop(&mut self) {
        // SAFETY: the list came from `vrsdk_list_topics` and is freed exactly
        // once, here.
        unsafe { sys::vrsdk_topic_list_free(self.0.as_ptr()) };
    }
}

/// Whether this build can enumerate both transports. False means camera
/// streams will not appear in [`list_topics`] even when they are running;
/// worth printing beside an empty result.
#[must_use]
pub fn discovery_covers_all_transports() -> bool {
    // SAFETY: takes no arguments and cannot fail.
    unsafe { sys::vrsdk_discovery_covers_all_transports() }
}

/// What watching one topic for a window found, from [`measure_rate`].
///
/// "Absent" is spelled as a `have_*` flag beside a zeroed value, so every field
/// is safe to read; the flag says whether it means anything.
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct RateReport {
    /// The key that was watched.
    pub key: String,
    /// Which wire it was watched on.
    pub transport: Transport,
    /// How long the subscriber stayed open, seconds.
    pub window_s: f64,
    /// Payloads received.
    pub samples: u64,
    /// First arrival to last arrival, seconds. `0.0` for fewer than 2 samples.
    pub span_s: f64,
    /// Observed rate over `span_s`, Hz, not `samples / window_s`. `0.0` for
    /// fewer than 2 samples.
    pub hz: f64,
    /// Mean gap between arrivals, ms.
    pub mean_interval_ms: f64,
    /// Shortest gap between arrivals, ms.
    pub min_interval_ms: f64,
    /// Longest gap between arrivals, ms: the worst stall a control loop saw.
    pub max_interval_ms: f64,
    /// Population standard deviation of the gaps, ms.
    pub jitter_ms: f64,
    /// Total payload bytes received.
    pub bytes: u64,
    /// Whether any payload carried a sequence number; false makes the four
    /// fields below meaningless.
    pub have_seq: bool,
    /// First sequence number seen.
    pub first_seq: u64,
    /// Last sequence number seen.
    pub last_seq: u64,
    /// How many times the sequence jumped forward by more than one.
    pub seq_gaps: u64,
    /// Total samples missed across all gaps.
    pub missed: u64,
    /// How many times the sequence went backwards or repeated: a restarted
    /// camera stream, or two publishers on one zenoh topic.
    pub seq_restarts: u64,
    /// Whether any payload carried a timestamp.
    pub have_latency: bool,
    /// Mean publish-to-arrival delay, ms. Meaningful same-host only.
    pub mean_latency_ms: f64,
    /// Smallest publish-to-arrival delay, ms. Negative means the clocks
    /// disagree.
    pub min_latency_ms: f64,
    /// Largest publish-to-arrival delay, ms.
    pub max_latency_ms: f64,
    /// Payloads that did not parse. Counted, never fatal.
    pub undecodable: u64,
    mean_bytes: f64,
    bytes_per_second: f64,
}

impl RateReport {
    /// Mean payload size, bytes. `0.0` when nothing arrived.
    #[must_use]
    pub fn mean_bytes(&self) -> f64 {
        self.mean_bytes
    }

    /// Average throughput over the window, bytes per second.
    #[must_use]
    pub fn bytes_per_second(&self) -> f64 {
        self.bytes_per_second
    }

    fn from_raw(raw: &sys::vrsdk_rate_report_t) -> RateReport {
        RateReport {
            key: fixed_str(&raw.key),
            transport: Transport::from_code(raw.transport),
            window_s: raw.window_s,
            samples: raw.samples,
            span_s: raw.span_s,
            hz: raw.hz,
            mean_interval_ms: raw.mean_interval_ms,
            min_interval_ms: raw.min_interval_ms,
            max_interval_ms: raw.max_interval_ms,
            jitter_ms: raw.jitter_ms,
            bytes: raw.bytes,
            have_seq: raw.have_seq,
            first_seq: raw.first_seq,
            last_seq: raw.last_seq,
            seq_gaps: raw.seq_gaps,
            missed: raw.missed,
            seq_restarts: raw.seq_restarts,
            have_latency: raw.have_latency,
            mean_latency_ms: raw.mean_latency_ms,
            min_latency_ms: raw.min_latency_ms,
            max_latency_ms: raw.max_latency_ms,
            undecodable: raw.undecodable,
            mean_bytes: raw.mean_bytes,
            bytes_per_second: raw.bytes_per_second,
        }
    }
}

/// Watch one topic for `window` and report its rate, jitter, drops and latency.
///
/// `key` is one full topic name, e.g. `vrobots/1/z/state` or
/// `vrobots/1/i/cam/front_left/720p_rgba8`; the transport segment picks the
/// wire. A window that sees nothing is an answer (`samples == 0`), not a
/// failure. The question it answers for a control loop is `max_interval_ms`.
///
/// # Errors
///
/// [`VrError::InvalidArgument`] for an empty or wildcard key or a zero window;
/// [`VrError::Session`] if zenoh will not open; [`VrError::Timeout`] if an
/// iceoryx2 stream has no publisher at all.
pub fn measure_rate(key: &str, window: Duration) -> VrResult<RateReport> {
    measure(key, window, None)
}

/// [`measure_rate`] with explicit options.
///
/// # Errors
///
/// As [`measure_rate`], plus [`VrError::Config`] for an unusable endpoint.
pub fn measure_rate_with(
    key: &str,
    window: Duration,
    options: &ConnectOptions,
) -> VrResult<RateReport> {
    measure(key, window, Some(options))
}

fn measure(key: &str, window: Duration, options: Option<&ConnectOptions>) -> VrResult<RateReport> {
    let key = ffi::c_string(key, "key")?;
    let options = options.map(ConnectOptions::to_c).transpose()?;
    let options_ptr = options.as_ref().map_or(ptr::null(), |o| o.as_ptr());
    let mut raw = sys::vrsdk_rate_report_t::default();
    // SAFETY: `key` is NUL-terminated and alive for the call, `options_ptr` is
    // NULL or an initialised struct alive for the call, and `raw` is writable
    // storage for one `vrsdk_rate_report_t`.
    check(unsafe {
        sys::vrsdk_measure_rate(key.as_ptr(), ffi::seconds(window), options_ptr, &mut raw)
    })?;
    Ok(RateReport::from_raw(&raw))
}
