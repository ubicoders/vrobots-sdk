//! Lidars: open a stream, read scans, turn ranges into points, re-mount.
//!
//! Every scan is one revolution, on one of two wires, and both read through
//! the same [`LidarStream`] into the same [`LidarScan`]:
//!
//! | | 3-D | 2-D |
//! |---|---|---|
//! | open with | [`open_lidar_3d`](VirtualRobot::open_lidar_3d) | [`open_lidar_2d`](VirtualRobot::open_lidar_2d) |
//! | topic | `vrobots/<id>/i/lidar/<name>` | `vrobots/<id>/z/lidar/<name>` |
//! | wire | iceoryx2, **same host only** | zenoh, same host or network |
//! | layout | 16 rings x 900 columns, 20 Hz | 1 ring x 360 columns, 10 Hz |
//! | carried by | truck, multirotor, forklift (`"lidar"`) | pallet robot (`"lidar_front_left"`, `"lidar_rear_right"`) |
//!
//! The wire carries one **range** per beam, metres from the optical centre,
//! and **`+inf` means no return, never 0**. Beam `ring * columns + column`
//! lies along azimuth `azimuth_start + column * azimuth_step` (column 0 is
//! forward, counter-clockwise seen from above) and elevation
//! `elevation_start + ring * elevation_step` (ring 0 is the lowest).
//! [`LidarScan::points`] and [`LidarScan::point`] place ranges on their beams
//! through the C library, in the lidar's frame: `"flu"` (x forward, y left,
//! z up) unless overridden through
//! [`set_frames`](VirtualRobot::set_frames) as device
//! [`device::lidar`](crate::device::lidar), which re-labels points and never
//! ranges. The mount in [`LidarParams`] is in the **robot's** frame.
//!
//! **Lidar is Pro.** On Community the simulator publishes no lidar topic at
//! all, so an open fails with [`VrError::Timeout`] exactly as for a lidar that
//! does not exist, and [`configure_lidars`](VirtualRobot::configure_lidars) is
//! refused with [`VrError::Service`].

use std::ffi::c_char;
use std::fmt;
use std::ptr::{self, NonNull};
use std::time::Duration;

use vrobots_sdk_sys as sys;

use crate::discovery::Transport;
use crate::error::{VrError, VrResult};
use crate::ffi::{self, check, expect_ok, fixed_str};
use crate::robot::{VirtualRobot, MESSAGE_CAPACITY};
use crate::state::Axes;

/// How often a read re-sizes its buffer when a scan outgrows it between the
/// size query and the copy. The count is `rings * columns`, fixed per lidar,
/// so a second attempt already succeeds; a third failure is a library bug.
const READ_ATTEMPTS: usize = 3;

/// What a lidar is, where it sits, and the beam layout that places its
/// ranges.
///
/// Carried by every scan, so a re-mounted lidar cannot desync from its
/// ranges. The wire carries every number as `f32`; they are widened here.
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct LidarParams {
    /// Index of this lidar on the robot, 0-based, in mount order.
    pub lidar_id: u32,
    /// The lidar's name on the robot, also its topic segment.
    pub name: String,
    /// The optical centre's offset from the robot origin, metres, in the
    /// **robot's** frame (`mount_coord_frame_id`), not the scan's.
    pub mount_position: [f64; 3],
    /// The mount rotation, radians, about the robot frame's x, y and z axes,
    /// in that frame's own Euler order.
    pub mount_euler_rad: [f64; 3],
    /// The convention the two mount fields are expressed in.
    pub mount_axis_convention: Axes,
    /// The frame id the two mount fields are expressed in: `"fru"` on the
    /// ground robots and `"frd"` on the multirotor by default. Authoritative
    /// when it and `mount_axis_convention` disagree.
    pub mount_coord_frame_id: String,
    /// Rings: 16 for the 3-D unit, 1 for a planar scanner.
    pub rings: u32,
    /// Beams per ring per revolution.
    pub columns: u32,
    /// Azimuth of column 0, radians; column 0 is forward.
    pub azimuth_start: f64,
    /// Azimuth from one column to the next, radians, counter-clockwise seen
    /// from above.
    pub azimuth_step: f64,
    /// Elevation of ring 0, the lowest, radians.
    pub elevation_start: f64,
    /// Elevation from one ring to the next, radians, positive up.
    pub elevation_step: f64,
    /// Returns nearer than this are not seen, metres.
    pub min_range: f64,
    /// Nothing within this range is a `+inf` range, metres.
    pub max_range: f64,
    /// Revolutions per second, fixed by the device: 20 for 3-D, 10 for 2-D.
    pub scan_rate: f64,
    /// Pose snapshots the revolution was cast from; 1 means no spin
    /// distortion. It never changes the scan rate.
    pub segments: u32,
}

impl LidarParams {
    fn from_raw(raw: &sys::vrsdk_lidar_params_t) -> LidarParams {
        LidarParams {
            lidar_id: raw.lidar_id,
            name: fixed_str(&raw.name),
            mount_position: raw.mount_position,
            mount_euler_rad: raw.mount_euler_rad,
            mount_axis_convention: Axes(raw.mount_axis_convention),
            mount_coord_frame_id: fixed_str(&raw.mount_coord_frame_id),
            rings: raw.rings,
            columns: raw.columns,
            azimuth_start: raw.azimuth_start,
            azimuth_step: raw.azimuth_step,
            elevation_start: raw.elevation_start,
            elevation_step: raw.elevation_step,
            min_range: raw.min_range,
            max_range: raw.max_range,
            scan_rate: raw.scan_rate,
            segments: raw.segments,
        }
    }

    fn to_raw(&self) -> sys::vrsdk_lidar_params_t {
        let mut raw = sys::vrsdk_lidar_params_t {
            lidar_id: self.lidar_id,
            mount_position: self.mount_position,
            mount_euler_rad: self.mount_euler_rad,
            mount_axis_convention: self.mount_axis_convention.0,
            rings: self.rings,
            columns: self.columns,
            azimuth_start: self.azimuth_start,
            azimuth_step: self.azimuth_step,
            elevation_start: self.elevation_start,
            elevation_step: self.elevation_step,
            min_range: self.min_range,
            max_range: self.max_range,
            scan_rate: self.scan_rate,
            segments: self.segments,
            ..Default::default()
        };
        ffi::write_fixed_str(&mut raw.name, &self.name);
        ffi::write_fixed_str(&mut raw.mount_coord_frame_id, &self.mount_coord_frame_id);
        raw
    }
}

/// One lidar revolution: the stamp, the layout and the ranges, owned.
///
/// A plain value copied out of the C library, so it outlives the stream, the
/// robot and the library's own copy. A 3-D scan is 14,400 ranges (57.6 kB):
/// move it rather than clone it. `Default` is an empty scan with no layout, for
/// building one by hand before calling [`points`](Self::points).
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct LidarScan {
    /// When the revolution completed, nanoseconds since the unix epoch: the
    /// clock of [`State::t_ns`](crate::State::t_ns), so the two subtract
    /// directly.
    pub t_ns: i64,
    /// Seconds since the robot's first **state** sample.
    pub elapsed: f64,
    /// The simulator's per-topic scan counter. It restarts at 0 when the topic
    /// reopens, so going backwards means "restarted", not "corrupt".
    pub seq: u64,
    /// The robot the lidar is on.
    pub sys_id: u32,
    /// The enum tag beside `coord_frame_id`; [`Axes::UNSPECIFIED`] for the
    /// default `"flu"`, which has no tag.
    pub axis_convention: Axes,
    /// The lidar's frame: `"flu"` unless overridden as device
    /// [`device::lidar`](crate::device::lidar). Authoritative when it and
    /// `axis_convention` disagree.
    pub coord_frame_id: String,
    /// The frame [`points`](Self::points) and [`point`](Self::point) express
    /// points in, as the library resolved it when the scan was read:
    /// `coord_frame_id` when the library knows it, otherwise `"flu"`, the frame
    /// the beam layout is defined in.
    pub points_frame_id: String,
    /// The unit, its mount and its beam layout.
    pub params: LidarParams,
    /// One range per beam, metres from the optical centre, in beam order (ring
    /// by ring, lowest first, each ring from column 0). `f32::INFINITY` where
    /// the beam found nothing within `max_range`; never 0.
    pub ranges: Vec<f32>,
}

impl LidarScan {
    /// The range on beam (`ring`, `column`), or `None` outside the layout.
    ///
    /// `+inf` comes back as `Some(f32::INFINITY)`: it is the wire's "no
    /// return", not a missing beam. [`point`](Self::point) is the call that
    /// folds it into `None`.
    #[must_use]
    pub fn range(&self, ring: u32, column: u32) -> Option<f32> {
        if ring >= self.params.rings || column >= self.params.columns {
            return None;
        }
        let index = usize::try_from(ring)
            .ok()?
            .checked_mul(usize::try_from(self.params.columns).ok()?)?
            .checked_add(usize::try_from(column).ok()?)?;
        self.ranges.get(index).copied()
    }

    /// Azimuth of `column`, radians: `azimuth_start + column * azimuth_step`.
    ///
    /// Column 0 is forward and the angle grows counter-clockwise seen from
    /// above. Not wrapped, and not bounds-checked: the formula is the layout.
    #[must_use]
    pub fn azimuth(&self, column: u32) -> f64 {
        self.params.azimuth_start + f64::from(column) * self.params.azimuth_step
    }

    /// Elevation of `ring`, radians: `elevation_start + ring * elevation_step`.
    ///
    /// Ring 0 is the lowest and the angle grows upwards. Not bounds-checked.
    #[must_use]
    pub fn elevation(&self, ring: u32) -> f64 {
        self.params.elevation_start + f64::from(ring) * self.params.elevation_step
    }

    /// Beam (`ring`, `column`) as a point `[x, y, z]`, metres, in
    /// [`points_frame_id`](Self::points_frame_id); `None` for no return or a
    /// beam outside the layout. Computed by the C library, as
    /// [`points`](Self::points) is, for this one beam.
    #[must_use]
    pub fn point(&self, ring: u32, column: u32) -> Option<[f32; 3]> {
        let range = self.range(ring, column)?;
        if !range.is_finite() {
            return None;
        }
        // A one-beam layout whose beam 0 lies along (ring, column): the
        // library then applies its own formula and frame to that range alone.
        let mut info = self.to_raw();
        info.params.rings = 1;
        info.params.columns = 1;
        info.params.azimuth_start = self.azimuth(column);
        info.params.elevation_start = self.elevation(ring);
        info.range_count = 1;
        let mut xyz = [0.0f32; 3];
        // SAFETY: `info` is an initialised `vrsdk_lidar_scan_info_t`, `range`
        // is one readable float, and `xyz` is writable for the 3 floats passed.
        let code =
            unsafe { sys::vrsdk_lidar_points(&info, &range, 1, xyz.as_mut_ptr(), xyz.len()) };
        expect_ok(code, "vrsdk_lidar_points");
        Some(xyz)
    }

    /// Every beam as a point: `[x0, y0, z0, x1, y1, z1, ...]`, metres, in
    /// [`points_frame_id`](Self::points_frame_id).
    ///
    /// Exactly `3 * ranges.len()` floats in beam order, so point `i` is beam
    /// `i` and the cloud stays organised by ring and column. A beam with no
    /// return is `[NaN, NaN, NaN]`; drop those (`p[0].is_nan()`) when an
    /// unorganised cloud is what you want. The points are in the **lidar's**
    /// frame, not the robot's: the mount is in [`params`](Self::params) for a
    /// consumer that wants them on the body. The C library computes them from
    /// this value, so a scan built by hand works as well as one a stream
    /// returned.
    ///
    /// ```no_run
    /// # use std::time::Duration;
    /// # use vrobots_sdk::{RobotType, VirtualRobot};
    /// let robot = VirtualRobot::connect(RobotType::Truck, Some(0))?;
    /// let lidar = robot.open_lidar_3d("lidar")?;
    /// lidar.wait_new_scan(Duration::from_secs(1))?;
    /// if let Some(scan) = lidar.latest() {
    ///     let xyz = scan.points();
    ///     let hits = xyz.chunks_exact(3).filter(|p| !p[0].is_nan()).count();
    ///     println!("{hits} returns in {}", scan.points_frame_id);
    /// }
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    #[must_use]
    pub fn points(&self) -> Vec<f32> {
        let mut xyz = vec![0.0f32; 3 * self.ranges.len()];
        if self.ranges.is_empty() {
            return xyz;
        }
        let info = self.to_raw();
        // SAFETY: `info` is an initialised `vrsdk_lidar_scan_info_t`, `ranges`
        // is readable for `ranges.len()` floats, and `xyz` is writable for the
        // `3 * ranges.len()` floats passed as its capacity.
        let code = unsafe {
            sys::vrsdk_lidar_points(
                &info,
                self.ranges.as_ptr(),
                self.ranges.len(),
                xyz.as_mut_ptr(),
                xyz.len(),
            )
        };
        expect_ok(code, "vrsdk_lidar_points");
        xyz
    }

    /// The mount this scan was cast from, in the **robot's** frame: what
    /// [`LidarConfigEntry::with_lidar_mount`] takes back unchanged.
    #[must_use]
    pub fn mount(&self) -> LidarMount {
        LidarMount::new(self.params.mount_position, self.params.mount_euler_rad)
    }

    fn from_raw(raw: &sys::vrsdk_lidar_scan_info_t, ranges: Vec<f32>) -> LidarScan {
        LidarScan {
            t_ns: raw.t_ns,
            elapsed: raw.elapsed,
            seq: raw.seq,
            sys_id: raw.sys_id,
            axis_convention: Axes(raw.axis_convention),
            coord_frame_id: fixed_str(&raw.coord_frame_id),
            points_frame_id: fixed_str(&raw.points_frame_id),
            params: LidarParams::from_raw(&raw.params),
            ranges,
        }
    }

    fn to_raw(&self) -> sys::vrsdk_lidar_scan_info_t {
        let mut raw = sys::vrsdk_lidar_scan_info_t {
            t_ns: self.t_ns,
            elapsed: self.elapsed,
            seq: self.seq,
            sys_id: self.sys_id,
            axis_convention: self.axis_convention.0,
            range_count: self.ranges.len(),
            params: self.params.to_raw(),
            ..Default::default()
        };
        ffi::write_fixed_str(&mut raw.coord_frame_id, &self.coord_frame_id);
        ffi::write_fixed_str(&mut raw.points_frame_id, &self.points_frame_id);
        raw
    }
}

/// Counters for one lidar stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub struct LidarStats {
    /// Scans received and published.
    pub received: u64,
    /// Payloads that did not decode. Counted, never fatal; see
    /// [`LidarStream::last_error`].
    pub decode_errors: u64,
    /// Times the sequence number jumped forward by more than one.
    pub seq_gaps: u64,
    /// Total scans missed across all gaps.
    pub missed_scans: u64,
    /// The last sequence number seen.
    pub last_seq: u64,
}

/// The shape of `vrsdk_lidar_fresh` and `vrsdk_lidar_latest`.
type Reader = unsafe extern "C" fn(
    *const sys::vrsdk_lidar_t,
    *mut sys::vrsdk_lidar_scan_info_t,
    *mut f32,
    usize,
    *mut bool,
) -> sys::vrsdk_err_t;

/// A live lidar scan stream, 3-D or 2-D, from
/// [`open_lidar_3d`](VirtualRobot::open_lidar_3d) or
/// [`open_lidar_2d`](VirtualRobot::open_lidar_2d).
///
/// Dropping it ends this subscription only; **the lidar keeps scanning**, as
/// dropping a [`VirtualRobot`] leaves the robot running. A 3-D stream owns its
/// subscriber and outlives the robot handle it came from; a 2-D stream rides
/// that robot's zenoh session and goes quiet once the robot handle is dropped.
/// `Send` and `Sync`: the library hands each scan out once, so two threads
/// racing on one stream never both receive the same scan.
pub struct LidarStream {
    raw: NonNull<sys::vrsdk_lidar_t>,
    name: String,
    topic: String,
    transport: Transport,
}

impl fmt::Debug for LidarStream {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LidarStream")
            .field("topic", &self.topic)
            .finish_non_exhaustive()
    }
}

impl LidarStream {
    /// Take ownership of a lidar handle a successful open returned, and read
    /// its identity once.
    fn adopt(out: *mut sys::vrsdk_lidar_t) -> VrResult<LidarStream> {
        let raw = NonNull::new(out).ok_or_else(|| {
            VrError::InvalidHandle(
                "the library reported a lidar stream but returned no handle; this is a bug in the \
                 VRobots SDK"
                    .to_string(),
            )
        })?;
        let mut stream = LidarStream {
            raw,
            name: String::new(),
            topic: String::new(),
            transport: Transport::Zenoh,
        };
        let mut name: [c_char; 256] = [0; 256];
        let mut topic: [c_char; 512] = [0; 512];
        let mut transport = sys::VRSDK_TRANSPORT_ZENOH;
        // SAFETY: the lidar handle is live, each buffer is writable for the
        // capacity passed with it, and `transport` is writable.
        check(unsafe {
            sys::vrsdk_lidar_info(
                raw.as_ptr(),
                name.as_mut_ptr(),
                name.len(),
                topic.as_mut_ptr(),
                topic.len(),
                &mut transport,
            )
        })?;
        stream.name = fixed_str(&name);
        stream.topic = fixed_str(&topic);
        stream.transport = Transport::from_code(transport);
        Ok(stream)
    }

    /// The latest scan **if it is new since the last call**, otherwise `None`.
    ///
    /// The read for a pipeline that does its work once per scan: polling at
    /// 100 Hz against a 20 Hz lidar yields a scan about 20 times a second and
    /// `None` the rest, and a poll with nothing new allocates nothing. Each
    /// scan is handed out once.
    #[must_use]
    pub fn fresh(&self) -> Option<LidarScan> {
        if !self.has_fresh() {
            return None;
        }
        self.read(sys::vrsdk_lidar_fresh, "vrsdk_lidar_fresh")
    }

    /// Whether a scan [`fresh`](Self::fresh) has not handed out yet is
    /// waiting. Claims nothing and copies nothing; `vrsdk_lidar_has_fresh`.
    #[must_use]
    pub fn has_fresh(&self) -> bool {
        // SAFETY: the lidar handle is live.
        unsafe { sys::vrsdk_lidar_has_fresh(self.raw.as_ptr()) }
    }

    /// The latest scan, new or not, or `None` before the first one. Does not
    /// consume freshness.
    #[must_use]
    pub fn latest(&self) -> Option<LidarScan> {
        self.read(sys::vrsdk_lidar_latest, "vrsdk_lidar_latest")
    }

    /// Block until a scan newer than the last stored one arrives: for
    /// scan-paced loops that run once per revolution.
    ///
    /// # Errors
    ///
    /// [`VrError::Timeout`] if none arrives in time, which never means the
    /// stream is broken: it is also how a lidar that stopped publishing (a tier
    /// change, a despawn) is detected. [`VrError::InvalidArgument`] for a zero
    /// timeout.
    pub fn wait_new_scan(&self, timeout: Duration) -> VrResult<()> {
        // SAFETY: the lidar handle is live.
        check(unsafe { sys::vrsdk_lidar_wait_new_scan(self.raw.as_ptr(), ffi::seconds(timeout)) })
    }

    /// The lidar's name on the robot.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The topic this stream reads, exactly as `vrobots topic list` prints it,
    /// e.g. `vrobots/0/i/lidar/lidar` for a 3-D unit.
    #[must_use]
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// Which wire the scans ride: [`Transport::Iceoryx2`] for a 3-D lidar,
    /// [`Transport::Zenoh`] for a 2-D one.
    #[must_use]
    pub fn transport(&self) -> Transport {
        self.transport
    }

    /// Stream counters: scans, decode failures, sequence gaps.
    #[must_use]
    pub fn stats(&self) -> LidarStats {
        let mut raw = sys::vrsdk_lidar_stats_t::default();
        // SAFETY: the lidar handle is live and `raw` is writable.
        let code = unsafe { sys::vrsdk_lidar_stats(self.raw.as_ptr(), &mut raw) };
        expect_ok(code, "vrsdk_lidar_stats");
        LidarStats {
            received: raw.received,
            decode_errors: raw.decode_errors,
            seq_gaps: raw.seq_gaps,
            missed_scans: raw.missed_scans,
            last_seq: raw.last_seq,
        }
    }

    /// The most recent decode failure on this stream, if any. Counted and
    /// dropped, never fatal.
    #[must_use]
    pub fn last_error(&self) -> Option<VrError> {
        let mut code = sys::VRSDK_OK;
        let mut message: [c_char; MESSAGE_CAPACITY] = [0; MESSAGE_CAPACITY];
        // SAFETY: the lidar handle is live; `code` is writable and `message` is
        // writable for its full length, the capacity passed.
        let status = unsafe {
            sys::vrsdk_lidar_last_error(
                self.raw.as_ptr(),
                &mut code,
                message.as_mut_ptr(),
                message.len(),
            )
        };
        expect_ok(status, "vrsdk_lidar_last_error");
        VrError::from_code(code, fixed_str(&message))
    }

    /// Whether the stream is still receiving: false after [`stop`](Self::stop).
    #[must_use]
    pub fn is_running(&self) -> bool {
        // SAFETY: the lidar handle is live.
        unsafe { sys::vrsdk_lidar_is_running(self.raw.as_ptr()) }
    }

    /// Stop reading. Idempotent, and dropping the stream does it too. The lidar
    /// keeps publishing; this only ends this subscription.
    pub fn stop(&self) {
        // SAFETY: the lidar handle is live.
        let code = unsafe { sys::vrsdk_lidar_stop(self.raw.as_ptr()) };
        expect_ok(code, "vrsdk_lidar_stop");
    }

    /// The C handle, for calling a function of [`crate::sys`] directly. Still
    /// owned by this value.
    #[must_use]
    pub fn as_raw(&self) -> *const sys::vrsdk_lidar_t {
        self.raw.as_ptr()
    }

    /// Size a buffer from the newest scan's info, then read into it.
    ///
    /// The library refuses a buffer too small for its scan before claiming
    /// the scan, so a refusal loses nothing; it happens only if the scan grew
    /// between the two calls, and the next attempt sizes again.
    fn read(&self, reader: Reader, what: &str) -> Option<LidarScan> {
        let mut code = sys::VRSDK_OK;
        for _ in 0..READ_ATTEMPTS {
            let mut newest = sys::vrsdk_lidar_scan_info_t::default();
            let mut any = false;
            // SAFETY: the lidar handle is live, `newest` and `any` are
            // writable, and a NULL range buffer asks for the info alone.
            let peek = unsafe {
                sys::vrsdk_lidar_latest(
                    self.raw.as_ptr(),
                    &mut newest,
                    ptr::null_mut(),
                    0,
                    &mut any,
                )
            };
            expect_ok(peek, "vrsdk_lidar_latest");
            if !any {
                return None;
            }
            // Never empty, so never NULL: a NULL buffer is the info-only read,
            // which claims nothing.
            let mut ranges = vec![0.0f32; newest.range_count.max(1)];
            let mut info = sys::vrsdk_lidar_scan_info_t::default();
            let mut valid = false;
            // SAFETY: `reader` is `vrsdk_lidar_fresh` or `vrsdk_lidar_latest`,
            // which take a live handle, a writable info, a range buffer
            // writable for the capacity passed, and a writable flag.
            code = unsafe {
                reader(
                    self.raw.as_ptr(),
                    &mut info,
                    ranges.as_mut_ptr(),
                    ranges.len(),
                    &mut valid,
                )
            };
            if code == sys::VRSDK_ERR_INVALID_ARGUMENT {
                continue;
            }
            expect_ok(code, what);
            if !valid {
                return None;
            }
            ranges.truncate(info.range_count);
            return Some(LidarScan::from_raw(&info, ranges));
        }
        expect_ok(code, what);
        None
    }
}

impl Drop for LidarStream {
    fn drop(&mut self) {
        // SAFETY: the handle came from the library, is owned only by this value
        // and freed exactly once, here. Freeing stops and joins a 3-D reader
        // thread or undeclares a 2-D subscriber; the lidar keeps scanning.
        unsafe { sys::vrsdk_lidar_free(self.raw.as_ptr()) };
    }
}

// SAFETY: the C API documents lidar handles as safe to use from several
// threads ("each scan is handed out once, so two threads racing on one stream
// cannot both receive it"); the handle is freed only in `Drop`, with exclusive
// access.
unsafe impl Send for LidarStream {}
// SAFETY: every method through `&LidarStream` passes the handle as `const`.
unsafe impl Sync for LidarStream {}

/// A lidar's mount pose: the one value
/// [`configure_lidars`](VirtualRobot::configure_lidars) moves a lidar with.
///
/// In the **robot's** frame, metres and radians in that frame's own Euler
/// order: the frame and shape [`LidarParams`] reports a mount in, so a mount
/// read off a scan ([`LidarScan::mount`]) goes back unchanged.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct LidarMount {
    /// The optical centre's offset from the robot origin, m.
    pub position: [f64; 3],
    /// The mount rotation, rad, about the robot frame's x, y and z axes.
    pub euler_rad: [f64; 3],
}

impl LidarMount {
    /// A mount pose, as [`LidarParams::mount_position`] and
    /// [`LidarParams::mount_euler_rad`] report one.
    #[must_use]
    pub fn new(position: [f64; 3], euler_rad: [f64; 3]) -> LidarMount {
        LidarMount {
            position,
            euler_rad,
        }
    }
}

/// One entry of [`configure_lidars`](VirtualRobot::configure_lidars): a
/// mounted lidar, by name, and what to change on it.
///
/// `None` leaves a setting alone. Build one with [`new`](Self::new) and the
/// `with_*` setters.
///
/// ```
/// use vrobots_sdk::LidarConfigEntry;
///
/// let entry = LidarConfigEntry::new("lidar").with_segments(1).with_layer_mask(0);
/// assert!(!entry.is_empty());
/// assert!(LidarConfigEntry::new("lidar").is_empty());
/// ```
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct LidarConfigEntry {
    /// The lidar, by the name it was mounted under: also its topic segment.
    pub name: String,
    /// Where the lidar sits, in the **robot's** frame. Position and rotation
    /// travel together: a pose is one value.
    pub mount: Option<LidarMount>,
    /// Pose snapshots per revolution; 1 means no spin distortion. Clamped by
    /// the simulator to [1, the device's maximum], 5 for a 2-D lidar and 2 for
    /// a 3-D one; it never changes the scan rate.
    pub segments: Option<u32>,
    /// The Unity physics layers the beams can hit, as a bit mask; 0 means the
    /// default raycast layers.
    pub layer_mask: Option<i32>,
}

impl LidarConfigEntry {
    /// An entry for the lidar mounted as `name`, changing nothing yet.
    #[must_use]
    pub fn new(name: impl Into<String>) -> LidarConfigEntry {
        LidarConfigEntry {
            name: name.into(),
            ..LidarConfigEntry::default()
        }
    }

    /// See [`mount`](Self::mount): a position and a rotation in the robot's
    /// frame.
    #[must_use]
    pub fn with_mount(mut self, position: [f64; 3], euler_rad: [f64; 3]) -> Self {
        self.mount = Some(LidarMount::new(position, euler_rad));
        self
    }

    /// See [`mount`](Self::mount): a [`LidarMount`], such as one read off a
    /// scan with [`LidarScan::mount`].
    #[must_use]
    pub fn with_lidar_mount(mut self, mount: LidarMount) -> Self {
        self.mount = Some(mount);
        self
    }

    /// See [`segments`](Self::segments).
    #[must_use]
    pub fn with_segments(mut self, segments: u32) -> Self {
        self.segments = Some(segments);
        self
    }

    /// See [`layer_mask`](Self::layer_mask).
    #[must_use]
    pub fn with_layer_mask(mut self, mask: i32) -> Self {
        self.layer_mask = Some(mask);
        self
    }

    /// Whether this entry would change nothing.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.mount.is_none() && self.segments.is_none() && self.layer_mask.is_none()
    }

    /// The C entry, its name pointing into `name`, which must outlive it.
    fn to_raw(&self, name: &std::ffi::CStr) -> sys::vrsdk_lidar_config_entry_t {
        let mut raw = sys::vrsdk_lidar_config_entry_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_lidar_config_entry_t`.
        unsafe { sys::vrsdk_lidar_config_entry_default(&mut raw) };
        raw.name = name.as_ptr();
        if let Some(mount) = self.mount {
            raw.has_mount = true;
            raw.mount_position = mount.position;
            raw.mount_euler_rad = mount.euler_rad;
        }
        if let Some(segments) = self.segments {
            raw.has_segments = true;
            raw.segments = segments;
        }
        if let Some(mask) = self.layer_mask {
            raw.has_layer_mask = true;
            raw.layer_mask = mask;
        }
        raw
    }
}

/// Lidars. See the [module documentation](crate::lidar).
impl VirtualRobot {
    /// Subscribe to one of the robot's 3-D lidars. **Never mutates the
    /// simulator.**
    ///
    /// The truck, the multirotor and the forklift each carry one, named
    /// `"lidar"`: 16 rings of 900 columns, 0.4 to 100 m, 20 Hz. Scans ride
    /// iceoryx2 on `vrobots/<id>/i/lidar/<name>`, so **the simulator must run
    /// on this host**.
    ///
    /// ```no_run
    /// # use vrobots_sdk::{RobotType, VirtualRobot};
    /// let robot = VirtualRobot::connect(RobotType::Truck, Some(0))?;
    /// let lidar = robot.open_lidar_3d("lidar")?;
    /// loop {
    ///     if let Some(scan) = lidar.fresh() {
    ///         let hits = scan.ranges.iter().filter(|r| r.is_finite()).count();
    ///         println!("scan {}: {hits} of {} beams hit", scan.seq, scan.ranges.len());
    ///     }
    ///     robot.rate(50.0);
    /// }
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`] for an empty name or one with a character
    /// other than a letter, a digit, `_` or `-`; [`VrError::Timeout`] if the
    /// topic is not found within
    /// [`ConnectOptions::camera_timeout`](crate::ConnectOptions::camera_timeout),
    /// which is also exactly what a Community login looks like, since lidar is
    /// Pro; [`VrError::Session`] if the stream exists but will not pair;
    /// [`VrError::Deleted`].
    pub fn open_lidar_3d(&self, name: &str) -> VrResult<LidarStream> {
        let name = ffi::c_string(name, "lidar name")?;
        let mut out: *mut sys::vrsdk_lidar_t = ptr::null_mut();
        // SAFETY: the robot handle is live, `name` is NUL-terminated and alive
        // for the call, and `out` is a writable slot.
        check(unsafe { sys::vrsdk_robot_open_lidar_3d(self.raw(), name.as_ptr(), &mut out) })?;
        LidarStream::adopt(out)
    }

    /// Subscribe to one of the robot's 2-D lidars. **Never mutates the
    /// simulator.**
    ///
    /// The pallet robot carries two, `"lidar_front_left"` and
    /// `"lidar_rear_right"`: one ring of 360 beams at 1 degree, 0.15 to 12 m,
    /// 10 Hz. Each publishes its full ring, so the inward quarter looks through
    /// the robot's own body. Scans ride zenoh on `vrobots/<id>/z/lidar/<name>`
    /// and reach across a network. zenoh has no registry, so this **waits for
    /// the first scan**, up to
    /// [`ConnectOptions::probe_timeout`](crate::ConnectOptions::probe_timeout),
    /// and the stream it returns already holds it.
    ///
    /// ```no_run
    /// # use vrobots_sdk::{RobotType, VirtualRobot};
    /// let robot = VirtualRobot::connect(RobotType::PalletRobot, Some(1))?;
    /// let front = robot.open_lidar_2d("lidar_front_left")?;
    /// let scan = front.latest().expect("open_lidar_2d returns after the first scan");
    /// // One ring: column c is c degrees counter-clockwise from the unit's forward.
    /// println!("dead ahead: {:?} m", scan.range(0, 0));
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    ///
    /// # Errors
    ///
    /// [`VrError::Timeout`] if no scan arrives in time: the topic was not
    /// found, which is also what a Community login looks like;
    /// [`VrError::Decode`] if samples arrived but none decoded; the name errors
    /// of [`open_lidar_3d`](Self::open_lidar_3d); [`VrError::Deleted`].
    pub fn open_lidar_2d(&self, name: &str) -> VrResult<LidarStream> {
        let name = ffi::c_string(name, "lidar name")?;
        let mut out: *mut sys::vrsdk_lidar_t = ptr::null_mut();
        // SAFETY: the robot handle is live, `name` is NUL-terminated and alive
        // for the call, and `out` is a writable slot.
        check(unsafe { sys::vrsdk_robot_open_lidar_2d(self.raw(), name.as_ptr(), &mut out) })?;
        LidarStream::adopt(out)
    }

    /// Re-mount and retune the robot's lidars, by name. `srv/lidars`. **Every
    /// robot that carries one; Pro only.**
    ///
    /// Each entry names one lidar the robot carries and changes only its fields
    /// that are `Some`. Lidars not named are left alone, and nothing here
    /// mounts, renames or unmounts one.
    ///
    /// ```no_run
    /// # use std::time::Duration;
    /// # use vrobots_sdk::{LidarConfigEntry, RobotType, VirtualRobot};
    /// let robot = VirtualRobot::connect(RobotType::Forklift, Some(0))?;
    /// let lidar = robot.open_lidar_3d("lidar")?;
    /// lidar.wait_new_scan(Duration::from_secs(1))?;
    /// let mut mount = lidar.latest().expect("a scan arrived").mount();
    /// mount.position[2] += 0.10; // 10 cm up: z is up in the forklift's FRU
    /// robot.configure_lidars(&[LidarConfigEntry::new("lidar").with_lidar_mount(mount)])?;
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    ///
    /// **The mount is in the robot's frame**, not your header frame: the request
    /// names no frame, and the simulator takes an unnamed sender frame as the
    /// robot's, the frame [`LidarParams`] reports the mount in. The ack is a
    /// receipt: the next scans carry the new mount and segments in their
    /// params, and an entry naming no mounted lidar is skipped by the
    /// simulator and acked.
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`] for no entries, a name that is empty, not a
    /// usable topic segment or given twice, an entry that sets nothing, or a
    /// mount value that is not finite; [`VrError::Service`] on the Community
    /// tier, carrying the simulator's own message; the errors of
    /// [`activate`](Self::activate) otherwise.
    pub fn configure_lidars(&self, entries: &[LidarConfigEntry]) -> VrResult<()> {
        let names = entries
            .iter()
            .map(|entry| ffi::c_string(&entry.name, "lidar name"))
            .collect::<VrResult<Vec<_>>>()?;
        let raw: Vec<sys::vrsdk_lidar_config_entry_t> = entries
            .iter()
            .zip(&names)
            .map(|(entry, name)| entry.to_raw(name))
            .collect();
        // SAFETY: the robot handle is live and `raw` is readable for
        // `raw.len()` entries whose names point into `names`, alive for the
        // call; an empty list passes a dangling but aligned, non-null pointer
        // with a zero count, which the library refuses without reading.
        check(unsafe { sys::vrsdk_robot_configure_lidars(self.raw(), raw.as_ptr(), raw.len()) })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_scan_round_trips_through_the_c_info() {
        let mut scan = LidarScan {
            t_ns: 42,
            seq: 7,
            sys_id: 1,
            coord_frame_id: "flu".to_string(),
            points_frame_id: "flu".to_string(),
            ranges: vec![1.0, 2.0],
            ..LidarScan::default()
        };
        scan.params.name = "lidar_front_left".to_string();
        scan.params.mount_coord_frame_id = "fru".to_string();
        scan.params.rings = 1;
        scan.params.columns = 2;
        scan.params.mount_position = [0.3, -0.2, 0.1];
        let info = scan.to_raw();
        assert_eq!(info.range_count, 2);
        assert_eq!(LidarScan::from_raw(&info, scan.ranges.clone()), scan);
    }

    #[test]
    fn a_config_entry_raises_exactly_its_flags() {
        let name = ffi::c_string("lidar", "lidar name").expect("no NUL");
        let raw = LidarConfigEntry::new("lidar")
            .with_segments(2)
            .to_raw(&name);
        assert!(raw.has_segments && !raw.has_mount && !raw.has_layer_mask);
        assert_eq!(raw.segments, 2);
        assert_eq!(raw.name, name.as_ptr());

        let raw = LidarConfigEntry::new("lidar")
            .with_mount([0.0, 0.0, 1.0], [0.0, 0.1, 0.0])
            .to_raw(&name);
        assert!(raw.has_mount && !raw.has_segments);
        assert_eq!(raw.mount_position, [0.0, 0.0, 1.0]);
    }
}
