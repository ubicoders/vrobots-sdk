//! Cameras: mount or open a stream, then read frames.
//!
//! Frames travel over iceoryx2 shared memory, **same host only**, and a stream
//! is identified by camera name, resolution and pixel format together: the
//! three are the service name, so a mismatch is a stream that never appears.
//! Every vrobot ships `front_left` and `front_right` at `"720p"` and `"rgba8"`,
//! which [`open_camera`](VirtualRobot::open_camera) attaches to without changing
//! the simulator. A [`Frame`] is row-major, top-down and tightly packed, with
//! channels in the renderer's order, **RGB(A), never BGR**. Frames and states
//! are separate streams that the SDK never pairs: compare `t_ns` to relate them.

use std::ffi::c_char;
use std::fmt;
use std::ops::Deref;
use std::ptr::{self, NonNull};
use std::time::Duration;

use vrobots_sdk_sys as sys;

use crate::error::{VrError, VrResult};
use crate::ffi::{self, check, expect_ok, fixed_str};
use crate::robot::{VirtualRobot, MESSAGE_CAPACITY};
use crate::state::Axes;
use crate::topics;

/// A camera stream resolution. One setting **per robot**, shared by every
/// camera it mounts. Widths are the 16:9 partners of the height.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[non_exhaustive]
pub enum Resolution {
    /// 640x360.
    P360,
    /// 1280x720. The default.
    P720,
    /// 1920x1080.
    P1080,
}

impl Resolution {
    /// Parse `"360p"`, `"720p"` or `"1080p"`; bare heights (`"720"`) are
    /// accepted too.
    #[must_use]
    pub fn parse(text: &str) -> Option<Resolution> {
        match text.trim().trim_end_matches(['p', 'P']) {
            "360" => Some(Resolution::P360),
            "720" => Some(Resolution::P720),
            "1080" => Some(Resolution::P1080),
            _ => None,
        }
    }

    /// The stream-name spelling: `"360p"`, `"720p"` or `"1080p"`.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Resolution::P360 => "360p",
            Resolution::P720 => "720p",
            Resolution::P1080 => "1080p",
        }
    }

    /// Pixel height.
    #[must_use]
    pub fn height(self) -> u32 {
        match self {
            Resolution::P360 => 360,
            Resolution::P720 => 720,
            Resolution::P1080 => 1080,
        }
    }

    /// Pixel width: the height's 16:9 partner.
    #[must_use]
    pub fn width(self) -> u32 {
        self.height() * 16 / 9
    }
}

impl fmt::Display for Resolution {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A camera pixel format, per camera. **RGB(A), never BGR**: OpenCV users
/// convert with `COLOR_RGB2BGR` or `COLOR_RGBA2BGR`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum PixelFormat {
    /// 8-bit greyscale, 1 byte per pixel.
    Mono8,
    /// 8-bit R,G,B, 3 bytes per pixel.
    Rgb8,
    /// 8-bit R,G,B,A, 4 bytes per pixel: the renderer's native readback and the
    /// format the default cameras publish.
    Rgba8,
}

impl PixelFormat {
    /// Parse `"mono8"`, `"rgb8"` or `"rgba8"`, case-insensitively.
    #[must_use]
    pub fn parse(text: &str) -> Option<PixelFormat> {
        match text.trim().to_ascii_lowercase().as_str() {
            "mono8" => Some(PixelFormat::Mono8),
            "rgb8" => Some(PixelFormat::Rgb8),
            "rgba8" => Some(PixelFormat::Rgba8),
            _ => None,
        }
    }

    /// The stream-name spelling.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            PixelFormat::Mono8 => "mono8",
            PixelFormat::Rgb8 => "rgb8",
            PixelFormat::Rgba8 => "rgba8",
        }
    }

    /// Bytes per pixel: 1, 3 or 4.
    #[must_use]
    pub fn bytes_per_pixel(self) -> u32 {
        match self {
            PixelFormat::Mono8 => 1,
            PixelFormat::Rgb8 => 3,
            PixelFormat::Rgba8 => 4,
        }
    }

    /// The numeric value the schema uses, `1` to `3` (the C API's
    /// `VRSDK_PIXEL_*`).
    #[must_use]
    pub fn wire_value(self) -> u32 {
        match self {
            PixelFormat::Mono8 => 1,
            PixelFormat::Rgb8 => 2,
            PixelFormat::Rgba8 => 3,
        }
    }

    /// Back from the schema's numeric value.
    #[must_use]
    pub fn from_wire_value(value: i32) -> Option<PixelFormat> {
        match value {
            sys::VRSDK_PIXEL_MONO8 => Some(PixelFormat::Mono8),
            sys::VRSDK_PIXEL_RGB8 => Some(PixelFormat::Rgb8),
            sys::VRSDK_PIXEL_RGBA8 => Some(PixelFormat::Rgba8),
            _ => None,
        }
    }

    fn from_bytes_per_pixel(bytes: u32) -> PixelFormat {
        match bytes {
            1 => PixelFormat::Mono8,
            3 => PixelFormat::Rgb8,
            _ => PixelFormat::Rgba8,
        }
    }
}

impl fmt::Display for PixelFormat {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Which stream: a camera name plus the resolution and format it publishes at.
/// All three are baked into the iceoryx2 service name.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub struct CameraSpec {
    /// The camera's name on the robot, e.g. `"front_left"`.
    pub name: String,
    /// Frame size, robot-wide.
    pub resolution: Resolution,
    /// Pixel format, per camera.
    pub format: PixelFormat,
}

impl CameraSpec {
    /// Validate and build a spec from the three strings
    /// [`open_camera`](VirtualRobot::open_camera) takes, without touching the
    /// simulator: for computing a stream name or a frame size up front.
    ///
    /// ```
    /// use vrobots_sdk::CameraSpec;
    ///
    /// let spec = CameraSpec::parse("front_left", "720p", "rgba8")?;
    /// assert_eq!(spec.service_name(1), "vrobots/1/i/cam/front_left/720p_rgba8");
    /// assert!(CameraSpec::parse("front left", "720p", "rgba8").is_err());
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`] for an empty name or one with a character
    /// other than a letter, a digit, `_` or `-` (the name becomes a segment of
    /// the stream name), an unknown resolution or an unknown format, each naming
    /// the accepted set.
    pub fn parse(name: &str, resolution: &str, format: &str) -> VrResult<CameraSpec> {
        let name = name.trim();
        if name.is_empty() {
            return Err(VrError::InvalidArgument(
                "camera name is empty; the simulator ignores unnamed entries".to_string(),
            ));
        }
        if let Some(bad) = name
            .chars()
            .find(|c| !(c.is_ascii_alphanumeric() || *c == '_' || *c == '-'))
        {
            return Err(VrError::InvalidArgument(format!(
                "camera name {name:?} contains {bad:?}; the name becomes a segment of the \
                 iceoryx2 service name, so keep it to letters, digits, '_' and '-'"
            )));
        }
        let parsed_resolution = Resolution::parse(resolution).ok_or_else(|| {
            VrError::InvalidArgument(format!(
                "resolution {resolution:?} is not one of \"360p\", \"720p\", \"1080p\""
            ))
        })?;
        let parsed_format = PixelFormat::parse(format).ok_or_else(|| {
            VrError::InvalidArgument(format!(
                "pixel format {format:?} is not one of \"mono8\", \"rgb8\", \"rgba8\""
            ))
        })?;
        Ok(CameraSpec {
            name: name.to_string(),
            resolution: parsed_resolution,
            format: parsed_format,
        })
    }

    /// The trailing segment of the stream name: `"720p_rgba8"`.
    #[must_use]
    pub fn stream_segment(&self) -> String {
        format!("{}_{}", self.resolution.as_str(), self.format.as_str())
    }

    /// The full iceoryx2 service name for this stream on `sys_id`.
    #[must_use]
    pub fn service_name(&self, sys_id: u32) -> String {
        topics::camera(sys_id, &self.name, &self.stream_segment())
    }

    /// Bytes of pixel data one frame carries.
    #[must_use]
    pub fn data_size(&self) -> usize {
        let pixels = u64::from(self.resolution.width()) * u64::from(self.resolution.height());
        usize::try_from(pixels * u64::from(self.format.bytes_per_pixel())).unwrap_or(usize::MAX)
    }

    fn from_strings(name: String, resolution: &str, format: &str) -> Option<CameraSpec> {
        Some(CameraSpec {
            name,
            resolution: Resolution::parse(resolution)?,
            format: PixelFormat::parse(format)?,
        })
    }
}

impl fmt::Display for CameraSpec {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} {}", self.name, self.stream_segment())
    }
}

/// Where a mounted camera sits and what lens it has, for
/// [`mount_camera_with`](VirtualRobot::mount_camera_with).
///
/// ```
/// use vrobots_sdk::CameraOptions;
///
/// let options = CameraOptions::default()
///     .with_mount_position([0.10, 0.20, 0.30]) // metres, your header frame
///     .with_mount_euler_deg([0.0, 0.0, 180.0]) // degrees, Unity-local
///     .with_focal_length(400.0)
///     .with_clip(0.2, 500.0);
/// assert_eq!((options.fx, options.fy), (400.0, 400.0));
/// ```
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct CameraOptions {
    /// Mount offset from the robot origin, metres, in **your** header frame.
    /// Default `[0, 0, 0]`.
    pub mount_position: [f64; 3],
    /// Mount rotation, **degrees**, Unity-local, taken as written. Default
    /// `[0, 0, 0]`: looking along the robot's forward axis.
    pub mount_euler_deg: [f64; 3],
    /// Horizontal focal length, pixels at the current resolution. Default 600.
    pub fx: f64,
    /// Vertical focal length, pixels. Default 600.
    pub fy: f64,
    /// Near clip plane, metres. Default 0.5.
    pub near_clip: f64,
    /// Far clip plane, metres. Default 1000.
    pub far_clip: f64,
}

impl Default for CameraOptions {
    /// The library's default mount and lens (`vrsdk_camera_options_default`).
    fn default() -> CameraOptions {
        let mut raw = sys::vrsdk_camera_options_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_camera_options_t`.
        unsafe { sys::vrsdk_camera_options_default(&mut raw) };
        CameraOptions {
            mount_position: raw.mount_position,
            mount_euler_deg: raw.mount_euler_deg,
            fx: raw.fx,
            fy: raw.fy,
            near_clip: raw.near_clip,
            far_clip: raw.far_clip,
        }
    }
}

impl CameraOptions {
    /// See [`mount_position`](Self::mount_position).
    #[must_use]
    pub fn with_mount_position(mut self, position: [f64; 3]) -> Self {
        self.mount_position = position;
        self
    }

    /// See [`mount_euler_deg`](Self::mount_euler_deg).
    #[must_use]
    pub fn with_mount_euler_deg(mut self, euler_deg: [f64; 3]) -> Self {
        self.mount_euler_deg = euler_deg;
        self
    }

    /// Set both focal lengths, pixels. `fx != fy` renders anamorphic.
    #[must_use]
    pub fn with_focal_length(mut self, f: f64) -> Self {
        self.fx = f;
        self.fy = f;
        self
    }

    /// Set both clip planes, metres.
    #[must_use]
    pub fn with_clip(mut self, near: f64, far: f64) -> Self {
        self.near_clip = near;
        self.far_clip = far;
        self
    }

    fn to_raw(&self) -> sys::vrsdk_camera_options_t {
        sys::vrsdk_camera_options_t {
            mount_position: self.mount_position,
            mount_euler_deg: self.mount_euler_deg,
            fx: self.fx,
            fy: self.fy,
            near_clip: self.near_clip,
            far_clip: self.far_clip,
        }
    }
}

/// The lens a frame was rendered through, read back from the live camera.
/// Rendered images are ideal pinhole: there is no distortion.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
pub struct Intrinsics {
    /// Horizontal focal length, pixels.
    pub fx: f64,
    /// Vertical focal length, pixels.
    pub fy: f64,
    /// Principal point x, pixels: the image centre.
    pub cx: f64,
    /// Principal point y, pixels: the image centre.
    pub cy: f64,
    /// Vertical field of view, **radians**.
    pub fov_y: f64,
    /// Near clip plane, metres.
    pub near_clip: f64,
    /// Far clip plane, metres.
    pub far_clip: f64,
}

/// Where the camera was mounted when a frame was taken. Rides with every frame,
/// so a re-mounted camera cannot desync from its images.
#[derive(Debug, Clone, PartialEq, Default)]
#[non_exhaustive]
pub struct MountPose {
    /// Offset from the robot origin, metres, in the **robot's** frame.
    pub position: [f64; 3],
    /// Mount rotation, **radians** (the mount request takes degrees).
    pub euler_rad: [f64; 3],
    /// The convention `position` is expressed in.
    pub axis_convention: Axes,
    /// The frame id `position` is expressed in.
    pub coord_frame_id: String,
}

/// A frame's pixels, borrowed straight from the C library's frame and released
/// when this value is dropped.
///
/// Dereferences to `&[u8]`, so `frame.data.len()`, `frame.data.chunks_exact(4)`
/// and `&frame.data` passed as a `&[u8]` all work. `height * step` bytes,
/// row-major **top-down**, tightly packed, channels **RGB(A)**. Nothing is
/// copied: the library already copied the pixels out of shared memory, so the
/// slice never aliases the simulator. Copy it with `to_vec()` to keep the pixels
/// beyond the frame.
pub struct FramePixels {
    handle: NonNull<sys::vrsdk_frame_t>,
    data: *const u8,
    len: usize,
}

impl FramePixels {
    /// Take ownership of a frame handle from `vrsdk_camera_fresh` or
    /// `vrsdk_camera_latest`.
    fn adopt(handle: NonNull<sys::vrsdk_frame_t>) -> FramePixels {
        // SAFETY: `handle` is a live frame handle; both calls accept one and
        // cannot fail on it.
        let (data, len) = unsafe {
            (
                sys::vrsdk_frame_data(handle.as_ptr()),
                sys::vrsdk_frame_data_len(handle.as_ptr()),
            )
        };
        FramePixels { handle, data, len }
    }

    /// The pixels as a slice.
    #[must_use]
    pub fn as_slice(&self) -> &[u8] {
        if self.data.is_null() || self.len == 0 {
            return &[];
        }
        // SAFETY: `data` came from `vrsdk_frame_data` for this handle, which the
        // header documents as pointing at `vrsdk_frame_data_len` bytes, valid
        // until `vrsdk_frame_free` and never mutated (frames are immutable once
        // handed out). The handle is freed only in `Drop`, which cannot run while
        // this borrow of `self` is alive.
        unsafe { std::slice::from_raw_parts(self.data, self.len) }
    }

    /// Copy the pixels into `dst` with the library's `vrsdk_frame_copy_data`,
    /// returning the byte count. Refuses a buffer that is too small rather than
    /// writing a partial image.
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`] if `dst` is shorter than the frame.
    pub fn copy_to(&self, dst: &mut [u8]) -> VrResult<usize> {
        let mut written = 0usize;
        // SAFETY: the frame handle is live; `dst` is writable for `dst.len()`
        // bytes and `written` is one writable `usize`.
        check(unsafe {
            sys::vrsdk_frame_copy_data(
                self.handle.as_ptr(),
                dst.as_mut_ptr(),
                dst.len(),
                &mut written,
            )
        })?;
        if dst.len() < written {
            return Err(VrError::InvalidArgument(format!(
                "the frame is {written} bytes and the buffer holds {}",
                dst.len()
            )));
        }
        Ok(written)
    }
}

impl Deref for FramePixels {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        self.as_slice()
    }
}

impl AsRef<[u8]> for FramePixels {
    fn as_ref(&self) -> &[u8] {
        self.as_slice()
    }
}

impl fmt::Debug for FramePixels {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("FramePixels")
            .field("len", &self.len)
            .finish_non_exhaustive()
    }
}

impl Drop for FramePixels {
    fn drop(&mut self) {
        // SAFETY: the handle came from the library, is owned only by this value,
        // and is freed exactly once, here; no slice borrowed from it can outlive
        // `self`.
        unsafe { sys::vrsdk_frame_free(self.handle.as_ptr()) };
    }
}

// SAFETY: a frame is immutable once handed out ("a handle stays valid however
// long it is kept"), so its pixels can be read from any thread, and freeing it
// from another thread than the one that received it is a plain deallocation.
unsafe impl Send for FramePixels {}
// SAFETY: every method through `&FramePixels` only reads immutable data.
unsafe impl Sync for FramePixels {}

/// One camera frame: its metadata copied out, its pixels borrowed from the C
/// library in [`data`](Self::data).
///
/// A `Frame` holds the library's frame until it is dropped; keep it as long as
/// you need the pixels, or copy them with `frame.data.to_vec()`.
#[derive(Debug)]
#[non_exhaustive]
pub struct Frame {
    /// Capture time, nanoseconds since the unix epoch, on the same clock as
    /// [`State::t_ns`](crate::State::t_ns): the one correct way to relate an
    /// image to a state.
    pub t_ns: i64,
    /// Seconds since the robot's first **state** sample.
    pub elapsed: f64,
    /// The publisher's per-stream sequence number. A gap is a real drop.
    pub seq: u64,
    /// Image width, pixels.
    pub width: u32,
    /// Image height, pixels.
    pub height: u32,
    /// Pixel format of `data`.
    pub format: PixelFormat,
    /// Bytes per row: `width * bytes_per_pixel`, with no padding.
    pub step: u32,
    /// The pixels, `height * step` bytes, row-major top-down.
    pub data: FramePixels,
    /// The robot the camera is on.
    pub sys_id: u32,
    /// The camera's name on the robot.
    pub camera_name: String,
    /// The camera's numeric id on the robot.
    pub camera_id: u32,
    /// The lens.
    pub intrinsics: Intrinsics,
    /// The mount.
    pub mount: MountPose,
    /// The convention the camera's own frame is expressed in.
    pub axis_convention: Axes,
    /// The camera's resolved coordinate frame id.
    pub coord_frame_id: String,
    /// The schema version the simulator stamped.
    pub schema_version: u32,
}

impl Frame {
    fn adopt(handle: NonNull<sys::vrsdk_frame_t>) -> Frame {
        let data = FramePixels::adopt(handle);
        let mut info = sys::vrsdk_frame_info_t::default();
        // SAFETY: `handle` is live (owned by `data` from here on) and `info` is
        // writable storage for one `vrsdk_frame_info_t`.
        let code = unsafe { sys::vrsdk_frame_info(handle.as_ptr(), &mut info) };
        expect_ok(code, "vrsdk_frame_info");
        let format = PixelFormat::from_wire_value(info.pixel_format)
            .unwrap_or_else(|| PixelFormat::from_bytes_per_pixel(info.bytes_per_pixel));
        Frame {
            t_ns: info.t_ns,
            elapsed: info.elapsed,
            seq: info.seq,
            width: info.width,
            height: info.height,
            format,
            step: info.step,
            data,
            sys_id: info.sys_id,
            camera_name: fixed_str(&info.camera_name),
            camera_id: info.camera_id,
            intrinsics: Intrinsics {
                fx: info.intrinsics.fx,
                fy: info.intrinsics.fy,
                cx: info.intrinsics.cx,
                cy: info.intrinsics.cy,
                fov_y: info.intrinsics.fov_y,
                near_clip: info.intrinsics.near_clip,
                far_clip: info.intrinsics.far_clip,
            },
            mount: MountPose {
                position: info.mount.position,
                euler_rad: info.mount.euler_rad,
                axis_convention: Axes(info.mount.axis_convention),
                coord_frame_id: fixed_str(&info.mount.coord_frame_id),
            },
            axis_convention: Axes(info.axis_convention),
            coord_frame_id: fixed_str(&info.coord_frame_id),
            schema_version: info.schema_version,
        }
    }

    /// Bytes per pixel: 1, 3 or 4.
    #[must_use]
    pub fn bytes_per_pixel(&self) -> u32 {
        self.format.bytes_per_pixel()
    }

    /// One row of pixels, top-down (`0` is the top of the image), or `None`
    /// past the last row. Row 0 of an outdoor camera is sky.
    #[must_use]
    pub fn row(&self, y: u32) -> Option<&[u8]> {
        if y >= self.height {
            return None;
        }
        let step = usize::try_from(self.step).ok()?;
        let start = usize::try_from(y).ok()?.checked_mul(step)?;
        self.data.get(start..start.checked_add(step)?)
    }

    /// The C frame handle, for calling a function of [`crate::sys`] directly.
    /// Still owned by this value.
    #[must_use]
    pub fn as_raw(&self) -> *const sys::vrsdk_frame_t {
        self.data.handle.as_ptr()
    }
}

/// Reader-thread counters for one camera stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub struct CameraStats {
    /// Frames received and published.
    pub received: u64,
    /// Slices that did not decode. Counted, never fatal.
    pub decode_errors: u64,
    /// Times the sequence number jumped forward by more than one.
    pub seq_gaps: u64,
    /// Total frames missed across all gaps.
    pub missed_frames: u64,
    /// The last sequence number seen.
    pub last_seq: u64,
}

/// A live camera frame stream, from [`mount_camera`](VirtualRobot::mount_camera)
/// or [`open_camera`](VirtualRobot::open_camera).
///
/// Dropping it stops this subscription's reader thread. **It does not unmount
/// the camera**: the robot keeps rendering and publishing, as dropping a
/// [`VirtualRobot`] leaves the robot running. It outlives the robot handle it
/// came from. `Send` and `Sync`, like every handle in this crate: the library
/// hands each frame out once, so two threads racing on one stream never both
/// receive the same frame.
pub struct CameraStream {
    raw: NonNull<sys::vrsdk_camera_t>,
    spec: CameraSpec,
    service_name: String,
}

impl fmt::Debug for CameraStream {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CameraStream")
            .field("service_name", &self.service_name)
            .finish_non_exhaustive()
    }
}

impl CameraStream {
    /// Take ownership of a camera handle and read its identity once.
    fn adopt(raw: NonNull<sys::vrsdk_camera_t>) -> VrResult<CameraStream> {
        let mut stream = CameraStream {
            raw,
            spec: CameraSpec {
                name: String::new(),
                resolution: Resolution::P720,
                format: PixelFormat::Rgba8,
            },
            service_name: String::new(),
        };
        let mut name: [c_char; 256] = [0; 256];
        let mut resolution: [c_char; 32] = [0; 32];
        let mut format: [c_char; 32] = [0; 32];
        let mut service: [c_char; 512] = [0; 512];
        // SAFETY: the camera handle is live and each buffer is writable for the
        // capacity passed with it.
        check(unsafe {
            sys::vrsdk_camera_info(
                raw.as_ptr(),
                name.as_mut_ptr(),
                name.len(),
                resolution.as_mut_ptr(),
                resolution.len(),
                format.as_mut_ptr(),
                format.len(),
                service.as_mut_ptr(),
                service.len(),
            )
        })?;
        let (resolution, format) = (fixed_str(&resolution), fixed_str(&format));
        stream.spec =
            CameraSpec::from_strings(fixed_str(&name), &resolution, &format).ok_or_else(|| {
                VrError::Decode(format!(
                    "the camera stream reports resolution {resolution:?} and format {format:?}, \
                     which this crate does not know"
                ))
            })?;
        stream.service_name = fixed_str(&service);
        Ok(stream)
    }

    /// The latest frame **if it is new since the last call**, otherwise `None`.
    ///
    /// The camera's freshness model, different from
    /// [`states`](VirtualRobot::states) on purpose: an image pipeline wants to
    /// work once per frame. Calling this at 100 Hz against a 60 fps stream
    /// yields a frame about 60 times a second. Each frame is handed out once.
    #[must_use]
    pub fn fresh(&self) -> Option<Frame> {
        let mut out: *mut sys::vrsdk_frame_t = ptr::null_mut();
        // SAFETY: the camera handle is live and `out` is a writable slot.
        let code = unsafe { sys::vrsdk_camera_fresh(self.raw.as_ptr(), &mut out) };
        expect_ok(code, "vrsdk_camera_fresh");
        NonNull::new(out).map(Frame::adopt)
    }

    /// The latest frame, new or not, or `None` before the first one. Does not
    /// consume freshness.
    #[must_use]
    pub fn latest(&self) -> Option<Frame> {
        let mut out: *mut sys::vrsdk_frame_t = ptr::null_mut();
        // SAFETY: the camera handle is live and `out` is a writable slot.
        let code = unsafe { sys::vrsdk_camera_latest(self.raw.as_ptr(), &mut out) };
        expect_ok(code, "vrsdk_camera_latest");
        NonNull::new(out).map(Frame::adopt)
    }

    /// Block until a frame newer than the last stored one arrives: for
    /// image-paced loops that run once per frame.
    ///
    /// # Errors
    ///
    /// [`VrError::Timeout`] if none arrives in time, which is also how a stopped
    /// camera or a paused simulator is detected; [`VrError::InvalidArgument`]
    /// for a zero timeout.
    pub fn wait_new_frame(&self, timeout: Duration) -> VrResult<()> {
        // SAFETY: the camera handle is live.
        check(unsafe { sys::vrsdk_camera_wait_new_frame(self.raw.as_ptr(), ffi::seconds(timeout)) })
    }

    /// Which stream this is: name, resolution and format.
    #[must_use]
    pub fn spec(&self) -> CameraSpec {
        self.spec.clone()
    }

    /// The camera's name on the robot.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.spec.name
    }

    /// The full iceoryx2 service name, exactly as `vrobots topic list` prints
    /// it, e.g. `vrobots/1/i/cam/front_left/720p_rgba8`.
    #[must_use]
    pub fn service_name(&self) -> &str {
        &self.service_name
    }

    /// Reader-thread counters: frames, decode failures, sequence gaps.
    #[must_use]
    pub fn stats(&self) -> CameraStats {
        let mut raw = sys::vrsdk_camera_stats_t::default();
        // SAFETY: the camera handle is live and `raw` is writable.
        let code = unsafe { sys::vrsdk_camera_stats(self.raw.as_ptr(), &mut raw) };
        expect_ok(code, "vrsdk_camera_stats");
        CameraStats {
            received: raw.received,
            decode_errors: raw.decode_errors,
            seq_gaps: raw.seq_gaps,
            missed_frames: raw.missed_frames,
            last_seq: raw.last_seq,
        }
    }

    /// The most recent decode failure on this stream, if any. Counted and
    /// dropped, never fatal.
    #[must_use]
    pub fn last_error(&self) -> Option<VrError> {
        let mut code = sys::VRSDK_OK;
        let mut message: [c_char; MESSAGE_CAPACITY] = [0; MESSAGE_CAPACITY];
        // SAFETY: the camera handle is live; `code` is writable and `message` is
        // writable for its full length, the capacity passed.
        let status = unsafe {
            sys::vrsdk_camera_last_error(
                self.raw.as_ptr(),
                &mut code,
                message.as_mut_ptr(),
                message.len(),
            )
        };
        expect_ok(status, "vrsdk_camera_last_error");
        VrError::from_code(code, fixed_str(&message))
    }

    /// Whether the reader thread is still running: false after
    /// [`stop`](Self::stop), after the camera was unmounted, or if the thread
    /// ended on its own.
    #[must_use]
    pub fn is_running(&self) -> bool {
        // SAFETY: the camera handle is live.
        unsafe { sys::vrsdk_camera_is_running(self.raw.as_ptr()) }
    }

    /// Stop reading. Idempotent, and dropping the stream does it too. The
    /// camera keeps publishing; this only ends this subscription.
    pub fn stop(&self) {
        // SAFETY: the camera handle is live.
        let code = unsafe { sys::vrsdk_camera_stop(self.raw.as_ptr()) };
        expect_ok(code, "vrsdk_camera_stop");
    }

    /// The C handle, for calling a function of [`crate::sys`] directly. Still
    /// owned by this value.
    #[must_use]
    pub fn as_raw(&self) -> *const sys::vrsdk_camera_t {
        self.raw.as_ptr()
    }
}

impl Drop for CameraStream {
    fn drop(&mut self) {
        // SAFETY: the handle came from the library, is owned only by this value
        // and freed exactly once, here. Freeing stops and joins the reader
        // thread; it does not unmount the camera.
        unsafe { sys::vrsdk_camera_free(self.raw.as_ptr()) };
    }
}

// SAFETY: the C API documents camera handles as safe to use from several
// threads ("two threads racing on the same stream cannot both receive the same
// frame"); the handle is freed only in `Drop`, with exclusive access.
unsafe impl Send for CameraStream {}
// SAFETY: every method through `&CameraStream` passes the handle as `const`.
unsafe impl Sync for CameraStream {}

/// Cameras. See the [module documentation](crate::camera).
impl VirtualRobot {
    /// Create a camera on the robot and subscribe to its frames, with the
    /// default mount and lens. **This mutates the simulator**; see
    /// [`mount_camera_with`](Self::mount_camera_with).
    ///
    /// # Errors
    ///
    /// As [`mount_camera_with`](Self::mount_camera_with).
    pub fn mount_camera(
        &self,
        name: &str,
        resolution: &str,
        format: &str,
    ) -> VrResult<CameraStream> {
        self.mount(name, resolution, format, None)
    }

    /// Create a camera with an explicit mount pose and lens, and subscribe to
    /// its frames.
    ///
    /// Reach for it only when the cameras the robot already carries cannot
    /// serve you; [`open_camera`](Self::open_camera) attaches to those without
    /// changing anything. `name` may contain letters, digits, `_` and `-`;
    /// `resolution` is `"360p"`, `"720p"` or `"1080p"` and is one setting for
    /// every camera on the robot; `format` is `"mono8"`, `"rgb8"` or `"rgba8"`.
    /// Mounting adds one camera and leaves the others alone; re-using a name
    /// reconfigures that camera in place. Remove it again with
    /// [`unmount_camera`](Self::unmount_camera).
    ///
    /// ```no_run
    /// # use vrobots_sdk::{CameraOptions, RobotType, VirtualRobot};
    /// # let robot = VirtualRobot::connect(RobotType::Multirotor, Some(1))?;
    /// let options = CameraOptions::default().with_mount_euler_deg([0.0, 0.0, 180.0]);
    /// let cam = robot.mount_camera_with("tilt", "720p", "rgb8", &options)?;
    /// println!("streaming on {}", cam.service_name());
    /// robot.unmount_camera("tilt")?;
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`] for an unusable name, an unknown resolution
    /// or format, or a resolution that conflicts with a camera already mounted;
    /// [`VrError::NoResponder`] or [`VrError::Timeout`] if `srv/cameras` does
    /// not answer; [`VrError::Timeout`] if no stream appears within
    /// [`ConnectOptions::camera_timeout`](crate::ConnectOptions::camera_timeout);
    /// [`VrError::Deleted`].
    pub fn mount_camera_with(
        &self,
        name: &str,
        resolution: &str,
        format: &str,
        options: &CameraOptions,
    ) -> VrResult<CameraStream> {
        self.mount(name, resolution, format, Some(options))
    }

    fn mount(
        &self,
        name: &str,
        resolution: &str,
        format: &str,
        options: Option<&CameraOptions>,
    ) -> VrResult<CameraStream> {
        let name = ffi::c_string(name, "camera name")?;
        let resolution = ffi::c_string(resolution, "resolution")?;
        let format = ffi::c_string(format, "format")?;
        let options = options.map(CameraOptions::to_raw);
        let options_ptr = options.as_ref().map_or(ptr::null(), ptr::from_ref);
        let mut out: *mut sys::vrsdk_camera_t = ptr::null_mut();
        // SAFETY: the robot handle is live; the three strings are NUL-terminated
        // and alive for the call; `options_ptr` is NULL or points at an
        // initialised struct alive for the call; `out` is a writable slot.
        check(unsafe {
            sys::vrsdk_robot_mount_camera(
                self.raw(),
                name.as_ptr(),
                resolution.as_ptr(),
                format.as_ptr(),
                options_ptr,
                &mut out,
            )
        })?;
        adopt_camera(out)
    }

    /// Subscribe to a camera that already exists. **Never mutates the
    /// simulator.**
    ///
    /// Every vrobot ships `front_left` and `front_right` at `"720p"` and
    /// `"rgba8"`, so this is the usual way to get pixels. The three strings must
    /// match the publisher exactly: on iceoryx2 they are the whole stream
    /// identity.
    ///
    /// ```no_run
    /// # use std::time::Duration;
    /// # use vrobots_sdk::{RobotType, VirtualRobot};
    /// # let robot = VirtualRobot::connect(RobotType::Multirotor, Some(1))?;
    /// let cam = robot.open_camera("front_left", "720p", "rgba8")?;
    /// cam.wait_new_frame(Duration::from_secs(2))?;
    /// if let Some(frame) = cam.fresh() {
    ///     println!("{}x{} {} bytes", frame.width, frame.height, frame.data.len());
    /// }
    /// # Ok::<(), vrobots_sdk::VrError>(())
    /// ```
    ///
    /// # Errors
    ///
    /// [`VrError::Timeout`] after
    /// [`camera_timeout`](crate::ConnectOptions::camera_timeout) when no such
    /// stream exists, which is what any mismatch in the three strings looks
    /// like; [`VrError::InvalidArgument`] for an unusable name, resolution or
    /// format; [`VrError::Session`] if the stream exists but will not pair.
    pub fn open_camera(
        &self,
        name: &str,
        resolution: &str,
        format: &str,
    ) -> VrResult<CameraStream> {
        let name = ffi::c_string(name, "camera name")?;
        let resolution = ffi::c_string(resolution, "resolution")?;
        let format = ffi::c_string(format, "format")?;
        let mut out: *mut sys::vrsdk_camera_t = ptr::null_mut();
        // SAFETY: the robot handle is live; the three strings are NUL-terminated
        // and alive for the call; `out` is a writable slot.
        check(unsafe {
            sys::vrsdk_robot_open_camera(
                self.raw(),
                name.as_ptr(),
                resolution.as_ptr(),
                format.as_ptr(),
                &mut out,
            )
        })?;
        adopt_camera(out)
    }

    /// Remove a camera this handle mounted, by name, and stop its stream. Every
    /// other camera keeps streaming, and a [`CameraStream`] still held goes quiet
    /// rather than dangling.
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`] for a camera this handle did not mount, such
    /// as `front_left` or one attached with [`open_camera`](Self::open_camera);
    /// the service errors of [`mount_camera_with`](Self::mount_camera_with).
    pub fn unmount_camera(&self, name: &str) -> VrResult<()> {
        let name = ffi::c_string(name, "camera name")?;
        // SAFETY: the robot handle is live and `name` is NUL-terminated and alive
        // for the call.
        check(unsafe { sys::vrsdk_robot_unmount_camera(self.raw(), name.as_ptr()) })
    }

    /// The cameras this handle mounted, in mount order: what this handle
    /// mounted, not a read-back from the simulator.
    #[must_use]
    pub fn mounted_cameras(&self) -> Vec<CameraSpec> {
        let mut count = 0usize;
        // SAFETY: the robot handle is live and `count` is writable.
        let code = unsafe { sys::vrsdk_robot_mounted_camera_count(self.raw(), &mut count) };
        expect_ok(code, "vrsdk_robot_mounted_camera_count");
        let mut out = Vec::with_capacity(count);
        for index in 0..count {
            let mut name: [c_char; 256] = [0; 256];
            let mut resolution: [c_char; 32] = [0; 32];
            let mut format: [c_char; 32] = [0; 32];
            // SAFETY: the robot handle is live and each buffer is writable for
            // the capacity passed with it.
            let code = unsafe {
                sys::vrsdk_robot_mounted_camera(
                    self.raw(),
                    index,
                    name.as_mut_ptr(),
                    name.len(),
                    resolution.as_mut_ptr(),
                    resolution.len(),
                    format.as_mut_ptr(),
                    format.len(),
                )
            };
            if code != sys::VRSDK_OK {
                // Unmounted by another thread between the count and this read.
                break;
            }
            if let Some(spec) = CameraSpec::from_strings(
                fixed_str(&name),
                &fixed_str(&resolution),
                &fixed_str(&format),
            ) {
                out.push(spec);
            }
        }
        out
    }
}

/// Wrap a camera handle a successful mount or open returned.
fn adopt_camera(out: *mut sys::vrsdk_camera_t) -> VrResult<CameraStream> {
    let raw = NonNull::new(out).ok_or_else(|| {
        VrError::InvalidHandle(
            "the library reported a camera stream but returned no handle; this is a bug in the \
             VRobots SDK"
                .to_string(),
        )
    })?;
    CameraStream::adopt(raw)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stream_identity_parses_and_prints() {
        assert_eq!(Resolution::parse("720p"), Some(Resolution::P720));
        assert_eq!(Resolution::parse("1080"), Some(Resolution::P1080));
        assert_eq!(Resolution::parse("4k"), None);
        assert_eq!(Resolution::P360.width(), 640);
        assert_eq!(PixelFormat::parse("RGBA8"), Some(PixelFormat::Rgba8));
        assert_eq!(PixelFormat::parse("bgr8"), None);
        for format in [PixelFormat::Mono8, PixelFormat::Rgb8, PixelFormat::Rgba8] {
            let wire = i32::try_from(format.wire_value()).expect("small");
            assert_eq!(PixelFormat::from_wire_value(wire), Some(format));
            assert_eq!(
                PixelFormat::from_bytes_per_pixel(format.bytes_per_pixel()),
                format
            );
        }

        let spec =
            CameraSpec::from_strings("front_left".to_string(), "720p", "rgba8").expect("known");
        assert_eq!(spec.stream_segment(), "720p_rgba8");
        assert_eq!(
            spec.service_name(1),
            "vrobots/1/i/cam/front_left/720p_rgba8"
        );
        assert_eq!(spec.data_size(), 1280 * 720 * 4);
        assert_eq!(spec.to_string(), "front_left 720p_rgba8");
    }

    #[test]
    fn camera_defaults_come_from_the_library() {
        let o = CameraOptions::default();
        assert_eq!((o.fx, o.fy), (600.0, 600.0));
        assert_eq!((o.near_clip, o.far_clip), (0.5, 1000.0));
        assert_eq!(o.mount_position, [0.0; 3]);
    }
}
