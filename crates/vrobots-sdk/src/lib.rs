//! Rust client SDK for the Ubicoders virtual robots simulator.
//!
//! A safe wrapper over the VRobots SDK C library, whose raw declarations live
//! in the `vrobots-sdk-sys` crate (re-exported here as [`sys`]). The same
//! library backs the C++ header and the Python package, so every surface of the
//! SDK connects, times out, retries and fails in exactly the same way; this
//! crate adds ownership, `Result` and owned Rust values, and nothing else.
//!
//! # The shape of a program
//!
//! ```no_run
//! use vrobots_sdk::{RobotType, VirtualRobot, VrError};
//!
//! fn main() -> Result<(), VrError> {
//!     // ===== setup =====
//!     vrobots_sdk::init_logging("info");
//!     // `Some(sys_id)` attaches to a robot; `None` creates a new one.
//!     let robot = VirtualRobot::connect(RobotType::Multirotor, Some(1))?;
//!
//!     // ===== loop =====
//!     loop {
//!         let s = robot.states(); // latest snapshot, never torn
//!         let [x, y, z] = s.kin.lin_pos;
//!         println!("t={:.3} pos=({x:.2},{y:.2},{z:.2})", s.elapsed);
//!
//!         robot.set_mr_pwm([1501.0; 4])?; // your controller's output
//!         robot.rate(100.0); // drift-compensated pacing, Hz
//!     }
//! }
//! ```
//!
//! # What the SDK guarantees
//!
//! - [`VirtualRobot::states`] never blocks and never returns a half-written
//!   value. A subscriber inside the library keeps it fresh; you read at your
//!   own rate.
//! - Every snapshot carries `t_ns` (simulator capture time, unix nanoseconds)
//!   and `elapsed` (seconds since the robot's first sample). State and camera
//!   streams are independent and never paired by the SDK: compare `t_ns`.
//! - Robots outlive the process. Dropping a [`VirtualRobot`] closes the session
//!   and leaves the robot running; [`VirtualRobot::delete`] is the only verb
//!   that removes one.
//! - Decode failures never tear the session down. They are counted
//!   ([`VirtualRobot::stats`]) and inspectable ([`VirtualRobot::last_error`]).
//! - You own the loop and its rate. [`VirtualRobot::rate`] and
//!   [`VirtualRobot::wait_new_state`] are helpers; the SDK never calls your
//!   code, except a log handler you register yourself.
//!
//! # Where things are
//!
//! | Module | What it holds |
//! |---|---|
//! | [`robot`] | [`VirtualRobot`]: connect, read state, pace a loop, delete |
//! | [`options`] | [`RobotType`] and [`ConnectOptions`] |
//! | [`state`] | [`State`] and its blocks, the [`Axes`] and wire [`EulerOrder`] tags |
//! | [`commands`] | the command methods, the [`cmd`] ids and [`CmdArgs`] |
//! | [`setpoint`] | reading other clients' commands: [`SetpointStream`] |
//! | [`camera`] | [`CameraStream`], [`Frame`] and the camera options |
//! | [`services`] | reset, activate and the configuration requests |
//! | [`topics`] | every wire name, built in one place |
//! | [`discovery`] | [`list_topics`] and [`measure_rate`] |
//! | [`rotations`] | quaternions, matrices, Euler angles and frame conversions |
//! | [`version`](mod@version) | [`version_info`] and [`check_version`] |
//! | [`logging`] | [`init_logging`], [`set_log_callback`] and [`set_log_level`] |
//! | [`error`] | [`VrError`], one variant per C error code |
//!
//! # Threads
//!
//! [`VirtualRobot`], [`CameraStream`], [`SetpointStream`] and [`Frame`] are
//! `Send` and `Sync`, so one handle can be shared between threads by reference
//! or through an `Arc`. The C library documents its handles as safe to use from
//! several threads at once: its header states that two threads racing on one
//! camera or setpoint stream cannot both receive the same value and keeps the
//! error detail of each failing call per thread, and the C++ header shipped
//! beside it states that the robot and camera handles may be used from several
//! threads. The one rule the library leaves to the caller, never to free a
//! handle while another thread is using it, is enforced here by ownership: each
//! handle is freed only in `Drop`, which cannot run while a borrow is alive.
//! A [`Frame`] is immutable once handed out.
//!
//! # Linking and running
//!
//! The `vrobots-sdk-sys` build script downloads the C bundle of this crate's
//! version from the GitHub Release of <https://github.com/ubicoders/vrobots-sdk>,
//! verifies it against the Release's `SHA256SUMS` and links the shared library
//! `vrobots_sdk_capi`; `cargo run` and `cargo test` find it on their own. Set
//! `VROBOTS_SDK_DIR` to an unpacked bundle to build offline, and enable the
//! `static` feature to link the static library, so that a program needs no
//! shared library at run time. The `vrobots-sdk-sys` documentation covers both
//! in detail.
//!
//! # Licence
//!
//! The VRobots SDK, including this crate and the C library it links, is
//! released under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
//! International licence (CC BY-NC-SA 4.0) with a patent addendum. The full
//! text is in `LICENSE` and `LICENSE-ADDENDUM` at the root of
//! <https://github.com/ubicoders/vrobots-sdk>.

#![warn(missing_docs)]
#![warn(unsafe_op_in_unsafe_fn)]
#![warn(clippy::undocumented_unsafe_blocks)]

/// The raw C declarations this crate wraps, for anything the safe API does not
/// cover. Every function in it is `unsafe` and follows the rules documented on
/// it.
pub use vrobots_sdk_sys as sys;

mod ffi;

pub mod camera;
pub mod commands;
pub mod discovery;
pub mod error;
pub mod logging;
pub mod options;
pub mod robot;
pub mod rotations;
pub mod services;
pub mod setpoint;
pub mod state;
pub mod topics;
pub mod version;

pub use camera::{
    CameraOptions, CameraSpec, CameraStats, CameraStream, Frame, FramePixels, Intrinsics,
    MountPose, PixelFormat, Resolution,
};
pub use commands::{cmd, CmdArgs};
pub use discovery::{
    discovery_covers_all_transports, list_topics, list_topics_with, measure_rate,
    measure_rate_with, RateReport, TopicInfo, Transport,
};
pub use error::{err, VrError, VrResult};
pub use logging::{
    clear_log_callback, init_logging, set_log_callback, set_log_level, LogEvent, LogLevel,
};
pub use options::{
    ConnectOptions, RobotType, DEFAULT_CLIENT_NAME, DEFAULT_COORD_FRAME_ID, DEFAULT_SRC_ID,
};
pub use robot::VirtualRobot;
// The rotation functions keep their own module namespace, as in the other SDK
// languages: `rotations::quat_to_euler` reads better at a call site than a bare
// `quat_to_euler`. `rotations::EulerOrder` is deliberately not lifted to the
// root, where `state::EulerOrder` (the wire tag) already owns the name.
pub use rotations::{AxisBasis, FrameTransform};
pub use services::{
    device, CartPoleConfig, DeviceFrame, DriveConfig, GpsNoise, GpsQuality, ImuNoise, MsdConfig,
    PhysicalParams, PwmBand, RotorSpec, SceneFrame, SensorConfig, INHERIT_FRAME,
};
pub use setpoint::{Setpoint, SetpointStats, SetpointStream};
pub use state::{
    Accelerometer, Actuator, Axes, Barometer, Environment, Estimate, EulerOrder, FrameDef,
    GeoPoint, Gnss, Gyroscope, Kinematics, Magnetometer, OpticalFlow, Sensors, State, StateStats,
    Wrench,
};
pub use version::{check_version, version, version_info, VersionInfo, VERSION};
