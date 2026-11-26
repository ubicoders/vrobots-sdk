//! Robot kinds, and the one struct that tunes a connection.
//!
//! A [`RobotType`] names what a handle drives; it selects which commands and
//! services make sense and is never read back off the wire. [`ConnectOptions`]
//! holds every timeout, the router endpoint and this client's identity, with
//! the library's own defaults, so nothing about a connection is a hidden
//! constant. [`SpawnPose`] places a robot the connection creates.

use std::ffi::CString;
use std::ptr;
use std::time::Duration;

use vrobots_sdk_sys as sys;

use crate::error::VrResult;
use crate::ffi;
use crate::rotations::IDENTITY_QUAT;
use crate::state::Axes;

/// The `src_id` this SDK stamps on every outbound header by default.
///
/// Every client needs a non-zero id of its own (0 is the simulator's), and
/// service replies route back by it. Change it with
/// [`ConnectOptions::with_src_id`] when several clients share a session.
pub const DEFAULT_SRC_ID: u32 = 122;

/// The coordinate frame stamped on outgoing headers by default: Unity's own.
pub const DEFAULT_COORD_FRAME_ID: &str = "unity";

/// The `header.name` this SDK stamps on outbound messages by default, naming
/// the client rather than the robot.
pub const DEFAULT_CLIENT_NAME: &str = "vrobots-sdk";

/// A robot kind in the simulator's spawn catalog.
///
/// The kind selects which typed commands and services make sense, nothing more.
/// It is not read back off the wire on an attach, and a robot silently ignores
/// commands it does not implement, so naming the wrong kind presents as
/// "nothing happens". The exception is a service query: `srv/rotors` on a truck
/// has no responder and fails with [`VrError::NoResponder`](crate::VrError::NoResponder).
///
/// # Not every kind can be created
///
/// The spawn catalog belongs to the scene. A key the running scene does not
/// know is refused with [`VrError::Service`](crate::VrError::Service), whose
/// message names the key you asked for and lists every key the scene does
/// know. The sandbox scene knows eight: `multirotor`, `truck`, `msd`,
/// `forklift`, `palletrobot`, `kr210_rail`, `tablearm` and `turret`. Look for
/// a key in that list by name; do not rely on the order the message gives
/// them in.
///
/// [`CartPole`](Self::CartPole), [`HalfDrone`](Self::HalfDrone) and
/// [`GlobalHawk`](Self::GlobalHawk) exist only as scene-authored robots in the
/// shipped scenes: attach to those with an explicit `sys_id`, which never
/// touches the create service. [`Forklift`](Self::Forklift) through
/// [`Turret`](Self::Turret) are in the spawner's catalog too, **not yet
/// verified on a live simulator**, and a runtime create needs the **Pro**
/// tier; the warehouse and table-arm scenes also carry them at pinned ids.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum RobotType {
    /// Quadrotor-class aircraft. Driven with
    /// [`set_mr_pwm`](crate::VirtualRobot::set_mr_pwm); serves
    /// [`configure_rotors`](crate::VirtualRobot::configure_rotors).
    Multirotor,
    /// Wheeled ground vehicle. Driven with
    /// [`set_car`](crate::VirtualRobot::set_car); serves
    /// [`configure_drive`](crate::VirtualRobot::configure_drive).
    Truck,
    /// One-dimensional mass-spring-damper. Driven with
    /// [`set_msd_force`](crate::VirtualRobot::set_msd_force); serves
    /// [`configure_msd`](crate::VirtualRobot::configure_msd).
    Msd,
    /// Cart on a rail with a pendulum on top. Driven with
    /// [`set_cartpole_force`](crate::VirtualRobot::set_cartpole_force); serves
    /// [`configure_cartpole`](crate::VirtualRobot::configure_cartpole), which
    /// owns every mass and length including the cart's. Scene-authored.
    CartPole,
    /// Two-rotor aircraft constrained to a plane. Driven with
    /// [`set_mr_pwm_n`](crate::VirtualRobot::set_mr_pwm_n) carrying exactly two
    /// pulse widths. Scene-authored.
    HalfDrone,
    /// RQ-4B Global Hawk, a fixed wing with six aero panels and one engine.
    /// Flies itself under an onboard rate loop, or hands the panels over with
    /// [`set_fw_ctrl_mode`](crate::VirtualRobot::set_fw_ctrl_mode).
    /// Scene-authored.
    GlobalHawk,
    /// Counterbalance forklift: the front pair drives, the rear pair steers,
    /// and the mast carries a lift and a fork-width servo. Driven with
    /// [`set_forklift`](crate::VirtualRobot::set_forklift); serves
    /// [`configure_forklift`](crate::VirtualRobot::configure_forklift); read
    /// with [`forklift_actuators`](crate::VirtualRobot::forklift_actuators).
    /// Carries one 3-D lidar. Pinned at `sys_id` 0 in the warehouse scene.
    Forklift,
    /// Mecanum pallet robot with a lifting, turning pad. Driven with
    /// [`set_palletrobot`](crate::VirtualRobot::set_palletrobot), and the wheel
    /// mixing is yours; serves
    /// [`configure_palletrobot`](crate::VirtualRobot::configure_palletrobot);
    /// read with
    /// [`palletrobot_actuators`](crate::VirtualRobot::palletrobot_actuators).
    /// Carries two 2-D lidars. Pinned at `sys_id` 1 in the warehouse scene.
    PalletRobot,
    /// KUKA KR210 on a linear rail with a vacuum pad: seven joints on
    /// [`set_arm_joints`](crate::VirtualRobot::set_arm_joints), the pad on
    /// [`set_arm_vacuum`](crate::VirtualRobot::set_arm_vacuum); serves
    /// [`configure_arm`](crate::VirtualRobot::configure_arm); read with
    /// [`rail_arm_actuators`](crate::VirtualRobot::rail_arm_actuators). Pinned
    /// at `sys_id` 2 in the warehouse scene; `kr210_rail` in the spawn catalog.
    RailArm,
    /// UR10 with a two-finger gripper: six joints on
    /// [`set_arm_joints`](crate::VirtualRobot::set_arm_joints), the gripper on
    /// [`set_arm_gripper`](crate::VirtualRobot::set_arm_gripper); serves
    /// [`configure_arm`](crate::VirtualRobot::configure_arm); read with
    /// [`table_arm_actuators`](crate::VirtualRobot::table_arm_actuators).
    /// Pinned at `sys_id` 0 in the table-arm scene.
    TableArm,
    /// Pan-tilt turret with a laser and a stereo camera pair. Driven with
    /// [`set_turret_rates`](crate::VirtualRobot::set_turret_rates),
    /// [`set_turret_joints`](crate::VirtualRobot::set_turret_joints) and
    /// [`set_turret_laser`](crate::VirtualRobot::set_turret_laser); serves
    /// [`configure_turret`](crate::VirtualRobot::configure_turret); read with
    /// [`turret_actuators`](crate::VirtualRobot::turret_actuators).
    /// Scene-authored in the sandbox scene with no pinned id: find it with
    /// [`list_topics`](crate::list_topics).
    Turret,
}

impl RobotType {
    /// The spawn-catalog key: `"multirotor"`, `"truck"`, `"msd"`, `"cartpole"`,
    /// `"halfdrone"`, `"globalhawk"`, `"forklift"`, `"palletrobot"`,
    /// `"kr210_rail"`, `"tablearm"` or `"turret"`.
    ///
    /// Matched exactly and case-sensitively by the simulator's manager.
    #[must_use]
    pub fn catalog_key(self) -> &'static str {
        match self {
            RobotType::Multirotor => "multirotor",
            RobotType::Truck => "truck",
            RobotType::Msd => "msd",
            RobotType::CartPole => "cartpole",
            RobotType::HalfDrone => "halfdrone",
            RobotType::GlobalHawk => "globalhawk",
            RobotType::Forklift => "forklift",
            RobotType::PalletRobot => "palletrobot",
            RobotType::RailArm => "kr210_rail",
            RobotType::TableArm => "tablearm",
            RobotType::Turret => "turret",
        }
    }

    /// Whether the simulator's spawn catalog knows this kind, so that
    /// [`VirtualRobot::connect`](crate::VirtualRobot::connect) with `None` can
    /// create one.
    ///
    /// True for the multirotor, the truck and the MSD, which are verified
    /// against the sandbox scene, and for the forklift, pallet robot, rail
    /// arm, table arm and turret, which are in the spawner's catalog but **not
    /// yet verified on a live simulator**. True is not a promise that a create
    /// succeeds: a runtime create needs the Pro tier, and on Community every
    /// create is refused.
    #[must_use]
    pub fn is_in_sandbox_catalog(self) -> bool {
        matches!(
            self,
            RobotType::Multirotor
                | RobotType::Truck
                | RobotType::Msd
                | RobotType::Forklift
                | RobotType::PalletRobot
                | RobotType::RailArm
                | RobotType::TableArm
                | RobotType::Turret
        )
    }

    /// Parse a catalog key back into a kind, case-insensitively.
    ///
    /// Accepts the synonyms the simulator's own UI uses as well: `"car"`,
    /// `"cart_pole"`, `"invpen"`, `"mass_spring_damper"`, `"half_drone"`,
    /// `"global_hawk"`, `"rq4b"`, `"pallet_robot"`, `"railarm"`, `"rail_arm"`,
    /// `"kuka_kr210_rail"` (the rail arm's wire name) and `"table_arm"`.
    #[must_use]
    pub fn from_catalog_key(key: &str) -> Option<RobotType> {
        match key.trim().to_ascii_lowercase().as_str() {
            "multirotor" => Some(RobotType::Multirotor),
            "truck" | "car" => Some(RobotType::Truck),
            "msd" | "mass_spring_damper" => Some(RobotType::Msd),
            "cartpole" | "cart_pole" | "invpen" => Some(RobotType::CartPole),
            "halfdrone" | "half_drone" => Some(RobotType::HalfDrone),
            "globalhawk" | "global_hawk" | "rq4b" => Some(RobotType::GlobalHawk),
            "forklift" => Some(RobotType::Forklift),
            "palletrobot" | "pallet_robot" => Some(RobotType::PalletRobot),
            "kr210_rail" | "railarm" | "rail_arm" | "kuka_kr210_rail" => Some(RobotType::RailArm),
            "tablearm" | "table_arm" => Some(RobotType::TableArm),
            "turret" => Some(RobotType::Turret),
            _ => None,
        }
    }

    /// The `VRSDK_ROBOT_*` code the C API takes.
    pub(crate) fn code(self) -> sys::vrsdk_robot_type_t {
        match self {
            RobotType::Multirotor => sys::VRSDK_ROBOT_MULTIROTOR,
            RobotType::Truck => sys::VRSDK_ROBOT_TRUCK,
            RobotType::Msd => sys::VRSDK_ROBOT_MSD,
            RobotType::CartPole => sys::VRSDK_ROBOT_CARTPOLE,
            RobotType::HalfDrone => sys::VRSDK_ROBOT_HALFDRONE,
            RobotType::GlobalHawk => sys::VRSDK_ROBOT_GLOBALHAWK,
            RobotType::Forklift => sys::VRSDK_ROBOT_FORKLIFT,
            RobotType::PalletRobot => sys::VRSDK_ROBOT_PALLETROBOT,
            RobotType::RailArm => sys::VRSDK_ROBOT_RAILARM,
            RobotType::TableArm => sys::VRSDK_ROBOT_TABLEARM,
            RobotType::Turret => sys::VRSDK_ROBOT_TURRET,
        }
    }

    /// The kind for a `VRSDK_ROBOT_*` code.
    pub(crate) fn from_code(code: sys::vrsdk_robot_type_t) -> Option<RobotType> {
        match code {
            sys::VRSDK_ROBOT_MULTIROTOR => Some(RobotType::Multirotor),
            sys::VRSDK_ROBOT_TRUCK => Some(RobotType::Truck),
            sys::VRSDK_ROBOT_MSD => Some(RobotType::Msd),
            sys::VRSDK_ROBOT_CARTPOLE => Some(RobotType::CartPole),
            sys::VRSDK_ROBOT_HALFDRONE => Some(RobotType::HalfDrone),
            sys::VRSDK_ROBOT_GLOBALHAWK => Some(RobotType::GlobalHawk),
            sys::VRSDK_ROBOT_FORKLIFT => Some(RobotType::Forklift),
            sys::VRSDK_ROBOT_PALLETROBOT => Some(RobotType::PalletRobot),
            sys::VRSDK_ROBOT_RAILARM => Some(RobotType::RailArm),
            sys::VRSDK_ROBOT_TABLEARM => Some(RobotType::TableArm),
            sys::VRSDK_ROBOT_TURRET => Some(RobotType::Turret),
            _ => None,
        }
    }
}

/// Where a created robot appears, in **raw Unity world coordinates**: the
/// [`ConnectOptions::initial_pose`] of a connection that creates one.
///
/// The simulator applies the pose exactly as sent and **ignores the frame
/// pair** of [`ConnectOptions::with_frame`], so nothing here is converted.
/// Unity's world is left-handed, in metres: +x right, +y up, +z forward. The
/// rotation is a Unity quaternion `[x, y, z, w]`, normalized before sending; a
/// heading of `yaw` radians about the up axis is
/// `[0, sin(yaw / 2), 0, cos(yaw / 2)]`, and a positive `yaw` turns +z toward
/// +x, clockwise seen from above.
///
/// It is also the robot's home, the pose [`reset`](crate::VirtualRobot::reset)
/// returns it to. Without one a created robot spawns at the world origin,
/// unrotated, so two robots created that way overlap.
///
/// ```
/// use vrobots_sdk::{ConnectOptions, SpawnPose};
///
/// // 4 m right of the origin, turned a quarter turn clockwise seen from above.
/// let half = std::f64::consts::FRAC_PI_4;
/// let options = ConnectOptions::default()
///     .with_initial_pose(SpawnPose::new([4.0, 0.0, 0.0], [0.0, half.sin(), 0.0, half.cos()]));
/// assert!(options.initial_pose.is_some());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct SpawnPose {
    /// Unity world position, metres: +x right, +y up, +z forward.
    pub position: [f64; 3],
    /// Unity world rotation, quaternion `[x, y, z, w]`.
    pub orientation: [f64; 4],
}

impl SpawnPose {
    /// A Unity world position and rotation.
    #[must_use]
    pub fn new(position: [f64; 3], orientation: [f64; 4]) -> SpawnPose {
        SpawnPose {
            position,
            orientation,
        }
    }

    /// A Unity world position with the identity rotation.
    #[must_use]
    pub fn at(position: [f64; 3]) -> SpawnPose {
        SpawnPose::new(position, IDENTITY_QUAT)
    }
}

/// Everything tunable about a connection.
///
/// Build it with [`ConnectOptions::default`], which asks the C library for its
/// defaults, and chain the `with_*` setters. The struct is `#[non_exhaustive]`,
/// so a knob added later is not a breaking change.
///
/// ```no_run
/// use std::time::Duration;
/// use vrobots_sdk::ConnectOptions;
///
/// let opts = ConnectOptions::default()
///     .with_router("tcp/192.168.1.10:7447") // sim on another machine
///     .with_src_id(200) // second client in the session
///     .with_probe_timeout(Duration::from_secs(20));
/// ```
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct ConnectOptions {
    /// Identity stamped into every outbound `header.src_id`. Must be non-zero:
    /// 0 is reserved for the simulator. Defaults to [`DEFAULT_SRC_ID`].
    pub src_id: u32,
    /// An explicit zenoh router, e.g. `"tcp/192.168.1.10:7447"`. `None` (the
    /// default) uses zenoh's own peer discovery, which finds a simulator on the
    /// same host or LAN segment.
    ///
    /// Zenoh traffic works remotely; iceoryx2 camera streams are same-host
    /// only, so a remote connection gets states and commands but not images.
    pub router_endpoint: Option<String>,
    /// Budget for opening the zenoh session. Default 5 s.
    pub connect_timeout: Duration,
    /// Budget for the first state sample. Default 15 s, because zenoh discovery
    /// takes several seconds after a simulator starts.
    pub probe_timeout: Duration,
    /// Budget for one service query. Default 8 s. Doubles as the capability
    /// probe: asking a truck for `srv/rotors` spends all of it.
    pub service_timeout: Duration,
    /// Budget for a camera stream to appear on iceoryx2. Default 5 s.
    pub camera_timeout: Duration,
    /// Wire name for a robot this connection creates. `None` uses the catalog
    /// type's default. Ignored when attaching to an existing id.
    pub robot_name: Option<String>,
    /// Where a robot this connection creates spawns, in raw Unity world
    /// coordinates: see [`SpawnPose`]. `None` (the default) spawns it at the
    /// world origin, unrotated. Ignored when attaching to an existing id.
    pub initial_pose: Option<SpawnPose>,
    /// The `header.name` stamped on outbound messages, naming this client.
    /// Defaults to [`DEFAULT_CLIENT_NAME`].
    pub client_name: String,
    /// Spawn a created robot already running rather than dormant. Default
    /// `true`; `false` is the deterministic configure-then-activate start (see
    /// [`activate`](crate::VirtualRobot::activate)).
    pub start_active: bool,
    /// Call `srv/activate` after a create. Default `true`: harmless when the
    /// robot is already active, required when it is not.
    pub activate_after_create: bool,
    /// The frame id stamped on outgoing headers, naming the convention your
    /// vectors are in. The robot converts into its own frame before acting.
    /// Defaults to [`DEFAULT_COORD_FRAME_ID`].
    pub coord_frame_id: String,
    /// The enum tag beside [`coord_frame_id`](Self::coord_frame_id). Keep the two
    /// consistent; the string wins when they disagree. Default [`Axes::UNITY`].
    pub axis_convention: Axes,
}

impl Default for ConnectOptions {
    /// The C library's defaults (`vrsdk_options_default`): `src_id` 122,
    /// timeouts of 5, 15, 8 and 5 seconds, no spawn pose, start active,
    /// activate after create, and the `"unity"` frame.
    fn default() -> ConnectOptions {
        let mut raw = sys::vrsdk_connect_options_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_connect_options_t`,
        // which is all `vrsdk_options_default` requires.
        unsafe { sys::vrsdk_options_default(&mut raw) };
        ConnectOptions {
            src_id: raw.src_id,
            router_endpoint: None,
            connect_timeout: duration_from_seconds(raw.connect_timeout_s),
            probe_timeout: duration_from_seconds(raw.probe_timeout_s),
            service_timeout: duration_from_seconds(raw.service_timeout_s),
            camera_timeout: duration_from_seconds(raw.camera_timeout_s),
            robot_name: None,
            initial_pose: raw
                .has_initial_pose
                .then(|| SpawnPose::new(raw.initial_position, raw.initial_orientation)),
            client_name: DEFAULT_CLIENT_NAME.to_string(),
            start_active: raw.start_active,
            activate_after_create: raw.activate_after_create,
            coord_frame_id: DEFAULT_COORD_FRAME_ID.to_string(),
            axis_convention: Axes(raw.axis_convention),
        }
    }
}

impl ConnectOptions {
    /// See [`src_id`](Self::src_id).
    #[must_use]
    pub fn with_src_id(mut self, src_id: u32) -> Self {
        self.src_id = src_id;
        self
    }

    /// See [`router_endpoint`](Self::router_endpoint).
    #[must_use]
    pub fn with_router(mut self, endpoint: impl Into<String>) -> Self {
        self.router_endpoint = Some(endpoint.into());
        self
    }

    /// See [`connect_timeout`](Self::connect_timeout).
    #[must_use]
    pub fn with_connect_timeout(mut self, timeout: Duration) -> Self {
        self.connect_timeout = timeout;
        self
    }

    /// See [`probe_timeout`](Self::probe_timeout).
    #[must_use]
    pub fn with_probe_timeout(mut self, timeout: Duration) -> Self {
        self.probe_timeout = timeout;
        self
    }

    /// See [`service_timeout`](Self::service_timeout).
    #[must_use]
    pub fn with_service_timeout(mut self, timeout: Duration) -> Self {
        self.service_timeout = timeout;
        self
    }

    /// See [`camera_timeout`](Self::camera_timeout).
    #[must_use]
    pub fn with_camera_timeout(mut self, timeout: Duration) -> Self {
        self.camera_timeout = timeout;
        self
    }

    /// See [`robot_name`](Self::robot_name).
    #[must_use]
    pub fn with_robot_name(mut self, name: impl Into<String>) -> Self {
        self.robot_name = Some(name.into());
        self
    }

    /// See [`initial_pose`](Self::initial_pose).
    #[must_use]
    pub fn with_initial_pose(mut self, pose: SpawnPose) -> Self {
        self.initial_pose = Some(pose);
        self
    }

    /// See [`client_name`](Self::client_name).
    #[must_use]
    pub fn with_client_name(mut self, name: impl Into<String>) -> Self {
        self.client_name = name.into();
        self
    }

    /// See [`start_active`](Self::start_active).
    #[must_use]
    pub fn with_start_active(mut self, start_active: bool) -> Self {
        self.start_active = start_active;
        self
    }

    /// See [`activate_after_create`](Self::activate_after_create).
    #[must_use]
    pub fn with_activate_after_create(mut self, activate: bool) -> Self {
        self.activate_after_create = activate;
        self
    }

    /// Set both halves of the outgoing frame tag at once, e.g.
    /// `with_frame("frd", Axes::FRD)`.
    #[must_use]
    pub fn with_frame(mut self, coord_frame_id: impl Into<String>, axes: Axes) -> Self {
        self.coord_frame_id = coord_frame_id.into();
        self.axis_convention = axes;
        self
    }

    /// The C struct for these options, with the strings it points into.
    ///
    /// # Errors
    ///
    /// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) if a string
    /// contains a NUL byte.
    pub(crate) fn to_c(&self) -> VrResult<CConnectOptions> {
        let router = self
            .router_endpoint
            .as_deref()
            .map(|s| ffi::c_string(s, "router endpoint"))
            .transpose()?;
        let robot_name = self
            .robot_name
            .as_deref()
            .map(|s| ffi::c_string(s, "robot name"))
            .transpose()?;
        let client_name = ffi::c_string(&self.client_name, "client name")?;
        let coord_frame_id = ffi::c_string(&self.coord_frame_id, "coord_frame_id")?;

        let mut raw = sys::vrsdk_connect_options_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_connect_options_t`.
        unsafe { sys::vrsdk_options_default(&mut raw) };
        raw.src_id = self.src_id;
        raw.router_endpoint = router.as_ref().map_or(ptr::null(), |s| s.as_ptr());
        raw.connect_timeout_s = ffi::seconds(self.connect_timeout);
        raw.probe_timeout_s = ffi::seconds(self.probe_timeout);
        raw.service_timeout_s = ffi::seconds(self.service_timeout);
        raw.camera_timeout_s = ffi::seconds(self.camera_timeout);
        raw.robot_name = robot_name.as_ref().map_or(ptr::null(), |s| s.as_ptr());
        if let Some(pose) = self.initial_pose {
            raw.has_initial_pose = true;
            raw.initial_position = pose.position;
            raw.initial_orientation = pose.orientation;
        }
        raw.client_name = client_name.as_ptr();
        raw.start_active = self.start_active;
        raw.activate_after_create = self.activate_after_create;
        raw.coord_frame_id = coord_frame_id.as_ptr();
        raw.axis_convention = self.axis_convention.0;

        Ok(CConnectOptions {
            raw,
            _strings: [router, robot_name, Some(client_name), Some(coord_frame_id)],
        })
    }
}

/// A `vrsdk_connect_options_t` together with the C strings it points into.
///
/// The pointers target the heap buffers of the `CString`s, which do not move
/// when this struct moves, so the C struct stays valid for as long as this value
/// lives. The C API copies whatever it keeps before returning.
pub(crate) struct CConnectOptions {
    raw: sys::vrsdk_connect_options_t,
    _strings: [Option<CString>; 4],
}

impl CConnectOptions {
    /// A pointer to the C struct, valid while `self` is alive.
    pub(crate) fn as_ptr(&self) -> *const sys::vrsdk_connect_options_t {
        &self.raw
    }
}

/// A default from the C library, in seconds, as a `Duration`.
fn duration_from_seconds(seconds: f64) -> Duration {
    Duration::try_from_secs_f64(seconds).unwrap_or(Duration::ZERO)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn catalog_keys_round_trip() {
        for kind in [
            RobotType::Multirotor,
            RobotType::Truck,
            RobotType::Msd,
            RobotType::CartPole,
            RobotType::HalfDrone,
            RobotType::GlobalHawk,
            RobotType::Forklift,
            RobotType::PalletRobot,
            RobotType::RailArm,
            RobotType::TableArm,
            RobotType::Turret,
        ] {
            assert_eq!(RobotType::from_catalog_key(kind.catalog_key()), Some(kind));
            assert_eq!(RobotType::from_code(kind.code()), Some(kind));
        }
        assert_eq!(RobotType::from_catalog_key("CAR"), Some(RobotType::Truck));
        assert_eq!(
            RobotType::from_catalog_key("rq4b"),
            Some(RobotType::GlobalHawk)
        );
        assert_eq!(
            RobotType::from_catalog_key("kuka_kr210_rail"),
            Some(RobotType::RailArm)
        );
        assert_eq!(
            RobotType::from_catalog_key("Pallet_Robot"),
            Some(RobotType::PalletRobot)
        );
        assert_eq!(RobotType::from_catalog_key("submarine"), None);
        assert_eq!(RobotType::from_code(99), None);
    }

    #[test]
    fn a_spawn_pose_reaches_the_c_struct_only_when_set() {
        let c = ConnectOptions::default().to_c().expect("no NUL bytes");
        // SAFETY: `as_ptr` points at the struct owned by `c`, alive here.
        let raw = unsafe { &*c.as_ptr() };
        assert!(!raw.has_initial_pose);

        let pose = SpawnPose::new([4.0, 0.5, -2.0], [0.0, 0.5, 0.0, 0.75]);
        let c = ConnectOptions::default()
            .with_initial_pose(pose)
            .to_c()
            .expect("no NUL bytes");
        // SAFETY: as above.
        let raw = unsafe { &*c.as_ptr() };
        assert!(raw.has_initial_pose);
        assert_eq!(raw.initial_position, [4.0, 0.5, -2.0]);
        assert_eq!(raw.initial_orientation, [0.0, 0.5, 0.0, 0.75]);
        assert_eq!(SpawnPose::at([1.0, 2.0, 3.0]).orientation, IDENTITY_QUAT);
    }

    #[test]
    fn strings_reach_the_c_struct() {
        let opts = ConnectOptions::default()
            .with_router("tcp/10.0.0.2:7447")
            .with_frame("frd", Axes::FRD);
        let c = opts.to_c().expect("no NUL bytes");
        // SAFETY: `as_ptr` points at the struct owned by `c`, alive here.
        let raw = unsafe { &*c.as_ptr() };
        assert!(raw.robot_name.is_null());
        assert_eq!(raw.axis_convention, sys::VRSDK_AXES_FRD);
        // SAFETY: the pointers target `CString`s owned by `c`, alive here.
        let router = unsafe { std::ffi::CStr::from_ptr(raw.router_endpoint) };
        assert_eq!(router.to_str(), Ok("tcp/10.0.0.2:7447"));
        // SAFETY: as above.
        let frame = unsafe { std::ffi::CStr::from_ptr(raw.coord_frame_id) };
        assert_eq!(frame.to_str(), Ok("frd"));
    }

    #[test]
    fn a_nul_byte_in_a_string_is_refused() {
        let opts = ConnectOptions::default().with_client_name("bad\0name");
        assert!(matches!(
            opts.to_c(),
            Err(crate::VrError::InvalidArgument(_))
        ));
    }
}
