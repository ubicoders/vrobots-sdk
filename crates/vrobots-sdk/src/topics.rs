//! Every wire name the simulator uses, built in one place.
//!
//! Names have the shape `vrobots/<sys_id>/<transport>/<subject...>`, where the
//! segment after the id names the transport, `z` for zenoh and `i` for
//! iceoryx2. The words `manager` and `scene` sit where a `sys_id` would, so a
//! scope key can never collide with a robot key. Both ends of a wire must match
//! byte for byte and a mismatch is silent, which is why these are functions and
//! not strings typed at call sites. `vrobots topic list` prints what is really
//! there.
//!
//! ```
//! use vrobots_sdk::topics;
//!
//! assert_eq!(topics::state(1), "vrobots/1/z/state");
//! assert_eq!(topics::srv_reset(1), "vrobots/1/z/srv/reset");
//! assert_eq!(topics::camera(1, "front_left", "720p_rgba8"), "vrobots/1/i/cam/front_left/720p_rgba8");
//! assert_eq!(topics::sys_id_of("vrobots/scene/z/srv/frame"), None);
//! ```

/// Prefix for everything one robot owns: `vrobots/<sys_id>`.
#[must_use]
pub fn root(sys_id: u32) -> String {
    format!("vrobots/{sys_id}")
}

/// Prefix for the swarm-wide manager, which belongs to no robot.
pub const MANAGER_ROOT: &str = "vrobots/manager";

/// Prefix for the scene scope.
pub const SCENE_ROOT: &str = "vrobots/scene";

/// Wildcard matching every vrobots zenoh key: what discovery subscribes to.
pub const ALL: &str = "vrobots/**";

/// Wildcard matching every robot's state topic.
pub const ALL_STATES: &str = "vrobots/*/z/state";

/// Wildcard matching every robot's safety-event topic: what
/// [`subscribe_all_safety_events`](crate::subscribe_all_safety_events) reads.
pub const ALL_SAFETY_EVENTS: &str = "vrobots/*/z/event/safety";

/// Wildcard matching every 2-D lidar on every robot. The 3-D lidars ride
/// iceoryx2, which has no wildcard.
pub const ALL_LIDARS_2D: &str = "vrobots/*/z/lidar/*";

/// The full state, published by the simulator at 25 Hz: `vrobots/<id>/z/state`.
#[must_use]
pub fn state(sys_id: u32) -> String {
    format!("vrobots/{sys_id}/z/state")
}

/// The coordinate-frame definitions, published on change plus a slow keepalive:
/// `vrobots/<id>/z/frames`. Read with
/// [`frame_def`](crate::VirtualRobot::frame_def).
#[must_use]
pub fn frames(sys_id: u32) -> String {
    format!("vrobots/{sys_id}/z/frames")
}

/// The attitude estimate a client publishes with
/// [`publish_estimate`](crate::VirtualRobot::publish_estimate):
/// `vrobots/<id>/z/estimate`.
#[must_use]
pub fn estimate(sys_id: u32) -> String {
    format!("vrobots/{sys_id}/z/estimate")
}

/// The command bus: `vrobots/<id>/z/cmd`. Many-to-many, so clients can read it
/// too; see [`subscribe_setpoint`](crate::VirtualRobot::subscribe_setpoint).
#[must_use]
pub fn command(sys_id: u32) -> String {
    format!("vrobots/{sys_id}/z/cmd")
}

/// One robot's safety events, one sample per incident and nothing in between:
/// `vrobots/<id>/z/event/safety`. See
/// [`subscribe_safety_events`](crate::VirtualRobot::subscribe_safety_events).
#[must_use]
pub fn safety_events(sys_id: u32) -> String {
    format!("vrobots/{sys_id}/z/event/safety")
}

/// What a robot is (mass, centre of gravity, inertia, actuator layout),
/// published when its wire opens, on any change and at 1 Hz:
/// `vrobots/<id>/z/properties`. Read with
/// [`properties`](crate::VirtualRobot::properties).
#[must_use]
pub fn properties(sys_id: u32) -> String {
    format!("vrobots/{sys_id}/z/properties")
}

/// Any per-robot service: `vrobots/<id>/z/srv/<segment>`.
#[must_use]
pub fn srv(sys_id: u32, segment: &str) -> String {
    format!("vrobots/{sys_id}/z/srv/{segment}")
}

/// The manager's create service. The one non-idempotent service in the system.
#[must_use]
pub fn manager_srv_create() -> String {
    format!("{MANAGER_ROOT}/z/srv/create")
}

/// The manager's delete service.
#[must_use]
pub fn manager_srv_delete() -> String {
    format!("{MANAGER_ROOT}/z/srv/delete")
}

/// `srv/activate`: releases a dormant robot's dynamics hold. See
/// [`activate`](crate::VirtualRobot::activate).
#[must_use]
pub fn srv_activate(sys_id: u32) -> String {
    srv(sys_id, "activate")
}

/// `srv/reset`: returns the robot to its home pose. A payload-less GET here
/// really does reset the robot. See [`reset`](crate::VirtualRobot::reset).
#[must_use]
pub fn srv_reset(sys_id: u32) -> String {
    srv(sys_id, "reset")
}

/// `srv/params`: mass and inertia. See
/// [`set_physical_params`](crate::VirtualRobot::set_physical_params).
#[must_use]
pub fn srv_params(sys_id: u32) -> String {
    srv(sys_id, "params")
}

/// `srv/skin`: appearance, and one of the two services that ever answer "no";
/// [`srv_lidars`] is the other. See [`set_skin`](crate::VirtualRobot::set_skin).
#[must_use]
pub fn srv_skin(sys_id: u32) -> String {
    srv(sys_id, "skin")
}

/// `srv/sensors`: noise models and GNSS quality. See
/// [`configure_sensors`](crate::VirtualRobot::configure_sensors).
#[must_use]
pub fn srv_sensors(sys_id: u32) -> String {
    srv(sys_id, "sensors")
}

/// `srv/frames`: which frame the robot and its devices report in. Distinct from
/// the [`frames`] state topic. See [`set_frames`](crate::VirtualRobot::set_frames).
#[must_use]
pub fn srv_frames(sys_id: u32) -> String {
    srv(sys_id, "frames")
}

/// `srv/drive`: a truck's drivetrain. Truck only. See
/// [`configure_drive`](crate::VirtualRobot::configure_drive).
#[must_use]
pub fn srv_drive(sys_id: u32) -> String {
    srv(sys_id, "drive")
}

/// `srv/rotors`: a multirotor's rotors. Multirotor only. See
/// [`configure_rotors`](crate::VirtualRobot::configure_rotors).
#[must_use]
pub fn srv_rotors(sys_id: u32) -> String {
    srv(sys_id, "rotors")
}

/// `srv/msd`: a mass-spring-damper's plant constants. Msd only. See
/// [`configure_msd`](crate::VirtualRobot::configure_msd).
#[must_use]
pub fn srv_msd(sys_id: u32) -> String {
    srv(sys_id, "msd")
}

/// `srv/cartpole`: a cart-pole's masses, lengths and limits. CartPole only. See
/// [`configure_cartpole`](crate::VirtualRobot::configure_cartpole).
#[must_use]
pub fn srv_cartpole(sys_id: u32) -> String {
    srv(sys_id, "cartpole")
}

/// `srv/cameras`: mount, reconfigure and unmount cameras. See
/// [`mount_camera`](crate::VirtualRobot::mount_camera).
#[must_use]
pub fn srv_cameras(sys_id: u32) -> String {
    srv(sys_id, "cameras")
}

/// `srv/turret`: joint limits and rates, the laser's range, the stereo pair.
/// Turret only. See [`configure_turret`](crate::VirtualRobot::configure_turret).
#[must_use]
pub fn srv_turret(sys_id: u32) -> String {
    srv(sys_id, "turret")
}

/// `srv/forklift`: drive, rear steering and mast. Forklift only. See
/// [`configure_forklift`](crate::VirtualRobot::configure_forklift).
#[must_use]
pub fn srv_forklift(sys_id: u32) -> String {
    srv(sys_id, "forklift")
}

/// `srv/palletrobot`: pulse band, virtual mecanum geometry, drive and pad.
/// PalletRobot only. See
/// [`configure_palletrobot`](crate::VirtualRobot::configure_palletrobot).
#[must_use]
pub fn srv_palletrobot(sys_id: u32) -> String {
    srv(sys_id, "palletrobot")
}

/// `srv/arm`: one key for the rail arm and the table arm. See
/// [`configure_arm`](crate::VirtualRobot::configure_arm).
#[must_use]
pub fn srv_arm(sys_id: u32) -> String {
    srv(sys_id, "arm")
}

/// `srv/lidars`: re-mount and retune a robot's lidars by name; every robot
/// serves it, and on Community it answers "no". See
/// [`configure_lidars`](crate::VirtualRobot::configure_lidars).
#[must_use]
pub fn srv_lidars(sys_id: u32) -> String {
    srv(sys_id, "lidars")
}

/// The scene's frame service: a payload-less GET whose answer is the scene's
/// active frame id. Scene scope. See
/// [`scene_frame`](crate::VirtualRobot::scene_frame).
#[must_use]
pub fn scene_srv_frame() -> String {
    format!("{SCENE_ROOT}/z/srv/frame")
}

/// A camera stream over iceoryx2, same host only:
/// `vrobots/<id>/i/cam/<camera>/<resolution>_<format>`. The stream segment is
/// [`CameraSpec::stream_segment`](crate::CameraSpec::stream_segment).
#[must_use]
pub fn camera(sys_id: u32, camera_name: &str, stream_segment: &str) -> String {
    format!("vrobots/{sys_id}/i/cam/{camera_name}/{stream_segment}")
}

/// A 3-D lidar's scans over iceoryx2, same host only:
/// `vrobots/<id>/i/lidar/<name>`. See
/// [`open_lidar_3d`](crate::VirtualRobot::open_lidar_3d).
#[must_use]
pub fn lidar_3d(sys_id: u32, lidar_name: &str) -> String {
    format!("vrobots/{sys_id}/i/lidar/{lidar_name}")
}

/// A 2-D lidar's scans over zenoh: `vrobots/<id>/z/lidar/<name>`. See
/// [`open_lidar_2d`](crate::VirtualRobot::open_lidar_2d).
#[must_use]
pub fn lidar_2d(sys_id: u32, lidar_name: &str) -> String {
    format!("vrobots/{sys_id}/z/lidar/{lidar_name}")
}

/// The leading-slash form of a key, as `header.metadata.topic_full_name`
/// carries it.
#[must_use]
pub fn full_name(topic: &str) -> String {
    if topic.starts_with('/') {
        topic.to_string()
    } else {
        format!("/{topic}")
    }
}

/// The `sys_id` in `vrobots/<id>/...`, or `None` when the key has no id there,
/// which includes the `manager` and `scene` scopes.
#[must_use]
pub fn sys_id_of(key: &str) -> Option<u32> {
    key.strip_prefix("vrobots/")?
        .split('/')
        .next()?
        .parse()
        .ok()
}

/// Every named wire key for one robot, in the order and under the names that
/// Python's `vrsdk.topics(sys_id)` returns them.
///
/// ```
/// let keys = vrobots_sdk::topics::all(1);
/// assert_eq!(keys[0], ("state", "vrobots/1/z/state".to_string()));
/// assert!(keys.contains(&("scene_frame", "vrobots/scene/z/srv/frame".to_string())));
/// ```
#[must_use]
pub fn all(sys_id: u32) -> Vec<(&'static str, String)> {
    vec![
        ("state", state(sys_id)),
        ("command", command(sys_id)),
        ("estimate", estimate(sys_id)),
        ("srv_cameras", srv_cameras(sys_id)),
        ("srv_activate", srv_activate(sys_id)),
        ("srv_reset", srv_reset(sys_id)),
        ("srv_params", srv_params(sys_id)),
        ("srv_skin", srv_skin(sys_id)),
        ("srv_sensors", srv_sensors(sys_id)),
        ("srv_frames", srv_frames(sys_id)),
        ("srv_drive", srv_drive(sys_id)),
        ("srv_rotors", srv_rotors(sys_id)),
        ("srv_msd", srv_msd(sys_id)),
        ("srv_cartpole", srv_cartpole(sys_id)),
        ("srv_turret", srv_turret(sys_id)),
        ("srv_forklift", srv_forklift(sys_id)),
        ("srv_palletrobot", srv_palletrobot(sys_id)),
        ("srv_arm", srv_arm(sys_id)),
        ("srv_lidars", srv_lidars(sys_id)),
        ("frames", frames(sys_id)),
        ("properties", properties(sys_id)),
        ("safety_events", safety_events(sys_id)),
        ("manager_create", manager_srv_create()),
        ("manager_delete", manager_srv_delete()),
        ("scene_frame", scene_srv_frame()),
    ]
}
