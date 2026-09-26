//! Connecting without a simulator, the library defaults behind every request
//! type, and the thread-safety markers. The one test that opens a network
//! session attaches to a `sys_id` no scene uses, so it fails the same way
//! whether or not a simulator happens to be running.

use std::time::{Duration, Instant};

use vrobots_sdk::{
    Axes, CameraOptions, CameraStream, CartPoleConfig, CmdArgs, ConnectOptions, DriveConfig, Frame,
    FramePixels, GpsNoise, GpsQuality, ImuNoise, MsdConfig, PhysicalParams, RobotType, RotorSpec,
    SensorConfig, SetpointStream, VirtualRobot, VrError, DEFAULT_CLIENT_NAME,
    DEFAULT_COORD_FRAME_ID, DEFAULT_SRC_ID,
};

/// Far above any id a scene hands out, so no simulator answers for it.
const NOBODY: u32 = 4_000_000_000;

#[test]
fn connecting_to_a_robot_nobody_publishes_times_out() {
    let probe = Duration::from_millis(800);
    let options = ConnectOptions::default()
        .with_connect_timeout(Duration::from_secs(5))
        .with_probe_timeout(probe);
    let started = Instant::now();
    let err = VirtualRobot::connect_with(RobotType::Multirotor, Some(NOBODY), options)
        .expect_err("no simulator publishes this id");
    let waited = started.elapsed();
    // The handle was built, never connected, and dropped inside `connect_with`.
    assert!(matches!(err, VrError::Timeout(_)), "{err:?}");
    assert!(err.detail().contains(&NOBODY.to_string()), "{err}");
    assert!(
        waited >= probe,
        "gave up after {waited:?}, before the probe budget"
    );
    assert!(waited < Duration::from_secs(30), "took {waited:?}");
}

#[test]
fn bad_options_are_refused_before_any_network_access() {
    for (options, needle) in [
        (ConnectOptions::default().with_src_id(0), "src_id"),
        (
            ConnectOptions::default().with_probe_timeout(Duration::ZERO),
            "probe_timeout",
        ),
        (
            ConnectOptions::default().with_client_name("bad\0name"),
            "client name",
        ),
    ] {
        let started = Instant::now();
        let err =
            VirtualRobot::connect_with(RobotType::Truck, Some(0), options).expect_err("refused");
        assert!(matches!(err, VrError::InvalidArgument(_)), "{err:?}");
        assert!(err.detail().contains(needle), "{needle:?} not in {err}");
        assert!(started.elapsed() < Duration::from_secs(1));
    }
}

#[test]
fn connect_options_start_from_the_library_defaults() {
    let o = ConnectOptions::default();
    assert_eq!(o.src_id, DEFAULT_SRC_ID);
    assert_eq!(o.router_endpoint, None);
    assert_eq!(o.connect_timeout, Duration::from_secs(5));
    assert_eq!(o.probe_timeout, Duration::from_secs(15));
    assert_eq!(o.service_timeout, Duration::from_secs(8));
    assert_eq!(o.camera_timeout, Duration::from_secs(5));
    assert_eq!(o.robot_name, None);
    assert_eq!(o.client_name, DEFAULT_CLIENT_NAME);
    assert!(o.start_active && o.activate_after_create);
    assert_eq!(o.coord_frame_id, DEFAULT_COORD_FRAME_ID);
    assert_eq!(o.axis_convention, Axes::UNITY);

    let o = o
        .with_router("tcp/192.168.1.10:7447")
        .with_src_id(200)
        .with_frame("frd", Axes::FRD)
        .with_robot_name("mine")
        .with_start_active(false)
        .with_activate_after_create(false);
    assert_eq!(o.router_endpoint.as_deref(), Some("tcp/192.168.1.10:7447"));
    assert_eq!(
        (o.src_id, o.coord_frame_id.as_str(), o.axis_convention),
        (200, "frd", Axes::FRD)
    );
    assert_eq!(o.robot_name.as_deref(), Some("mine"));
    assert!(!o.start_active && !o.activate_after_create);
}

#[test]
fn robot_types_round_trip_their_catalog_keys() {
    for (kind, key, sandbox) in [
        (RobotType::Multirotor, "multirotor", true),
        (RobotType::Truck, "truck", true),
        (RobotType::Msd, "msd", true),
        (RobotType::CartPole, "cartpole", false),
        (RobotType::HalfDrone, "halfdrone", false),
        (RobotType::GlobalHawk, "globalhawk", false),
    ] {
        assert_eq!(kind.catalog_key(), key);
        assert_eq!(RobotType::from_catalog_key(key), Some(kind));
        assert_eq!(kind.is_in_sandbox_catalog(), sandbox);
    }
    assert_eq!(RobotType::from_catalog_key("Car"), Some(RobotType::Truck));
    assert_eq!(RobotType::from_catalog_key("submarine"), None);
}

#[test]
fn request_types_start_from_the_library_defaults() {
    let imu = ImuNoise::ideal();
    assert_eq!(
        imu.scale_factor, [1.0; 3],
        "an unset gain would be a dead sensor"
    );
    assert_eq!(imu.white_std, [0.0; 3]);
    assert_eq!(ImuNoise::default(), imu);

    let quality = GpsQuality::default();
    assert_eq!((quality.eph, quality.epv, quality.fix_type), (1.5, 3.0, 3));
    assert_eq!(GpsNoise::default().position_std, [0.0; 3]);

    let rotor = RotorSpec::default();
    assert_eq!((rotor.pwm_min_us, rotor.pwm_max_us), (1100, 2000));
    assert_eq!(rotor.spin_dir, 0.0);
    assert!(rotor.thrust_a != 0.0 || rotor.thrust_b != 0.0 || rotor.thrust_c != 0.0);

    let camera = CameraOptions::default();
    assert_eq!((camera.fx, camera.fy), (600.0, 600.0));
    assert_eq!((camera.near_clip, camera.far_clip), (0.5, 1000.0));

    assert!(PhysicalParams::default().is_empty());
    assert!(!PhysicalParams::default().with_mass(1.6).is_empty());
    assert!(SensorConfig::default().is_empty());
    assert!(DriveConfig::default().is_empty());
    assert!(MsdConfig::default().is_empty());
    assert!(CartPoleConfig::default().is_empty());
    assert_eq!(CmdArgs::default().int_arr, Vec::<i32>::new());
    assert_eq!(CmdArgs::ints(&[1500, 1600]).int_arr, vec![1500, 1600]);
}

#[test]
fn handles_may_be_shared_between_threads() {
    fn assert_send_sync<T: Send + Sync>() {}
    assert_send_sync::<VirtualRobot>();
    assert_send_sync::<CameraStream>();
    assert_send_sync::<SetpointStream>();
    assert_send_sync::<Frame>();
    assert_send_sync::<FramePixels>();
    assert_send_sync::<vrobots_sdk::State>();
}
