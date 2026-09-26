//! Topic-name builders against the table in the book's appendix A and the
//! dictionary Python's `vrsdk.topics(1)` returns. Pure string building.

use vrobots_sdk::topics;

/// `vrsdk.topics(1)` in Python, key for key and in the same order.
const PYTHON_TOPICS_1: [(&str, &str); 18] = [
    ("state", "vrobots/1/z/state"),
    ("command", "vrobots/1/z/cmd"),
    ("estimate", "vrobots/1/z/estimate"),
    ("srv_cameras", "vrobots/1/z/srv/cameras"),
    ("srv_activate", "vrobots/1/z/srv/activate"),
    ("srv_reset", "vrobots/1/z/srv/reset"),
    ("srv_params", "vrobots/1/z/srv/params"),
    ("srv_skin", "vrobots/1/z/srv/skin"),
    ("srv_sensors", "vrobots/1/z/srv/sensors"),
    ("srv_frames", "vrobots/1/z/srv/frames"),
    ("srv_drive", "vrobots/1/z/srv/drive"),
    ("srv_rotors", "vrobots/1/z/srv/rotors"),
    ("srv_msd", "vrobots/1/z/srv/msd"),
    ("srv_cartpole", "vrobots/1/z/srv/cartpole"),
    ("frames", "vrobots/1/z/frames"),
    ("manager_create", "vrobots/manager/z/srv/create"),
    ("manager_delete", "vrobots/manager/z/srv/delete"),
    ("scene_frame", "vrobots/scene/z/srv/frame"),
];

#[test]
fn all_matches_python_topics_for_sys_id_1() {
    let ours = topics::all(1);
    assert_eq!(ours.len(), PYTHON_TOPICS_1.len());
    for ((name, key), (want_name, want_key)) in ours.iter().zip(PYTHON_TOPICS_1) {
        assert_eq!((*name, key.as_str()), (want_name, want_key));
    }
}

#[test]
fn every_builder_matches_appendix_a() {
    let id = 7;
    assert_eq!(topics::root(id), "vrobots/7");
    assert_eq!(topics::state(id), "vrobots/7/z/state");
    assert_eq!(topics::command(id), "vrobots/7/z/cmd");
    assert_eq!(topics::frames(id), "vrobots/7/z/frames");
    assert_eq!(topics::estimate(id), "vrobots/7/z/estimate");
    assert_eq!(topics::srv(id, "reset"), "vrobots/7/z/srv/reset");
    assert_eq!(topics::srv_activate(id), "vrobots/7/z/srv/activate");
    assert_eq!(topics::srv_reset(id), "vrobots/7/z/srv/reset");
    assert_eq!(topics::srv_params(id), "vrobots/7/z/srv/params");
    assert_eq!(topics::srv_skin(id), "vrobots/7/z/srv/skin");
    assert_eq!(topics::srv_sensors(id), "vrobots/7/z/srv/sensors");
    assert_eq!(topics::srv_frames(id), "vrobots/7/z/srv/frames");
    assert_eq!(topics::srv_drive(id), "vrobots/7/z/srv/drive");
    assert_eq!(topics::srv_rotors(id), "vrobots/7/z/srv/rotors");
    assert_eq!(topics::srv_msd(id), "vrobots/7/z/srv/msd");
    assert_eq!(topics::srv_cartpole(id), "vrobots/7/z/srv/cartpole");
    assert_eq!(topics::srv_cameras(id), "vrobots/7/z/srv/cameras");
    assert_eq!(
        topics::camera(1, "front_left", "720p_rgba8"),
        "vrobots/1/i/cam/front_left/720p_rgba8"
    );
    assert_eq!(topics::manager_srv_create(), "vrobots/manager/z/srv/create");
    assert_eq!(topics::manager_srv_delete(), "vrobots/manager/z/srv/delete");
    assert_eq!(topics::scene_srv_frame(), "vrobots/scene/z/srv/frame");
    assert_eq!(topics::MANAGER_ROOT, "vrobots/manager");
    assert_eq!(topics::SCENE_ROOT, "vrobots/scene");
    assert_eq!(topics::ALL, "vrobots/**");
    assert_eq!(topics::ALL_STATES, "vrobots/*/z/state");
}

#[test]
fn keys_parse_back() {
    assert_eq!(topics::sys_id_of("vrobots/42/z/state"), Some(42));
    assert_eq!(
        topics::sys_id_of("vrobots/1/i/cam/front_left/720p_rgba8"),
        Some(1)
    );
    assert_eq!(topics::sys_id_of("vrobots/manager/z/srv/create"), None);
    assert_eq!(topics::sys_id_of("vrobots/scene/z/srv/frame"), None);
    assert_eq!(topics::sys_id_of("other/1/z/state"), None);
    assert_eq!(topics::full_name("vrobots/1/z/state"), "/vrobots/1/z/state");
    assert_eq!(
        topics::full_name("/vrobots/1/z/state"),
        "/vrobots/1/z/state"
    );
}

#[test]
fn a_camera_spec_names_its_stream() {
    use vrobots_sdk::{CameraSpec, PixelFormat, Resolution};

    let spec = CameraSpec::parse("front_right", "720p", "rgba8").expect("valid spec");
    assert_eq!(spec.name, "front_right");
    assert_eq!(spec.resolution, Resolution::P720);
    assert_eq!(spec.format, PixelFormat::Rgba8);
    assert_eq!(spec.stream_segment(), "720p_rgba8");
    assert_eq!(
        spec.service_name(3),
        "vrobots/3/i/cam/front_right/720p_rgba8"
    );
    assert_eq!(spec.data_size(), 1280 * 720 * 4);
    assert_eq!(spec.to_string(), "front_right 720p_rgba8");

    for (name, resolution, format) in [
        ("", "720p", "rgba8"),
        ("front/left", "720p", "rgba8"),
        ("front", "4k", "rgba8"),
        ("front", "720p", "bgr8"),
    ] {
        let err = CameraSpec::parse(name, resolution, format).expect_err("refused");
        assert!(
            matches!(err, vrobots_sdk::VrError::InvalidArgument(_)),
            "{err:?}"
        );
    }
}
