//! Lidar scans built by hand and turned into points by the C library: the beam
//! layout, `+inf` as no return, the lidar's own frame, and one beam agreeing
//! with the whole cloud. No simulator needed.

use std::f64::consts::FRAC_PI_2;

use vrobots_sdk::{device, LidarConfigEntry, LidarMount, LidarScan};

/// One ring of four beams, 90 degrees apart, in FLU, as a 2-D lidar reports.
fn ring_of_four() -> LidarScan {
    let mut scan = LidarScan::default();
    scan.coord_frame_id = "flu".to_string();
    scan.params.name = "lidar_front_left".to_string();
    scan.params.rings = 1;
    scan.params.columns = 4;
    scan.params.azimuth_step = FRAC_PI_2;
    scan.params.mount_position = [0.35, 0.30, 0.20];
    scan.params.mount_euler_rad = [0.0, 0.0, 0.785];
    scan.ranges = vec![2.0, f32::INFINITY, 1.0, 0.5];
    scan
}

/// Two rings of three beams, tilted, so elevation and azimuth both matter.
fn two_rings() -> LidarScan {
    let mut scan = LidarScan::default();
    scan.coord_frame_id = "flu".to_string();
    scan.params.rings = 2;
    scan.params.columns = 3;
    scan.params.azimuth_start = -0.3;
    scan.params.azimuth_step = 0.3;
    scan.params.elevation_start = -0.1;
    scan.params.elevation_step = 0.2;
    scan.ranges = vec![5.0, 6.0, f32::INFINITY, 7.0, 8.0, 9.0];
    scan
}

fn close(a: [f32; 3], b: [f32; 3]) -> bool {
    a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-6)
}

#[test]
fn the_layout_helpers_index_the_ranges() {
    let scan = ring_of_four();
    assert_eq!(scan.range(0, 0), Some(2.0));
    assert_eq!(
        scan.range(0, 1),
        Some(f32::INFINITY),
        "no return is a value"
    );
    assert_eq!(scan.range(0, 4), None, "past the last column");
    assert_eq!(scan.range(1, 0), None, "past the last ring");
    assert_eq!(scan.azimuth(1), FRAC_PI_2);
    assert_eq!(scan.elevation(0), 0.0);

    let scan = two_rings();
    assert_eq!(scan.range(1, 2), Some(9.0));
    assert!((scan.elevation(1) - 0.1).abs() < 1e-12);
    assert!((scan.azimuth(2) - 0.3).abs() < 1e-12);
}

#[test]
fn points_follow_the_beam_layout_in_flu() {
    let scan = ring_of_four();
    let xyz = scan.points();
    assert_eq!(xyz.len(), 12, "three floats per beam, organised");
    assert_eq!(&xyz[..3], &[2.0, 0.0, 0.0], "column 0 is forward");
    assert!(xyz[3..6].iter().all(|v| v.is_nan()), "no return is NaN");
    let behind: [f32; 3] = xyz[6..9].try_into().expect("three");
    assert!(close(behind, [-1.0, 0.0, 0.0]), "{behind:?}");
    // Counter-clockwise from above: column 3 is to the right, -y in FLU.
    let right: [f32; 3] = xyz[9..12].try_into().expect("three");
    assert!(close(right, [0.0, -0.5, 0.0]), "{right:?}");
}

#[test]
fn one_beam_is_exactly_its_slice_of_the_cloud() {
    for scan in [ring_of_four(), two_rings()] {
        let cloud = scan.points();
        for ring in 0..scan.params.rings {
            for column in 0..scan.params.columns {
                let i = usize::try_from(ring * scan.params.columns + column).expect("small");
                let slice = &cloud[3 * i..3 * i + 3];
                match scan.point(ring, column) {
                    Some(p) => {
                        let bits = |v: &[f32]| v.iter().map(|f| f.to_bits()).collect::<Vec<_>>();
                        assert_eq!(bits(&p), bits(slice), "beam ({ring}, {column})");
                    }
                    None => assert!(slice.iter().all(|v| v.is_nan()), "beam ({ring}, {column})"),
                }
            }
        }
    }
    assert_eq!(ring_of_four().point(0, 1), None, "no return");
    assert_eq!(ring_of_four().point(0, 9), None, "outside the layout");
}

#[test]
fn a_frame_override_relabels_the_points_never_the_ranges() {
    let mut scan = ring_of_four();
    scan.ranges[1] = 3.0;
    let flu = scan.point(0, 1).expect("a return");
    assert!(close(flu, [0.0, 3.0, 0.0]), "{flu:?}");

    // The same beam with the lidar overridden to FRD: y and z flip sign.
    scan.coord_frame_id = "frd".to_string();
    let frd = scan.point(0, 1).expect("a return");
    assert!(close(frd, [0.0, -3.0, 0.0]), "{frd:?}");
    assert_eq!(scan.range(0, 1), Some(3.0));
    assert_eq!(device::lidar("lidar_front_left"), "lidar/lidar_front_left");
}

#[test]
fn an_empty_scan_has_no_points() {
    assert!(LidarScan::default().points().is_empty());
    assert_eq!(LidarScan::default().point(0, 0), None);
}

#[test]
fn a_mount_read_off_a_scan_goes_back_unchanged() {
    let scan = ring_of_four();
    let mount = scan.mount();
    assert_eq!(
        mount,
        LidarMount::new([0.35, 0.30, 0.20], [0.0, 0.0, 0.785])
    );
    let entry = LidarConfigEntry::new("lidar_front_left").with_lidar_mount(mount);
    assert_eq!(entry.mount, Some(mount));
    assert_eq!(
        LidarConfigEntry::new("lidar_front_left")
            .with_mount(mount.position, mount.euler_rad)
            .mount,
        Some(mount)
    );
    assert!(!entry.is_empty());
}
