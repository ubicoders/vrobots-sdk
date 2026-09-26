//! Rotation conversions against the numbers the book's rotation chapter
//! (`ch02-concepts/08-rotation-conversions.md`) and the rotations example
//! print. Pure math through the C library; no simulator needed.

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2};

use vrobots_sdk::rotations::{self, EulerOrder, GIMBAL_LOCK_COS, IDENTITY_QUAT, IDENTITY_ROTMAT};
use vrobots_sdk::{sys, Axes, AxisBasis, FrameDef, FrameTransform, VrError};

const EPS: f64 = 1e-12;

const ORDERS: [EulerOrder; 6] = [
    EulerOrder::Xyz,
    EulerOrder::Xzy,
    EulerOrder::Yxz,
    EulerOrder::Yzx,
    EulerOrder::Zxy,
    EulerOrder::Zyx,
];

fn close<const N: usize>(a: [f64; N], b: [f64; N]) -> bool {
    a.iter().zip(b).all(|(x, y)| (x - y).abs() < EPS)
}

fn close_m(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> bool {
    (0..3).all(|i| close(a[i], b[i]))
}

#[test]
fn euler_orders_carry_the_wire_values_and_names_of_the_table() {
    let table = [
        (EulerOrder::Xyz, 1, "xyz"),
        (EulerOrder::Xzy, 2, "xzy"),
        (EulerOrder::Yxz, 3, "yxz"),
        (EulerOrder::Yzx, 4, "yzx"),
        (EulerOrder::Zxy, 5, "zxy"),
        (EulerOrder::Zyx, 6, "zyx"),
    ];
    for (order, wire, name) in table {
        assert_eq!(order.to_wire(), wire);
        assert_eq!(order.name(), name);
        assert_eq!(EulerOrder::from_wire(wire), Some(order));
        // The wire tag at the crate root names the same order.
        assert_eq!(vrobots_sdk::EulerOrder(wire).name(), name);
    }
    assert_eq!(EulerOrder::from_wire(0), None);
    assert_eq!(vrobots_sdk::EulerOrder::UNSPECIFIED.name(), "");
}

#[test]
fn the_identities_equal_the_library_ones() {
    let mut q = [9.0; 4];
    let mut r = [9.0; 9];
    // SAFETY: `q` is writable for 4 doubles and `r` for 9, as the two identity
    // functions require.
    unsafe {
        VrError::check(sys::vrsdk_identity_quat(q.as_mut_ptr())).unwrap();
        VrError::check(sys::vrsdk_identity_rotmat(r.as_mut_ptr())).unwrap();
    }
    assert_eq!(q, IDENTITY_QUAT);
    assert_eq!(
        r,
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "row-major identity"
    );
    assert_eq!(IDENTITY_ROTMAT[1], [0.0, 1.0, 0.0]);
    assert!((GIMBAL_LOCK_COS - f64::EPSILON.sqrt()).abs() < 1e-24);
}

#[test]
fn every_order_round_trips_away_from_the_pole() {
    let euler = [0.3, -0.4, 1.1];
    for order in ORDERS {
        let q = rotations::euler_to_quat(euler, order);
        assert!(
            close(rotations::quat_to_euler(q, order), euler),
            "{order:?}"
        );
        let r = rotations::euler_to_rotmat(euler, order);
        assert!(
            close(rotations::rotmat_to_euler(r, order), euler),
            "{order:?}"
        );
        assert!(close_m(rotations::quat_to_rotmat(q), r), "{order:?}");
        let back = rotations::rotmat_to_quat(r);
        assert!(
            back[3] >= 0.0,
            "the representative has a non-negative scalar"
        );
        assert!(
            close(back, q) || close(back, q.map(|c| -c)),
            "{order:?}: {back:?} vs {q:?}"
        );
    }
}

#[test]
fn the_same_triple_under_two_orders_is_two_rotations() {
    let euler = [0.3, -0.4, 1.1];
    let zyx = rotations::euler_to_quat(euler, EulerOrder::Zyx);
    let xyz = rotations::euler_to_quat(euler, EulerOrder::Xyz);
    assert!(!close(zyx, xyz));
}

#[test]
fn quaternion_primitives_behave() {
    // Yaw +90 degrees in ZYX: body x lands on world y.
    let yaw = rotations::euler_to_quat([0.0, 0.0, FRAC_PI_2], EulerOrder::Zyx);
    assert!(close(yaw, [0.0, 0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2]));
    assert!(close(
        rotations::rotate_vec3(yaw, [1.0, 0.0, 0.0]),
        [0.0, 1.0, 0.0]
    ));

    let product = rotations::quat_multiply(yaw, rotations::quat_conjugate(yaw));
    assert!(close(product, IDENTITY_QUAT));
    // Two quarter turns are a half turn.
    let half = rotations::quat_multiply(yaw, yaw);
    assert!(close(
        rotations::rotate_vec3(half, [1.0, 0.0, 0.0]),
        [-1.0, 0.0, 0.0]
    ));

    // A zero-length quaternion normalizes to the identity rather than to NaN.
    assert_eq!(rotations::quat_normalize([0.0; 4]), IDENTITY_QUAT);
    assert!(close(
        rotations::quat_normalize([0.0, 0.0, 0.0, 2.0]),
        IDENTITY_QUAT
    ));
}

#[test]
fn matrix_primitives_behave() {
    let r = rotations::euler_to_rotmat([0.2, 0.1, -0.7], EulerOrder::Zyx);
    let rt = rotations::rotmat_transpose(r);
    assert!(close_m(rotations::rotmat_multiply(r, rt), IDENTITY_ROTMAT));
    assert!((rotations::rotmat_det(r) - 1.0).abs() < EPS);
    let reflect = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]];
    assert!((rotations::rotmat_det(reflect) + 1.0).abs() < EPS);
    assert_eq!(rotations::rotmat_abs(reflect), IDENTITY_ROTMAT);
    assert_eq!(
        rotations::rotmat_apply(reflect, [1.0, 2.0, 3.0]),
        [1.0, 2.0, -3.0]
    );
}

#[test]
fn gimbal_lock_pins_yaw_and_rebuilds_the_same_rotation() {
    // The book: in (0, 90, 40) deg, out (-40, 90, 0) deg.
    let locked = [0.0_f64, 90.0, 40.0].map(f64::to_radians);
    let q = rotations::euler_to_quat(locked, EulerOrder::Zyx);
    let out = rotations::quat_to_euler(q, EulerOrder::Zyx).map(f64::to_degrees);
    assert!((out[0] + 40.0).abs() < 1e-9, "roll {}", out[0]);
    assert!((out[1] - 90.0).abs() < 1e-9, "pitch {}", out[1]);
    assert!(out[2].abs() < 1e-9, "yaw {}", out[2]);
    let rebuilt = rotations::euler_to_rotmat(out.map(f64::to_radians), EulerOrder::Zyx);
    assert!(close_m(rebuilt, rotations::quat_to_rotmat(q)));
}

#[test]
fn the_four_built_in_frames_match_the_table() {
    // (basis, handedness, order, north, east, down)
    let table = [
        (
            AxisBasis::unity(),
            false,
            EulerOrder::Zxy,
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ),
        (
            AxisBasis::frd(),
            true,
            EulerOrder::Zyx,
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ),
        (
            AxisBasis::fru(),
            false,
            EulerOrder::Zyx,
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, -1.0],
        ),
        (
            AxisBasis::cv(),
            true,
            EulerOrder::Zyx,
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ),
    ];
    for (basis, right_handed, order, north, east, down) in table {
        assert_eq!(basis.is_right_handed(), right_handed, "{basis:?}");
        assert_eq!(basis.det(), if right_handed { -1.0 } else { 1.0 });
        assert_eq!(basis.euler_order, order);
        assert_eq!(basis.north(), north, "{basis:?}");
        assert_eq!(basis.east(), east, "{basis:?}");
        assert_eq!(basis.down(), down, "{basis:?}");
    }
    assert_eq!(AxisBasis::unity().rows, IDENTITY_ROTMAT);
    assert_eq!(AxisBasis::from_frame_id("fru"), Some(AxisBasis::fru()));
    assert_eq!(AxisBasis::from_axes(Axes::FRD), Some(AxisBasis::frd()));
    assert_eq!(AxisBasis::from_axes(Axes::UNSPECIFIED), None);
    assert_eq!(AxisBasis::from_frame_id("ned"), None);
    assert_eq!(
        AxisBasis::new(AxisBasis::cv().rows, EulerOrder::Zyx),
        AxisBasis::cv()
    );
}

#[test]
fn a_frame_definition_becomes_a_basis_only_with_an_order() {
    let mut def = FrameDef::default();
    def.id = "fru".to_string();
    def.rows = AxisBasis::fru().rows;
    // No order and no tag to default it from: nothing trustworthy to report in.
    assert_eq!(AxisBasis::from_frame_def(&def), None);
    def.euler_order = vrobots_sdk::EulerOrder::ZYX;
    assert_eq!(AxisBasis::from_frame_def(&def), Some(AxisBasis::fru()));
    // The tag fills in a missing order.
    def.euler_order = vrobots_sdk::EulerOrder::UNSPECIFIED;
    def.axis_convention = Axes::UNITY;
    def.rows = IDENTITY_ROTMAT;
    let basis = AxisBasis::from_frame_def(&def).expect("unity's default order");
    assert_eq!(basis.euler_order, EulerOrder::Zxy);
}

#[test]
fn polar_and_axial_differ_exactly_across_a_handedness_flip() {
    // The truck's fru to frd: M = diag(1, 1, -1), det -1.
    let t = FrameTransform::between(AxisBasis::fru(), AxisBasis::frd());
    assert_eq!(t.det, -1.0);
    assert!(t.flips_handedness());
    assert_eq!(
        t.matrix,
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]
    );
    let v = [0.1, -0.2, 0.3];
    assert_eq!(t.apply_vec3(v), [0.1, -0.2, -0.3]);
    assert_eq!(t.apply_axial_vec3(v), [-0.1, 0.2, 0.3]);
    assert_eq!(t.apply_inertia_vec3([0.1, 0.2, 0.3]), [0.1, 0.2, 0.3]);
    let back = t.inverse();
    assert_eq!(back.apply_vec3(t.apply_vec3(v)), v);
    assert_eq!(back.apply_axial_vec3(t.apply_axial_vec3(v)), v);

    // The book's two-line trap: a roll rate of [0, 0, 1] in Unity axes.
    let u = FrameTransform::between(AxisBasis::unity(), AxisBasis::frd());
    assert_eq!(u.apply_axial_vec3([0.0, 0.0, 1.0]), [-1.0, 0.0, 0.0]);
    assert_eq!(u.apply_vec3([0.0, 0.0, 1.0]), [1.0, 0.0, 0.0]);

    let same = FrameTransform::between(AxisBasis::frd(), AxisBasis::cv());
    assert!(!same.flips_handedness());
    assert!(!FrameTransform::identity().flips_handedness());
    assert_eq!(FrameTransform::identity().matrix, IDENTITY_ROTMAT);
}

#[test]
fn attitudes_convert_as_the_example_prints() {
    // +90 degrees about Unity's up axis is yaw +90 in frd, and the quaternion
    // conversion is [x, y, z, w] -> [-z, -x, y, w].
    let yaw90_unity = rotations::euler_to_quat([0.0, FRAC_PI_2, 0.0], EulerOrder::Zxy);
    let yaw90_frd = rotations::convert_quat(yaw90_unity, Axes::UNITY, Axes::FRD).unwrap();
    let [x, y, z, w] = yaw90_unity;
    assert!(close(yaw90_frd, [-z, -x, y, w]));
    let yaw = rotations::quat_to_euler(yaw90_frd, EulerOrder::Zyx)[2].to_degrees();
    assert!((yaw - 90.0).abs() < 1e-9, "{yaw}");

    let t = FrameTransform::between(AxisBasis::unity(), AxisBasis::frd());
    assert!(close(t.apply_quat(yaw90_unity), yaw90_frd));
    let r = rotations::quat_to_rotmat(yaw90_unity);
    assert!(close_m(
        t.apply_rotmat(r),
        rotations::quat_to_rotmat(yaw90_frd)
    ));
}

#[test]
fn tag_keyed_conversions_match_the_example_and_refuse_unspecified() {
    let up = rotations::convert_vec3([0.0, 10.0, 0.0], Axes::UNITY, Axes::FRD).unwrap();
    assert_eq!(up, [0.0, 0.0, -10.0]);
    let roll = rotations::convert_axial_vec3([0.0, 0.0, 1.0], Axes::UNITY, Axes::FRD).unwrap();
    assert_eq!(roll, [-1.0, 0.0, 0.0]);
    let moi = rotations::convert_inertia_vec3([0.1, 0.2, 0.3], Axes::UNITY, Axes::FRD).unwrap();
    assert_eq!(moi, [0.3, 0.1, 0.2]);
    let r = rotations::convert_rotmat(IDENTITY_ROTMAT, Axes::UNITY, Axes::CV).unwrap();
    assert_eq!(r, IDENTITY_ROTMAT);
    let euler = rotations::convert_euler([0.0, 0.0, 0.0], Axes::UNITY, Axes::FRD).unwrap();
    assert!(close(euler, [0.0; 3]));

    assert_eq!(
        rotations::default_euler_order(Axes::UNITY),
        Some(EulerOrder::Zxy)
    );
    assert_eq!(
        rotations::default_euler_order(Axes::FRD),
        Some(EulerOrder::Zyx)
    );
    assert_eq!(
        rotations::default_euler_order(Axes::CV),
        Some(EulerOrder::Zyx)
    );
    assert_eq!(rotations::default_euler_order(Axes::UNSPECIFIED), None);

    let err = rotations::convert_vec3([1.0, 2.0, 3.0], Axes::UNSPECIFIED, Axes::FRD)
        .expect_err("the default fru frame has no tag");
    assert!(matches!(err, VrError::InvalidArgument(_)), "{err:?}");
    assert!(!err.detail().is_empty());
}
