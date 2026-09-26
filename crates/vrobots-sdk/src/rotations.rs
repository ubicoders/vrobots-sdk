//! Rotations and frame conversions: pure math, no robot and no simulator.
//!
//! Storage never changes: a quaternion is `[x, y, z, w]` (Hamilton, unit,
//! body-to-world), a matrix is row-major 3x3 so `r[1][2]` is row 1, column 2,
//! and an Euler triple is always `[about x, about y, about z]` in radians. The
//! [`EulerOrder`] argument says in which sequence the three intrinsic rotations
//! are applied; it never reorders the array. [`AxisBasis`] and
//! [`FrameTransform`] re-express vectors and attitudes between conventions, and
//! which `apply_*` is right depends on what the vector is (polar, axial,
//! inertia or orientation).
//!
//! Every function forwards to the C library, so Rust, C++ and Python get one
//! answer from one implementation. The math is total: no `NaN` out of finite
//! input and a documented answer for every degenerate case. The functions that
//! return [`VrResult`] fail only on a caller error, a tag that names no frame.
//!
//! ```
//! use vrobots_sdk::rotations::{self, EulerOrder};
//!
//! // Pitch exactly at the pole: the angle about z is pinned to zero and the
//! // whole determined combination goes into roll, as the simulator does.
//! let locked = [0.0_f64, 90.0, 40.0].map(f64::to_radians);
//! let q = rotations::euler_to_quat(locked, EulerOrder::Zyx);
//! let [roll, pitch, yaw] = rotations::quat_to_euler(q, EulerOrder::Zyx);
//! assert!((roll.to_degrees() + 40.0).abs() < 1e-9);
//! assert!((pitch.to_degrees() - 90.0).abs() < 1e-9);
//! assert!(yaw.abs() < 1e-9);
//! ```

use vrobots_sdk_sys as sys;

use crate::error::VrResult;
use crate::ffi::{self, expect_ok, flatten, unflatten};
use crate::state::{self, Axes, FrameDef};

/// The identity rotation as a quaternion, `[0, 0, 0, 1]` in `[x, y, z, w]`
/// order. Equal to the library's `vrsdk_identity_quat`.
pub const IDENTITY_QUAT: [f64; 4] = [0.0, 0.0, 0.0, 1.0];

/// The identity rotation as a row-major 3x3 matrix. Equal to the library's
/// `vrsdk_identity_rotmat`.
pub const IDENTITY_ROTMAT: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// How near the pole [`rotmat_to_euler`] switches to its gimbal-lock branch,
/// measured as `cos(middle angle)`: about 1.49e-8 rad, or 8.5e-7 degrees.
///
/// The square root of the `f64` machine epsilon, the same rule the simulator
/// applies to `f32`. It is where the errors of the two branches cross, and
/// neither branch is ever worse than about 2e-8 in the matrix it rebuilds. The
/// value the C library uses; Python exposes it as `rotations.GIMBAL_LOCK_COS`.
pub const GIMBAL_LOCK_COS: f64 = 1.490_116_119_384_765_6e-8;

/// The sequence three intrinsic Tait-Bryan rotations are applied in.
///
/// The name lists the application order; the angles are always stored
/// `[about x, about y, about z]`. So [`Zyx`](Self::Zyx) reads `euler[2]` (yaw)
/// first, then `euler[1]`, then `euler[0]`. There is no `[yaw, pitch, roll]`
/// layout anywhere in this SDK.
///
/// A closed enum, because a conversion handed a value that names no order has
/// nothing correct to do. The wire tag, which can be "unspecified", is
/// [`vrobots_sdk::EulerOrder`](crate::EulerOrder); cross with
/// [`from_wire`](Self::from_wire).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(i32)]
pub enum EulerOrder {
    /// About x, then the new y, then the newest z.
    Xyz = sys::VRSDK_EULER_XYZ,
    /// About x, then the new z, then the newest y.
    Xzy = sys::VRSDK_EULER_XZY,
    /// About y, then the new x, then the newest z.
    Yxz = sys::VRSDK_EULER_YXZ,
    /// About y, then the new z, then the newest x.
    Yzx = sys::VRSDK_EULER_YZX,
    /// About z, then the new x, then the newest y. Unity's native order, and
    /// what `"unity"` reports in.
    Zxy = sys::VRSDK_EULER_ZXY,
    /// About z, then the new y, then the newest x. The aerospace order, and
    /// what `"frd"`, `"fru"` and `"cv"` report in.
    Zyx = sys::VRSDK_EULER_ZYX,
}

impl EulerOrder {
    /// The order for a wire value, or `None` for `0` (unspecified) and anything
    /// the schema does not define.
    #[must_use]
    pub fn from_wire(value: i32) -> Option<EulerOrder> {
        match value {
            sys::VRSDK_EULER_XYZ => Some(EulerOrder::Xyz),
            sys::VRSDK_EULER_XZY => Some(EulerOrder::Xzy),
            sys::VRSDK_EULER_YXZ => Some(EulerOrder::Yxz),
            sys::VRSDK_EULER_YZX => Some(EulerOrder::Yzx),
            sys::VRSDK_EULER_ZXY => Some(EulerOrder::Zxy),
            sys::VRSDK_EULER_ZYX => Some(EulerOrder::Zyx),
            _ => None,
        }
    }

    /// The wire value, `1` to `6`.
    #[must_use]
    pub fn to_wire(self) -> i32 {
        self as i32
    }

    /// The three axis indices in application order: `Zyx` is `[2, 1, 0]`.
    #[must_use]
    pub fn axes(self) -> [u8; 3] {
        match self {
            EulerOrder::Xyz => [0, 1, 2],
            EulerOrder::Xzy => [0, 2, 1],
            EulerOrder::Yxz => [1, 0, 2],
            EulerOrder::Yzx => [1, 2, 0],
            EulerOrder::Zxy => [2, 0, 1],
            EulerOrder::Zyx => [2, 1, 0],
        }
    }

    /// The lowercase name: `"xyz"`, ..., `"zyx"`. The library's own
    /// `vrsdk_euler_order_name`.
    #[must_use]
    pub fn name(self) -> &'static str {
        state::EulerOrder(self.to_wire()).name()
    }
}

// ---------------------------------------------------------------------------
// quaternion primitives
// ---------------------------------------------------------------------------

/// The Hamilton product `a * b`: the rotation "b first, then a".
///
/// Not normalized: two unit quaternions give a unit quaternion, and rescaling
/// silently would hide an input that was never unit.
#[must_use]
pub fn quat_multiply(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let mut out = [0.0; 4];
    // SAFETY: `a` and `b` are readable for 4 doubles and `out` writable for 4,
    // as `vrsdk_quat_multiply` requires.
    let code = unsafe { sys::vrsdk_quat_multiply(a.as_ptr(), b.as_ptr(), out.as_mut_ptr()) };
    expect_ok(code, "vrsdk_quat_multiply");
    out
}

/// The conjugate: vector part negated, scalar kept. For a unit quaternion this
/// is the inverse rotation.
#[must_use]
pub fn quat_conjugate(q: [f64; 4]) -> [f64; 4] {
    let mut out = [0.0; 4];
    // SAFETY: `q` is readable and `out` writable for 4 doubles.
    let code = unsafe { sys::vrsdk_quat_conjugate(q.as_ptr(), out.as_mut_ptr()) };
    expect_ok(code, "vrsdk_quat_conjugate");
    out
}

/// Scale to unit length; a zero-length or non-finite input becomes
/// [`IDENTITY_QUAT`] rather than a `NaN`.
#[must_use]
pub fn quat_normalize(q: [f64; 4]) -> [f64; 4] {
    let mut out = [0.0; 4];
    // SAFETY: `q` is readable and `out` writable for 4 doubles.
    let code = unsafe { sys::vrsdk_quat_normalize(q.as_ptr(), out.as_mut_ptr()) };
    expect_ok(code, "vrsdk_quat_normalize");
    out
}

/// Rotate a vector by a quaternion: body components in, world components out.
/// `quat` is normalized first.
#[must_use]
pub fn rotate_vec3(quat: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let mut out = [0.0; 3];
    // SAFETY: `quat` is readable for 4 doubles, `v` for 3, and `out` writable
    // for 3, as `vrsdk_rotate_vec3` requires.
    let code = unsafe { sys::vrsdk_rotate_vec3(quat.as_ptr(), v.as_ptr(), out.as_mut_ptr()) };
    expect_ok(code, "vrsdk_rotate_vec3");
    out
}

// ---------------------------------------------------------------------------
// matrix primitives
// ---------------------------------------------------------------------------

/// The matrix product `a * b`.
#[must_use]
pub fn rotmat_multiply(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let (a, b) = (flatten(a), flatten(b));
    let mut out = [0.0; 9];
    // SAFETY: `a` and `b` are readable and `out` writable for 9 doubles.
    let code = unsafe { sys::vrsdk_rotmat_multiply(a.as_ptr(), b.as_ptr(), out.as_mut_ptr()) };
    expect_ok(code, "vrsdk_rotmat_multiply");
    unflatten(out)
}

/// The transpose, which for a rotation is also the inverse.
#[must_use]
pub fn rotmat_transpose(r: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let r = flatten(r);
    let mut out = [0.0; 9];
    // SAFETY: `r` is readable and `out` writable for 9 doubles.
    let code = unsafe { sys::vrsdk_rotmat_transpose(r.as_ptr(), out.as_mut_ptr()) };
    expect_ok(code, "vrsdk_rotmat_transpose");
    unflatten(out)
}

/// The determinant: `+1` for a rotation, `-1` for a rotation with a reflection.
#[must_use]
pub fn rotmat_det(r: [[f64; 3]; 3]) -> f64 {
    let r = flatten(r);
    let mut out = 0.0;
    // SAFETY: `r` is readable for 9 doubles and `out` is one writable double.
    let code = unsafe { sys::vrsdk_rotmat_det(r.as_ptr(), &mut out) };
    expect_ok(code, "vrsdk_rotmat_det");
    out
}

/// Elementwise absolute value. For a signed permutation this recovers the
/// unsigned one, which is how a diagonal inertia converts.
#[must_use]
pub fn rotmat_abs(r: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let r = flatten(r);
    let mut out = [0.0; 9];
    // SAFETY: `r` is readable and `out` writable for 9 doubles.
    let code = unsafe { sys::vrsdk_rotmat_abs(r.as_ptr(), out.as_mut_ptr()) };
    expect_ok(code, "vrsdk_rotmat_abs");
    unflatten(out)
}

/// Apply a matrix to a column vector: `r * v`.
#[must_use]
pub fn rotmat_apply(r: [[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    let r = flatten(r);
    let mut out = [0.0; 3];
    // SAFETY: `r` is readable for 9 doubles, `v` for 3, `out` writable for 3.
    let code = unsafe { sys::vrsdk_rotmat_apply(r.as_ptr(), v.as_ptr(), out.as_mut_ptr()) };
    expect_ok(code, "vrsdk_rotmat_apply");
    out
}

// ---------------------------------------------------------------------------
// the six conversions
// ---------------------------------------------------------------------------

/// Euler angles to a quaternion.
///
/// The sign is whatever the product produces and is not canonicalized, so
/// [`EulerOrder::Zyx`] reproduces the simulator's own FRD quaternion term for
/// term.
#[must_use]
pub fn euler_to_quat(euler: [f64; 3], order: EulerOrder) -> [f64; 4] {
    let mut out = [0.0; 4];
    // SAFETY: `euler` is readable for 3 doubles and `out` writable for 4; the
    // order is a defined sequence, so the call has no failure left.
    let code =
        unsafe { sys::vrsdk_euler_to_quat(euler.as_ptr(), order.to_wire(), out.as_mut_ptr()) };
    expect_ok(code, "vrsdk_euler_to_quat");
    out
}

/// Euler angles to a rotation matrix: the product of the three axis rotations
/// in application order.
#[must_use]
pub fn euler_to_rotmat(euler: [f64; 3], order: EulerOrder) -> [[f64; 3]; 3] {
    let mut out = [0.0; 9];
    // SAFETY: `euler` is readable for 3 doubles and `out` writable for 9.
    let code =
        unsafe { sys::vrsdk_euler_to_rotmat(euler.as_ptr(), order.to_wire(), out.as_mut_ptr()) };
    expect_ok(code, "vrsdk_euler_to_rotmat");
    unflatten(out)
}

/// A quaternion to a rotation matrix. `quat` is normalized first, so a
/// slightly-off-unit quaternion from the wire still gives an orthonormal matrix.
#[must_use]
pub fn quat_to_rotmat(quat: [f64; 4]) -> [[f64; 3]; 3] {
    let mut out = [0.0; 9];
    // SAFETY: `quat` is readable for 4 doubles and `out` writable for 9.
    let code = unsafe { sys::vrsdk_quat_to_rotmat(quat.as_ptr(), out.as_mut_ptr()) };
    expect_ok(code, "vrsdk_quat_to_rotmat");
    unflatten(out)
}

/// A rotation matrix to a quaternion, by Shepperd's method, returning the
/// representative with a non-negative scalar part.
#[must_use]
pub fn rotmat_to_quat(r: [[f64; 3]; 3]) -> [f64; 4] {
    let r = flatten(r);
    let mut out = [0.0; 4];
    // SAFETY: `r` is readable for 9 doubles and `out` writable for 4.
    let code = unsafe { sys::vrsdk_rotmat_to_quat(r.as_ptr(), out.as_mut_ptr()) };
    expect_ok(code, "vrsdk_rotmat_to_quat");
    out
}

/// A rotation matrix to Euler angles in the given application order.
///
/// The middle angle lands in `[-pi/2, pi/2]` and the outer two in `[-pi, pi]`.
/// At gimbal lock the angle about z is pinned to zero and the determined
/// combination goes into the other outer angle, as the simulator does.
#[must_use]
pub fn rotmat_to_euler(r: [[f64; 3]; 3], order: EulerOrder) -> [f64; 3] {
    let r = flatten(r);
    let mut out = [0.0; 3];
    // SAFETY: `r` is readable for 9 doubles and `out` writable for 3.
    let code = unsafe { sys::vrsdk_rotmat_to_euler(r.as_ptr(), order.to_wire(), out.as_mut_ptr()) };
    expect_ok(code, "vrsdk_rotmat_to_euler");
    out
}

/// A quaternion to Euler angles in the given application order. Goes through
/// the matrix, so it inherits the ranges and gimbal-lock behaviour of
/// [`rotmat_to_euler`].
#[must_use]
pub fn quat_to_euler(quat: [f64; 4], order: EulerOrder) -> [f64; 3] {
    let mut out = [0.0; 3];
    // SAFETY: `quat` is readable for 4 doubles and `out` writable for 3.
    let code =
        unsafe { sys::vrsdk_quat_to_euler(quat.as_ptr(), order.to_wire(), out.as_mut_ptr()) };
    expect_ok(code, "vrsdk_quat_to_euler");
    out
}

/// The Euler order a built-in convention reports angles in: `Zxy` for
/// [`Axes::UNITY`], `Zyx` for [`Axes::FRD`] and [`Axes::CV`], and `None` for a
/// tag that names no frame, which includes [`Axes::UNSPECIFIED`] and therefore
/// every robot in the default `"fru"` frame.
#[must_use]
pub fn default_euler_order(axes: Axes) -> Option<EulerOrder> {
    let mut out = sys::VRSDK_EULER_UNSPECIFIED;
    // SAFETY: `out` is one writable `vrsdk_euler_order_t`.
    let code = unsafe { sys::vrsdk_default_euler_order(axes.0, &mut out) };
    match code {
        sys::VRSDK_OK => EulerOrder::from_wire(out),
        sys::VRSDK_ERR_INVALID_ARGUMENT => None,
        other => {
            expect_ok(other, "vrsdk_default_euler_order");
            None
        }
    }
}

// ---------------------------------------------------------------------------
// frames
// ---------------------------------------------------------------------------

/// One axis convention: the Unity-to-frame matrix, plus the order the frame
/// reports Euler angles in.
///
/// Plain data: build one with a named constructor, copy it, store it.
///
/// ```
/// use vrobots_sdk::AxisBasis;
///
/// let frd = AxisBasis::frd();
/// assert_eq!(frd.north(), [1.0, 0.0, 0.0]); // forward is north
/// assert!(frd.is_right_handed());
/// assert!(!AxisBasis::fru().is_right_handed());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct AxisBasis {
    /// `R`, the signed-permutation matrix from Unity into this frame, row-major.
    /// Row `i` is the combination of Unity axes that maps onto frame axis `i`,
    /// so `R * v` re-expresses a Unity vector and `R^T` goes back.
    pub rows: [[f64; 3]; 3],
    /// The order this convention reports Euler angles in. Not derivable from
    /// `rows`: it is a reporting convention, and two frames with identical axes
    /// can differ here.
    pub euler_order: EulerOrder,
}

impl AxisBasis {
    /// A basis from a frame definition's nine numbers and its angle order.
    ///
    /// Nothing is validated: rows that are not orthonormal give conversions
    /// that are not rotations.
    #[must_use]
    pub fn new(rows: [[f64; 3]; 3], euler_order: EulerOrder) -> AxisBasis {
        let flat = flatten(rows);
        let mut raw = sys::vrsdk_axis_basis_t::default();
        // SAFETY: `flat` is readable for 9 doubles and `raw` is writable storage
        // for one `vrsdk_axis_basis_t`; the order is a defined sequence.
        let code =
            unsafe { sys::vrsdk_axis_basis_new(flat.as_ptr(), euler_order.to_wire(), &mut raw) };
        expect_ok(code, "vrsdk_axis_basis_new");
        AxisBasis::from_raw(&raw)
    }

    /// `"unity"`: +x right, +y up, +z forward. Left-handed; the frame every
    /// other one is defined against, so `R` is the identity.
    #[must_use]
    pub fn unity() -> AxisBasis {
        AxisBasis::built_in(sys::vrsdk_axis_basis_unity, "vrsdk_axis_basis_unity")
    }

    /// `"frd"`: +x forward, +y right, +z down. Right-handed; as a world frame it
    /// is NED.
    #[must_use]
    pub fn frd() -> AxisBasis {
        AxisBasis::built_in(sys::vrsdk_axis_basis_frd, "vrsdk_axis_basis_frd")
    }

    /// `"fru"`: +x forward, +y right, +z up. Left-handed, the scene default on a
    /// fresh launch, and the one built-in with no [`Axes`] tag.
    #[must_use]
    pub fn fru() -> AxisBasis {
        AxisBasis::built_in(sys::vrsdk_axis_basis_fru, "vrsdk_axis_basis_fru")
    }

    /// `"cv"`: +x right, +y down, +z forward, the OpenCV camera convention.
    /// Right-handed.
    #[must_use]
    pub fn cv() -> AxisBasis {
        AxisBasis::built_in(sys::vrsdk_axis_basis_cv, "vrsdk_axis_basis_cv")
    }

    /// The built-in for an axis tag, or `None` for [`Axes::UNSPECIFIED`] and
    /// anything unrecognised. There is no tag for `"fru"`; use
    /// [`from_frame_id`](Self::from_frame_id).
    #[must_use]
    pub fn from_axes(axes: Axes) -> Option<AxisBasis> {
        let mut raw = sys::vrsdk_axis_basis_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_axis_basis_t`.
        let code = unsafe { sys::vrsdk_axis_basis_from_axes(axes.0, &mut raw) };
        AxisBasis::lookup(code, &raw, "vrsdk_axis_basis_from_axes")
    }

    /// The built-in registered under a `coord_frame_id`: `"unity"`, `"frd"`,
    /// `"fru"` or `"cv"`, or `None` for any other name. A frame the scene
    /// registered at run time is read with
    /// [`frame_def`](crate::VirtualRobot::frame_def) and converted with
    /// [`from_frame_def`](Self::from_frame_def) instead.
    #[must_use]
    pub fn from_frame_id(coord_frame_id: &str) -> Option<AxisBasis> {
        let id = ffi::c_string(coord_frame_id, "coord_frame_id").ok()?;
        let mut raw = sys::vrsdk_axis_basis_t::default();
        // SAFETY: `id` is a NUL-terminated string borrowed for the call and
        // `raw` is writable storage for one `vrsdk_axis_basis_t`.
        let code = unsafe { sys::vrsdk_axis_basis_from_frame_id(id.as_ptr(), &mut raw) };
        AxisBasis::lookup(code, &raw, "vrsdk_axis_basis_from_frame_id")
    }

    /// A basis from a definition read off `z/frames`: the bridge that lets a
    /// frame registered at run time convert end to end.
    ///
    /// The angle order comes from the definition; when the publisher left it
    /// unspecified, the built-in order for its `axis_convention` fills in.
    /// `None` only when neither side names an order, because a basis with a
    /// guessed order extracts angles that look plausible and mean nothing.
    #[must_use]
    pub fn from_frame_def(def: &FrameDef) -> Option<AxisBasis> {
        let source = def.to_raw();
        let mut raw = sys::vrsdk_axis_basis_t::default();
        // SAFETY: `source` is one initialised `vrsdk_frame_def_t` and `raw` is
        // writable storage for one `vrsdk_axis_basis_t`.
        let code = unsafe { sys::vrsdk_axis_basis_from_frame_def(&source, &mut raw) };
        AxisBasis::lookup(code, &raw, "vrsdk_axis_basis_from_frame_def")
    }

    /// `det(R)`. **`-1` means right-handed**: the determinant is of the map out
    /// of left-handed Unity, so a right-handed frame is the one that flips it.
    #[must_use]
    pub fn det(&self) -> f64 {
        let raw = self.to_raw();
        let mut out = 0.0;
        // SAFETY: `raw` is one initialised `vrsdk_axis_basis_t` and `out` one
        // writable double.
        let code = unsafe { sys::vrsdk_axis_basis_det(&raw, &mut out) };
        expect_ok(code, "vrsdk_axis_basis_det");
        out
    }

    /// Whether this convention is right-handed, i.e. whether
    /// [`det`](Self::det) is negative.
    #[must_use]
    pub fn is_right_handed(&self) -> bool {
        let raw = self.to_raw();
        // SAFETY: `raw` is one initialised `vrsdk_axis_basis_t`.
        unsafe { sys::vrsdk_axis_basis_is_right_handed(&raw) }
    }

    /// Geographic north in this frame's components: `R` applied to Unity's +z.
    #[must_use]
    pub fn north(&self) -> [f64; 3] {
        self.anchor(sys::vrsdk_axis_basis_north, "vrsdk_axis_basis_north")
    }

    /// Geographic east in this frame's components: `R` applied to Unity's +x.
    #[must_use]
    pub fn east(&self) -> [f64; 3] {
        self.anchor(sys::vrsdk_axis_basis_east, "vrsdk_axis_basis_east")
    }

    /// Down in this frame's components: `R` applied to Unity's -y. Up is its
    /// negation.
    #[must_use]
    pub fn down(&self) -> [f64; 3] {
        self.anchor(sys::vrsdk_axis_basis_down, "vrsdk_axis_basis_down")
    }

    pub(crate) fn to_raw(self) -> sys::vrsdk_axis_basis_t {
        sys::vrsdk_axis_basis_t {
            rows: flatten(self.rows),
            euler_order: self.euler_order.to_wire(),
        }
    }

    fn from_raw(raw: &sys::vrsdk_axis_basis_t) -> AxisBasis {
        let euler_order = EulerOrder::from_wire(raw.euler_order).unwrap_or_else(|| {
            panic!(
                "the C library returned an axis basis with no Euler order ({}); this is a bug \
                 in the VRobots SDK",
                raw.euler_order
            )
        });
        AxisBasis {
            rows: unflatten(raw.rows),
            euler_order,
        }
    }

    fn built_in(
        constructor: unsafe extern "C" fn(*mut sys::vrsdk_axis_basis_t) -> sys::vrsdk_err_t,
        what: &str,
    ) -> AxisBasis {
        let mut raw = sys::vrsdk_axis_basis_t::default();
        // SAFETY: every built-in constructor takes one writable
        // `vrsdk_axis_basis_t` and nothing else.
        let code = unsafe { constructor(&mut raw) };
        expect_ok(code, what);
        AxisBasis::from_raw(&raw)
    }

    fn lookup(
        code: sys::vrsdk_err_t,
        raw: &sys::vrsdk_axis_basis_t,
        what: &str,
    ) -> Option<AxisBasis> {
        match code {
            sys::VRSDK_OK => Some(AxisBasis::from_raw(raw)),
            sys::VRSDK_ERR_INVALID_ARGUMENT => None,
            other => {
                expect_ok(other, what);
                None
            }
        }
    }

    fn anchor(
        &self,
        function: unsafe extern "C" fn(
            *const sys::vrsdk_axis_basis_t,
            *mut f64,
        ) -> sys::vrsdk_err_t,
        what: &str,
    ) -> [f64; 3] {
        let raw = self.to_raw();
        let mut out = [0.0; 3];
        // SAFETY: the north, east and down functions each take one initialised
        // `vrsdk_axis_basis_t` and a pointer writable for 3 doubles.
        let code = unsafe { function(&raw, out.as_mut_ptr()) };
        expect_ok(code, what);
        out
    }
}

/// The change of basis between two conventions, `M = R_to * R_from^T`.
///
/// Build it once and apply it many times. For the built-in frames every entry
/// is `0` or `+/-1`, so converting is exact: components move and signs flip,
/// and no rounding error enters. A vector's physical category decides which
/// `apply_*` is right:
///
/// | Category | Method | State fields |
/// |---|---|---|
/// | polar | [`apply_vec3`](Self::apply_vec3) | `lin_pos`, `lin_vel`, `lin_acc`, `wrench.force`, `env.gravity`, accelerometer, magnetometer |
/// | axial | [`apply_axial_vec3`](Self::apply_axial_vec3) | `ang_vel`, `ang_acc`, `wrench.torque`, gyroscope |
/// | diagonal inertia | [`apply_inertia_vec3`](Self::apply_inertia_vec3) | [`PhysicalParams::moi`](crate::PhysicalParams::moi) |
/// | orientation | [`apply_quat`](Self::apply_quat), [`apply_rotmat`](Self::apply_rotmat) | `quat` |
///
/// ```
/// use vrobots_sdk::{AxisBasis, FrameTransform};
///
/// // fru to frd flips the third axis, and with it the handedness.
/// let t = FrameTransform::between(AxisBasis::fru(), AxisBasis::frd());
/// assert_eq!(t.det, -1.0);
/// assert_eq!(t.apply_vec3([1.0, 2.0, 3.0]), [1.0, 2.0, -3.0]);
/// assert_eq!(t.apply_axial_vec3([1.0, 2.0, 3.0]), [-1.0, -2.0, 3.0]);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub struct FrameTransform {
    /// `M`, row-major.
    pub matrix: [[f64; 3]; 3],
    /// `det(M)`: `+1` when both frames have the same handedness, `-1` when the
    /// conversion crosses it. Every method recomputes it from `matrix`.
    pub det: f64,
}

impl FrameTransform {
    /// The transform that changes nothing.
    #[must_use]
    pub fn identity() -> FrameTransform {
        let mut raw = sys::vrsdk_frame_transform_t::default();
        // SAFETY: `raw` is writable storage for one `vrsdk_frame_transform_t`.
        let code = unsafe { sys::vrsdk_frame_transform_identity(&mut raw) };
        expect_ok(code, "vrsdk_frame_transform_identity");
        FrameTransform::from_raw(&raw)
    }

    /// The transform out of `from` and into `to`: `M = R_to * R_from^T`.
    ///
    /// A fact about the two matrices only; neither basis's Euler order is read.
    #[must_use]
    pub fn between(from: AxisBasis, to: AxisBasis) -> FrameTransform {
        let (source, target) = (from.to_raw(), to.to_raw());
        let mut raw = sys::vrsdk_frame_transform_t::default();
        // SAFETY: `source` and `target` are initialised `vrsdk_axis_basis_t`s and
        // `raw` is writable storage for one `vrsdk_frame_transform_t`.
        let code = unsafe { sys::vrsdk_frame_transform_between(&source, &target, &mut raw) };
        expect_ok(code, "vrsdk_frame_transform_between");
        FrameTransform::from_raw(&raw)
    }

    /// The transform back the other way. `M` is orthonormal, so this is the
    /// transpose.
    #[must_use]
    pub fn inverse(self) -> FrameTransform {
        let source = self.to_raw();
        let mut raw = sys::vrsdk_frame_transform_t::default();
        // SAFETY: `source` is one initialised `vrsdk_frame_transform_t` and
        // `raw` writable storage for one.
        let code = unsafe { sys::vrsdk_frame_transform_inverse(&source, &mut raw) };
        expect_ok(code, "vrsdk_frame_transform_inverse");
        FrameTransform::from_raw(&raw)
    }

    /// Whether the two frames disagree about handedness.
    #[must_use]
    pub fn flips_handedness(self) -> bool {
        let raw = self.to_raw();
        // SAFETY: `raw` is one initialised `vrsdk_frame_transform_t`.
        unsafe { sys::vrsdk_frame_transform_flips_handedness(&raw) }
    }

    /// Re-express a **polar** vector (position, velocity, force, a magnetometer
    /// reading): `M * v`.
    ///
    /// Wrong for angular velocity, angular acceleration, torque and gyro
    /// readings, which are axial; see [`apply_axial_vec3`](Self::apply_axial_vec3).
    #[must_use]
    pub fn apply_vec3(self, v: [f64; 3]) -> [f64; 3] {
        self.apply3(
            sys::vrsdk_frame_transform_apply_vec3,
            v,
            "vrsdk_frame_transform_apply_vec3",
        )
    }

    /// Re-express an **axial** vector (angular velocity, angular acceleration,
    /// torque, a gyro reading): `det(M) * M * v`.
    ///
    /// The determinant is the handedness flip, which falls out of the definition
    /// of a cross product: reflect the basis and a counter-clockwise rotation is
    /// clockwise.
    #[must_use]
    pub fn apply_axial_vec3(self, v: [f64; 3]) -> [f64; 3] {
        self.apply3(
            sys::vrsdk_frame_transform_apply_axial_vec3,
            v,
            "vrsdk_frame_transform_apply_axial_vec3",
        )
    }

    /// Re-express a **diagonal inertia**, the three principal moments:
    /// `abs(M) * v`, a permutation with no sign, since a moment of inertia is
    /// positive.
    #[must_use]
    pub fn apply_inertia_vec3(self, moi: [f64; 3]) -> [f64; 3] {
        self.apply3(
            sys::vrsdk_frame_transform_apply_inertia_vec3,
            moi,
            "vrsdk_frame_transform_apply_inertia_vec3",
        )
    }

    /// Re-express an attitude matrix: `M * C * M^T`, a proper rotation even when
    /// the conversion flips handedness.
    #[must_use]
    pub fn apply_rotmat(self, r: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
        let raw = self.to_raw();
        let r = flatten(r);
        let mut out = [0.0; 9];
        // SAFETY: `raw` is one initialised `vrsdk_frame_transform_t`, `r` is
        // readable and `out` writable for 9 doubles.
        let code =
            unsafe { sys::vrsdk_frame_transform_apply_rotmat(&raw, r.as_ptr(), out.as_mut_ptr()) };
        expect_ok(code, "vrsdk_frame_transform_apply_rotmat");
        unflatten(out)
    }

    /// Re-express an attitude quaternion: `[det(M) * (M * q_xyz), q_w]`.
    ///
    /// Equivalent to [`apply_rotmat`](Self::apply_rotmat) on the same rotation,
    /// exact for the built-in frames, and it keeps the caller's choice between
    /// `q` and `-q`.
    #[must_use]
    pub fn apply_quat(self, q: [f64; 4]) -> [f64; 4] {
        let raw = self.to_raw();
        let mut out = [0.0; 4];
        // SAFETY: `raw` is one initialised `vrsdk_frame_transform_t`, `q` is
        // readable and `out` writable for 4 doubles.
        let code =
            unsafe { sys::vrsdk_frame_transform_apply_quat(&raw, q.as_ptr(), out.as_mut_ptr()) };
        expect_ok(code, "vrsdk_frame_transform_apply_quat");
        out
    }

    fn to_raw(self) -> sys::vrsdk_frame_transform_t {
        sys::vrsdk_frame_transform_t {
            matrix: flatten(self.matrix),
            det: self.det,
        }
    }

    fn from_raw(raw: &sys::vrsdk_frame_transform_t) -> FrameTransform {
        FrameTransform {
            matrix: unflatten(raw.matrix),
            det: raw.det,
        }
    }

    fn apply3(
        self,
        function: unsafe extern "C" fn(
            *const sys::vrsdk_frame_transform_t,
            *const f64,
            *mut f64,
        ) -> sys::vrsdk_err_t,
        v: [f64; 3],
        what: &str,
    ) -> [f64; 3] {
        let raw = self.to_raw();
        let mut out = [0.0; 3];
        // SAFETY: the three vector appliers each take one initialised
        // `vrsdk_frame_transform_t`, a pointer readable for 3 doubles and one
        // writable for 3.
        let code = unsafe { function(&raw, v.as_ptr(), out.as_mut_ptr()) };
        expect_ok(code, what);
        out
    }
}

// ---------------------------------------------------------------------------
// one-shot conversions keyed by axis tag
// ---------------------------------------------------------------------------

/// The shared shape of the six tag-keyed conversions.
type Convert = unsafe extern "C" fn(
    *const f64,
    sys::vrsdk_axes_t,
    sys::vrsdk_axes_t,
    *mut f64,
) -> sys::vrsdk_err_t;

/// Run one tag-keyed conversion.
///
/// # Safety
///
/// `function` must read `N` doubles from its first argument and write `N` to
/// its last, which is the documented contract of every `vrsdk_convert_*`
/// function for the `N` its caller passes.
unsafe fn convert<const N: usize>(
    function: Convert,
    input: [f64; N],
    from: Axes,
    to: Axes,
) -> VrResult<[f64; N]> {
    let mut out = [0.0; N];
    // SAFETY: per this function's contract, `input` is readable and `out`
    // writable for exactly the count `function` uses.
    let code = unsafe { function(input.as_ptr(), from.0, to.0, out.as_mut_ptr()) };
    ffi::check(code)?;
    Ok(out)
}

/// Re-express a **polar** vector from one convention in another: position,
/// velocity, acceleration, force, a magnetometer reading.
///
/// # Errors
///
/// [`VrError::InvalidArgument`](crate::VrError::InvalidArgument) if either tag names no frame, which includes
/// [`Axes::UNSPECIFIED`] and so the default `"fru"` frame. Use
/// [`AxisBasis::from_frame_id`] and [`FrameTransform::between`] for that one.
pub fn convert_vec3(v: [f64; 3], from: Axes, to: Axes) -> VrResult<[f64; 3]> {
    // SAFETY: `vrsdk_convert_vec3` reads 3 doubles and writes 3.
    unsafe { convert(sys::vrsdk_convert_vec3, v, from, to) }
}

/// Re-express an **axial** vector from one convention in another: angular
/// velocity, angular acceleration, torque, a gyro reading. Carries the
/// handedness sign flip that [`convert_vec3`] must not.
///
/// # Errors
///
/// As [`convert_vec3`].
pub fn convert_axial_vec3(v: [f64; 3], from: Axes, to: Axes) -> VrResult<[f64; 3]> {
    // SAFETY: `vrsdk_convert_axial_vec3` reads 3 doubles and writes 3.
    unsafe { convert(sys::vrsdk_convert_axial_vec3, v, from, to) }
}

/// Re-express three principal moments of inertia: a permutation with no sign.
///
/// # Errors
///
/// As [`convert_vec3`].
pub fn convert_inertia_vec3(moi: [f64; 3], from: Axes, to: Axes) -> VrResult<[f64; 3]> {
    // SAFETY: `vrsdk_convert_inertia_vec3` reads 3 doubles and writes 3.
    unsafe { convert(sys::vrsdk_convert_inertia_vec3, moi, from, to) }
}

/// Re-express an attitude quaternion, `[x, y, z, w]` in and out.
///
/// # Errors
///
/// As [`convert_vec3`].
pub fn convert_quat(q: [f64; 4], from: Axes, to: Axes) -> VrResult<[f64; 4]> {
    // SAFETY: `vrsdk_convert_quat` reads 4 doubles and writes 4.
    unsafe { convert(sys::vrsdk_convert_quat, q, from, to) }
}

/// Re-express an attitude matrix.
///
/// # Errors
///
/// As [`convert_vec3`].
pub fn convert_rotmat(r: [[f64; 3]; 3], from: Axes, to: Axes) -> VrResult<[[f64; 3]; 3]> {
    // SAFETY: `vrsdk_convert_rotmat` reads 9 doubles and writes 9.
    let out = unsafe { convert(sys::vrsdk_convert_rotmat, flatten(r), from, to) }?;
    Ok(unflatten(out))
}

/// Re-express Euler angles, **each in its own default order**: read in
/// `from`'s order and returned in `to`'s, so this changes the order as well as
/// the axes. For other orders go through [`euler_to_quat`], [`convert_quat`]
/// and [`quat_to_euler`].
///
/// # Errors
///
/// As [`convert_vec3`].
pub fn convert_euler(euler: [f64; 3], from: Axes, to: Axes) -> VrResult<[f64; 3]> {
    // SAFETY: `vrsdk_convert_euler` reads 3 doubles and writes 3.
    unsafe { convert(sys::vrsdk_convert_euler, euler, from, to) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_order_round_trips_through_its_wire_value() {
        for order in [
            EulerOrder::Xyz,
            EulerOrder::Xzy,
            EulerOrder::Yxz,
            EulerOrder::Yzx,
            EulerOrder::Zxy,
            EulerOrder::Zyx,
        ] {
            assert_eq!(EulerOrder::from_wire(order.to_wire()), Some(order));
            let letters: String = order
                .axes()
                .iter()
                .map(|&axis| char::from(b"xyz"[usize::from(axis)]))
                .collect();
            assert_eq!(order.name(), letters);
        }
        assert_eq!(EulerOrder::from_wire(0), None);
        assert_eq!(EulerOrder::from_wire(7), None);
    }

    #[test]
    fn a_failed_lookup_is_none_not_a_panic() {
        assert_eq!(default_euler_order(Axes::UNSPECIFIED), None);
        assert!(AxisBasis::from_axes(Axes(42)).is_none());
        assert!(AxisBasis::from_frame_id("nope").is_none());
        assert!(AxisBasis::from_frame_id("f\0rd").is_none());
    }

    #[test]
    fn convert_refuses_an_unspecified_tag() {
        let err = convert_vec3([1.0, 2.0, 3.0], Axes::UNSPECIFIED, Axes::FRD)
            .expect_err("unspecified names no frame");
        assert!(matches!(err, crate::VrError::InvalidArgument(_)), "{err:?}");
    }
}
