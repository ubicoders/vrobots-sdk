//! ex36 -- rotations: the frame math, end to end.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex36_rotations
//! ```
//!
//! ex25 changed which frame a robot **reports** in, by asking the simulator to.
//! This does the conversion itself, with [`vrobots_sdk::rotations`] -- a module
//! that talks to nothing. Pure, total functions from the C library, the same
//! implementation behind the C++ header and the Python package, so that
//! "convert this quaternion to roll/pitch/yaw" has exactly one answer with
//! exactly one set of conventions behind it.
//!
//! A truck is created, driven in an arc for a few seconds, and deleted. The arc
//! is not decoration: a robot standing still has a zero position, a zero rate and
//! an identity attitude, and a sign flip in a zero proves nothing.
//!
//! # Storage order never changes; application order is an argument
//!
//! Angles are stored `[about x, about y, about z]` and quaternions `[x, y, z, w]`
//! with the scalar **last**, always -- the wire's `Vec4` order.
//! [`EulerOrder`] says which rotation is applied first and it never reorders the
//! array: [`Zyx`] is the aerospace yaw-then-pitch-then-roll, and it reads
//! `euler[2]`, `euler[1]`, `euler[0]` in that order. There is no
//! `[yaw, pitch, roll]` layout anywhere in this SDK.
//!
//! # A frame is nine numbers, and the wire carries them
//!
//! A convention is one signed-permutation matrix `R` whose rows are the Unity
//! axes that map onto that frame's axes, so `R * v` re-expresses a Unity vector
//! in frame components. That is exactly what `z/frames` publishes, and it is all
//! a peer needs: the inverse is the transpose, the handedness is `det(R)`, and
//! north/east/down are `R` applied to the Unity world anchor.
//!
//! Which matters because the registry is **string-keyed**. `"fru"` -- the
//! default this truck reports in -- has no [`Axes`] constant at all, so its state
//! header carries `AXES_UNSPECIFIED` and the [`Axes`]-keyed shortcuts cannot
//! reach it. [`frame_def`] and [`AxisBasis::from_frame_def`] can, and a frame
//! some scene registered at runtime is the same problem one step further out.
//!
//! # A vector's physical category decides how it converts
//!
//! `M * v` is right for a **polar** vector and wrong for an **axial** one:
//!
//! | category | rule | fields |
//! |---|---|---|
//! | polar | `M * v` | position, velocity, acceleration, force, accelerometer, magnetometer |
//! | axial | `det(M) * M * v` | angular velocity, angular acceleration, torque, **gyroscope** |
//! | diagonal inertia | `abs(M) * v` | principal moments -- a permutation with no sign |
//! | orientation | `M * C * M^T` | the attitude quaternion |
//!
//! Between two frames of the same handedness `det(M) = +1` and the first two
//! rules coincide, which is precisely why using the wrong one survives testing
//! until somebody converts a body rate across a handedness flip. `fru` is
//! left-handed and `frd` is right-handed, so the run below is that case.
//!
//! # Gimbal lock has an answer, and it is the simulator's
//!
//! With the nose vertical the two outer rotations act about the same axis and
//! only their difference survives; there is no unique triple left. The last
//! section asks for one anyway. `heading` is the meaningless quantity there, so
//! the angle about z is pinned to zero and the whole determined combination goes
//! into the other outer angle -- the same choice `CoordFrame.MatrixToEulerDeg`
//! makes, so a locked attitude decodes here to the triple the simulator shows.
//!
//! Created, not scene-authored: deleted at the end.
//!
//! [`Zyx`]: vrobots_sdk::rotations::EulerOrder::Zyx
//! [`frame_def`]: vrobots_sdk::VirtualRobot::frame_def

use std::f64::consts::FRAC_PI_2;

use vrobots_sdk::rotations::{self, AxisBasis, EulerOrder, FrameTransform};
use vrobots_sdk::{Axes, RobotType, VirtualRobot, VrError};

const STEER_US: f64 = 1300.0; // well left of centre, for a yaw rate worth converting
const THROTTLE_US: f64 = 1650.0; // light forward
const BRAKE_US: f64 = 1100.0; // released -- brake is bottom-anchored, not centred
const HZ: f64 = 25.0;
const DRIVE_SAMPLES: u32 = 75; // ~3 s to build a heading, a position and a yaw rate

/// Illustrative principal moments, kg m^2. Inertia is never in the state
/// message, so there is nothing live to convert -- but the rule is its own case.
const MOI: [f64; 3] = [0.10, 0.20, 0.30];

/// Roll, pitch, yaw in degrees, with pitch **at** the pole.
const LOCKED_DEG: [f64; 3] = [0.0, 90.0, 40.0];

/// Round-trip tolerance. A signed permutation moves components and flips signs,
/// so the error is not "small", it is zero -- this only guards the arithmetic.
const EPS: f64 = 1e-12;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Truck, None)?;
    let sys_id = robot.sys_id();
    let drive_s = f64::from(DRIVE_SAMPLES) / HZ;
    println!("created sys_id={sys_id} ({:?})", robot.robot_type());
    println!("driving a left arc for {drive_s:.1} s, so that there is something to convert");
    for _ in 0..DRIVE_SAMPLES {
        // Latches, so this only has to be sent once -- a real controller streams,
        // so this one does too.
        robot.set_car(STEER_US, THROTTLE_US, Some(BRAKE_US))?;
        robot.rate(HZ);
    }

    // ===== 1: the frame, as the nine numbers that define it =====
    let def = robot.frame_def()?;
    let state = robot.states();
    println!("\n== 1. the frame this robot reports in, off z/frames ==");
    println!("  id={:?}  name={:?}", def.id, def.name);
    println!(
        "  axis_convention={} ({:?})   euler_order={} ({:?})",
        def.axis_convention.0,
        def.axis_convention.name(),
        def.euler_order.0,
        def.euler_order.name()
    );
    println!("  R, unity -> {}, one row per frame axis:", def.id);
    for row in def.rows {
        println!("    [{:+6.3} {:+6.3} {:+6.3}]", row[0], row[1], row[2]);
    }
    if def.id != state.coord_frame_id {
        println!(
            "  NOTE: the state header says {:?}. z/frames carries one message per distinct \
             frame the robot and its devices use, and this read takes the first that arrives.",
            state.coord_frame_id
        );
    }

    let basis = AxisBasis::from_frame_def(&def).ok_or_else(|| {
        VrError::InvalidArgument(format!(
            "frame {:?} names no euler order and its axis convention has no built-in default, \
             so there is no order to report angles in",
            def.id
        ))
    })?;
    println!(
        "  det(R)={:+.0}, right-handed={} -- det -1 IS right-handed here, because the map is \
         OUT of left-handed unity",
        basis.det(),
        basis.is_right_handed()
    );
    println!(
        "  north={}  east={}  down={}  (unit vectors, in this frame's own components)",
        vec3(basis.north()),
        vec3(basis.east()),
        vec3(basis.down())
    );
    if def.axis_convention == Axes::UNSPECIFIED {
        println!(
            "  axis_convention is UNSPECIFIED: this frame has no Axes constant on the wire, so \
             those nine numbers are the only description of it there is. Section 4 walks into \
             that."
        );
    }

    // ===== 2: the attitude, which is a quaternion on the wire =====
    let order = basis.euler_order;
    let euler = rotations::quat_to_euler(state.kin.quat, order);
    println!("\n== 2. the attitude as angles ==");
    println!("  kin.quat [x,y,z,w] = {}", vec4(state.kin.quat));
    println!(
        "  quat_to_euler in {}  roll={:+7.2} deg  pitch={:+7.2} deg  yaw={:+7.2} deg",
        order.name(),
        euler[0].to_degrees(),
        euler[1].to_degrees(),
        euler[2].to_degrees()
    );
    println!(
        "  No state message carries an Euler triple, and this is why: the numbers depend on \
         an application order that is a property of the frame, not of the attitude."
    );

    // ===== 3: re-expressing this robot's state in frd =====
    let target = AxisBasis::frd();
    let t = FrameTransform::between(basis, target);
    println!("\n== 3. re-expressing the live state in frd ==");
    println!(
        "  M = R_frd * R_{}^T, det(M)={:+.0}, flips handedness={}",
        def.id,
        t.det,
        t.flips_handedness()
    );
    for row in t.matrix {
        println!("    [{:+6.3} {:+6.3} {:+6.3}]", row[0], row[1], row[2]);
    }

    // polar: position
    let position = state.kin.lin_pos;
    let position_frd = t.apply_vec3(position);
    println!(
        "  position  polar  {} -> {} m",
        vec3(position),
        vec3(position_frd)
    );

    // axial: the gyro, and the mistake beside it
    let gyro = state.sensors.gyroscope.angular_velocity;
    let gyro_frd = t.apply_axial_vec3(gyro);
    let gyro_wrong = t.apply_vec3(gyro);
    println!(
        "  gyro      axial  {} -> {} rad/s   apply_axial_vec3, det(M) * M * v",
        vec3(gyro),
        vec3(gyro_frd)
    );
    println!(
        "  gyro      polar  {} -> {} rad/s   apply_vec3, the WRONG rule for a rate",
        vec3(gyro),
        vec3(gyro_wrong)
    );
    if t.flips_handedness() {
        println!(
            "    det(M)={:+.0}, so every component is negated. Convert a body rate with the \
             polar rule across this pair and the robot spins the other way.",
            t.det
        );
    } else {
        println!(
            "    det(M)={:+.0}, so the two rules agree here -- which is exactly how the \
             mistake survives testing until somebody crosses a handedness flip.",
            t.det
        );
    }

    // orientation: M * C * M^T, in closed form
    let quat_frd = t.apply_quat(state.kin.quat);
    let euler_frd = rotations::quat_to_euler(quat_frd, target.euler_order);
    println!(
        "  quat      orient {} -> {}",
        vec4(state.kin.quat),
        vec4(quat_frd)
    );
    println!(
        "    in frd's own order ({})  roll={:+7.2} deg  pitch={:+7.2} deg  yaw={:+7.2} deg",
        target.euler_order.name(),
        euler_frd[0].to_degrees(),
        euler_frd[1].to_degrees(),
        euler_frd[2].to_degrees()
    );

    // and back, which has to be exact
    let back = t.inverse();
    let round_trip = [
        max_abs_diff(&position, &back.apply_vec3(position_frd)),
        max_abs_diff(&gyro, &back.apply_axial_vec3(gyro_frd)),
        max_abs_diff(&state.kin.quat, &back.apply_quat(quat_frd)),
    ]
    .into_iter()
    .fold(0.0_f64, f64::max);
    println!("  inverse() round trip: worst component error {round_trip:.1e}");
    assert!(
        round_trip < EPS,
        "a change of basis and its inverse lost {round_trip} -- that is not rounding"
    );

    // ===== 4: the Axes-keyed shortcuts, and where they stop =====
    // Fixed unity-frame inputs, so these lines print the same numbers on every
    // run: they are the four rules, not this drive.
    let up_unity = [0.0, 10.0, 0.0]; // ten metres up
    let roll_unity = [0.0, 0.0, 1.0]; // 1 rad/s about unity's forward axis
    let yaw90_unity = rotations::euler_to_quat([0.0, FRAC_PI_2, 0.0], EulerOrder::Zxy);
    let yaw90_frd = rotations::convert_quat(yaw90_unity, Axes::UNITY, Axes::FRD)?;
    println!("\n== 4. the Axes-keyed shortcuts, on fixed unity input ==");
    println!(
        "  convert_vec3         {} -> {} m       ten metres up",
        vec3(up_unity),
        vec3(rotations::convert_vec3(up_unity, Axes::UNITY, Axes::FRD)?)
    );
    println!(
        "  convert_axial_vec3   {} -> {} rad/s   a roll rate, as the axial vector it is",
        vec3(roll_unity),
        vec3(rotations::convert_axial_vec3(
            roll_unity,
            Axes::UNITY,
            Axes::FRD
        )?)
    );
    println!(
        "  convert_vec3         {} -> {} rad/s   the same rate, the polar rule, wrong",
        vec3(roll_unity),
        vec3(rotations::convert_vec3(roll_unity, Axes::UNITY, Axes::FRD)?)
    );
    println!(
        "  convert_inertia_vec3 {} -> {} kg m^2  abs(M): a moment of inertia stays positive",
        vec3(MOI),
        vec3(rotations::convert_inertia_vec3(
            MOI,
            Axes::UNITY,
            Axes::FRD
        )?)
    );
    println!(
        "  convert_quat         {} -> {}",
        vec4(yaw90_unity),
        vec4(yaw90_frd)
    );
    println!(
        "    +90 deg about unity's up axis is yaw={:+.2} deg in frd, and the whole conversion \
         is [x,y,z,w] -> [-z,-x,y,w]: a permutation and two signs, exact.",
        rotations::quat_to_euler(yaw90_frd, EulerOrder::Zyx)[2].to_degrees()
    );

    // The same call keyed on the tag this robot actually publishes.
    match rotations::convert_vec3(position, state.axis_convention, Axes::FRD) {
        Ok(v) => println!("  this robot's position by tag -> {} m", vec3(v)),
        Err(e) => println!(
            "  this robot's position by tag -> [{}] {}",
            e.code(),
            e.detail()
        ),
    }
    println!(
        "    which is section 1's whole reason for existing: the tag is a convenience and the \
         nine numbers are the fact."
    );

    // ===== 5: gimbal lock =====
    let locked = LOCKED_DEG.map(f64::to_radians);
    let quat = rotations::euler_to_quat(locked, EulerOrder::Zyx);
    let extracted = rotations::quat_to_euler(quat, EulerOrder::Zyx);
    let rebuilt = max_abs_diff(
        &rotations::quat_to_rotmat(quat).concat(),
        &rotations::euler_to_rotmat(extracted, EulerOrder::Zyx).concat(),
    );
    println!("\n== 5. gimbal lock, in zyx, with the nose vertical ==");
    println!(
        "  in    roll={:+7.2} deg  pitch={:+7.2} deg  yaw={:+7.2} deg",
        LOCKED_DEG[0], LOCKED_DEG[1], LOCKED_DEG[2]
    );
    println!(
        "  out   roll={:+7.2} deg  pitch={:+7.2} deg  yaw={:+7.2} deg",
        extracted[0].to_degrees(),
        extracted[1].to_degrees(),
        extracted[2].to_degrees()
    );
    println!("  same rotation, rebuilt: worst matrix element error {rebuilt:.1e}");
    assert!(
        rebuilt < EPS,
        "the locked triple does not rebuild its own rotation (error {rebuilt})"
    );
    println!(
        "  Yaw is pinned to 0 and roll carries the whole determined combination. The naive \
         atan2 pair returns neither: both of its terms are rounding noise at the pole, which \
         is how (0, 90, 40) comes back from a hand-inlined formula as (26.6, 90, 90)."
    );

    robot.delete()?;
    println!("\ndeleted sys_id={sys_id}");
    Ok(())
}

/// Three numbers, aligned so two rows can be compared by eye.
fn vec3(v: [f64; 3]) -> String {
    format!("[{:+7.3},{:+7.3},{:+7.3}]", v[0], v[1], v[2])
}

/// A quaternion, `[x, y, z, w]`, scalar last.
fn vec4(q: [f64; 4]) -> String {
    format!("[{:+7.4},{:+7.4},{:+7.4},{:+7.4}]", q[0], q[1], q[2], q[3])
}

/// The worst component-wise difference between two equal-length runs of numbers.
fn max_abs_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0_f64, f64::max)
}
