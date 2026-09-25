// ex36 -- rotations: the frame math, end to end.
//
//     target/cpp-build/Release/ex36_rotations
//
// ex25 changed which frame a robot REPORTS in, by asking the simulator to. This
// does the conversion itself, with `vrsdk::rotations` -- a namespace that talks
// to nothing. Pure, total functions, shared with the Rust and Python bindings, so
// that "convert this quaternion to roll/pitch/yaw" has exactly one answer with
// exactly one set of conventions behind it.
//
// A truck is created, driven in an arc for a few seconds, and deleted. The arc is
// not decoration: a robot standing still has a zero position, a zero rate and an
// identity attitude, and a sign flip in a zero proves nothing.
//
// STORAGE ORDER NEVER CHANGES; APPLICATION ORDER IS AN ARGUMENT
//
//   Angles are stored [about x, about y, about z] and quaternions [x, y, z, w]
//   with the scalar LAST, always -- the wire's Vec4 order. `EulerOrder` says
//   which rotation is applied first and it never reorders the array: `Zyx` is
//   the aerospace yaw-then-pitch-then-roll, and it reads euler[2], euler[1],
//   euler[0] in that order. There is no [yaw, pitch, roll] layout anywhere in
//   this SDK.
//
// A FRAME IS NINE NUMBERS, AND THE WIRE CARRIES THEM
//
//   A convention is one signed-permutation matrix R whose rows are the Unity
//   axes that map onto that frame's axes, so R * v re-expresses a Unity vector
//   in frame components. That is exactly what `z/frames` publishes, and it is
//   all a peer needs: the inverse is the transpose, the handedness is det(R),
//   and north/east/down are R applied to the Unity world anchor.
//
//   Which matters because the registry is STRING-KEYED. "fru" -- the default
//   this truck reports in -- has no `vrsdk_axes_t` constant at all, so its state
//   header carries `VRSDK_AXES_UNSPECIFIED` and the tag-keyed shortcuts cannot
//   reach it. `frame_def()` and `AxisBasis::from_frame_def` can, and a frame
//   some scene registered at runtime is the same problem one step further out.
//
// A VECTOR'S PHYSICAL CATEGORY DECIDES HOW IT CONVERTS
//
//   M * v is right for a POLAR vector and wrong for an AXIAL one:
//
//   | category         | rule            | fields                                                        |
//   |------------------|-----------------|---------------------------------------------------------------|
//   | polar            | M * v           | position, velocity, acceleration, force, accelerometer, magnetometer |
//   | axial            | det(M) * M * v  | angular velocity, angular acceleration, torque, GYROSCOPE     |
//   | diagonal inertia | abs(M) * v      | principal moments -- a permutation with no sign               |
//   | orientation      | M * C * M^T     | the attitude quaternion                                       |
//
//   Between two frames of the same handedness det(M) = +1 and the first two
//   rules coincide, which is precisely why using the wrong one survives testing
//   until somebody converts a body rate across a handedness flip. `fru` is
//   left-handed and `frd` is right-handed, so the run below is that case.
//
// GIMBAL LOCK HAS AN ANSWER, AND IT IS THE SIMULATOR'S
//
//   With the nose vertical the two outer rotations act about the same axis and
//   only their difference survives; there is no unique triple left. The last
//   section asks for one anyway. Heading is the meaningless quantity there, so
//   the angle about z is pinned to zero and the whole determined combination
//   goes into the other outer angle -- the same choice
//   `CoordFrame.MatrixToEulerDeg` makes, so a locked attitude decodes here to
//   the triple the simulator shows.
//
// Created, not scene-authored: deleted at the end.

#include <cmath>
#include <cstdio>
#include <string>

#include <vrobots_sdk.hpp>

constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double FRAC_PI_2 = PI / 2.0;

constexpr double STEER_US = 1300.0;     // well left of centre, for a yaw rate worth converting
constexpr double THROTTLE_US = 1650.0;  // light forward
constexpr double BRAKE_US = 1100.0;     // released -- brake is bottom-anchored, not centred
constexpr double HZ = 25.0;
constexpr int DRIVE_SAMPLES = 75;  // ~3 s to build a heading, a position and a yaw rate

// Illustrative principal moments, kg m^2. Inertia is never in the state message,
// so there is nothing live to convert -- but the rule is its own case.
constexpr vrsdk::Vec3 MOI = {0.10, 0.20, 0.30};

// Roll, pitch, yaw in degrees, with pitch AT the pole.
constexpr vrsdk::Vec3 LOCKED_DEG = {0.0, 90.0, 40.0};

// Round-trip tolerance. A signed permutation moves components and flips signs,
// so the error is not "small", it is zero -- this only guards the arithmetic.
constexpr double EPS = 1e-12;

namespace {

/// Three numbers, aligned so two rows can be compared by eye.
std::string vec3(const vrsdk::Vec3& v) {
    char buffer[96];
    std::snprintf(buffer, sizeof buffer, "[%+7.3f,%+7.3f,%+7.3f]", v[0], v[1], v[2]);
    return std::string(buffer);
}

/// A quaternion, [x, y, z, w], scalar last.
std::string vec4(const vrsdk::Quat& q) {
    char buffer[96];
    std::snprintf(buffer, sizeof buffer, "[%+7.4f,%+7.4f,%+7.4f,%+7.4f]", q[0], q[1], q[2], q[3]);
    return std::string(buffer);
}

/// The worst component-wise difference between two equal-length runs of numbers.
double max_abs_diff(const double* a, const double* b, std::size_t n) {
    double worst = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        worst = std::fmax(worst, std::fabs(a[i] - b[i]));
    }
    return worst;
}

double max_abs_diff(const vrsdk::Vec3& a, const vrsdk::Vec3& b) {
    return max_abs_diff(a.data(), b.data(), a.size());
}

double max_abs_diff(const vrsdk::Quat& a, const vrsdk::Quat& b) {
    return max_abs_diff(a.data(), b.data(), a.size());
}

/// The attitude quaternion, as a value.
vrsdk::Quat quat_of(const vrsdk::State& s) {
    const double* q = s.kin().quat;
    return {q[0], q[1], q[2], q[3]};
}

/// The world position, as a value.
vrsdk::Vec3 position_of(const vrsdk::State& s) {
    const double* p = s.kin().lin_pos;
    return {p[0], p[1], p[2]};
}

/// The gyro's body rates, as a value.
vrsdk::Vec3 gyro_of(const vrsdk::State& s) {
    const double* w = s.sensors().gyroscope.angular_velocity;
    return {w[0], w[1], w[2]};
}

/// A 3x3 matrix as the nine numbers a comparison needs.
std::array<double, 9> flat(const vrsdk::Rotmat& m) {
    return {m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]};
}

/// One matrix, one row per line.
void print_rows(const vrsdk::Rotmat& m) {
    for (const vrsdk::Vec3& row : m) {
        std::printf("    [%+6.3f %+6.3f %+6.3f]\n", row[0], row[1], row[2]);
    }
}

}  // namespace

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot = vrsdk::VirtualRobot::create(vrsdk::RobotType::Truck);
        robot.connect();
        const std::uint32_t sys_id = robot.sys_id();
        const double drive_s = static_cast<double>(DRIVE_SAMPLES) / HZ;
        std::printf("created sys_id=%u (Truck)\n", sys_id);
        std::printf("driving a left arc for %.1f s, so that there is something to convert\n",
                    drive_s);
        for (int i = 0; i < DRIVE_SAMPLES; ++i) {
            // Latches, so this only has to be sent once -- a real controller
            // streams, so this one does too.
            robot.set_car(STEER_US, THROTTLE_US, BRAKE_US);
            robot.rate(HZ);
        }

        // ===== 1: the frame, as the nine numbers that define it =====
        const vrsdk::FrameDef def = robot.frame_def();
        const vrsdk::State state = robot.states();
        std::printf("\n== 1. the frame this robot reports in, off z/frames ==\n");
        std::printf("  id=\"%s\"  name=\"%s\"\n", def.id.c_str(), def.name.c_str());
        std::printf("  axis_convention=%d (\"%s\")   euler_order=%d (\"%s\")\n",
                    def.axis_convention, vrsdk::rotations::axes_name(def.axis_convention),
                    static_cast<int>(def.euler_order), vrsdk::to_string(def.euler_order));
        std::printf("  R, unity -> %s, one row per frame axis:\n", def.id.c_str());
        print_rows(def.rows);
        if (def.id != state.coord_frame_id) {
            std::printf(
                "  NOTE: the state header says \"%s\". z/frames carries one message per distinct "
                "frame the robot and its devices use, and this read takes the first that "
                "arrives.\n",
                state.coord_frame_id.c_str());
        }

        // Throws when the definition names no order and its axis convention has
        // no built-in default: there is then no order to report angles in.
        const vrsdk::rotations::AxisBasis basis = vrsdk::rotations::AxisBasis::from_frame_def(def);
        std::printf(
            "  det(R)=%+.0f, right-handed=%s -- det -1 IS right-handed here, because the map is "
            "OUT of left-handed unity\n",
            basis.det(), basis.is_right_handed() ? "true" : "false");
        std::printf("  north=%s  east=%s  down=%s  (unit vectors, in this frame's own components)\n",
                    vec3(basis.north()).c_str(), vec3(basis.east()).c_str(),
                    vec3(basis.down()).c_str());
        if (def.axis_convention == VRSDK_AXES_UNSPECIFIED) {
            std::printf(
                "  axis_convention is UNSPECIFIED: this frame has no Axes constant on the wire, "
                "so those nine numbers are the only description of it there is. Section 4 walks "
                "into that.\n");
        }

        // ===== 2: the attitude, which is a quaternion on the wire =====
        const vrsdk::EulerOrder order = basis.euler_order;
        const vrsdk::Vec3 euler = vrsdk::rotations::quat_to_euler(quat_of(state), order);
        std::printf("\n== 2. the attitude as angles ==\n");
        std::printf("  kin.quat [x,y,z,w] = %s\n", vec4(quat_of(state)).c_str());
        std::printf("  quat_to_euler in %s  roll=%+7.2f deg  pitch=%+7.2f deg  yaw=%+7.2f deg\n",
                    vrsdk::to_string(order), euler[0] * RAD_TO_DEG, euler[1] * RAD_TO_DEG,
                    euler[2] * RAD_TO_DEG);
        std::printf(
            "  No state message carries an Euler triple, and this is why: the numbers depend on "
            "an application order that is a property of the frame, not of the attitude.\n");

        // ===== 3: re-expressing this robot's state in frd =====
        const vrsdk::rotations::AxisBasis target = vrsdk::rotations::AxisBasis::frd();
        const vrsdk::rotations::FrameTransform t =
            vrsdk::rotations::FrameTransform::between(basis, target);
        std::printf("\n== 3. re-expressing the live state in frd ==\n");
        std::printf("  M = R_frd * R_%s^T, det(M)=%+.0f, flips handedness=%s\n", def.id.c_str(),
                    t.det(), t.flips_handedness() ? "true" : "false");
        print_rows(t.matrix());

        // polar: position
        const vrsdk::Vec3 position = position_of(state);
        const vrsdk::Vec3 position_frd = t.apply_vec3(position);
        std::printf("  position  polar  %s -> %s m\n", vec3(position).c_str(),
                    vec3(position_frd).c_str());

        // axial: the gyro, and the mistake beside it
        const vrsdk::Vec3 gyro = gyro_of(state);
        const vrsdk::Vec3 gyro_frd = t.apply_axial_vec3(gyro);
        const vrsdk::Vec3 gyro_wrong = t.apply_vec3(gyro);
        std::printf("  gyro      axial  %s -> %s rad/s   apply_axial_vec3, det(M) * M * v\n",
                    vec3(gyro).c_str(), vec3(gyro_frd).c_str());
        std::printf("  gyro      polar  %s -> %s rad/s   apply_vec3, the WRONG rule for a rate\n",
                    vec3(gyro).c_str(), vec3(gyro_wrong).c_str());
        if (t.flips_handedness()) {
            std::printf(
                "    det(M)=%+.0f, so every component is negated. Convert a body rate with the "
                "polar rule across this pair and the robot spins the other way.\n",
                t.det());
        } else {
            std::printf(
                "    det(M)=%+.0f, so the two rules agree here -- which is exactly how the "
                "mistake survives testing until somebody crosses a handedness flip.\n",
                t.det());
        }

        // orientation: M * C * M^T, in closed form
        const vrsdk::Quat quat_frd = t.apply_quat(quat_of(state));
        const vrsdk::Vec3 euler_frd =
            vrsdk::rotations::quat_to_euler(quat_frd, target.euler_order);
        std::printf("  quat      orient %s -> %s\n", vec4(quat_of(state)).c_str(),
                    vec4(quat_frd).c_str());
        std::printf(
            "    in frd's own order (%s)  roll=%+7.2f deg  pitch=%+7.2f deg  yaw=%+7.2f deg\n",
            vrsdk::to_string(target.euler_order), euler_frd[0] * RAD_TO_DEG,
            euler_frd[1] * RAD_TO_DEG, euler_frd[2] * RAD_TO_DEG);

        // and back, which has to be exact
        const vrsdk::rotations::FrameTransform back = t.inverse();
        double round_trip = max_abs_diff(position, back.apply_vec3(position_frd));
        round_trip = std::fmax(round_trip, max_abs_diff(gyro, back.apply_axial_vec3(gyro_frd)));
        round_trip =
            std::fmax(round_trip, max_abs_diff(quat_of(state), back.apply_quat(quat_frd)));
        std::printf("  inverse() round trip: worst component error %.1e\n", round_trip);
        if (round_trip >= EPS) {
            std::fprintf(stderr,
                         "a change of basis and its inverse lost %g -- that is not rounding\n",
                         round_trip);
            return 1;
        }

        // ===== 4: the Axes-keyed shortcuts, and where they stop =====
        // Fixed unity-frame inputs, so these lines print the same numbers on
        // every run: they are the four rules, not this drive.
        const vrsdk::Vec3 up_unity = {0.0, 10.0, 0.0};    // ten metres up
        const vrsdk::Vec3 roll_unity = {0.0, 0.0, 1.0};   // 1 rad/s about unity's forward axis
        const vrsdk::Quat yaw90_unity =
            vrsdk::rotations::euler_to_quat({0.0, FRAC_PI_2, 0.0}, vrsdk::EulerOrder::Zxy);
        const vrsdk::Quat yaw90_frd =
            vrsdk::rotations::convert_quat(yaw90_unity, VRSDK_AXES_UNITY, VRSDK_AXES_FRD);
        std::printf("\n== 4. the Axes-keyed shortcuts, on fixed unity input ==\n");
        std::printf(
            "  convert_vec3         %s -> %s m       ten metres up\n", vec3(up_unity).c_str(),
            vec3(vrsdk::rotations::convert_vec3(up_unity, VRSDK_AXES_UNITY, VRSDK_AXES_FRD))
                .c_str());
        std::printf(
            "  convert_axial_vec3   %s -> %s rad/s   a roll rate, as the axial vector it is\n",
            vec3(roll_unity).c_str(),
            vec3(vrsdk::rotations::convert_axial_vec3(roll_unity, VRSDK_AXES_UNITY,
                                                      VRSDK_AXES_FRD))
                .c_str());
        std::printf(
            "  convert_vec3         %s -> %s rad/s   the same rate, the polar rule, wrong\n",
            vec3(roll_unity).c_str(),
            vec3(vrsdk::rotations::convert_vec3(roll_unity, VRSDK_AXES_UNITY, VRSDK_AXES_FRD))
                .c_str());
        std::printf(
            "  convert_inertia_vec3 %s -> %s kg m^2  abs(M): a moment of inertia stays positive\n",
            vec3(MOI).c_str(),
            vec3(vrsdk::rotations::convert_inertia_vec3(MOI, VRSDK_AXES_UNITY, VRSDK_AXES_FRD))
                .c_str());
        std::printf("  convert_quat         %s -> %s\n", vec4(yaw90_unity).c_str(),
                    vec4(yaw90_frd).c_str());
        std::printf(
            "    +90 deg about unity's up axis is yaw=%+.2f deg in frd, and the whole conversion "
            "is [x,y,z,w] -> [-z,-x,y,w]: a permutation and two signs, exact.\n",
            vrsdk::rotations::quat_to_euler(yaw90_frd, vrsdk::EulerOrder::Zyx)[2] * RAD_TO_DEG);

        // The same call keyed on the tag this robot actually publishes.
        try {
            const vrsdk::Vec3 by_tag = vrsdk::rotations::convert_vec3(
                position, state.raw.axis_convention, VRSDK_AXES_FRD);
            std::printf("  this robot's position by tag -> %s m\n", vec3(by_tag).c_str());
        } catch (const vrsdk::Error& e) {
            // `detail()` rather than `what()`: the bare sentence the SDK
            // produced, with the code printed beside it instead of prefixed
            // into it.
            std::printf("  this robot's position by tag -> [%d] %s\n", e.code(),
                        e.detail().c_str());
        }
        std::printf(
            "    which is section 1's whole reason for existing: the tag is a convenience and "
            "the nine numbers are the fact.\n");

        // ===== 5: gimbal lock =====
        const vrsdk::Vec3 locked = {LOCKED_DEG[0] * DEG_TO_RAD, LOCKED_DEG[1] * DEG_TO_RAD,
                                    LOCKED_DEG[2] * DEG_TO_RAD};
        const vrsdk::Quat quat = vrsdk::rotations::euler_to_quat(locked, vrsdk::EulerOrder::Zyx);
        const vrsdk::Vec3 extracted =
            vrsdk::rotations::quat_to_euler(quat, vrsdk::EulerOrder::Zyx);
        const std::array<double, 9> from_quat = flat(vrsdk::rotations::quat_to_rotmat(quat));
        const std::array<double, 9> from_euler =
            flat(vrsdk::rotations::euler_to_rotmat(extracted, vrsdk::EulerOrder::Zyx));
        const double rebuilt = max_abs_diff(from_quat.data(), from_euler.data(), from_quat.size());
        std::printf("\n== 5. gimbal lock, in zyx, with the nose vertical ==\n");
        std::printf("  in    roll=%+7.2f deg  pitch=%+7.2f deg  yaw=%+7.2f deg\n", LOCKED_DEG[0],
                    LOCKED_DEG[1], LOCKED_DEG[2]);
        std::printf("  out   roll=%+7.2f deg  pitch=%+7.2f deg  yaw=%+7.2f deg\n",
                    extracted[0] * RAD_TO_DEG, extracted[1] * RAD_TO_DEG,
                    extracted[2] * RAD_TO_DEG);
        std::printf("  same rotation, rebuilt: worst matrix element error %.1e\n", rebuilt);
        if (rebuilt >= EPS) {
            std::fprintf(stderr,
                         "the locked triple does not rebuild its own rotation (error %g)\n",
                         rebuilt);
            return 1;
        }
        std::printf(
            "  Yaw is pinned to 0 and roll carries the whole determined combination. The naive "
            "atan2 pair returns neither: both of its terms are rounding noise at the pole, which "
            "is how (0, 90, 40) comes back from a hand-inlined formula as (26.6, 90, 90).\n");

        robot.remove();
        std::printf("\ndeleted sys_id=%u\n", sys_id);
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
