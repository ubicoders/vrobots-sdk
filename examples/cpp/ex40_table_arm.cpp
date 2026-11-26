// ex40 -- table_arm: six joints, a two-finger gripper in metres, and what the fingers report.
//
//     target/cpp-build/Release/ex40_table_arm          # the table-arm scene's UR10, sys_id 0
//     target/cpp-build/Release/ex40_table_arm <id>     # one at another id
//
// A UR10 with a Robotiq 2F-85 gripper, at the back of the table-arm scene's
// sorting cell, pinned at `sys_id` 0. The base never moves.
//
// JOINTS
//
//   `SET_ARM_JOINTS` takes six setpoints, A1 to A6 -- base, shoulder, elbow and
//   three wrists -- in RADIANS, each within -360 to 360 deg, reached at 120 deg/s
//   (A1-A2) or 180 deg/s (A3-A6). It is the rail arm's command (ex39) with one
//   joint fewer, and the same rules: a latch, every joint clamped and driven
//   independently, every step tested against the scene first. This arm also
//   refuses to drive into ITSELF. Home is whatever pose the arm loaded in, and
//   nothing on the wire publishes it -- so read the joints once after
//   connecting, which is also how this example knows where to come back to.
//
// THE GRIPPER IS IN METRES
//
//   `SET_ARM_GRIPPER` takes the gap between the pads in METRES, clamped to 0 to
//   0.085 (`VRSDK_TABLE_ARM_GRIPPER_STROKE_M`, the 2F-85's stroke). The clamp is
//   silent, so a millimetre figure such as `40.0` opens the gripper fully. The
//   fingers move at 0.15 m/s of opening, the whole stroke in under 0.6 s.
//
//   Closing on an object stops the pads at its width, and the object is HELD
//   when both pads press it from inside and it is a loose object within the
//   5 kg payload. Closed on thin air, as here, the pads simply arrive and nothing
//   is held.
//
// THE THREE ENTRIES AFTER THE JOINTS
//
//   | index | entry   | `normalized`             | `measured`                               |
//   |-------|---------|--------------------------|------------------------------------------|
//   | 6     | GRIPPER | the latch, as a fraction | the gap between the pads now, m          |
//   |       |         | of the stroke            |                                          |
//   | 7     | CONTACT | 0                        | 0 none, 1 tool contact, 2 collision stop |
//   | 8     | HOLD    | 1 holding, 0 not         | the held mass, kg                        |
//
//   Nine entries, like the rail arm's, and laid out differently after the
//   joints: `table_arm_actuators()` decodes them by robot type, never by count.
//   A finger stopping on an object is NOT a contact -- that is how the gripper
//   feels it -- so CONTACT speaks only for the arm's joint steps.
//
// LEAVING IT AS FOUND
//
//   The joints and the gripper latch are read at the start and put back at the
//   end. An arm found holding something is left alone: opening would drop it.
//   Scene-authored: attached by `sys_id`, never deleted.

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>

#include <vrobots_sdk.hpp>

constexpr const char* USAGE =
    "The table-arm scene pins its UR10 at sys_id 0, which is the default. For one at\n"
    "another id, pass it:\n"
    "\n"
    "    ex40_table_arm 0\n"
    "\n"
    "The scene has to be loaded and in Play mode; its scene card needs the Pro tier.\n"
    "List what is publishing with:\n"
    "\n"
    "    vrobots topic list";

constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double DEG_TO_RAD = PI / 180.0;

constexpr std::uint32_t SYS_ID = 0;   // the table-arm scene's UR10
constexpr double A1_TURN_DEG = 15.0;  // swing the base
constexpr double A6_TURN_DEG = 45.0;  // and turn the gripper about its own axis
constexpr double GRIP_M = 0.040;      // METRES, so 40 mm. A bare 40.0 would open fully
constexpr double HZ = 25.0;
constexpr int MOVE_SAMPLES = 125;   // 5 s; this move takes well under 1
constexpr int GRIP_SAMPLES = 125;   // 5 s: the whole stroke takes 0.6, or 4.3 at srv/arm's slowest
constexpr double TOLERANCE = 1e-3;  // rad
constexpr double GRIP_TOLERANCE_M = 0.0005;  // half a millimetre

/// One setpoint or reading per joint, A1 to A6.
using Joints = std::array<double, VRSDK_TABLE_ARM_JOINT_COUNT>;

namespace {

/// The three entries after the joints, decoded.
std::string tool(const vrsdk::TableArmActuators& arm) {
    char out[256];
    std::snprintf(out, sizeof out,
                  "GRIPPER [%d]: latch %.1f mm, pads %.1f mm apart   HOLD [%d]: holding=%s, %.2f kg"
                  "   CONTACT [%d]: %s (code %d)",
                  VRSDK_TABLE_ARM_GRIPPER, arm.gripper_command_m * 1e3, arm.gripper_opening_m * 1e3,
                  VRSDK_TABLE_ARM_HOLD, arm.holding ? "true" : "false", arm.held_mass_kg,
                  VRSDK_TABLE_ARM_CONTACT, vrsdk::to_string(arm.contact),
                  static_cast<int>(arm.contact));
    return out;
}

/// The largest gap between where the joints are and where they were sent.
double largest_error(const Joints& now, const Joints& goal) {
    double worst = 0.0;
    for (std::size_t i = 0; i < now.size(); ++i) {
        worst = std::fmax(worst, std::fabs(now[i] - goal[i]));
    }
    return worst;
}

/// A1 to A6, in degrees.
std::string joints(const Joints& j) {
    char out[160];
    std::snprintf(out, sizeof out,
                  "A1 %+7.1f  A2 %+7.1f  A3 %+7.1f  A4 %+7.1f  A5 %+7.1f  A6 %+7.1f deg",
                  j[VRSDK_TABLE_ARM_A1] * RAD_TO_DEG, j[VRSDK_TABLE_ARM_A2] * RAD_TO_DEG,
                  j[VRSDK_TABLE_ARM_A3] * RAD_TO_DEG, j[VRSDK_TABLE_ARM_A4] * RAD_TO_DEG,
                  j[VRSDK_TABLE_ARM_A5] * RAD_TO_DEG, j[VRSDK_TABLE_ARM_A6] * RAD_TO_DEG);
    return out;
}

/// Numbers as a list, `[a, b, c]`.
std::string list_of(const Joints& values) {
    std::string out = "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        char buffer[32];
        std::snprintf(buffer, sizeof buffer, "%s%.3f", i ? ", " : "", values[i]);
        out += buffer;
    }
    return out + "]";
}

/// Degrees to the radians the command takes.
double deg(double degrees) { return degrees * DEG_TO_RAD; }

/// Attach to `sys_id`, and make sure it really is the table arm.
///
/// `connect()` takes the robot type on trust, and the rail arm reports nine
/// actuator entries too. `z/properties` names each one; once this handle has
/// read it (ex44), `table_arm_actuators()` refuses a robot whose layout is not
/// six servos, GRIPPER, CONTACT and HOLD.
vrsdk::VirtualRobot attach(std::uint32_t sys_id) {
    vrsdk::VirtualRobot robot(vrsdk::RobotType::TableArm, sys_id);
    robot.connect();
    (void)robot.properties();
    (void)robot.table_arm_actuators();
    return robot;
}

/// Latch six setpoints and watch the measured joints converge -- or stop short,
/// with CONTACT saying why.
void move_to(vrsdk::VirtualRobot& robot, const Joints& goal) {
    const double t0 = robot.states().elapsed;
    for (int i = 0; i < MOVE_SAMPLES; ++i) {
        // A latch: once would do, and every iteration heals a lost command.
        robot.set_arm_joints(goal);
        robot.rate(HZ);
        const vrsdk::TableArmActuators arm = robot.table_arm_actuators();
        const double t = robot.states().elapsed - t0;
        const double worst = largest_error(arm.joints, goal);
        if (worst < TOLERANCE) {
            std::printf("   t=%5.2fs  %s  -- arrived\n", t, joints(arm.joints).c_str());
            return;
        }
        if (arm.contact != vrsdk::ArmContact::NoContact) {
            std::printf("   t=%5.2fs  %s  -- held short: CONTACT %s (code %d)\n", t,
                        joints(arm.joints).c_str(), vrsdk::to_string(arm.contact),
                        static_cast<int>(arm.contact));
            return;
        }
        if (i % 5 == 0) {
            std::printf("   t=%5.2fs  %s  worst error %.4f\n", t, joints(arm.joints).c_str(),
                        worst);
        }
    }
    const vrsdk::TableArmActuators arm = robot.table_arm_actuators();
    std::printf(
        "   not there after %.0f s. joints_normalized %s: a joint at -1 or 1 is clamped at its "
        "limit\n",
        MOVE_SAMPLES / HZ, list_of(arm.joints_normalized).c_str());
}

/// Command an opening, metres, and read GRIPPER, HOLD and CONTACT once the pads
/// have arrived -- or once they have closed on something.
void gripper(vrsdk::VirtualRobot& robot, double opening_m) {
    std::printf("\n-- gripper to %.1f mm --\n", opening_m * 1e3);
    const double t0 = robot.states().elapsed;
    for (int i = 0; i < GRIP_SAMPLES; ++i) {
        robot.set_arm_gripper(opening_m);
        robot.rate(HZ);
        const vrsdk::TableArmActuators arm = robot.table_arm_actuators();
        if (arm.holding || std::fabs(arm.gripper_opening_m - opening_m) < GRIP_TOLERANCE_M) {
            std::printf("   t=%5.2fs  %s\n", robot.states().elapsed - t0, tool(arm).c_str());
            return;
        }
    }
    // Stopped short with nothing held: something is between the pads that the
    // gripper will not hold -- fixed scenery, or over the payload.
    std::printf("   stopped short, holding nothing: %s\n",
                tool(robot.table_arm_actuators()).c_str());
}

/// The table arm's sys_id: the one command-line argument, or `SYS_ID`.
///
/// A sys_id is a `uint32_t` on the wire, so anything that does not fit one is
/// refused rather than wrapped: truncating `4294967297` to `1` would attach to
/// somebody else's robot and look like it worked. `strtoul` reports the overflow
/// through `errno` but wraps a leading '-' silently, so both are checked, and the
/// range check catches the rest on a 64-bit `unsigned long`.
std::uint32_t sys_id_from_args(int argc, char** argv) {
    const char* program = argc > 0 ? argv[0] : "ex40_table_arm";
    if (argc < 2) {
        return SYS_ID;
    }
    char* end = nullptr;
    errno = 0;
    const unsigned long parsed = std::strtoul(argv[1], &end, 10);
    if (end == argv[1] || *end != '\0' || argv[1][0] == '-' || errno == ERANGE ||
        parsed > std::numeric_limits<std::uint32_t>::max()) {
        std::fprintf(stderr, "usage: %s [sys_id]\n\n%s\n", program, USAGE);
        std::exit(2);
    }
    return static_cast<std::uint32_t>(parsed);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        // ===== setup =====
        vrsdk::check_version();
        const std::uint32_t sys_id = sys_id_from_args(argc, argv);
        std::optional<vrsdk::VirtualRobot> attached;
        try {
            attached = attach(sys_id);
        } catch (const vrsdk::Error& e) {
            std::fprintf(stderr, "no table arm at sys_id %u: [%d] %s\n\n%s\n", sys_id, e.code(),
                         e.what(), USAGE);
            return 1;
        }
        vrsdk::VirtualRobot& robot = *attached;
        const vrsdk::State s = robot.states();
        std::printf("attached to sys_id=%u '%s'\n", robot.sys_id(), s.name.c_str());

        // ===== as found =====
        // There is no home pose on the wire: these joints are the only record of it.
        const vrsdk::TableArmActuators found = robot.table_arm_actuators();
        std::printf("found: %s\n", joints(found.joints).c_str());
        std::printf("       %s\n", tool(found).c_str());
        if (found.holding) {
            std::printf(
                "the gripper is holding %.2f kg, and this example opens it: nothing was sent. "
                "Let go of it first, or reset() the arm.\n",
                found.held_mass_kg);
            return 0;
        }

        // ===== joints =====
        Joints goal = found.joints;
        goal[VRSDK_TABLE_ARM_A1] += deg(A1_TURN_DEG);
        goal[VRSDK_TABLE_ARM_A6] += deg(A6_TURN_DEG);
        std::printf("\n-- A1 %+g deg, A6 %+g deg: to %s --\n", A1_TURN_DEG, A6_TURN_DEG,
                    joints(goal).c_str());
        move_to(robot, goal);

        // ===== close the gripper to a width, then open it =====
        gripper(robot, GRIP_M);
        gripper(robot, found.gripper_command_m);

        // ===== hand it back =====
        std::printf("\n-- back to where it was found --\n");
        move_to(robot, found.joints);
        std::printf(
            "joints and gripper back. Scene-authored: left as it was found, never deleted.\n");
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
