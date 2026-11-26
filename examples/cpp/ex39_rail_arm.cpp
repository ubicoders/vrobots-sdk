// ex39 -- rail_arm: seven joints to a pose, the vacuum on and off, and back.
//
//     target/cpp-build/Release/ex39_rail_arm          # the warehouse rail arm, sys_id 2
//     target/cpp-build/Release/ex39_rail_arm <id>     # one at another id
//
// A KUKA KR210 riding an 8 m linear rail, with a 400 x 300 mm vacuum pad on its
// flange. The warehouse scene pins it at `sys_id` 2. Its base never moves -- the
// state's kinematics are the base's -- so everything this arm does is in its
// actuator report.
//
// ONE SETPOINT PER JOINT, SI
//
//   `SET_ARM_JOINTS` carries seven numbers in actuator order: the rail in
//   METRES, then A1 to A6 in RADIANS (`VRSDK_RAIL_ARM_*` names the indices). The
//   numbers are the KUKA controller's axis values, and at home they are:
//
//   | joint            | range               | top speed | home |
//   |------------------|---------------------|-----------|------|
//   | rail             | 0 to 8 m            | 1.2 m/s   | 4 m  |
//   | A1, swivel       | -185 to 185 deg     | 105 deg/s | 0    |
//   | A2, shoulder     | -140 to -5 deg      | 94 deg/s  | -60  |
//   | A3, elbow        | -120 to 168 deg     | 100 deg/s | 90   |
//   | A4, forearm roll | -350 to 350 deg     | 136 deg/s | 0    |
//   | A5, wrist bend   | -122.5 to 122.5 deg | 129 deg/s | 60   |
//   | A6, flange turn  | -350 to 350 deg     | 206 deg/s | 0    |
//
//   With A4 at 0 the pad faces straight down whenever A2 + A3 + A5 = 90 deg, as
//   at home; the pose below keeps that sum. The command is a LATCH: every joint
//   is clamped to its range and driven toward its setpoint at its own top speed,
//   independently, so they arrive at different times. Nothing replies. The
//   report is the only proof, and it can end a move three ways:
//
//   - ARRIVED: every joint within a tolerance of its setpoint. The values cross
//     the wire as `float` both ways, so never compare with `==`.
//   - HELD SHORT: CONTACT is not `no_contact`. Every step is tested against the
//     scene before it is taken, and a step into anything is refused -- the
//     protective stop -- with the setpoint still latched.
//   - AT A LIMIT: a setpoint past a joint's range is clamped, so that joint
//     stops short with `joints_normalized` at -1 or 1.
//
// THE TWO ENTRIES AFTER THE JOINTS
//
//   | index | entry   | `normalized`              | `measured`                               |
//   |-------|---------|---------------------------|------------------------------------------|
//   | 7     | VACUUM  | the latch, 0 off and 1 on | the held mass, kg; 0 = nothing           |
//   | 8     | CONTACT | 0                         | 0 none, 1 tool contact, 2 collision stop |
//
//   `SET_ARM_VACUUM` is a latch too. Switched on with nothing under the pad it
//   grips nothing -- it seals only on a loose object within 1 cm of the pad,
//   square to it -- so up in the air the proof is the latch echoing back with a
//   held mass of 0. The table arm reports nine entries too, laid out differently
//   after the joints: `rail_arm_actuators()` decodes by robot type, never by
//   count (ex40).
//
// LEAVING IT AS FOUND
//
//   The joints are read at the start and driven back there at the end, and the
//   vacuum latch is put back as it was. An arm found holding a load is left
//   alone: switching the vacuum off would drop it. `reset()` would also put the
//   arm home -- instantly, with its setpoints -- but home is not necessarily
//   where it was found. Scene-authored: attached by `sys_id`, never deleted.

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
    "The warehouse scene pins its rail arm at sys_id 2, which is the default. For one\n"
    "at another id, pass it:\n"
    "\n"
    "    ex39_rail_arm 2\n"
    "\n"
    "The scene has to be loaded and in Play mode, and on the Community tier only the\n"
    "focused robot is on the wire at all. List what is publishing with:\n"
    "\n"
    "    vrobots topic list";

constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double DEG_TO_RAD = PI / 180.0;

constexpr std::uint32_t SYS_ID = 2;  // the warehouse scene's rail arm
constexpr double HZ = 25.0;
constexpr int MOVE_SAMPLES = 125;   // 5 s; the pose below takes about 0.8
constexpr int VACUUM_SAMPLES = 10;  // 0.4 s for the latch to come back
constexpr double TOLERANCE = 1e-3;  // m on the rail, rad on the others

/// One setpoint or reading per joint: the rail, then A1 to A6.
using Joints = std::array<double, VRSDK_RAIL_ARM_JOINT_COUNT>;

namespace {

/// The largest gap between where the joints are and where they were sent.
double largest_error(const Joints& now, const Joints& goal) {
    double worst = 0.0;
    for (std::size_t i = 0; i < now.size(); ++i) {
        worst = std::fmax(worst, std::fabs(now[i] - goal[i]));
    }
    return worst;
}

/// The rail in metres, A1 to A6 in degrees.
std::string joints(const Joints& j) {
    char out[160];
    std::snprintf(
        out, sizeof out,
        "rail %.3f m  A1 %+6.1f  A2 %+6.1f  A3 %+6.1f  A4 %+6.1f  A5 %+6.1f  A6 %+6.1f deg",
        j[VRSDK_RAIL_ARM_RAIL], j[VRSDK_RAIL_ARM_A1] * RAD_TO_DEG,
        j[VRSDK_RAIL_ARM_A2] * RAD_TO_DEG, j[VRSDK_RAIL_ARM_A3] * RAD_TO_DEG,
        j[VRSDK_RAIL_ARM_A4] * RAD_TO_DEG, j[VRSDK_RAIL_ARM_A5] * RAD_TO_DEG,
        j[VRSDK_RAIL_ARM_A6] * RAD_TO_DEG);
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

/// Attach to `sys_id`, and make sure it really is the rail arm.
///
/// `connect()` takes the robot type on trust, and the table arm reports nine
/// actuator entries too. `z/properties` names each one; once this handle has
/// read it (ex44), `rail_arm_actuators()` refuses a robot whose layout is not
/// the rail's linear joint, six servos, VACUUM and CONTACT.
vrsdk::VirtualRobot attach(std::uint32_t sys_id) {
    vrsdk::VirtualRobot robot(vrsdk::RobotType::RailArm, sys_id);
    robot.connect();
    (void)robot.properties();
    (void)robot.rail_arm_actuators();
    return robot;
}

/// Latch one setpoint per joint, and watch the measured joints converge.
///
/// The setpoint latches, so once would do; sending it every iteration costs
/// nothing and heals a command lost on the way.
void move_to(vrsdk::VirtualRobot& robot, const Joints& goal) {
    const double t0 = robot.states().elapsed;
    for (int i = 0; i < MOVE_SAMPLES; ++i) {
        robot.set_arm_joints(goal);
        robot.rate(HZ);
        const vrsdk::RailArmActuators arm = robot.rail_arm_actuators();
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
    const vrsdk::RailArmActuators arm = robot.rail_arm_actuators();
    std::printf(
        "   not there after %.0f s. joints_normalized %s: a joint at -1 or 1 is clamped at its "
        "limit\n",
        MOVE_SAMPLES / HZ, list_of(arm.joints_normalized).c_str());
}

/// Switch the vacuum, then read the two entries after the joints.
void vacuum(vrsdk::VirtualRobot& robot, bool on) {
    std::printf("\n-- vacuum %s --\n", on ? "on" : "off");
    for (int i = 0; i < VACUUM_SAMPLES; ++i) {
        robot.set_arm_vacuum(on);
        robot.rate(HZ);
    }
    const vrsdk::RailArmActuators arm = robot.rail_arm_actuators();
    std::printf("   VACUUM  [%d]: on=%s, holding=%s, held %.2f kg\n", VRSDK_RAIL_ARM_VACUUM,
                arm.vacuum_on ? "true" : "false", arm.holding ? "true" : "false", arm.held_mass_kg);
    std::printf("   CONTACT [%d]: %s (code %d)\n", VRSDK_RAIL_ARM_CONTACT,
                vrsdk::to_string(arm.contact), static_cast<int>(arm.contact));
}

/// The rail arm's sys_id: the one command-line argument, or `SYS_ID`.
///
/// A sys_id is a `uint32_t` on the wire, so anything that does not fit one is
/// refused rather than wrapped: truncating `4294967297` to `1` would attach to
/// somebody else's robot and look like it worked. `strtoul` reports the overflow
/// through `errno` but wraps a leading '-' silently, so both are checked, and the
/// range check catches the rest on a 64-bit `unsigned long`.
std::uint32_t sys_id_from_args(int argc, char** argv) {
    const char* program = argc > 0 ? argv[0] : "ex39_rail_arm";
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
            std::fprintf(stderr, "no rail arm at sys_id %u: [%d] %s\n\n%s\n", sys_id, e.code(),
                         e.what(), USAGE);
            return 1;
        }
        vrsdk::VirtualRobot& robot = *attached;
        const vrsdk::State s = robot.states();
        std::printf("attached to sys_id=%u '%s'\n", robot.sys_id(), s.name.c_str());

        // ===== as found =====
        const vrsdk::RailArmActuators found = robot.rail_arm_actuators();
        std::printf("found: %s\n", joints(found.joints).c_str());
        std::printf("       vacuum on=%s, holding=%s (%.2f kg), CONTACT %s\n",
                    found.vacuum_on ? "true" : "false", found.holding ? "true" : "false",
                    found.held_mass_kg, vrsdk::to_string(found.contact));
        if (found.holding) {
            std::printf(
                "the pad is holding %.2f kg, and this example switches the vacuum off: nothing "
                "was sent. Release it first, or reset() the arm.\n",
                found.held_mass_kg);
            return 0;
        }

        // ===== to a pose =====
        // Home is (4 m, 0, -60, 90, 0, 60, 0). This pose slides the carriage a
        // metre, swings A1 30 deg and takes the shoulder 10 deg back, and A5
        // follows so that A2 + A3 + A5 stays 90 deg: the pad keeps facing
        // straight down.
        const Joints pose = {5.0, deg(30.0), deg(-70.0), deg(90.0), 0.0, deg(70.0), 0.0};
        std::printf("\n-- to %s --\n", joints(pose).c_str());
        move_to(robot, pose);

        // ===== the vacuum, on and off =====
        vacuum(robot, true);
        vacuum(robot, false);

        // ===== hand it back =====
        std::printf("\n-- back to where it was found --\n");
        move_to(robot, found.joints);
        robot.set_arm_vacuum(found.vacuum_on);
        std::printf(
            "joints back, vacuum on=%s. Scene-authored: left as it was found, never deleted.\n",
            found.vacuum_on ? "true" : "false");
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
