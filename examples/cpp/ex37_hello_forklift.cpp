// ex37 -- hello_forklift: forks up and out, a short drive, a brake, and everything put back.
//
//     target/cpp-build/Release/ex37_hello_forklift          # the warehouse forklift, sys_id 0
//     target/cpp-build/Release/ex37_hello_forklift <id>     # a forklift at another id
//
// The warehouse scene pins its counterbalance forklift at `sys_id` 0, so that is
// the default. Its front pair drives, its REAR PAIR STEERS -- the tail swings out
// in a turn -- and its mast carries two POSITION SERVOS. One command,
// `SET_FORKLIFT`, drives all four, RC style:
//
//   | channel                  | 1100 us      | 1500 us                 | 1900 us               |
//   |--------------------------|--------------|-------------------------|-----------------------|
//   | steer                    | full left    | straight                | full right (45 deg)   |
//   | throttle                 | full reverse | stop, on the idle brake | full forward (~5 m/s) |
//   | lift                     | -0.215 m     | 0.448 m                 | 1.11 m                |
//   | fork width               | 0.158 m      | 0.746 m                 | 1.334 m               |
//   | brake, an optional fifth | released     | half                    | full                  |
//
// TWO KINDS OF CHANNEL
//
//   Steer and throttle centre on 1500 with a +/-20 us deadband, like the truck's
//   (ex05). Lift and fork width have no centre at all: every microsecond is a
//   place on the mast, linear across the band -- the arithmetic in `lift_m()` and
//   `width_m()` below. And the mast SLEWS there, the lift at 0.5 m/s and the width
//   at only 0.1 m/s, so `actuator.pwm` echoes a new pulse on the very next sample
//   while the forks are still on their way. The move is done when
//   `actuator.measured` says so, and `forklift_actuators()` decodes that into
//   metres: the typed actuator report is the confirmation, not the echo.
//
// THE REPORT IS NOT IN COMMAND ORDER
//
//   The command is `[steer, throttle, lift, width]`. The report leads with the
//   drive -- `[drive, steering, lift, width]` -- so `actuator.pwm[0]` echoes the
//   THROTTLE. `VRSDK_FORKLIFT_CH_*` indexes the command, `VRSDK_FORKLIFT_ACT_*`
//   the report.
//
// THE BRAKE
//
//   A fifth channel, sent as `set_forklift`'s optional last argument, and
//   BOTTOM-ANCHORED: 1100 is released and 1900 is full, not 1500. It latches with
//   the rest of the frame, and the next four-channel `set_forklift` releases it.
//   The report has no brake entry; the proof is the drive speed falling to zero.
//
// LEAVING IT AS FOUND
//
//   Every frame latches, so whatever this program sends last is what the
//   forklift keeps doing after it exits. The echo *is* the latch, so the forks'
//   pulses are read off it at the start and sent again at the end, with the drive
//   centred and the brake released. The forklift drives back as far as it went --
//   open loop, so to within a few centimetres. `reset()` would be quicker -- it
//   snaps the mast to the reset pulses instead of slewing it -- but it also
//   teleports the forklift to where the scene started it, which is "home", not
//   "where it was found" (ex21).
//
//   Let it run to the end: a Ctrl-C mid-drive leaves the throttle latched, and
//   the forklift keeps driving. Scene-authored: attached by `sys_id`, never
//   deleted.

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>

#include <vrobots_sdk.hpp>

constexpr const char* USAGE =
    "The warehouse scene pins its forklift at sys_id 0, which is the default. For a\n"
    "forklift at another id, pass it:\n"
    "\n"
    "    ex37_hello_forklift 0\n"
    "\n"
    "The scene has to be loaded and in Play mode, and on the Community tier only the\n"
    "focused robot is on the wire at all. List what is publishing with:\n"
    "\n"
    "    vrobots topic list";

constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;

constexpr std::uint32_t SYS_ID = 0;  // the warehouse scene's forklift

// The shipped mast ranges, for lift_m() and width_m(). srv/forklift can move
// them (ex44), and then the expectations printed below are wrong.
constexpr double LIFT_MIN_M = -0.215;
constexpr double LIFT_MAX_M = 1.11;
constexpr double WIDTH_MIN_M = 0.158;
constexpr double WIDTH_MAX_M = 1.334;
constexpr double TYRE_RADIUS_M = 0.36;  // front tyres: drive rad/s times this is m/s

constexpr double NEUTRAL_US = 1500.0;     // straight ahead, and stopped on the idle brake
constexpr double LIFT_UP_US = 1500.0;     // the carriage to 0.448 m
constexpr double WIDTH_OUT_US = 1500.0;   // the forks to 0.746 m apart
constexpr double FORWARD_US = 1600.0;     // a quarter of the forward half: gentle
constexpr double BACK_US = 1400.0;        // the same quarter in reverse
constexpr double BRAKE_FULL_US = 1900.0;  // bottom-anchored: 1100 is released
constexpr double HZ = 20.0;
constexpr int DRIVE_SAMPLES = 50;  // 2.5 s each way, about a metre
constexpr int BRAKE_SAMPLES = 40;  // 2 s, far more than a stop needs
constexpr int MAST_SAMPLES = 200;  // 10 s; the slow width needs ~3 s for this move
constexpr double TOLERANCE_M = 0.005;
constexpr double STOPPED_RAD_S = 0.05;

namespace {

/// Where a lift pulse puts the carriage: linear across the shipped band, no
/// deadband.
double lift_m(double us) { return LIFT_MIN_M + (us - 1100.0) / 800.0 * (LIFT_MAX_M - LIFT_MIN_M); }

/// Where a fork-width pulse puts the forks, centre to centre: the same rule.
double width_m(double us) {
    return WIDTH_MIN_M + (us - 1100.0) / 800.0 * (WIDTH_MAX_M - WIDTH_MIN_M);
}

/// The decoded report, in the units a person reads.
std::string report(const vrsdk::ForkliftActuators& a) {
    char out[160];
    std::snprintf(out, sizeof out,
                  "lift %+.3f m  width %.3f m  drive %+5.2f rad/s (%+.2f m/s)  steering %+5.1f deg",
                  a.lift_height, a.fork_width, a.drive_speed, a.drive_speed * TYRE_RADIUS_M,
                  a.steering_angle * RAD_TO_DEG);
    return out;
}

/// Straight-line distance between two positions, in whatever frame they share.
double distance(const double a[3], const double b[3]) {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// One echoed pulse width, or `fallback` when the report is shorter.
double echo_us(const vrsdk_actuator_t& a, std::uint32_t index, std::uint32_t fallback) {
    return index < a.pwm_count ? a.pwm[index] : fallback;
}

/// The echoed pulse widths, as a list.
std::string pwm_of(const vrsdk_actuator_t& a) {
    std::string out = "[";
    for (std::uint32_t i = 0; i < a.pwm_count; ++i) {
        char buffer[16];
        std::snprintf(buffer, sizeof buffer, "%s%u", i ? ", " : "", a.pwm[i]);
        out += buffer;
    }
    return out + "]";
}

/// Attach to `sys_id`, and make sure it really is a forklift.
///
/// `connect()` takes the robot type on trust: nothing in a state message names
/// it, and a truck at this id would attach just as happily. `z/properties` does
/// name it -- the kind of every actuator entry -- and once this handle has read
/// it (ex44 prints the rest), `forklift_actuators()` refuses a robot whose
/// layout is not `[wheel, servo, linear, linear]` instead of decoding its
/// report into wrong numbers.
vrsdk::VirtualRobot attach(std::uint32_t sys_id) {
    vrsdk::VirtualRobot robot(vrsdk::RobotType::Forklift, sys_id);
    robot.connect();
    (void)robot.properties();
    (void)robot.forklift_actuators();
    return robot;
}

/// Stream one mast frame -- straight, stopped, brake released -- until the
/// forks are where the band says those pulses put them.
///
/// The frame latches, so sending it once would do; sending it every iteration
/// costs nothing and heals a frame lost on the way.
void move_mast(vrsdk::VirtualRobot& robot, double lift_us, double width_us) {
    const double want_lift = lift_m(lift_us);
    const double want_width = width_m(width_us);
    const double t0 = robot.states().elapsed;
    for (int i = 0; i < MAST_SAMPLES; ++i) {
        robot.set_forklift(NEUTRAL_US, NEUTRAL_US, lift_us, width_us);
        robot.rate(HZ);
        const vrsdk::ForkliftActuators a = robot.forklift_actuators();
        const double t = robot.states().elapsed - t0;
        const bool there = std::fabs(a.lift_height - want_lift) < TOLERANCE_M &&
                           std::fabs(a.fork_width - want_width) < TOLERANCE_M;
        if (there) {
            std::printf("   t=%5.2fs  %s  -- there\n", t, report(a).c_str());
            return;
        }
        if (i % 10 == 0) {
            std::printf("   t=%5.2fs  %s\n", t, report(a).c_str());
        }
    }
    std::printf("   not there after %.0f s: is the band or a range not the shipped one (ex44)?\n",
                MAST_SAMPLES / HZ);
}

/// Hold one throttle, straight ahead, with the forks where they are.
///
/// A frame carries all four channels, so the mast pulses ride along in every
/// drive frame: one that left them out would move the forks as well.
void drive(vrsdk::VirtualRobot& robot, const char* label, double throttle_us, double lift_us,
           double width_us) {
    std::printf("\n-- %s: throttle %g us for %.1f s --\n", label, throttle_us, DRIVE_SAMPLES / HZ);
    for (int i = 0; i < DRIVE_SAMPLES; ++i) {
        robot.set_forklift(NEUTRAL_US, throttle_us, lift_us, width_us);
        robot.rate(HZ);
        if (i % 10 == 9) {
            std::printf("   %s\n", report(robot.forklift_actuators()).c_str());
        }
    }
}

/// Centre the throttle and add the fifth channel at full until the front axle
/// stops. The brake stays latched until the next four-channel frame.
void brake(vrsdk::VirtualRobot& robot, double lift_us, double width_us) {
    const double from = robot.forklift_actuators().drive_speed;
    const double t0 = robot.states().elapsed;
    for (int i = 0; i < BRAKE_SAMPLES; ++i) {
        robot.set_forklift(NEUTRAL_US, NEUTRAL_US, lift_us, width_us, BRAKE_FULL_US);
        robot.rate(HZ);
        const vrsdk::ForkliftActuators a = robot.forklift_actuators();
        if (std::fabs(a.drive_speed) < STOPPED_RAD_S) {
            std::printf("   brake %g us: drive %+.2f -> %+.2f rad/s in %.2f s\n", BRAKE_FULL_US,
                        from, a.drive_speed, robot.states().elapsed - t0);
            return;
        }
    }
    std::printf("   still rolling after %.0f s of full brake\n", BRAKE_SAMPLES / HZ);
}

/// The forklift's sys_id: the one command-line argument, or `SYS_ID`.
///
/// A sys_id is a `uint32_t` on the wire, so anything that does not fit one is
/// refused rather than wrapped: truncating `4294967297` to `1` would attach to
/// somebody else's robot and look like it worked. `strtoul` reports the overflow
/// through `errno` but wraps a leading '-' silently, so both are checked, and the
/// range check catches the rest on a 64-bit `unsigned long`.
std::uint32_t sys_id_from_args(int argc, char** argv) {
    const char* program = argc > 0 ? argv[0] : "ex37_hello_forklift";
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
            std::fprintf(stderr, "no forklift at sys_id %u: [%d] %s\n\n%s\n", sys_id, e.code(),
                         e.what(), USAGE);
            return 1;
        }
        vrsdk::VirtualRobot& robot = *attached;
        const vrsdk::State s = robot.states();
        std::printf("attached to sys_id=%u '%s', reporting in \"%s\"\n", robot.sys_id(),
                    s.name.c_str(), s.coord_frame_id.c_str());

        // ===== as found =====
        // The echo is the latch -- the last frame anybody sent -- so it is also
        // what to send back at the end. Report order: pwm[DRIVE] is the throttle.
        const double found_lift_us = echo_us(s.actuator(), VRSDK_FORKLIFT_ACT_LIFT, 1100);
        const double found_width_us = echo_us(s.actuator(), VRSDK_FORKLIFT_ACT_FORK_WIDTH, 1301);
        std::printf("found: echo %s = [throttle, steer, lift, width] us\n",
                    pwm_of(s.actuator()).c_str());
        std::printf("       %s\n", report(robot.forklift_actuators()).c_str());

        // ===== forks up, forks out =====
        std::printf("\n-- lift %g us, width %g us: by the band %.3f m and %.3f m --\n", LIFT_UP_US,
                    WIDTH_OUT_US, lift_m(LIFT_UP_US), width_m(WIDTH_OUT_US));
        move_mast(robot, LIFT_UP_US, WIDTH_OUT_US);

        // ===== a gentle drive forward, and the brake =====
        const vrsdk::State start = robot.states();
        drive(robot, "forward", FORWARD_US, LIFT_UP_US, WIDTH_OUT_US);
        brake(robot, LIFT_UP_US, WIDTH_OUT_US);

        // ===== and back the same way =====
        drive(robot, "back", BACK_US, LIFT_UP_US, WIDTH_OUT_US);
        brake(robot, LIFT_UP_US, WIDTH_OUT_US);

        // ===== hand it back =====
        // A four-channel frame, so the brake is released too.
        std::printf(
            "\n-- hand it back: straight, stopped, brake released, forks to %g / %g us --\n",
            found_lift_us, found_width_us);
        move_mast(robot, found_lift_us, found_width_us);
        const vrsdk::State end = robot.states();
        std::printf(
            "ended %.2f m from where it started (open loop, so not quite 0). Scene-authored: left "
            "as it was found, never deleted.\n",
            distance(start.kin().lin_pos, end.kin().lin_pos));
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
