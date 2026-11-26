// ex38 -- hello_palletrobot: four wheel channels, no twist command, and the mix is yours.
//
//     target/cpp-build/Release/ex38_hello_palletrobot          # the warehouse pallet robot, sys_id 1
//     target/cpp-build/Release/ex38_hello_palletrobot <id>     # one at another id
//
// The pallet robot is the low mecanum robot that drives under a pedestal and
// lifts it on a turning pad. The warehouse scene pins it at `sys_id` 1.
// `SET_PALLETROBOT` is six pulse widths, in one order for the command and the
// actuator report alike (`VRSDK_PALLETROBOT_CH_*`):
//
//   | index | channel                    | 1100 us   | 1500 us | 1900 us                         |
//   |-------|----------------------------|-----------|---------|---------------------------------|
//   | 0-3   | wheel ESC FL, FR, RL, RR   | -28 rad/s | stop    | +28 rad/s, rolling forward      |
//   | 4     | pad lift, a position servo | 0 m       | 0.03 m  | 0.06 m                          |
//   | 5     | pad turn, a rate           | -90 deg/s | hold    | +90 deg/s, clockwise from above |
//
// THE ROBOT TAKES NO TWIST
//
//   There is no "go right at 0.3 m/s" command. Each wheel has its own ESC, and
//   turning a body twist into four wheel speeds -- the mecanum mix -- is the
//   client's job. It is a few lines of arithmetic, in `mix()` below, for rollers
//   in the standard X layout, on the robot's virtual geometry R = 0.054 m,
//   lx = 0.35 m and ly = 0.30 m:
//
//     FL = (vx + vy + (lx+ly) wz) / R        FR = (vx - vy - (lx+ly) wz) / R
//     RL = (vx - vy + (lx+ly) wz) / R        RR = (vx + vy - (lx+ly) wz) / R
//     us = round(1500 + 400 * w / 28)        for each wheel speed w in rad/s
//
//   `vx` is forward and `vy` right in m/s, `wz` clockwise seen from above in
//   rad/s: FRU, the frame this robot reports in unless `srv/frames` changed it,
//   so the state's `kin.lin_vel[0]`, `kin.lin_vel[1]` and `kin.ang_vel[2]` are
//   the same three numbers coming back. The wheel channels themselves are
//   physical (+ rolls forward) and are never frame-converted. If any wheel would
//   pass 28 rad/s, all four scale down by the same factor: clipping one alone
//   changes the direction of travel.
//
//   The geometry is virtual -- the model has no rollers -- and nothing on the
//   wire reads it back, so a client keeps its own copy. If `srv/palletrobot` has
//   changed it, the constants below must follow.
//
// DEADBAND, RAMPS, AND WHAT CONFIRMS A MOVE
//
//   The wheels and the pad turn have a +/-20 us deadband, which makes about
//   0.08 m/s the slowest controllable speed; the pad lift has none. The wheels
//   ramp at 60 rad/s^2 and the pad climbs at 0.03 m/s, so the actuator report,
//   `palletrobot_actuators()`, is where a move is confirmed. Its wheel speeds are
//   the motors' own: a blocked robot still reports them, and `kin` is what the
//   body actually did.
//
// LEAVING IT AS FOUND
//
//   Every move is undone by its mirror image -- strafe right then left, turn one
//   way then back -- so the robot ends where it started, to within the open-loop
//   error. The pad returns to the pulse it was found at, and the last frame
//   stops the wheels; with all four stopped the robot holds where it parked. Let
//   it run to the end: a Ctrl-C leaves the last frame latched. Scene-authored:
//   attached by `sys_id`, never deleted.

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
    "The warehouse scene pins its pallet robot at sys_id 1, which is the default. For\n"
    "one at another id, pass it:\n"
    "\n"
    "    ex38_hello_palletrobot 1\n"
    "\n"
    "The scene has to be loaded and in Play mode, and on the Community tier only the\n"
    "focused robot is on the wire at all. List what is publishing with:\n"
    "\n"
    "    vrobots topic list";

constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;

constexpr std::uint32_t SYS_ID = 1;  // the warehouse scene's pallet robot

// The virtual mecanum geometry: the shipped srv/palletrobot values.
constexpr double WHEEL_RADIUS_M = 0.054;   // R
constexpr double HALF_WHEELBASE_M = 0.35;  // lx, centre to the front and rear axles
constexpr double HALF_TRACK_M = 0.30;      // ly, centre to the left and right wheels
constexpr double MAX_WHEEL_RAD_S = 28.0;   // a wheel at 400 us off neutral
constexpr double PAD_MAX_M = 0.06;         // the pad at the top of the band

constexpr double NEUTRAL_US = 1500.0;  // a wheel stopped, the pad turn holding
constexpr double PAD_TOP_US = 1900.0;
constexpr double STRAFE_M_S = 0.3;
constexpr double TURN_RAD_S = 0.5;  // ~29 deg/s
constexpr double HZ = 25.0;
constexpr int MOVE_SAMPLES = 50;  // 2 s per move
constexpr int PAD_SAMPLES = 100;  // 4 s; the whole 0.06 m takes 2
constexpr double PAD_TOLERANCE_M = 0.001;

namespace {

/// The mecanum mix: a body twist into four wheel speeds, rad/s, `[FL, FR, RL, RR]`.
///
/// `vx` forward and `vy` right in m/s, `wz` clockwise from above in rad/s.
std::array<double, 4> mix(double vx, double vy, double wz) {
    const double k = HALF_WHEELBASE_M + HALF_TRACK_M;  // lx + ly
    std::array<double, 4> wheels = {
        (vx + vy + k * wz) / WHEEL_RADIUS_M,
        (vx - vy - k * wz) / WHEEL_RADIUS_M,
        (vx - vy + k * wz) / WHEEL_RADIUS_M,
        (vx + vy - k * wz) / WHEEL_RADIUS_M,
    };

    // Too fast for one wheel: slow all four by the same factor, which keeps the
    // direction of travel. Clipping only the fast one would bend it.
    double fastest = 0.0;
    for (const double w : wheels) {
        fastest = std::fmax(fastest, std::fabs(w));
    }
    if (fastest > MAX_WHEEL_RAD_S) {
        for (double& w : wheels) {
            w *= MAX_WHEEL_RAD_S / fastest;
        }
    }
    return wheels;
}

/// One wheel speed as the pulse that asks for it: 400 us either side of neutral
/// is `MAX_WHEEL_RAD_S`.
///
/// Rounded to a whole microsecond, which is all the wire carries. The SDK rounds
/// to the nearest one as well, so this only makes the printed pulse the one that
/// goes out.
double to_us(double rad_per_s) {
    return std::round(NEUTRAL_US + 400.0 * rad_per_s / MAX_WHEEL_RAD_S);
}

/// The decoded report, in the units a person reads.
std::string report(const vrsdk::PalletRobotActuators& a) {
    const auto [fl, fr, rl, rr] = a.wheel_speeds;
    char out[160];
    std::snprintf(out, sizeof out,
                  "wheels [%+5.1f, %+5.1f, %+5.1f, %+5.1f] rad/s  pad %.3f m  pad angle %+.1f deg",
                  fl, fr, rl, rr, a.pad_height, a.pad_angle * RAD_TO_DEG);
    return out;
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

/// Attach to `sys_id`, and make sure it really is a pallet robot.
///
/// `connect()` takes the robot type on trust; `z/properties` names it. Once
/// this handle has read it (ex44), `palletrobot_actuators()` refuses a robot
/// whose layout is not four wheels, a linear pad lift and a servo pad turn.
vrsdk::VirtualRobot attach(std::uint32_t sys_id) {
    vrsdk::VirtualRobot robot(vrsdk::RobotType::PalletRobot, sys_id);
    robot.connect();
    (void)robot.properties();
    (void)robot.palletrobot_actuators();
    return robot;
}

/// Mix one body twist into wheel pulses and hold it for `MOVE_SAMPLES`.
///
/// The pad pulse rides along in every frame: a frame is all six channels, and
/// one that left the pad out would move it.
void drive(vrsdk::VirtualRobot& robot, const char* label, double vx, double vy, double wz,
           double pad_us) {
    const std::array<double, 4> w = mix(vx, vy, wz);
    const std::array<double, 4> us = {to_us(w[0]), to_us(w[1]), to_us(w[2]), to_us(w[3])};
    std::printf("\n-- %s: vx %+.2f m/s, vy %+.2f m/s, wz %+.2f rad/s --\n", label, vx, vy, wz);
    std::printf(
        "   mix: [FL, FR, RL, RR] = [%+.2f, %+.2f, %+.2f, %+.2f] rad/s -> [%.0f, %.0f, %.0f, "
        "%.0f] us\n",
        w[0], w[1], w[2], w[3], us[0], us[1], us[2], us[3]);
    for (int i = 0; i < MOVE_SAMPLES; ++i) {
        robot.set_palletrobot(us[0], us[1], us[2], us[3], pad_us, NEUTRAL_US);
        robot.rate(HZ);
        if (i % 25 == 24) {
            // FRU: lin_vel is [forward, right, up], ang_vel[2] clockwise from above.
            const vrsdk::State s = robot.states();
            std::printf("   %s  body vx %+.2f vy %+.2f m/s, wz %+.2f rad/s\n",
                        report(robot.palletrobot_actuators()).c_str(), s.kin().lin_vel[0],
                        s.kin().lin_vel[1], s.kin().ang_vel[2]);
        }
    }
}

/// Move the pad to `pad_us`, wheels stopped, until the report says it is there.
void pad(vrsdk::VirtualRobot& robot, double pad_us) {
    // A position servo with no deadband: linear from 0 m at 1100 to the top at 1900.
    const double want = PAD_MAX_M * (pad_us - 1100.0) / 800.0;
    std::printf("\n-- pad lift %g us: %.3f m by the band --\n", pad_us, want);
    const double t0 = robot.states().elapsed;
    for (int i = 0; i < PAD_SAMPLES; ++i) {
        robot.set_palletrobot(NEUTRAL_US, NEUTRAL_US, NEUTRAL_US, NEUTRAL_US, pad_us, NEUTRAL_US);
        robot.rate(HZ);
        const vrsdk::PalletRobotActuators a = robot.palletrobot_actuators();
        const double t = robot.states().elapsed - t0;
        if (std::fabs(a.pad_height - want) < PAD_TOLERANCE_M) {
            std::printf("   t=%5.2fs  %s  -- there\n", t, report(a).c_str());
            return;
        }
        if (i % 10 == 0) {
            std::printf("   t=%5.2fs  %s\n", t, report(a).c_str());
        }
    }
    std::printf("   not there after %.0f s: has srv/palletrobot changed the pad?\n",
                PAD_SAMPLES / HZ);
}

/// The pallet robot's sys_id: the one command-line argument, or `SYS_ID`.
///
/// A sys_id is a `uint32_t` on the wire, so anything that does not fit one is
/// refused rather than wrapped: truncating `4294967297` to `1` would attach to
/// somebody else's robot and look like it worked. `strtoul` reports the overflow
/// through `errno` but wraps a leading '-' silently, so both are checked, and the
/// range check catches the rest on a 64-bit `unsigned long`.
std::uint32_t sys_id_from_args(int argc, char** argv) {
    const char* program = argc > 0 ? argv[0] : "ex38_hello_palletrobot";
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
            std::fprintf(stderr, "no pallet robot at sys_id %u: [%d] %s\n\n%s\n", sys_id, e.code(),
                         e.what(), USAGE);
            return 1;
        }
        vrsdk::VirtualRobot& robot = *attached;
        const vrsdk::State s = robot.states();
        std::printf("attached to sys_id=%u '%s', reporting in \"%s\"\n", robot.sys_id(),
                    s.name.c_str(), s.coord_frame_id.c_str());

        // ===== as found =====
        // The echo is the latch, in command order, so the pad's pulse is read off
        // it here and sent back at the end.
        const double found_pad_us = echo_us(s.actuator(), VRSDK_PALLETROBOT_CH_PAD_LIFT, 1100);
        std::printf("found: echo %s = [FL, FR, RL, RR, pad lift, pad turn] us\n",
                    pwm_of(s.actuator()).c_str());
        std::printf("       %s\n", report(robot.palletrobot_actuators()).c_str());

        // ===== strafe right, then back =====
        drive(robot, "strafe right", 0.0, STRAFE_M_S, 0.0, found_pad_us);
        drive(robot, "strafe back", 0.0, -STRAFE_M_S, 0.0, found_pad_us);

        // ===== turn on the spot, clockwise, then back =====
        drive(robot, "turn clockwise", 0.0, 0.0, TURN_RAD_S, found_pad_us);
        drive(robot, "turn back", 0.0, 0.0, -TURN_RAD_S, found_pad_us);

        // ===== stop =====
        // A zero twist mixes to 1500 on every wheel: a stop is just another frame.
        drive(robot, "stop", 0.0, 0.0, 0.0, found_pad_us);

        // ===== the pad: up to the top, then back =====
        pad(robot, PAD_TOP_US);
        pad(robot, found_pad_us);

        // ===== hand it back =====
        // pad() ends on a frame with the wheels stopped and the pad turn holding,
        // and that frame stays latched.
        std::printf(
            "\nwheels stopped, pad back at %g us. Scene-authored: left as it was found, never "
            "deleted.\n",
            found_pad_us);
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
