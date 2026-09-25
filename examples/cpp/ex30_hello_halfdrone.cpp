// ex30 -- hello_halfdrone: two rotors, one degree of freedom, and a capability
// probe.
//
//     target/cpp-build/Release/ex30_hello_halfdrone <sys_id>
//
// A half-drone is a multirotor cut in half: a bar on a pivot with a rotor at
// each end of it, free to ROLL -- about the FRD forward axis, with the two arms
// lying along the left-right one -- and constrained in everything else. It
// exists so that attitude control can be taught without six degrees of freedom
// arguing back: the plant is
// `I*theta'' = L2*F2 - L1*F1 + g*cos(theta)*m*(L1 - L2) - c*theta'`, and the
// only input is the difference between two pulse widths.
//
// Like the cart-pole it is SCENE-AUTHORED: not in the spawn catalog, so attach
// by sys_id, and the ids move between sessions (`vrobots topic list`). That is
// the one thing in these examples that cannot be a constant, so it is the single
// positional argument.
//
// WHAT MAKES THIS EXAMPLE WORTH ITS OWN FILE: THE PROBE
//
//   Every robot serves the same seven services -- activate, reset, params, skin,
//   cameras, sensors, frames -- and then each TYPE adds its own: `srv/rotors`
//   for a multirotor, `srv/drive` for a truck, `srv/msd`, `srv/cartpole`. **The
//   half-drone adds nothing.** Two rotors and a hinge need no configuration
//   service.
//
//   So asking it for `srv/rotors` is a GET to a key nobody serves, and that
//   answers as VRSDK_ERR_NO_RESPONDER after the service timeout. THAT IS THE
//   CAPABILITY PROBE, and it is the only one this API has: nothing in a state
//   message names the robot's type, and a command for the wrong type is silently
//   ignored rather than refused. If you need to know what you are attached to,
//   ask it for a service only that type serves and time the answer.
//
//   The cost is real -- a probe is a timeout, not a lookup -- so this example
//   shortens `service_timeout_s` to 3 s for the run. It is also
//   indistinguishable from a simulator that is not running, which
//   `vrobots topic list` tells apart.
//
// EXACTLY TWO PULSE WIDTHS
//
//   `SET_MR_PWM` carries one entry per rotor and the robot refuses any other
//   count with a warning you cannot see. `set_mr_pwm` takes a vector, so the
//   count is whatever you pass. The run below sends the four-entry form first,
//   on purpose, and shows the echo not moving -- ex08's lesson with a robot that
//   really does implement the id.
//
//   | index | rotor                       | effect                              |
//   |-------|-----------------------------|-------------------------------------|
//   | 0     | rotor1, the FRD LEFT arm    | rotor1 high rolls FRD roll POSITIVE |
//   | 1     | rotor2, the FRD RIGHT arm   | rotor2 high rolls it back           |
//
//   THIS AIRFRAME'S BAND TOPS OUT AT 1900 us, not the stock rotor's 2000. The
//   SDK validates against the wider 1100-2000 (it does not know the airframe),
//   so 1950 is accepted here and clamped there. A fresh spawn -- and every reset
//   -- latches [1100, 1100].
//
// READING THE TILT
//
//   There are no Euler angles on the state wire, so every consumer derives them
//   the same way, from `kin.quat`, ordered [x, y, z, w]:
//
//       roll = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))     rad, FRD
//       rate = kin.ang_vel[0]                              rad/s, FRD roll rate
//
//   The bar dead-stops at its mechanical travel (70 degrees each side by
//   default), so a large enough difference just parks it against the stop.
//
// Scene-authored: this example never deletes the robot, and it idles both rotors
// on the way out because a pulse width LATCHES.

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include <vrobots_sdk.hpp>

constexpr const char* USAGE =
    "The half-drone is scene-authored, and sys ids are handed out at scene load and\n"
    "keep incrementing, so there is no id this file could hard-code. Pass the live\n"
    "one:\n"
    "\n"
    "    ex30_hello_halfdrone 4\n"
    "\n"
    "List what is publishing with:\n"
    "\n"
    "    vrobots topic list";

constexpr double PROBE_TIMEOUT_S = 3.0;
constexpr double THROTTLE_US = 1600.0;
constexpr double PWM_MIN_US = 1100.0;  // this airframe's floor -- and its idle
constexpr double PWM_MAX_US = 1900.0;  // and its ceiling, NOT the stock rotor's 2000
constexpr double HZ = 25.0;
constexpr int HOLD_SAMPLES = 50;  // ~2 s per differential setting

/// Differential pulse width added to rotor1 and subtracted from rotor2.
constexpr double SWEEP_US[] = {0.0, 90.0, 0.0, -90.0, 0.0};

namespace {

/// FRD roll in degrees from the state quaternion, ordered [x, y, z, w].
///
/// There are no Euler angles on the wire, so this conversion is every consumer's
/// job -- the simulator's own panels do exactly this.
double roll_deg(const double q[4]) {
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    return std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * 180.0 /
           3.14159265358979323846;
}

/// The pwm echo or the measured channels, whichever array is asked for.
std::string list_u32(const std::uint32_t* values, std::uint32_t count) {
    std::string out = "[";
    for (std::uint32_t i = 0; i < count; ++i) {
        char buffer[16];
        std::snprintf(buffer, sizeof buffer, "%s%u", i ? "," : "", values[i]);
        out += buffer;
    }
    return out + "]";
}

std::string list_f64(const double* values, std::uint32_t count) {
    std::string out = "[";
    for (std::uint32_t i = 0; i < count; ++i) {
        char buffer[32];
        std::snprintf(buffer, sizeof buffer, "%s%+.3f", i ? "," : "", values[i]);
        out += buffer;
    }
    return out + "]";
}

/// The scene-authored half-drone's sys_id, from the one command-line argument.
///
/// A sys_id is a `uint32_t` on the wire, so anything that does not fit one is
/// refused rather than wrapped: truncating `4294967297` to `1` would attach to
/// somebody else's robot and look like it worked. `strtoul` reports the overflow
/// through `errno` but wraps a leading '-' silently, so both are checked, and the
/// range check catches the rest on a 64-bit `unsigned long`.
std::uint32_t sys_id_from_args(int argc, char** argv) {
    const char* program = argc > 0 ? argv[0] : "ex30_hello_halfdrone";
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s <sys_id>\n\n%s\n", program, USAGE);
        std::exit(2);
    }
    char* end = nullptr;
    errno = 0;
    const unsigned long parsed = std::strtoul(argv[1], &end, 10);
    if (end == argv[1] || *end != '\0' || argv[1][0] == '-' || errno == ERANGE ||
        parsed > std::numeric_limits<std::uint32_t>::max()) {
        std::fprintf(stderr, "usage: %s <sys_id>\n\n%s\n", program, USAGE);
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

        // A capability probe is spent entirely on waiting for an answer that is
        // not coming, so budget it deliberately rather than taking the 8 s
        // default.
        vrsdk_connect_options_t options{};
        vrsdk_options_default(&options);
        options.service_timeout_s = PROBE_TIMEOUT_S;

        vrsdk::VirtualRobot robot(vrsdk::RobotType::HalfDrone, sys_id, &options);
        robot.connect();
        std::printf("attached to sys_id=%u (HalfDrone), frame=\"%s\"\n", robot.sys_id(),
                    robot.states().coord_frame_id.c_str());

        // ===== the probe =====
        std::printf("\n-- what does this robot serve? --\n");
        try {
            robot.reset();
            std::printf("  srv/reset    answered  -> one of the seven every robot serves\n");
        } catch (const vrsdk::Error& e) {
            std::printf("  srv/reset    [%d] %s\n", e.code(), e.what());
        }
        try {
            robot.configure_rotors({vrsdk::rotor_spec(), vrsdk::rotor_spec()});
            std::printf("  srv/rotors   UNEXPECTED: something answered\n");
        } catch (const vrsdk::Error& e) {
            if (e.code() == VRSDK_ERR_NO_RESPONDER) {
                std::printf(
                    "  srv/rotors   NO RESPONDER after %.0fs -> not a multirotor. (%s)\n",
                    PROBE_TIMEOUT_S, e.what());
            } else {
                std::printf("  srv/rotors   [%d] %s\n", e.code(), e.what());
            }
        }
        std::printf(
            "  That timeout IS the type discovery. Nothing in a state message names the type.\n");

        // ===== the wrong number of pulse widths =====
        const vrsdk::State start = robot.states();
        const std::string before = list_u32(start.actuator().pwm, start.actuator().pwm_count);
        std::printf("\n-- SET_MR_PWM with four entries, on a two-rotor airframe --\n");
        for (int i = 0; i < HOLD_SAMPLES; ++i) {
            // Published happily; dropped there.
            robot.set_mr_pwm({THROTTLE_US, THROTTLE_US, THROTTLE_US, THROTTLE_US});
            robot.rate(HZ);
        }
        const vrsdk::State after = robot.states();
        std::printf(
            "  returned without throwing every time; echo %s -> %s. A wrong length is refused by "
            "a log line no client can see.\n",
            before.c_str(), list_u32(after.actuator().pwm, after.actuator().pwm_count).c_str());

        // ===== two entries, which is the robot type, not a setting =====
        std::printf(
            "\n-- SET_MR_PWM with two entries: [rotor1 (FRD left), rotor2 (FRD right)] --\n");
        for (const double diff : SWEEP_US) {
            const std::vector<double> pwm = {
                std::fmin(std::fmax(THROTTLE_US + diff, PWM_MIN_US), PWM_MAX_US),
                std::fmin(std::fmax(THROTTLE_US - diff, PWM_MIN_US), PWM_MAX_US),
            };
            std::printf("  diff=%+6.0f us -> pwm=[%.0f,%.0f]\n", diff, pwm[0], pwm[1]);
            for (int i = 0; i < HOLD_SAMPLES; ++i) {
                robot.set_mr_pwm(pwm);
                if (i % 25 == 0) {
                    const vrsdk::State s = robot.states();
                    const vrsdk_actuator_t& a = s.actuator();
                    std::printf(
                        "     t=%7.2fs roll=%+7.2f deg  roll_rate=%+6.2f deg/s  echo=%s  "
                        "measured=%s\n",
                        s.elapsed, roll_deg(s.kin().quat),
                        s.kin().ang_vel[0] * 180.0 / 3.14159265358979323846,
                        list_u32(a.pwm, a.pwm_count).c_str(),
                        list_f64(a.measured, a.measured_count).c_str());
                }
                robot.rate(HZ);
            }
        }

        // ===== hand it back =====
        // Every pulse width latches: without this the bar holds the last
        // difference forever. Idle is this airframe's floor, and also what a
        // reset re-latches.
        robot.set_mr_pwm({PWM_MIN_US, PWM_MIN_US});
        std::printf("\nidled at [%.0f, %.0f]. Scene-authored robot: left running, never deleted.\n",
                    PWM_MIN_US, PWM_MIN_US);
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
