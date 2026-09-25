// ex31 -- globalhawk_direct: take the surfaces off the autopilot.
//
//     target/cpp-build/Release/ex31_globalhawk_direct <sys_id>
//
// The RQ-4B Global Hawk is the first robot in this book that flies ITSELF: six
// aero panels and one engine, with onboard rate PIDs and an airspeed hold
// closing the loop against whatever SET_ANGVEL setpoint arrives. Left alone it
// cruises at 72.8 m/s and needs nothing from you.
//
// `set_fw_ctrl_mode(FwCtrlMode::DirectSurface)` switches that off. The rate loop
// is bypassed entirely and the six panels take your radians verbatim: YOU ARE
// THE AUTOPILOT, mixing included. This example proves that the per-panel path is
// real, then shows the three ways it will surprise you.
//
// Scene-authored, like ex29 and ex30 -- `globalhawk` is not in any spawn
// catalog, so attach by sys_id from `vrobots topic list`. It lives in the IMU
// scene, not the sandbox.
//
// THERE IS NO MIXER
//
//   Each entry drives its own panel, in this order, in RADIANS, clamped to the
//   airframe's 20-degree limit:
//
//   | index | panel                 | what the onboard mixer does with it     |
//   |-------|-----------------------|-----------------------------------------|
//   | 0     | left outboard flap    | aileron, gain +1                        |
//   | 1     | right outboard flap   | aileron, gain -1                        |
//   | 2     | LEFT INNER FLAP       | NOTHING -- gain 0 on all three channels |
//   | 3     | RIGHT INNER FLAP      | NOTHING                                 |
//   | 4     | rear left ruddervator | elevator -1, rudder +1                  |
//   | 5     | rear right ruddervator| elevator -1, rudder -1                  |
//
//   Indices 2 and 3 are the proof. The simulator's own mixer has ZERO GAIN
//   there, so it can never move them: an inner flap that follows your command is
//   a deflection that could only have come through the per-panel path. The
//   second pose below deflects nothing else, on purpose.
//
//   The length must equal the panel count exactly -- a wrong-length array makes
//   the simulator drop the WHOLE command, never apply it partially.
//
// THE ECHO IS THE ONLY RECEIPT, AND INDEX 6 IS NOT A PULSE WIDTH
//
//   `actuator.measured` has PANELS + 1 entries:
//
//       measured[0..=5]  per-panel deflection, RADIANS
//       measured[6]      the engine, NEWTONS -- not normalised, not a pulse width
//
//   `set_fw_thrust` takes newtons too (clamped to 20 kN), and measured[6] is the
//   only thing that ever confirms it. A simulator too old for this work
//   publishes SIX entries rather than seven -- that count is the version check.
//
// THREE THINGS THAT WILL CATCH YOU OUT
//
//   LATCHING, WITH NO WATCHDOG. Stop sending and the aircraft keeps flying your
//   last command forever, exactly like a dead PWM client on a multirotor. The
//   run below stops for two seconds to show the deflections not moving.
//
//   `reset()` REVERTS THE MODE. Deliberately: keeping direct control with the
//   surface latches zeroed would relaunch the aircraft unflyable. So a
//   direct-surface client must re-assert `set_fw_ctrl_mode` after EVERY reset,
//   and the aircraft you thought you were flying has quietly gone back to its
//   autopilot. Demonstrated, then undone.
//
//   BUMPLESS IS NOT ZEROED. Entering direct mode seeds the latches from what the
//   plant is doing *now* -- including the thrust the airspeed hold happened to
//   be holding -- so nothing jolts. Which means a client that wants a particular
//   thrust must send `set_fw_thrust` AFTER EVERY MODE ENTRY, and after every
//   reset. Skip it and the engine keeps whatever the autopilot left, which is
//   usually about 3.8 kN at trim.
//
// `set_fw_thrust_bias` is not used here at all: it is an onboard-loop concept
// and is ignored in direct mode.
//
// Scene-authored, so nothing is deleted. The aircraft is handed back to its
// autopilot on the way out -- a Ctrl-C is not, and leaves it flying the last
// deflections.

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include <vrobots_sdk.hpp>

constexpr const char* USAGE =
    "The Global Hawk is scene-authored (IMU scene) and sys ids are handed out at\n"
    "scene load and keep incrementing, so there is no id this file could hard-code.\n"
    "Pass the live one:\n"
    "\n"
    "    ex31_globalhawk_direct 15\n"
    "\n"
    "List what is publishing with:\n"
    "\n"
    "    vrobots topic list";

constexpr std::size_t PANELS = 6;
constexpr double DEFLECT_RAD = 0.15;  // ~8.6 deg, inside the airframe's 20 deg limit
constexpr double CRUISE_N = 3800.0;   // about what the airspeed hold carries at trim
constexpr double CLIMB_N = 8000.0;    // a step big enough to be unmistakable
constexpr double HZ = 25.0;
constexpr int HOLD_SAMPLES = 60;  // ~2.4 s per pose
constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;

namespace {

/// Named poses, in panel order [LF, RF, LIF, RIF, RLF, RRF].
struct Pose {
    const char* label;
    std::vector<double> surfaces;
};

/// Six deflections, aligned so two rows can be compared by eye.
std::string fmt(const double* values, std::size_t count) {
    std::string out = "[";
    for (std::size_t i = 0; i < count; ++i) {
        char buffer[32];
        std::snprintf(buffer, sizeof buffer, "%s%+6.3f", i ? "," : "", values[i]);
        out += buffer;
    }
    return out + "]";
}

std::string fmt(const std::vector<double>& values) { return fmt(values.data(), values.size()); }

/// The actuator echo: panels in radians, then the engine in newtons.
void print_echo(vrsdk::VirtualRobot& robot, const char* prefix) {
    const vrsdk::State s = robot.states();
    const vrsdk_actuator_t& a = s.actuator();
    const std::uint32_t panels = a.measured_count > 0 ? a.measured_count - 1 : 0;
    const double engine = a.measured_count > 0 ? a.measured[a.measured_count - 1] : 0.0;
    std::printf("%s t=%7.2fs panels=%s engine=%8.0f N  rates=(%+6.2f,%+6.2f,%+6.2f) deg/s\n",
                prefix, s.elapsed, fmt(a.measured, panels).c_str(), engine,
                s.kin().ang_vel[0] * RAD_TO_DEG, s.kin().ang_vel[1] * RAD_TO_DEG,
                s.kin().ang_vel[2] * RAD_TO_DEG);
}

/// The engine channel, newtons.
///
/// `states()` returns the State BY VALUE, and `actuator()` hands out a reference
/// into it, so the State has to be a named local: binding the reference straight
/// off `robot.states().actuator()` leaves it pointing into a temporary that dies
/// at the end of that statement.
double thrust_of(vrsdk::VirtualRobot& robot) {
    const vrsdk::State s = robot.states();
    const vrsdk_actuator_t& a = s.actuator();
    return a.measured_count > 0 ? a.measured[a.measured_count - 1] : 0.0;
}

/// Stream one pose for a while and print commanded against measured.
void hold(vrsdk::VirtualRobot& robot, const char* label, const std::vector<double>& surfaces,
          double thrust_n) {
    std::printf("-- %s: %s --\n", label, fmt(surfaces).c_str());
    for (int i = 0; i < HOLD_SAMPLES; ++i) {
        // Latching means this does not have to be re-sent -- but a real
        // controller streams, so this one does too.
        robot.set_fw_surfaces(surfaces);
        robot.set_fw_thrust(thrust_n);
        if (i % 30 == 0) {
            print_echo(robot, "  ");
        }
        robot.rate(HZ);
    }
}

/// The scene-authored aircraft's sys_id, from the one command-line argument.
///
/// A sys_id is a `uint32_t` on the wire, so anything that does not fit one is
/// refused rather than wrapped: truncating `4294967297` to `1` would attach to
/// somebody else's robot and look like it worked. `strtoul` reports the overflow
/// through `errno` but wraps a leading '-' silently, so both are checked, and the
/// range check catches the rest on a 64-bit `unsigned long`.
std::uint32_t sys_id_from_args(int argc, char** argv) {
    const char* program = argc > 0 ? argv[0] : "ex31_globalhawk_direct";
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
        vrsdk::VirtualRobot robot(vrsdk::RobotType::GlobalHawk, sys_id);
        robot.connect();

        const std::uint32_t channels = robot.states().actuator().measured_count;
        std::printf("attached to sys_id=%u (GlobalHawk), frame=\"%s\", %u actuator channels\n",
                    robot.sys_id(), robot.states().coord_frame_id.c_str(), channels);
        if (channels != PANELS + 1) {
            std::printf(
                "WARNING: expected %zu channels (%zu panels + the engine). %u means a different "
                "airframe -- or a simulator too old for the per-panel path, which publishes %zu "
                "and silently ignores SET_FW_SURFACES.\n",
                PANELS + 1, PANELS, channels, PANELS);
        }

        // ===== hand the panels over =====
        // Order matters: mode first, then thrust. Entering direct mode inherits
        // whatever thrust the airspeed hold was carrying -- bumpless, not zeroed
        // -- so the thrust command has to come AFTER the mode, every time.
        robot.set_fw_ctrl_mode(vrsdk::FwCtrlMode::DirectSurface);
        robot.set_fw_thrust(CRUISE_N);
        std::printf("\nmode -> DIRECT_SURFACE, thrust -> %.0f N\n\n", CRUISE_N);

        // ===== the sweep =====
        const Pose sweep[] = {
            {"neutral", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            // Only the inner flaps. The onboard mixer CANNOT produce this pose.
            {"inner flaps only", {0.0, 0.0, DEFLECT_RAD, -DEFLECT_RAD, 0.0, 0.0}},
            {"roll right", {DEFLECT_RAD, -DEFLECT_RAD, 0.0, 0.0, 0.0, 0.0}},
            {"nose up", {0.0, 0.0, 0.0, 0.0, -DEFLECT_RAD, -DEFLECT_RAD}},
            {"yaw right", {0.0, 0.0, 0.0, 0.0, DEFLECT_RAD, -DEFLECT_RAD}},
            {"neutral", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
        };
        for (const Pose& pose : sweep) {
            hold(robot, pose.label, pose.surfaces, CRUISE_N);
        }

        // ===== the engine is in newtons =====
        hold(robot, "thrust step", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, CLIMB_N);
        std::printf(
            "  measured[%zu] = %.0f N for a commanded %.0f N -- newtons in, newtons back, no "
            "pulse width anywhere.\n",
            PANELS, thrust_of(robot), CLIMB_N);

        // ===== latching, and no watchdog =====
        const std::vector<double> latched = {DEFLECT_RAD, -DEFLECT_RAD, DEFLECT_RAD,
                                             -DEFLECT_RAD, 0.0,         0.0};
        std::printf("\n-- nothing sent for ~2 s --\n");
        robot.set_fw_surfaces(latched);
        for (int i = 0; i < HOLD_SAMPLES; ++i) {
            if (i % 25 == 0) {
                print_echo(robot, "  latched");
            }
            robot.rate(HZ);
        }
        std::printf("  unchanged. A command is a setpoint; there is no failsafe behind it.\n");

        // ===== reset takes the aircraft back =====
        std::printf("\n-- reset() --\n");
        robot.reset();
        for (int i = 0; i < HOLD_SAMPLES; ++i) {
            // Same command as before the reset, still being sent, and now
            // ignored: the mode went back to onboard, so the mixer is flying the
            // panels.
            robot.set_fw_surfaces(latched);
            robot.rate(HZ);
        }
        print_echo(robot, "  after reset");
        std::printf(
            "  The inner flaps (2, 3) are back at 0 with the same command still streaming: that "
            "is the mode reverting, not a dropped packet.\n");

        // ===== re-assert, which is all it takes =====
        std::printf("\n-- set_fw_ctrl_mode(DirectSurface) again, then thrust again --\n");
        robot.set_fw_ctrl_mode(vrsdk::FwCtrlMode::DirectSurface);
        robot.set_fw_thrust(CRUISE_N);
        hold(robot, "recovered", {0.0, 0.0, DEFLECT_RAD, -DEFLECT_RAD, 0.0, 0.0}, CRUISE_N);

        // ===== hand it back =====
        // Bumpless in this direction too: the rate PIDs reset and the airspeed
        // hold is seeded from the current thrust, so the aircraft does not sag.
        robot.set_fw_ctrl_mode(vrsdk::FwCtrlMode::OnboardRate);
        std::printf("\nmode -> ONBOARD_RATE. Scene-authored robot: left flying, never deleted.\n");
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
