// ex21 -- reset: put a robot back where it started, without restarting anything.
//
//     target/cpp-build/Release/ex21_reset 1     # the scene's multirotor
//     target/cpp-build/Release/ex21_reset       # create one instead
//
// `reset()` is the first of the twelve service calls this book covers, and the
// smallest: it teleports the robot to the pose captured at its FIRST PHYSICS
// STEP, zeroes linear and angular velocity, rests the actuators and re-latches
// the robot's initial command. Exactly what the simulator's own Reset button
// does, and nothing more.
//
// Four things about it are worth more than the call itself.
//
//   A BARE GET IS THE REQUEST. `srv/reset` carries no payload -- the simulator's
//   vendored C# zenoh client cannot attach one, so an empty query had to mean
//   something. The practical consequence: `srv/reset` is the one service key
//   where the usual "probe it with an empty GET and see if anybody answers"
//   capability trick PERFORMS THE ACTION. Probe every other key freely; never
//   probe this one.
//
//   THE ACK IS A RECEIPT, NOT A RESULT. The reply is packed the instant the
//   query lands; the teleport happens in phase 0 of the next physics step, under
//   20 ms later at 50 Hz. A call that returns means "the robot heard you". The
//   state stream is the confirmation, here as everywhere.
//
//   A LIVE PUBLISHER WINS ONE STEP LATER. Commands latch, and the reset
//   re-latches the robot's *initial* command -- 1100 us idle on a multirotor. A
//   loop that keeps publishing 1700 climbs straight back out of the reset and
//   barely notices it happened, which is exactly what you want from a controller
//   under test. This example therefore STOPS COMMANDING across the reset, so the
//   effect is visible: watch `echo` fall from 1700 to 1100 with nothing sent.
//
//   TIME DOES NOT RESTART. `seq` and `elapsed` keep advancing straight through
//   -- only the robot moves. A frozen `elapsed` means the simulator stopped
//   (ex19), never that it reset.
//
// "HOME" IS NOT "WHERE YOU FOUND IT"
//
//   Home is the pose captured at the robot's FIRST PHYSICS STEP. Attach to a
//   scene robot that has been flying since the scene loaded and the two are
//   nothing like each other -- measured live, an attach found one 27 m from its
//   home, so a program that treated the attach position as home reported the
//   reset as having moved the robot *away* from where it belonged.
//
//   There is no service that reads the home pose out, so the only honest way to
//   learn it is to go there: RESET ONCE, SETTLE, AND READ THE POSITION OFF THE
//   STATE STREAM -- which is exactly what this example does on the attach path,
//   and exactly the trick ex29 uses to find a cart-pole's rail centre. On the
//   create path it is unnecessary: nothing has happened to the robot yet, so the
//   first sample already is home.
//
//   What survives a reset: everything the *other* services set -- mass, inertia,
//   noise models, rotor curves, skin, coordinate frames. It is a state reset, not
//   a factory reset. What does not survive: on a fixed wing the control mode goes
//   back to onboard and the estimate source back to truth (ex31, ex33).
//
// WHICH MULTIROTOR, AND ONE SIMULATOR BUG
//
//   With no argument this CREATES its own multirotor and removes it again, so
//   that running it twice disturbs nothing (ex04's lifecycle). Give it a
//   `sys_id` and it attaches to the scene's own instead and leaves it running --
//   resetting a scene robot is benign, it is the same thing the sim's Reset
//   button does.
//
//   Prefer the argument for now. AS OF SIM v3.0.0 A CLIENT-CREATED MULTIROTOR
//   SPAWNS WITH A RIGIDBODY THAT NEVER INTEGRATES: it hangs where it spawned,
//   ignores every pulse width and even a direct body force, while its actuator
//   echo and rotor-speed model answer perfectly normally. Trucks and MSDs created
//   the same way have live physics, and the scene's own multirotor flies. So on a
//   created robot the climb below goes nowhere and the "the latch is still flying
//   it" phase is only a story -- the reset assertions still hold, because a
//   teleport is a teleport, but you will not see it fly.
//
// The multirotor publishes in "frd", so `lin_pos[2]` is DOWN and altitude is its
// negation.

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>

#include <vrobots_sdk.hpp>

constexpr const char* USAGE =
    "With no argument this CREATES a multirotor and removes it again. With a sys_id it\n"
    "attaches to the scene's own and never removes it:\n"
    "\n"
    "    ex21_reset 1\n"
    "\n"
    "Prefer the argument until the created-multirotor physics bug is fixed (see the\n"
    "file header). List what is publishing with:\n"
    "\n"
    "    vrobots topic list";

constexpr double CLIMB_US = 1700.0;   // well over hover: it leaves in a hurry
constexpr double HZ = 25.0;           // the state rate -- one line per sample's worth of time
constexpr int CLIMB_SAMPLES = 75;     // ~3 s under power
constexpr int COAST_SAMPLES = 25;     // ~1 s with nothing sent: the latch keeps flying it
constexpr int SETTLE_SAMPLES = 50;    // ~2 s watching it come home

namespace {

/// The commanded pulse widths the robot latched.
std::string echo_of(const vrsdk::State& s) {
    const vrsdk_actuator_t& a = s.actuator();
    std::string out = "[";
    for (std::uint32_t i = 0; i < a.pwm_count; ++i) {
        char buffer[16];
        std::snprintf(buffer, sizeof buffer, "%s%u", i ? "," : "", a.pwm[i]);
        out += buffer;
    }
    return out + "]";
}

/// One state line: the fields a reset is visible in.
std::string line(const vrsdk::State& s) {
    const double* p = s.kin().lin_pos;
    const double* v = s.kin().lin_vel;
    const double speed = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    const std::string echo = echo_of(s);

    char out[320];
    std::snprintf(out, sizeof out,
                  "seq=%-6llu t=%6.2fs pos=(%+7.2f,%+7.2f,%+7.2f) [%s] alt=%5.2f m |v|=%5.2f m/s "
                  "echo=%s",
                  static_cast<unsigned long long>(s.seq), s.elapsed, p[0], p[1], p[2],
                  s.coord_frame_id.c_str(),
                  -p[2],  // "frd": the third component is DOWN
                  speed, echo.c_str());
    return out;
}

/// Straight-line distance between two positions, in whatever frame they share.
double distance(const double a[3], const double b[3]) {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// Learn the pose `reset()` returns this robot to.
///
/// "Home" is the pose captured at the robot's FIRST PHYSICS STEP, and that is
/// emphatically not "wherever you found it". On a robot this program just created
/// the two coincide, because nothing has happened to it yet -- the first sample
/// `connect()` waited for IS the home pose.
///
/// On a SCENE robot they can be tens of metres apart: it has been flying since
/// the scene loaded, and whatever it drifted to before you attached says nothing
/// about where it started. Measured live: an attach found one 27 m from its home,
/// and a program that assumed otherwise reported the reset as having moved the
/// robot *away*.
///
/// So on the attach path, go there and look: reset once, let the teleport land,
/// and read the position off the state stream. The same trick ex29 uses to find a
/// cart-pole's rail centre -- when the simulator will not tell you a reference,
/// put the robot on it and measure.
vrsdk::State learn_home(vrsdk::VirtualRobot& robot, bool created) {
    if (created) {
        return robot.states();
    }
    std::printf("attached: resetting once to find out where home actually is\n");
    robot.reset();
    for (int i = 0; i < SETTLE_SAMPLES; ++i) {
        robot.rate(HZ);
    }
    return robot.states();
}

/// The robot to reset: a scene-authored `sys_id` if one was given, otherwise
/// empty, which creates a fresh one. See USAGE.
///
/// A sys_id is a `uint32_t` on the wire, so anything that does not fit one is
/// refused rather than wrapped: truncating `4294967297` to `1` would attach to
/// somebody else's robot and look like it worked. `strtoul` reports the overflow
/// through `errno` but wraps a leading '-' silently, so both are checked, and the
/// range check catches the rest on a 64-bit `unsigned long`.
std::optional<std::uint32_t> target_from_args(int argc, char** argv) {
    const char* program = argc > 0 ? argv[0] : "ex21_reset";
    if (argc < 2) {
        return std::nullopt;
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
        const std::optional<std::uint32_t> target = target_from_args(argc, argv);
        vrsdk::VirtualRobot robot =
            target ? vrsdk::VirtualRobot(vrsdk::RobotType::Multirotor, *target)
                   : vrsdk::VirtualRobot::create(vrsdk::RobotType::Multirotor);
        robot.connect();
        const std::uint32_t sys_id = robot.sys_id();
        std::printf("%s sys_id=%u, service key = vrobots/%u/z/srv/reset\n",
                    target ? "attached to" : "created", sys_id, sys_id);

        // ===== phase 0: where IS home? =====
        const vrsdk::State home = learn_home(robot, !target.has_value());
        std::printf("home  %s\n\n", line(home).c_str());

        // ===== phase 1: leave home =====
        std::printf("-- climbing at %.0f us --\n", CLIMB_US);
        for (int i = 0; i < CLIMB_SAMPLES; ++i) {
            robot.set_mr_pwm({CLIMB_US, CLIMB_US, CLIMB_US, CLIMB_US});
            if (i % 25 == 0) {
                std::printf("fly   %s\n", line(robot.states()).c_str());
            }
            robot.rate(HZ);
        }

        // Nothing is sent from here on. The last command LATCHES, so the robot
        // keeps climbing -- a command is a setpoint, not an impulse. (On a
        // CREATED multirotor it never left the ground in the first place; see the
        // header.)
        std::printf("\n-- nothing sent: the 1700 us latch is still flying it --\n");
        for (int i = 0; i < COAST_SAMPLES; ++i) {
            if (i % 12 == 0) {
                std::printf("coast %s\n", line(robot.states()).c_str());
            }
            robot.rate(HZ);
        }

        // ===== phase 2: reset =====
        const vrsdk::State before = robot.states();
        std::printf("\n-- reset() (a bare GET) --\n");
        robot.reset();
        std::printf(
            "acked. That is a RECEIPT: the teleport lands in phase 0 of the next physics step, "
            "and the state stream is the proof.\n");

        for (int i = 0; i < SETTLE_SAMPLES; ++i) {
            const vrsdk::State s = robot.states();
            if (i < 4 || i % 12 == 0) {
                std::printf("home? %s  d(home)=%5.2f m\n", line(s).c_str(),
                            distance(s.kin().lin_pos, home.kin().lin_pos));
            }
            robot.rate(HZ);
        }

        // ===== what actually happened =====
        const vrsdk::State after = robot.states();
        std::printf("\nposition:   %.2f m from home before, %.2f m after\n",
                    distance(before.kin().lin_pos, home.kin().lin_pos),
                    distance(after.kin().lin_pos, home.kin().lin_pos));
        std::printf(
            "actuators:  echo %s -> %s  (nothing was sent -- reset re-latched the robot's INITIAL "
            "command)\n",
            echo_of(before).c_str(), echo_of(after).c_str());
        std::printf(
            "time:       seq %llu -> %llu, elapsed %.2fs -> %.2fs  (the clock never resets; only "
            "the robot moved)\n",
            static_cast<unsigned long long>(before.seq),
            static_cast<unsigned long long>(after.seq), before.elapsed, after.elapsed);

        if (!target) {
            robot.remove();
            std::printf("deleted sys_id=%u\n", sys_id);
        } else {
            std::printf("sys_id=%u belongs to the scene: left running, never deleted.\n", sys_id);
        }
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
