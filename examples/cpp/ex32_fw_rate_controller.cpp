// ex32 -- fw_rate_controller: your gains, the operator's stick, the sim's
// aircraft.
//
//     target/cpp-build/Release/ex32_fw_rate_controller <sys_id>
//
// ex31 flew the panels open loop. This closes the loop, and it is the shape the
// fixed wing was built for: THE SIMULATOR'S RATE PIDs BYPASSED, AN EXTERNAL
// CONTROLLER IN THEIR PLACE, AND THE OPERATOR STILL FLYING WITH THE IN-GAME
// STICK. Three streams meet in one program:
//
//     z/cmd    ->  SET_ANGVEL from the sim's IMU panel   the setpoint (read, not written)
//     z/state  ->  kin.ang_vel                           the measurement
//     z/cmd    <-  SET_FW_SURFACES + SET_FW_THRUST       your output
//
// COMMANDS ARE READABLE, BECAUSE ZENOH IS A BUS
//
//   Everywhere else in this SDK a command is write-only, and rightly so: it has
//   no reply and the state stream is the proof. `subscribe_setpoint()` is the
//   one exception. The robot's `z/cmd` key is many-to-many, so the setpoints the
//   in-game panel publishes at 50 Hz are readable by anyone who subscribes to
//   the same key -- and a controller that wants the stick as an *input* rather
//   than as a competitor subscribes to it.
//
//   Everything anyone sends to this robot arrives there, THIS PROCESS'S OWN
//   TRAFFIC INCLUDED. `Setpoint::src_id()` is the sender; compare it against
//   your own `src_id` and skip your own. Non-matching command ids are counted in
//   the stream's `filtered`, which climbing fast is normal.
//
//   (The C++ surface has no accessor for the options in effect, so this example
//   sets `src_id` explicitly rather than reading the default back. That is the
//   honest way to know your own id here.)
//
// `latest()`, NOT `fresh()`
//
//   A setpoint LATCHES: the last one stands until the next arrives, and a
//   publisher that stops has not commanded zero. A rate loop therefore wants
//   "the current command" every iteration, which is `latest()`. `fresh()` hands
//   each value out exactly once and is the right read for something that must
//   not act twice on one operator input -- not this. Before the first setpoint
//   ever arrives `latest()` is empty, and this loop treats that as "hold zero
//   rates", which is a decision, not a default.
//
// FRAMES, AND THE ONE PLACE THERE ARE NONE
//
//   The setpoint arrives IN THE SENDER'S FRAME, UNCONVERTED -- the panel stamps
//   the target robot's own frame; the Global Hawk publishes `frd` (verified
//   live, sim v3.0.0), so it reads [p, q, r] in rad/s. `kin.ang_vel` is in the
//   robot's frame too, so demand and measurement are directly subtractable and
//   the loop below does no conversion at all. It checks the tag rather than
//   assuming.
//
//   The *output* has no frame: SET_FW_SURFACES is a float array, one number per
//   panel, so nothing is re-expressed on the way out and the mixing is entirely
//   yours.
//
// THE MIXER, SINCE THERE IS NOT ONE
//
//   This reproduces the simulator's own, so the aircraft flies the way it did
//   before you took it over -- panel gains from the airframe:
//
//     panel 0 = +aileron          panel 3 = 0          (inner flaps: the mixer
//     panel 1 = -aileron          panel 4 = -elevator + rudder   has zero gain
//     panel 2 = 0                 panel 5 = -elevator - rudder   there. ex31.)
//
//   and gains, feed-forward and gain schedule likewise: surface effectiveness
//   grows as airspeed squared, so gains tuned at the 72.8 m/s trim point are far
//   too hot at speed and the loop chatters rail to rail. Scaling the error and
//   the feed-forward by (V_trim / v)^2 is the same thing as scaling Kp, Ki and
//   FF live.
//
// WHAT THIS LOOP IS NOT
//
//   It is a RATE controller and nothing else. The onboard loop it replaced also
//   carried a wings-level assist, an altitude hold and an airspeed hold; in
//   DIRECT_SURFACE none of those exist. With the stick centred the aircraft will
//   hold zero body rates and still wander off in bank and altitude, and the
//   thrust is whatever you last sent. That is the honest cost of taking the
//   airframe.
//
// Scene-authored: never deleted, and handed back to its autopilot at the end.

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <vrobots_sdk.hpp>

constexpr const char* USAGE =
    "The Global Hawk is scene-authored (IMU scene) and sys ids are handed out at\n"
    "scene load and keep incrementing, so there is no id this file could hard-code.\n"
    "Pass the live one:\n"
    "\n"
    "    ex32_fw_rate_controller 15\n"
    "\n"
    "Then fly it with the sim's in-game IMU panel: this loop reads the stick off the\n"
    "robot's own command topic and closes the loop around it.";

constexpr std::size_t PANELS = 6;
constexpr double PI = 3.14159265358979323846;
constexpr double LIMIT_RAD = 20.0 * PI / 180.0;  // the airframe's clamp
constexpr double TRIM_MPS = 72.8;                // the gain schedule's anchor
constexpr double CRUISE_N = 3800.0;              // about what the airspeed hold carries at trim

// This client's identity on the bus. Set explicitly so the loop can recognise
// -- and ignore -- its own traffic coming back on z/cmd.
constexpr std::uint32_t OWN_SRC_ID = 122;

// The simulator's own rate gains, per FRD axis [roll, pitch, yaw].
constexpr double KP[3] = {0.16, 0.24, 0.35};
constexpr double KI[3] = {0.05, 0.08, 0.10};
constexpr double FF[3] = {0.29, 0.09, 0.14};

constexpr int RUN_SAMPLES = 1500;      // ~60 s at the 25 Hz state rate
constexpr double SAMPLE_TIMEOUT = 0.5;
constexpr double MAX_DT_S = 0.2;  // a stalled stream must not dump seconds into the integrator
constexpr int REPORT_EVERY = 25;

namespace {

/// Per-axis rate loop: feed-forward plus PI, with the integrator held at the
/// deflection limit so it cannot wind up behind a saturated surface.
class RatePid {
  public:
    /// Returns {aileron, elevator, rudder} in radians.
    std::vector<double> step(const double demand[3], const double measured[3], double q_scale,
                             double dt) {
        std::vector<double> out(3, 0.0);
        for (int axis = 0; axis < 3; ++axis) {
            const double error = q_scale * (demand[axis] - measured[axis]);
            integral_[axis] += error * dt;

            // Anti-windup: the integral alone can never exceed the clamp, so a
            // stuck surface cannot store minutes of error behind it.
            const double integral_limit = KI[axis] > 0.0 ? LIMIT_RAD / KI[axis] : 0.0;
            integral_[axis] = std::fmin(std::fmax(integral_[axis], -integral_limit), integral_limit);

            const double raw = FF[axis] * demand[axis] * q_scale + KP[axis] * error +
                               KI[axis] * integral_[axis];
            out[axis] = std::fmin(std::fmax(raw, -LIMIT_RAD), LIMIT_RAD);
        }
        return out;
    }

  private:
    double integral_[3] = {};
};

/// Three channels onto six panels, with the airframe's own gains.
///
/// The inner flaps stay at zero because that is what the onboard mixer does --
/// not because they cannot move. ex31 moves them, and that is how you know the
/// per-panel path is real.
std::vector<double> mix(double aileron, double elevator, double rudder) {
    std::vector<double> panels = {
        aileron, -aileron, 0.0, 0.0, -elevator + rudder, -elevator - rudder,
    };
    for (double& d : panels) {
        d = std::fmin(std::fmax(d, -LIMIT_RAD), LIMIT_RAD);
    }
    return panels;
}

std::string fmt(const std::vector<double>& values) {
    std::string out = "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        char buffer[32];
        std::snprintf(buffer, sizeof buffer, "%s%+6.3f", i ? "," : "", values[i]);
        out += buffer;
    }
    return out + "]";
}

/// The scene-authored aircraft's sys_id, from the one command-line argument.
///
/// A sys_id is a `uint32_t` on the wire, so anything that does not fit one is
/// refused rather than wrapped: truncating `4294967297` to `1` would attach to
/// somebody else's robot and look like it worked. `strtoul` reports the overflow
/// through `errno` but wraps a leading '-' silently, so both are checked, and the
/// range check catches the rest on a 64-bit `unsigned long`.
std::uint32_t sys_id_from_args(int argc, char** argv) {
    const char* program = argc > 0 ? argv[0] : "ex32_fw_rate_controller";
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

        // Stamping FRD is documentation: this loop thinks in [p, q, r] and the
        // aircraft publishes in FRD. It sends no vector commands, so nothing is
        // converted either way -- but a header that lies is worse than one that
        // is merely unused.
        vrsdk_connect_options_t options{};
        vrsdk_options_default(&options);
        options.coord_frame_id = "frd";
        options.axis_convention = VRSDK_AXES_FRD;
        options.src_id = OWN_SRC_ID;

        vrsdk::VirtualRobot robot(vrsdk::RobotType::GlobalHawk, sys_id, &options);
        robot.connect();

        // Subscribe BEFORE taking the aircraft: a stick input during the
        // handover would otherwise be missed, and the loop would start from "no
        // setpoint".
        vrsdk::SetpointStream setpoints = robot.subscribe_setpoint();
        std::printf(
            "attached to sys_id=%u (GlobalHawk); watching %s for SET_ANGVEL (id %u), ignoring "
            "src_id=%u\n",
            robot.sys_id(), setpoints.key().c_str(), setpoints.cmd_id(), OWN_SRC_ID);

        const std::uint32_t channels = robot.states().actuator().measured_count;
        if (channels != PANELS + 1) {
            std::fprintf(stderr,
                         "this robot publishes %u actuator channels; this mixer is written for "
                         "%zu panels + an engine. Six channels means a simulator too old for the "
                         "per-panel path.\n",
                         channels, PANELS);
            return 1;
        }

        // Mode first, thrust second -- entering direct mode inherits the thrust
        // the airspeed hold was carrying, so a specific thrust has to be asked
        // for after every mode entry (ex31).
        robot.set_fw_ctrl_mode(vrsdk::FwCtrlMode::DirectSurface);
        robot.set_fw_thrust(CRUISE_N);
        std::printf(
            "mode -> DIRECT_SURFACE, thrust -> %.0f N. Fly it with the sim's IMU panel.\n\n",
            CRUISE_N);

        RatePid pid;
        double previous_elapsed = robot.states().elapsed;
        bool frame_warned = false;

        // ===== loop =====
        for (int i = 0; i < RUN_SAMPLES; ++i) {
            // One control cycle per received state sample, so dt is the state
            // period.
            try {
                robot.wait_new_state(SAMPLE_TIMEOUT);
            } catch (const vrsdk::Error& e) {
                if (e.code() != VRSDK_ERR_TIMEOUT) {
                    throw;
                }
                // The surfaces latch, so holding is the correct response to
                // silence.
                std::printf("no new state -- holding the last deflections (they latch)\n");
                continue;
            }
            const vrsdk::State s = robot.states();
            const double dt = std::fmin(std::fmax(s.elapsed - previous_elapsed, 0.0), MAX_DT_S);
            previous_elapsed = s.elapsed;

            // --- the setpoint: latched, so read the current one every iteration
            const std::optional<vrsdk::Setpoint> setpoint = setpoints.latest();
            double demand[3] = {0.0, 0.0, 0.0};
            if (setpoint && setpoint->src_id() != OWN_SRC_ID) {
                // Our own traffic comes back on this bus too. It is not a
                // setpoint.
                const std::array<double, 3> value = setpoint->value();
                demand[0] = value[0];
                demand[1] = value[1];
                demand[2] = value[2];
                if (!frame_warned && !setpoint->coord_frame_id.empty() &&
                    setpoint->coord_frame_id != s.coord_frame_id) {
                    frame_warned = true;
                    std::printf(
                        "NOTE: the setpoint is stamped \"%s\" and this robot reports \"%s\". The "
                        "vector is NOT converted for you -- convert before subtracting.\n",
                        setpoint->coord_frame_id.c_str(), s.coord_frame_id.c_str());
                }
            }
            // Nobody has ever published one, or it was ours: "hold zero rates"
            // is a decision.

            // --- the measurement: body rates in the robot's own frame ---
            const double* measured = s.kin().ang_vel;

            // --- gain schedule: surface effectiveness grows as v^2 ---
            const double* v = s.kin().lin_vel;
            const double airspeed =
                std::fmax(std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]), 1.0);
            const double q_scale =
                std::fmin(std::fmax(TRIM_MPS * TRIM_MPS / (airspeed * airspeed), 0.05), 2.0);

            // --- three axes in, three channels out ---
            const std::vector<double> aer = pid.step(demand, measured, q_scale, dt);

            // --- and the mixer the simulator would have run ---
            const std::vector<double> surfaces = mix(aer[0], aer[1], aer[2]);
            robot.set_fw_surfaces(surfaces);
            robot.set_fw_thrust(CRUISE_N);

            if (i % REPORT_EVERY == 0) {
                const vrsdk_setpoint_stats_t stats = setpoints.stats();
                const double age = setpoint ? s.elapsed - setpoint->elapsed()
                                            : std::numeric_limits<double>::quiet_NaN();
                std::printf(
                    "t=%7.2fs demand=(%+6.3f,%+6.3f,%+6.3f) measured=(%+6.3f,%+6.3f,%+6.3f) "
                    "rad/s  err=(%+6.3f,%+6.3f,%+6.3f)\n",
                    s.elapsed, demand[0], demand[1], demand[2], measured[0], measured[1],
                    measured[2], demand[0] - measured[0], demand[1] - measured[1],
                    demand[2] - measured[2]);
                std::printf(
                    "        a/e/r=(%+6.3f,%+6.3f,%+6.3f) rad  panels=%s  v=%5.1f m/s q=%4.2f  "
                    "setpoints received=%llu filtered=%llu age=%.2fs\n",
                    aer[0], aer[1], aer[2], fmt(surfaces).c_str(), airspeed, q_scale,
                    static_cast<unsigned long long>(stats.received),
                    static_cast<unsigned long long>(stats.filtered), age);
            }
        }

        // ===== hand it back =====
        robot.set_fw_ctrl_mode(vrsdk::FwCtrlMode::OnboardRate);
        const vrsdk_setpoint_stats_t stats = setpoints.stats();
        std::printf(
            "\nmode -> ONBOARD_RATE. setpoints received=%llu filtered=%llu decode_errors=%llu "
            "seq_gaps=%llu. Scene-authored robot: left flying, never deleted.\n",
            static_cast<unsigned long long>(stats.received),
            static_cast<unsigned long long>(stats.filtered),
            static_cast<unsigned long long>(stats.decode_errors),
            static_cast<unsigned long long>(stats.seq_gaps));
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
