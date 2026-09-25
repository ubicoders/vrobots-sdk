// ex33 -- fw_est_source: let the autopilot believe your estimator instead of the
// truth.
//
//     target/cpp-build/Release/ex33_fw_est_source <sys_id>
//
// Every control loop in the simulator is fed the EXACT attitude, because it is a
// simulator and it can be. SET_FW_EST_SOURCE (command id 311) is the one switch
// that takes that away:
//
//   | value                       | the onboard loop is fed                    |
//   |-----------------------------|--------------------------------------------|
//   | `FwEstSource::Truth` (0)    | the simulator's true attitude              |
//   | `FwEstSource::Observer` (1) | whatever is published on `z/estimate`      |
//
// The controller itself never knows which. The swap happens upstream of it, in
// the robot -- which is exactly how a real flight computer gets fooled, and the
// whole point of the experiment: A FOOLED AUTOPILOT IS JUST AN AUTOPILOT WITH A
// LYING SENSOR. Write an attitude estimator, publish it, and the aircraft flies
// on your errors.
//
// Two properties make it safe to try:
//
//   IT ONLY AFFECTS THE ONBOARD LOOP. `FwCtrlMode::DirectSurface` (ex31, ex32)
//   never consults an attitude at all, so the switch is inert there. This
//   example therefore stays in `OnboardRate` throughout.
//
//   A STALE ESTIMATE FALLS BACK TO TRUTH. An estimate older than 0.5 s is not
//   trusted, and the loop silently reverts to truth until a fresh one arrives
//   (the simulator logs a warning saying so). And like the control mode,
//   `reset()` puts the source back to truth.
//
// WHAT THIS RUN CAN AND CANNOT SHOW YOU
//
//   Nothing publishes `z/estimate` here, so selecting the observer is a
//   DEMONSTRATION OF THE FALLBACK, not of the swap: the loop asks for an
//   estimate, finds none within the staleness window, and keeps flying on
//   truth. The evidence is that the tracking error does not change -- printed
//   below, across all three phases.
//
//   The swap itself is ex35, which selects the observer AND publishes on:
//
//       vrobots/{sys_id}/z/estimate      swarmbotix.states.EstimateState
//
//   `publish_estimate` and `publish_estimate_euler` build that message for you
//   -- `estimate.valid`, `estimate.kinematics.pose.orientation`, frame-tagged
//   from the connect header -- and one of ex35's phases publishes a pitch 5
//   degrees above the true one, so the aircraft visibly flies the wrong
//   attitude. An estimate does not latch the way a command does: the simulator
//   ages it from arrival and stops trusting it after 0.5 s, so a publisher has
//   to repeat itself at 20 Hz or better.
//
// WHY YAW
//
//   The onboard loop tracks SET_ANGVEL, so this example publishes one and
//   measures how well it is followed. It commands YAW rate specifically: roll
//   and pitch demands are summed with a wings-level assist and an altitude-hold
//   trim, so their steady tracking error is not zero and would make a poor
//   yardstick. Yaw has no assist -- the rate loop's integrator drives its error
//   to zero -- so it measures the loop and nothing else.
//
//   SET_ANGVEL has a typed wrapper, set_angvel -- ex35 uses it -- but this
//   example keeps the generic `send_cmd` path on purpose: it is ex08's escape
//   hatch, used in anger, and the two spell the same bytes. It carries a
//   VEC3, so unlike the surface array it *is* re-expressed from your header
//   frame into the robot's, as an axial vector: the options below stamp "frd" so
//   that {0, 0, r} means what it looks like.
//
// And like everything else on this aircraft the setpoint LATCHES -- except
// across a `reset()`, which clears it. Phase 3 re-sends it for that reason.
//
// Scene-authored (IMU scene): attach by sys_id, never delete.

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>

#include <vrobots_sdk.hpp>

constexpr const char* USAGE =
    "The Global Hawk is scene-authored (IMU scene) and sys ids are handed out at\n"
    "scene load and keep incrementing, so there is no id this file could hard-code.\n"
    "Pass the live one:\n"
    "\n"
    "    ex33_fw_est_source 15\n"
    "\n"
    "List what is publishing with:\n"
    "\n"
    "    vrobots topic list";

constexpr double YAW_RATE = 0.05;  // rad/s, nose right -- gentle, well inside authority
constexpr double HZ = 25.0;
constexpr int SETTLE_SAMPLES = 100;   // ~4 s for the integrator to null the error
constexpr int MEASURE_SAMPLES = 150;  // ~6 s averaged

namespace {

/// One tracking window.
struct Tracking {
    std::string label;
    double mean_rate = 0.0;
    double mean_error = 0.0;
    double mean_abs_error = 0.0;
};

/// Publish one rate setpoint. FRD [p, q, r], and it IS converted from the
/// header frame into the robot's -- which is why the options stamp "frd".
void send_rate(vrsdk::VirtualRobot& robot, const double demand[3]) {
    vrsdk_cmd_args_t args{};
    vrsdk_cmd_args_default(&args);
    // Deliberately the generic path (ex08), not set_angvel: same bytes,
    // demonstrated once here. The vec3 field is the one this id reads;
    // everything else stays off the wire.
    args.vec3 = demand;
    robot.send_cmd(VRSDK_CMD_SET_ANGVEL, &args);
}

/// The engine channel, newtons.
double thrust_of(const vrsdk::State& s) {
    const vrsdk_actuator_t& a = s.actuator();
    return a.measured_count > 0 ? a.measured[a.measured_count - 1] : 0.0;
}

/// Stream a steady yaw-rate setpoint, let the loop settle, then average the
/// error over a window.
Tracking track(vrsdk::VirtualRobot& robot, const char* label) {
    std::printf("-- %s --\n", label);
    const double demand[3] = {0.0, 0.0, YAW_RATE};  // FRD [p, q, r]

    for (int i = 0; i < SETTLE_SAMPLES; ++i) {
        send_rate(robot, demand);
        robot.rate(HZ);
    }

    double rate_sum = 0.0;
    double error_sum = 0.0;
    double abs_sum = 0.0;
    for (int i = 0; i < MEASURE_SAMPLES; ++i) {
        send_rate(robot, demand);
        const vrsdk::State s = robot.states();
        const double r = s.kin().ang_vel[2];  // FRD: the third body rate is yaw
        rate_sum += r;
        error_sum += YAW_RATE - r;
        abs_sum += std::fabs(YAW_RATE - r);
        if (i % 50 == 0) {
            std::printf(
                "   t=%7.2fs r=%+7.4f rad/s  err=%+7.4f  bank rate p=%+7.4f  engine=%8.0f N\n",
                s.elapsed, r, YAW_RATE - r, s.kin().ang_vel[0], thrust_of(s));
        }
        robot.rate(HZ);
    }

    const double n = static_cast<double>(MEASURE_SAMPLES);
    Tracking out;
    out.label = label;
    out.mean_rate = rate_sum / n;
    out.mean_error = error_sum / n;
    out.mean_abs_error = abs_sum / n;
    return out;
}

/// The scene-authored aircraft's sys_id, from the one command-line argument.
///
/// A sys_id is a `uint32_t` on the wire, so anything that does not fit one is
/// refused rather than wrapped: truncating `4294967297` to `1` would attach to
/// somebody else's robot and look like it worked. `strtoul` reports the overflow
/// through `errno` but wraps a leading '-' silently, so both are checked, and the
/// range check catches the rest on a 64-bit `unsigned long`.
std::uint32_t sys_id_from_args(int argc, char** argv) {
    const char* program = argc > 0 ? argv[0] : "ex33_fw_est_source";
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

        // The rate setpoint is a vec3 and IS converted from this frame into the
        // robot's. Stamping the aircraft's own frame makes {0, 0, r} mean r.
        vrsdk_connect_options_t options{};
        vrsdk_options_default(&options);
        options.coord_frame_id = "frd";
        options.axis_convention = VRSDK_AXES_FRD;

        vrsdk::VirtualRobot robot(vrsdk::RobotType::GlobalHawk, sys_id, &options);
        robot.connect();
        std::printf("attached to sys_id=%u (GlobalHawk), frame=\"%s\"\n", robot.sys_id(),
                    robot.states().coord_frame_id.c_str());

        // This switch is an onboard-loop concept, so make sure the onboard loop
        // is the one flying. Bumpless, and it does not teleport the aircraft.
        robot.set_fw_ctrl_mode(vrsdk::FwCtrlMode::OnboardRate);
        std::printf(
            "mode -> ONBOARD_RATE (the only mode where an attitude is consulted at all)\n\n");

        // ===== phase 1: truth, the default =====
        robot.set_fw_est_source(vrsdk::FwEstSource::Truth);
        const Tracking truth = track(robot, "truth (source 0)");

        // ===== phase 2: observer, with nobody publishing an estimate =====
        robot.set_fw_est_source(vrsdk::FwEstSource::Observer);
        std::printf(
            "\nsource -> OBSERVER. Nothing publishes z/estimate here, so within 0.5 s the "
            "estimate is stale and the loop is fed truth again -- the simulator logs a warning "
            "saying exactly that.\n");
        const Tracking observer = track(robot, "observer (source 1), no publisher");

        // ===== phase 3: reset puts it back =====
        std::printf("\n-- reset() --\n");
        robot.reset();
        std::printf(
            "  the source is back to truth and the mode back to onboard, and the SET_ANGVEL "
            "latch was cleared -- so it has to be re-sent.\n");
        const Tracking after_reset = track(robot, "after reset");

        std::printf("\nsteady yaw-rate tracking, commanded %.2f rad/s:\n", YAW_RATE);
        for (const Tracking* run : {&truth, &observer, &after_reset}) {
            std::printf("  %-34s mean r=%+7.4f rad/s  mean error=%+7.4f  |error|=%6.4f\n",
                        run->label.c_str(), run->mean_rate, run->mean_error, run->mean_abs_error);
        }
        std::printf(
            "Phase 2 matching phase 1 IS the fallback: the loop asked for an estimate, found none "
            "fresh, and kept flying on truth.\n");

        // ===== the value the SDK will not send =====
        // The typed enum makes an out-of-range mode unrepresentable in C++, so
        // this reaches for the raw C entry point to show the guard is real.
        const vrsdk_err_t code = vrsdk_robot_set_fw_est_source(robot.handle(), 2);
        if (code == VRSDK_OK) {
            std::printf("\nUNEXPECTED: source 2 was accepted\n");
        } else {
            std::printf("\nset_fw_est_source(2) -> [%d] %s\n", code, vrsdk_last_error_message());
        }

        // ===== hand it back =====
        const double zero[3] = {0.0, 0.0, 0.0};
        send_rate(robot, zero);
        std::printf("rate setpoint zeroed. Scene-authored robot: left flying, never deleted.\n");
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
