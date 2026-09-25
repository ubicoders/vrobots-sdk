// ex35 -- publish_estimate: hand the autopilot an attitude you made up.
//
//     target/cpp-build/Release/ex35_publish_estimate <sys_id>
//
// ex33 selected the observer with nothing on `z/estimate`, and got the fallback:
// the loop asked for an estimate, found none fresh, and kept flying on truth.
// This is the other half. `publish_estimate` puts a
// `swarmbotix.states.EstimateState` on that topic, so
// `set_fw_est_source(FwEstSource::Observer)` finally has something to select --
// and the aircraft flies on YOUR attitude, errors and all.
//
// Four phases, each measured the same way so the numbers can be compared:
//
//   | phase | source   | published on `z/estimate`  | what to expect                         |
//   |-------|----------|----------------------------|----------------------------------------|
//   | 1     | truth    | the robot's own quaternion | nothing changes; the `_Est` gauges move|
//   | 2     | observer | the robot's own quaternion | still nothing: your estimate is right  |
//   | 3     | observer | the same, pitched up 5 deg | the nose trims down by about 5 deg     |
//   | 4     | observer | nothing at all             | 0.5 s later the loop is back on truth  |
//
// Phase 1 is what makes phase 3 mean anything. A truth-copy estimator is the one
// estimator whose error is exactly zero, so a difference between phases 1 and 2
// would be the plumbing rather than the estimate, and there is nowhere left for
// phase 3's shift to have come from.
//
// THE SETPOINT LATCHES; THE ESTIMATE DOES NOT
//
//   `set_angvel` is sent ONCE, at the top, and stands for the whole run -- that
//   is what a command is, and ex33's `send_cmd(SET_ANGVEL, ..)` escape hatch is
//   retired now that there is a typed wrapper for it. The estimate is the
//   opposite: the simulator ages it FROM ARRIVAL, in sim time, and stops trusting
//   it after 0.5 s, so it has to be republished every iteration at 20 Hz or
//   better. Phase 4 is that difference made visible -- the loop stops publishing
//   and sends nothing else, and the aircraft keeps the rate setpoint while losing
//   the attitude.
//
// WHAT THE LIE ACTUALLY DOES
//
//   The onboard loop adds two attitude assists on top of the rate setpoint, and
//   both read roll and pitch out of the BELIEVED attitude: a wings-leveller, and
//   an altitude hold that biases the pitch demand. Tell it the nose is 5 degrees
//   higher than it is and the pitch assist trims 5 degrees of nose-down to
//   "correct" it -- at the airframe's 1.5 rad/s per rad that is an extra 7.5
//   deg/s of nose-down demand while it settles, and the steady state is a true
//   pitch about 5 degrees below where phase 2 held it. The aircraft then sinks
//   until the altitude hold has bought those 5 degrees back, which takes tens of
//   metres.
//
//   THE RATE LOOP IS NEVER WRONG. It tracks its demand as well in phase 3 as in
//   phase 1; it is being asked for the wrong thing. That is the whole lesson: a
//   fooled autopilot is just an autopilot with a lying sensor.
//
//   Five degrees is deliberately small. The altitude assist is clamped at 10
//   degrees, so a lie inside the clamp settles the aircraft lower instead of
//   departing -- raise PITCH_BIAS_DEG past 10 and it cannot recover.
//
// `valid`, AND THE TWO WAYS OF SAYING NOTHING
//
//   Everything here publishes `valid = true`. `false` is not a status flag: the
//   simulator drops the message before it reads the quaternion AND DOES NOT RESET
//   THE AGE COUNTER, so a stream of invalid estimates is indistinguishable,
//   sim-side, from phase 4's silence. If your filter has not converged that is
//   still the honest thing to send -- just do not expect to be able to tell it
//   apart from a dead publisher.
//
// THE RATES RIDE ALONG, AND NOTHING READS THEM
//
//   The gyro vector fills the estimate's `twist`, because a filter that has rates
//   should say so and the field is there. NO CONSUMER READS IT TODAY -- the fixed
//   wing takes the quaternion and nothing else. `std::nullopt` would leave
//   `twist` off the wire entirely, which is a different statement from sending
//   zeros.
//
// THE FRAME IS THE HEADER'S
//
//   The SDK leaves the estimate's own frame pair unset, so it inherits the
//   header's -- the "frd" this connect stamps. That is why the truth copy can be
//   `s.kin().quat` straight off the state stream: this aircraft reports in `frd`
//   too, so both ends of the round trip name the same convention. The run checks
//   that rather than assuming it.
//
// Scene-authored (IMU scene): attach by sys_id, never delete.

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>

#include <vrobots_sdk.hpp>

constexpr const char* USAGE =
    "The Global Hawk is scene-authored (IMU scene) and sys ids are handed out at\n"
    "scene load and keep incrementing, so there is no id this file could hard-code.\n"
    "Pass the live one:\n"
    "\n"
    "    ex35_publish_estimate 15\n"
    "\n"
    "List what is publishing with:\n"
    "\n"
    "    vrobots topic list";

constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double DEG_TO_RAD = PI / 180.0;

// The aircraft's frame, and therefore the order its angles decompose in.
constexpr const char* FRAME_ID = "frd";
constexpr vrsdk::EulerOrder EULER_ORDER = vrsdk::EulerOrder::Zyx;

// The rate setpoint, FRD [p, q, r] in rad/s. Sent once: it latches.
constexpr vrsdk::Vec3 RATE_SETPOINT = {0.0, 0.0, 0.0};

// How much nose-up the phase-3 estimate invents, degrees. Under the onboard
// loop's 10-degree altitude-assist clamp, so the aircraft sags rather than
// departs.
constexpr double PITCH_BIAS_DEG = 5.0;

constexpr double HZ = 25.0;           // the publish rate; the sim wants 20 Hz or better
constexpr double STALE_S = 0.5;       // the sim's own staleness window, aged from arrival
constexpr int SETTLE_SAMPLES = 100;   // ~4 s for the assists and the rate loop to settle
constexpr int MEASURE_SAMPLES = 150;  // ~6 s averaged
constexpr int REPORT_EVERY = 50;      // a progress line every ~2 s

namespace {

/// What the publisher does during one tracking window.
enum class Estimator {
    /// The robot's own attitude, published straight back at it. Zero error by
    /// construction, which is what makes it the control.
    TruthCopy,
    /// The same attitude with PITCH_BIAS_DEG of nose-up composed on.
    PitchBias,
    /// Nothing goes on the wire.
    Silent,
};

/// One measurement window.
struct Tracking {
    std::string label;
    double mean_pitch_deg = 0.0;
    double mean_roll_deg = 0.0;
    double mean_yaw_rate = 0.0;
    /// Altitude at the end of the window, metres.
    double altitude_m = 0.0;
    /// How much of that was gained or lost across the window, metres.
    double climb_m = 0.0;
};

/// The gyro's body rates, as a value the estimate can carry.
vrsdk::Vec3 gyro_of(const vrsdk::State& s) {
    const double* w = s.sensors().gyroscope.angular_velocity;
    return {w[0], w[1], w[2]};
}

/// The attitude quaternion, as a value.
vrsdk::Quat quat_of(const vrsdk::State& s) {
    const double* q = s.kin().quat;
    return {q[0], q[1], q[2], q[3]};
}

/// The invented pitch offset, as a rotation about the body's pitch axis.
///
/// The order is irrelevant for a single-axis rotation and is named anyway: it is
/// the frame's own, and a triple is always stored [about x, about y, about z]
/// whatever the application order.
vrsdk::Quat pitch_bias_quat() {
    return vrsdk::rotations::euler_to_quat({0.0, PITCH_BIAS_DEG * DEG_TO_RAD, 0.0}, EULER_ORDER);
}

/// One estimate, or none. Returns what went on the wire.
std::optional<vrsdk::Quat> publish(vrsdk::VirtualRobot& robot, const vrsdk::State& s,
                                   Estimator estimator) {
    if (estimator == Estimator::Silent) {
        return std::nullopt;
    }
    // quat_multiply(a, b) is "b first, then a", so post-multiplying applies the
    // bias about the BODY pitch axis: with the wings level -- which the leveller
    // sees to -- that is exactly PITCH_BIAS_DEG of believed nose-up.
    const vrsdk::Quat quat =
        estimator == Estimator::TruthCopy
            ? quat_of(s)
            : vrsdk::rotations::quat_multiply(quat_of(s), pitch_bias_quat());
    // valid = true throughout. The gyro rates go along for the ride; nothing
    // reads them yet.
    robot.publish_estimate(quat, gyro_of(s), true);
    return quat;
}

/// Publish for a while, let the assists settle, then average over a window.
///
/// Identical in every phase so the four rows can be read side by side: only
/// `estimator` changes, and in phase 4 not even that -- it publishes nothing.
Tracking track(vrsdk::VirtualRobot& robot, const std::string& label, Estimator estimator) {
    std::printf("-- %s --\n", label.c_str());

    for (int i = 0; i < SETTLE_SAMPLES; ++i) {
        publish(robot, robot.states(), estimator);
        robot.rate(HZ);
    }

    double pitch_sum = 0.0;
    double roll_sum = 0.0;
    double rate_sum = 0.0;
    double first_alt = 0.0;
    double last_alt = 0.0;
    for (int i = 0; i < MEASURE_SAMPLES; ++i) {
        const vrsdk::State s = robot.states();
        const std::optional<vrsdk::Quat> published = publish(robot, s, estimator);

        // There is no attitude on the wire, only a quaternion: these are the
        // angles ex36 extracts, in the frame's own order.
        const vrsdk::Vec3 euler = vrsdk::rotations::quat_to_euler(quat_of(s), EULER_ORDER);
        const double roll = euler[0];
        const double pitch = euler[1];
        // FRD is NED as a world frame, so the third component is DOWN.
        const double altitude = -s.kin().lin_pos[2];

        pitch_sum += pitch * RAD_TO_DEG;
        roll_sum += roll * RAD_TO_DEG;
        rate_sum += s.kin().ang_vel[2];
        if (i == 0) {
            first_alt = altitude;
        }
        last_alt = altitude;

        if (i % REPORT_EVERY == 0) {
            // What the loop believes, beside what is true. It is the published
            // pitch the assist drives towards its target, never the real one.
            char believed[24] = "    --    ";
            if (published) {
                std::snprintf(
                    believed, sizeof believed, "%+6.2f deg",
                    vrsdk::rotations::quat_to_euler(*published, EULER_ORDER)[1] * RAD_TO_DEG);
            }
            std::printf(
                "   t=%7.2fs true pitch=%+6.2f deg  published=%s  roll=%+6.2f deg  "
                "r=%+7.4f rad/s  alt=%9.1f m\n",
                s.elapsed, pitch * RAD_TO_DEG, believed, roll * RAD_TO_DEG, s.kin().ang_vel[2],
                altitude);
        }
        robot.rate(HZ);
    }

    const double n = static_cast<double>(MEASURE_SAMPLES);
    Tracking out;
    out.label = label;
    out.mean_pitch_deg = pitch_sum / n;
    out.mean_roll_deg = roll_sum / n;
    out.mean_yaw_rate = rate_sum / n;
    out.altitude_m = last_alt;
    out.climb_m = last_alt - first_alt;
    return out;
}

/// The scene-authored aircraft's sys_id, from the one command-line argument.
///
/// A sys_id is a `uint32_t` on the wire, so anything that does not fit one is
/// refused rather than wrapped: truncating `4294967297` to `1` would attach to
/// somebody else's robot and look like it worked. `strtoul` reports the overflow
/// through `errno`, and the range check catches the rest on a 64-bit `long`.
std::uint32_t sys_id_from_args(int argc, char** argv) {
    const char* program = argc > 0 ? argv[0] : "ex35_publish_estimate";
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s <sys_id>\n\n%s\n", program, USAGE);
        std::exit(2);
    }
    char* end = nullptr;
    errno = 0;
    const unsigned long parsed = std::strtoul(argv[1], &end, 10);
    const bool bad = end == argv[1] || *end != '\0' || argv[1][0] == '-' || errno == ERANGE ||
                     parsed > std::numeric_limits<std::uint32_t>::max();
    if (bad) {
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

        // The estimate inherits this header frame, and the rate setpoint is
        // re-expressed out of it, so stamping the aircraft's own makes both mean
        // what they look like.
        vrsdk_connect_options_t options{};
        vrsdk_options_default(&options);
        options.coord_frame_id = FRAME_ID;
        options.axis_convention = VRSDK_AXES_FRD;

        vrsdk::VirtualRobot robot(vrsdk::RobotType::GlobalHawk, sys_id, &options);
        robot.connect();
        std::printf("attached to sys_id=%u (GlobalHawk), frame=\"%s\"\n", robot.sys_id(),
                    robot.states().coord_frame_id.c_str());
        // The C++ surface has no topic-name builder; the shapes are fixed by the
        // wire, so compose it here.
        std::printf("publishing estimates on vrobots/%u/z/estimate\n", sys_id);

        // The truth copy below is `s.kin().quat`, which is in the robot's
        // reporting frame, published under a header that says FRAME_ID. If those
        // two ever disagree the simulator re-bases a quaternion that was already
        // correct.
        const std::string reported = robot.states().coord_frame_id;
        if (reported != FRAME_ID) {
            std::printf(
                "NOTE: this robot reports in \"%s\" and this run stamps \"%s\". The truth copy "
                "would be re-based on arrival -- convert it first, or connect with the robot's "
                "own frame.\n",
                reported.c_str(), FRAME_ID);
        }

        // The estimate source is an onboard-loop concept, so make sure the
        // onboard loop is the one flying. Bumpless, and it does not teleport the
        // aircraft.
        robot.set_fw_ctrl_mode(vrsdk::FwCtrlMode::OnboardRate);

        // Sent ONCE. A command latches: this setpoint is still in force forty
        // seconds from now, including through phase 4's silence.
        robot.set_angvel(RATE_SETPOINT);
        std::printf(
            "mode -> ONBOARD_RATE, SET_ANGVEL -> [%.1f, %.1f, %.1f] rad/s (sent once -- it "
            "latches)\n",
            RATE_SETPOINT[0], RATE_SETPOINT[1], RATE_SETPOINT[2]);

        // Both entry points build the same message. This is the Euler one, used
        // once: the aircraft's own attitude, decomposed in the frame's order and
        // rebuilt by the SDK on the way out.
        {
            const vrsdk::State s = robot.states();
            const vrsdk::Vec3 euler = vrsdk::rotations::quat_to_euler(quat_of(s), EULER_ORDER);
            robot.publish_estimate_euler(euler, EULER_ORDER, gyro_of(s), true);
            std::printf(
                "publish_estimate_euler once: roll/pitch/yaw = (%+.2f,%+.2f,%+.2f) deg in order "
                "%s -- the same wire message publish_estimate builds, from angles instead of a "
                "quaternion\n\n",
                euler[0] * RAD_TO_DEG, euler[1] * RAD_TO_DEG, euler[2] * RAD_TO_DEG,
                vrsdk::to_string(EULER_ORDER));
        }

        // ===== phase 1: the control -- a perfect estimator nobody is listening to =====
        robot.set_fw_est_source(vrsdk::FwEstSource::Truth);
        std::printf(
            "source -> TRUTH. Publishing anyway: the _Est gauges move, the flight does not.\n");
        const Tracking truth =
            track(robot, "1 truth source, truth-copy estimate", Estimator::TruthCopy);

        // ===== phase 2: the swap, with an estimate that happens to be right =====
        robot.set_fw_est_source(vrsdk::FwEstSource::Observer);
        std::printf(
            "\nsource -> OBSERVER. Same publisher, same quaternion -- but the loop is flying "
            "YOUR attitude now, and there is no field anywhere that says so.\n");
        const Tracking observer =
            track(robot, "2 observer source, truth-copy estimate", Estimator::TruthCopy);

        // ===== phase 3: the same loop, flying a lie =====
        std::printf(
            "\nsame source, same publisher, +%.1f deg of pitch composed onto every estimate. The "
            "aircraft is about to be told its nose is higher than it is.\n",
            PITCH_BIAS_DEG);
        char lie_label[64];
        std::snprintf(lie_label, sizeof lie_label, "3 observer source, +%.1f deg pitch lie",
                      PITCH_BIAS_DEG);
        const Tracking lie = track(robot, lie_label, Estimator::PitchBias);
        std::printf(
            "  against phase 2: %+.2f deg of true pitch, %+.1f m of altitude. The lie was "
            "+%.1f deg.\n",
            lie.mean_pitch_deg - observer.mean_pitch_deg, lie.altitude_m - observer.altitude_m,
            PITCH_BIAS_DEG);

        // ===== put the aircraft back where phase 1 found it =====
        // Phase 4 is only comparable from a level, on-altitude start, and the lie
        // spent tens of metres. Reset relaunches at trim -- and takes the estimate
        // source and the SET_ANGVEL latch with it, so both are re-sent.
        std::printf("\n-- reset() --\n");
        robot.reset();
        robot.set_fw_est_source(vrsdk::FwEstSource::Observer);
        robot.set_angvel(RATE_SETPOINT);
        std::printf(
            "  relaunched at trim, and the source and the rate setpoint re-sent: reset() clears "
            "both.\n");

        // ===== phase 4: the fallback, from the publisher's side this time =====
        std::printf(
            "\nstill OBSERVER, and nothing published from here on. After %g s of silence the "
            "estimate is stale and the loop is fed truth again -- the simulator logs a warning "
            "saying exactly that.\n",
            STALE_S);
        const Tracking stale =
            track(robot, "4 observer source, nothing published", Estimator::Silent);

        // ===== the four numbers =====
        const double window_s = static_cast<double>(MEASURE_SAMPLES) / HZ;
        std::printf("\nwhat each phase flew on, and what the airframe did about it:\n");
        for (const Tracking* run : {&truth, &observer, &lie, &stale}) {
            std::printf(
                "  %-38s pitch=%+6.2f deg  roll=%+6.2f deg  r=%+7.4f rad/s  "
                "alt=%9.1f m (%+7.1f m over %.1f s)\n",
                run->label.c_str(), run->mean_pitch_deg, run->mean_roll_deg, run->mean_yaw_rate,
                run->altitude_m, run->climb_m, window_s);
        }
        std::printf(
            "Phase 2 matching phase 1 is the swap working: the loop flew your estimate and your "
            "estimate was right. Phase 3 is the same loop, the same gains and the same setpoint, "
            "given one wrong number. Phase 4 is the staleness clock handing it back.\n");

        // ===== hand it back =====
        robot.set_fw_est_source(vrsdk::FwEstSource::Truth);
        robot.set_angvel(RATE_SETPOINT);
        std::printf(
            "source -> TRUTH, rate setpoint zeroed. Scene-authored robot: left flying, never "
            "deleted.\n");
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
