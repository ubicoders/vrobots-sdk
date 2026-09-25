// ex27 -- rotor_config: rebuild the airframe while it is flying.
//
//     target/cpp-build/Release/ex27_rotor_config 1   # the scene's multirotor
//     target/cpp-build/Release/ex27_rotor_config     # create one instead
//
// `srv/rotors` is the multirotor's own service, and it carries everything that
// turns a pulse width into a force:
//
//     thrust = (thrust_a*pwm^2 + thrust_b*pwm + thrust_c) * g            [N]
//     torque = spin_dir * (torque_a*pwm^2 + torque_b*pwm + torque_c) * g [N.m]
//     omega  = ang_vel_slope*pwm + ang_vel_intercept                     [rad/s]
//
// Roll, pitch and yaw are not in that list because THERE IS NO MIXING MATRIX
// ANYWHERE IN THE SIMULATOR -- moments fall out of the rotor *positions*, so
// moving a rotor really does change the airframe's response, and an asymmetric
// aircraft is just a different rotor list.
//
// THREE RULES, AND THE THIRD ONE IS THE TRAP
//
//   1. THE LIST IS REPLACED, NOT MERGED. One verb, no upsert: the vector you
//      send becomes the whole rotor list.
//
//   2. SO IT MUST DESCRIBE EVERY ROTOR, IN INDEX ORDER. The count is fixed when
//      the airframe spawns and is `actuator.pwm_count` in the state stream --
//      read it, do not assume four. A list of the wrong length makes the
//      simulator drop the ENTIRE request (never a partial apply) and ack `ok`
//      anyway. The second run below does exactly that on purpose, with a curve
//      that would make the aircraft fall out of the sky, and proves it was
//      dropped by climbing anyway.
//
//   3. THERE IS NO READ-BACK, AND NO PER-FIELD FLAGS INSIDE AN ENTRY. You cannot
//      ask what the geometry currently is, and a zero is a zero coefficient
//      rather than "leave it alone" -- so `vrsdk::rotor_spec()`, the simulator's
//      own reference rotor, is the base to build on. Note what a bare one means
//      for `position`: {0, 0, 0}, every rotor at the origin, an aircraft with
//      thrust and no control authority. Whatever you send IS the airframe now.
//
//   Positions are measured FROM THE ROBOT'S ORIGIN, NOT FROM ITS CENTRE OF MASS
//   -- the simulator subtracts the CoM offset itself, so a CoM-relative value
//   gets it subtracted twice. They are read in *your* header frame, which here is
//   the default "unity": +x right, +y up, +z forward, so a flat rotor ring lives
//   in the x-z plane at y = 0.
//
//   `spin_dir` is the sign of the yaw reaction torque, +1 clockwise and -1
//   counter-clockwise -- and 0 LETS THE SIMULATOR ALTERNATE BY INDEX (index 0
//   clockwise), which is how the stock airframes are built.
//
// WHAT THE STATE STREAM WILL AND WILL NOT TELL YOU
//
//   `actuator.measured` is rotor speed in rad/s, and it comes from
//   `ang_vel_slope`/`ang_vel_intercept` -- a REPORTED line, computed from the
//   pulse width and nothing else. The third run below cuts the thrust curve to
//   70% and leaves that line alone: the aircraft stops climbing while `measured`
//   does not move a digit. The rotor-speed echo is not evidence about thrust.
//   Height is.
//
// WHICH MULTIROTOR, AND WHY ATTACHING IS A ONE-WAY DOOR
//
//   With no argument this CREATES a multirotor and removes it again. Give it a
//   `sys_id` and it attaches to the scene's own -- and then NEVER REMOVES IT,
//   which matters more here than anywhere else in the book:
//
//     This service REPLACES the rotor list and there is NO READ-BACK. Run it
//     against the scene's multirotor and that aircraft is flying the ring
//     geometry and the 70% thrust curve set below -- for every other client,
//     until the scene is reloaded. `reset()` will not undo it (ex21: state
//     reset, not factory reset), and the SDK cannot restore what it was never
//     able to read. Reload the scene when you are done.
//
//   Prefer the argument for now anyway. AS OF SIM v3.0.0 A CLIENT-CREATED
//   MULTIROTOR SPAWNS WITH A RIGIDBODY THAT NEVER INTEGRATES: it hangs where it
//   spawned and ignores every pulse width and even a direct body force, while
//   its actuator echo and rotor-speed model answer perfectly normally. All three
//   climb runs then read 0.00 m/s and prove nothing. Trucks and MSDs created the
//   same way have live physics; the scene's multirotor flies.

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
    "With no argument this CREATES a multirotor and removes it again. With a sys_id it\n"
    "attaches to the scene's own, never removes it, and LEAVES ITS ROTORS REBUILT\n"
    "until the scene is reloaded -- there is no read-back to restore them from:\n"
    "\n"
    "    ex27_rotor_config 1\n"
    "\n"
    "Prefer the argument until the created-multirotor physics bug is fixed (see the\n"
    "file header) -- on a created one every climb run reads 0.00 m/s. List what is\n"
    "publishing with:\n"
    "\n"
    "    vrobots topic list";

constexpr double PI = 3.14159265358979323846;
constexpr double MASS_KG = 1.0;  // pinned, so the three runs are comparable
constexpr double ARM_M = 0.25;   // hub distance from the robot origin
// High enough that the retuned airframe still climbs: a run that sits on the
// ground reads 0.00 m/s whether the thrust curve changed or the request was lost.
constexpr double COLLECTIVE_US = 1800.0;
constexpr double THRUST_SCALE = 0.70;  // run 3: 70% of the reference thrust curve
constexpr double HZ = 25.0;
constexpr int SETTLE_SAMPLES = 25;   // ~1 s after a reset
constexpr int MEASURE_SAMPLES = 75;  // ~3 s of climb

namespace {

/// One climb measurement.
struct Run {
    std::string label;
    double climb = 0.0;
    std::string measured;
};

/// One decimal, so a column of rotor speeds stays readable.
std::string round1(const vrsdk_actuator_t& a) {
    std::string out = "[";
    for (std::uint32_t i = 0; i < a.measured_count; ++i) {
        char buffer[32];
        std::snprintf(buffer, sizeof buffer, "%s%.1f", i ? "," : "",
                      std::round(a.measured[i] * 10.0) / 10.0);
        out += buffer;
    }
    return out + "]";
}

/// `n` reference rotors laid out on a flat ring of radius ARM_M.
///
/// In the default "unity" header frame the horizontal plane is x-z and +y is up,
/// so the ring sits at y = 0. `spin_dir` is left at 0 so the simulator alternates
/// clockwise/counter-clockwise by index, which is what keeps the yaw torques
/// cancelling.
std::vector<vrsdk_rotor_spec_t> ring(std::size_t n) {
    std::vector<vrsdk_rotor_spec_t> out;
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double angle =
            PI / 4.0 + static_cast<double>(i) * 2.0 * PI / static_cast<double>(n > 0 ? n : 1);
        vrsdk_rotor_spec_t rotor = vrsdk::rotor_spec();
        rotor.position[0] = ARM_M * std::sin(angle);
        rotor.position[1] = 0.0;
        rotor.position[2] = ARM_M * std::cos(angle);
        out.push_back(rotor);
    }
    return out;
}

/// Reset, hold a fixed collective, and return the mean climb rate.
///
/// Height is `kin.lin_pos[2]`, negated because the multirotor publishes in "frd"
/// and the third component is DOWN. NOT `env.agl`: as of sim v3.0.0 that field is
/// a hard-coded zero for every robot -- filling it needs a downward raycast, and
/// the simulator publishes 0 rather than guessing, because `env` is the truth
/// block and an invented height is worse than an absent one.
///
/// `set_mr_pwm` takes one pulse width per rotor, however many that is -- a wrong
/// length is silently ignored by the robot.
Run climb(vrsdk::VirtualRobot& robot, const char* label, const std::vector<double>& collective) {
    std::printf("-- %s --\n", label);
    robot.reset();
    for (int i = 0; i < SETTLE_SAMPLES; ++i) {
        robot.set_mr_pwm(collective);
        robot.rate(HZ);
    }

    const vrsdk::State start = robot.states();
    for (int i = 0; i < MEASURE_SAMPLES; ++i) {
        robot.set_mr_pwm(collective);
        if (i % 25 == 0) {
            const vrsdk::State s = robot.states();
            std::printf("   t=%6.2fs alt=%7.2f m  climb=%+6.2f m/s  measured=%s\n", s.elapsed,
                        -s.kin().lin_pos[2],  // "frd": the third component is DOWN
                        -s.kin().lin_vel[2], round1(s.actuator()).c_str());
        }
        robot.rate(HZ);
    }
    const vrsdk::State end = robot.states();

    const double seconds = end.elapsed - start.elapsed;
    Run out;
    out.label = label;
    // "frd": z counts DOWN, so a climb is a DECREASE. The summary and the live
    // `climb=` column above must agree, or one of them is lying.
    out.climb = seconds > 0.0 ? (start.kin().lin_pos[2] - end.kin().lin_pos[2]) / seconds : 0.0;
    out.measured = round1(end.actuator());
    return out;
}

/// The multirotor to fly: a scene-authored `sys_id` if one was given, otherwise
/// empty, which creates a fresh one. See USAGE.
///
/// A sys_id is a `uint32_t` on the wire, so anything that does not fit one is
/// refused rather than wrapped: truncating `4294967297` to `1` would attach to
/// somebody else's robot and look like it worked. `strtoul` reports the overflow
/// through `errno` but wraps a leading '-' silently, so both are checked, and the
/// range check catches the rest on a 64-bit `unsigned long`.
std::optional<std::uint32_t> target_from_args(int argc, char** argv) {
    const char* program = argc > 0 ? argv[0] : "ex27_rotor_config";
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
        std::printf("%s sys_id=%u, service key = vrobots/%u/z/srv/rotors\n",
                    target ? "attached to" : "created", robot.sys_id(), robot.sys_id());
        if (target) {
            std::printf(
                "NOTE: this robot belongs to the scene. The rotor list is REPLACED below and "
                "cannot be read back, so it stays rebuilt until the scene is reloaded.\n");
        }
        {
            auto params = vrsdk::physical_params();
            params.has_mass = true;
            params.mass = MASS_KG;
            robot.set_physical_params(params);
        }

        // The count is fixed at spawn. This is where it lives -- not in a
        // constant.
        const std::size_t rotors = robot.states().actuator().pwm_count;
        const std::vector<double> collective(rotors, COLLECTIVE_US);
        std::printf("this airframe has %zu rotor(s) (actuator.pwm_count), mass pinned at %.1f kg\n\n",
                    rotors, MASS_KG);

        // ===== run 1: as spawned =====
        const Run stock = climb(robot, "as spawned", collective);

        // ===== run 2: the wrong number of entries =====
        // One short, and with a curve that makes almost no thrust. If the
        // simulator applied it the aircraft would drop; it does not, because a
        // wrong-length list is dropped whole -- and acked `ok` regardless.
        std::vector<vrsdk_rotor_spec_t> shortlist = ring(rotors > 0 ? rotors - 1 : 0);
        for (vrsdk_rotor_spec_t& r : shortlist) {
            r.thrust_a = 0.0;
            r.thrust_b = 0.0;
            r.thrust_c = 0.02;
        }
        std::printf("configure_rotors with %zu entries for %zu rotors ...\n", shortlist.size(),
                    rotors);
        robot.configure_rotors(shortlist);
        std::printf("... returned without throwing. That is a receipt, and the request was "
                    "dropped:\n");
        const Run dropped = climb(robot, "after the short list", collective);

        // ===== run 3: every rotor, 70% thrust =====
        std::vector<vrsdk_rotor_spec_t> weak = ring(rotors);
        const vrsdk_rotor_spec_t reference = vrsdk::rotor_spec();
        for (vrsdk_rotor_spec_t& r : weak) {
            r.thrust_a = reference.thrust_a * THRUST_SCALE;
            r.thrust_b = reference.thrust_b * THRUST_SCALE;
            r.thrust_c = reference.thrust_c * THRUST_SCALE;
        }
        robot.configure_rotors(weak);
        const Run retuned = climb(robot, "70% thrust curve", collective);

        std::printf("\n%.0f us on every rotor, three times:\n", COLLECTIVE_US);
        for (const Run* run : {&stock, &dropped, &retuned}) {
            std::printf("  %-24s climb=%+6.2f m/s   rotor speed echo=%s\n", run->label.c_str(),
                        run->climb, run->measured.c_str());
        }
        std::printf(
            "Run 2 matches run 1: the short list never applied. Run 3 stops climbing while the "
            "rotor-speed echo is unchanged -- that echo is the reported ang_vel line, not a "
            "thrust measurement.\n");

        // ===== the one length the SDK does refuse =====
        try {
            robot.configure_rotors({});
            std::printf("\nUNEXPECTED: an empty rotor list was accepted\n");
        } catch (const vrsdk::Error& e) {
            std::printf("\nempty list -> [%d] %s\n", e.code(), e.what());
        }

        if (!target) {
            robot.remove();
            std::printf("deleted sys_id=%u\n", robot.sys_id());
        } else {
            std::printf(
                "sys_id=%u belongs to the scene: left running on the ring geometry and the %.2f "
                "thrust curve. Reload the scene to get its own airframe back.\n",
                robot.sys_id(), THRUST_SCALE);
        }
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
