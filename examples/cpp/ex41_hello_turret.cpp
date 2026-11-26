// ex41 -- hello_turret: find it, turn it by rate and by target, fire the laser, narrow its limits.
//
//     target/cpp-build/Release/ex41_hello_turret          # find the turret on the wire
//     target/cpp-build/Release/ex41_hello_turret <id>     # or name it
//
// A fixed pan-tilt turret: a pylon turns (HEADING, positive to the right seen
// from above) and a head tilts (PITCH, positive raises the barrel), with a laser
// along the barrel and a stereo camera pair on the head. It reports in CV, and
// its attitude IS its joints -- but joint angles carry no axes, so every number
// here means the same whatever the frame.
//
// FINDING IT
//
//   The sandbox scene authors a turret without pinning its id, so with no
//   argument this example looks for it, in two steps:
//
//   1. WHO IS ON THE WIRE. ex11's `list_topics`: every robot publishes a state
//      topic, so the `vrobots/{id}/z/state` keys are the robots.
//   2. WHICH ONE IS A TURRET. Nothing in a state message names the type, and the
//      number of actuator entries does not either -- a cart-pole reports three
//      as well. `z/properties` does: the turret is the robot whose layout is
//      `[servo, servo, laser]`. `read_properties` reads a candidate's with no
//      handle to it, so nothing is attached until the turret is found, and
//      reading changes nothing.
//
// TWO MOTION COMMANDS, ONE LATCH
//
//   | command             | carries | the turret                                            |
//   |---------------------|---------|-------------------------------------------------------|
//   | `set_turret_rates`  | rad/s   | turns at that rate, clamped to the max, until a limit |
//   | `set_turret_joints` | rad     | turns to that target at the max rate, then holds      |
//
//   Both latch, and they share one latch: whichever arrived last is in charge. A
//   rate is the command for closing an aiming loop yourself; a target is the one
//   for "go there". Stop sending rates and the pylon keeps turning, all the way
//   to its limit -- which is why the rate phase below ends on a rate of zero.
//
// THE LASER
//
//   A latch of its own. While on, the simulator measures the distance from the
//   muzzle to the first thing the beam hits, up to 200 m, every physics step, and
//   that one number is what reaches the wire: `laser_range`, 0 when the laser is
//   off OR nothing is within range. The barrel is aimed 5 deg down below so that
//   on open, flat ground the beam meets the floor about 24 m out.
//
// `srv/turret`, AND WHY THE TARGET SURVIVES IT
//
//   `configure_turret` sets the joint limits and rates (also the laser's range
//   and the stereo pair). Limits that exclude the current pose do not snap it:
//   the joint walks back to the nearest limit at its max rate. The TARGET STAYS
//   LATCHED -- the limit only clamps it -- so when the shipped limits come back
//   the turret walks out to its old target again. Nothing reads the limits back,
//   so "restore" means sending the shipped values; a scene that had tuned them
//   differently gets the shipped ones.
//
// LEAVING IT AS FOUND
//
//   The joints and the laser latch are read at the start and put back at the
//   end, the limits are restored, and the last motion command is a target -- the
//   found pose -- which the turret holds. Scene-authored: never deleted.

#include <algorithm>
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
    "With no argument this example finds the turret: the sandbox scene authors one\n"
    "but does not pin its id. To skip the search, pass the id:\n"
    "\n"
    "    ex41_hello_turret 3\n"
    "\n"
    "The scene has to be loaded and in Play mode, and on the Community tier only the\n"
    "focused robot is on the wire at all. List what is publishing with:\n"
    "\n"
    "    vrobots topic list";

constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double DEG_TO_RAD = PI / 180.0;

constexpr double WINDOW_S = 1.5;  // ex11's listening window
constexpr double HZ = 25.0;
constexpr double RATE = 0.3;               // rad/s of heading, to the right: ~17 deg/s
constexpr int RATE_SAMPLES = 38;           // ~1.5 s of it
constexpr double AIM_HEADING_DEG = -30.0;  // left of rest
constexpr double AIM_PITCH_DEG = -5.0;     // barrel a little down, so the laser meets the floor
constexpr int MOVE_SAMPLES = 100;          // 4 s; any move within the limits takes under 3
constexpr int LASER_SAMPLES = 10;          // 0.4 s: measured every physics step, on from the next
constexpr int WATCH_SAMPLES = 25;          // 1 s to watch a limit change land
constexpr double TOLERANCE = 1e-3;         // rad
constexpr double NARROW_DEG = 20.0;        // heading limits narrowed to +/- this
constexpr double SHIPPED_HEADING_DEG = 90.0;  // the shipped heading limits, +/- this

namespace {

/// The decoded report, in degrees.
std::string report(const vrsdk::TurretActuators& t) {
    char out[160];
    std::snprintf(
        out, sizeof out,
        "heading %+6.1f deg (%+4.0f%% of max rate)  pitch %+5.1f deg (%+4.0f%%)  laser %s %.2f m",
        t.heading * RAD_TO_DEG, t.heading_rate_fraction * 100.0, t.pitch * RAD_TO_DEG,
        t.pitch_rate_fraction * 100.0, t.laser_on ? "on " : "off", t.laser_range);
    return out;
}

/// Degrees to the radians the commands take.
double deg(double degrees) { return degrees * DEG_TO_RAD; }

/// Robot ids as a list, `[0, 1, 2]`.
std::string ids_of(const std::vector<std::uint32_t>& ids) {
    std::string out = "[";
    for (std::size_t i = 0; i < ids.size(); ++i) {
        out += (i ? ", " : "") + std::to_string(ids[i]);
    }
    return out + "]";
}

/// Actuator kinds as a list of their names, `["servo", "servo", "laser"]`.
std::string kinds_of(const std::vector<vrsdk::ActuatorKind>& kinds) {
    std::string out = "[";
    for (std::size_t i = 0; i < kinds.size(); ++i) {
        out += i ? ", \"" : "\"";
        out += vrsdk::to_string(kinds[i]);
        out += "\"";
    }
    return out + "]";
}

/// Attach to `sys_id`, and make sure it really is a turret.
///
/// `connect()` takes the robot type on trust; once this handle has read
/// `z/properties` (ex44), `turret_actuators()` refuses a robot whose layout is
/// not `[servo, servo, laser]`.
vrsdk::VirtualRobot attach(std::uint32_t sys_id) {
    vrsdk::VirtualRobot robot(vrsdk::RobotType::Turret, sys_id);
    robot.connect();
    (void)robot.properties();
    (void)robot.turret_actuators();
    return robot;
}

/// Find the turret: list the robots on the wire, then ask each what it is.
///
/// Returns the attached handle, which has already read the layout -- so its
/// `turret_actuators()` is checked against it from here on -- or empty when no
/// robot on the wire is a turret.
std::optional<vrsdk::VirtualRobot> find_turret() {
    // Step 1, ex11's: a zenoh topic is listed only if it published during the
    // window, and a robot's state publishes at 25 Hz.
    std::printf("listening for %gs for the robots on the wire ...\n", WINDOW_S);
    const std::vector<vrsdk::TopicInfo> topics = vrsdk::list_topics(WINDOW_S);
    std::vector<std::uint32_t> candidates;
    for (const vrsdk::TopicInfo& t : topics) {
        if (!t.sys_id) {
            continue;
        }
        const std::uint32_t sys_id = *t.sys_id;
        if (t.key == "vrobots/" + std::to_string(sys_id) + "/z/state" &&
            std::find(candidates.begin(), candidates.end(), sys_id) == candidates.end()) {
            candidates.push_back(sys_id);
        }
    }
    std::printf("robots on the wire: %s\n", ids_of(candidates).c_str());

    // Step 2: the layout on z/properties is what a robot says it is, and
    // reading it takes no handle.
    for (const std::uint32_t sys_id : candidates) {
        vrsdk::RobotProperties p;
        try {
            p = vrsdk::read_properties(sys_id);
        } catch (const vrsdk::Error& e) {
            std::printf("   sys_id %u: no layout: [%d] %s\n", sys_id, e.code(), e.what());
            continue;
        }
        const std::string kinds = kinds_of(p.actuator_types);
        if (p.actuator_types == vrsdk::turret_actuator::KINDS) {
            std::printf("   sys_id %u: %s -- the turret\n", sys_id, kinds.c_str());
            return attach(sys_id);
        }
        std::printf("   sys_id %u: %s -- not a turret\n", sys_id, kinds.c_str());
    }
    return std::nullopt;
}

/// Latch a joint target and watch both joints arrive.
///
/// The target latches, so once would do; sending it every iteration costs
/// nothing and heals a command lost on the way.
void aim(vrsdk::VirtualRobot& robot, double heading, double pitch) {
    std::printf("\n-- aim: heading %+.1f deg, pitch %+.1f deg --\n", heading * RAD_TO_DEG,
                pitch * RAD_TO_DEG);
    const double t0 = robot.states().elapsed;
    for (int i = 0; i < MOVE_SAMPLES; ++i) {
        robot.set_turret_joints(heading, pitch);
        robot.rate(HZ);
        const vrsdk::TurretActuators now = robot.turret_actuators();
        const double t = robot.states().elapsed - t0;
        if (std::fabs(now.heading - heading) < TOLERANCE &&
            std::fabs(now.pitch - pitch) < TOLERANCE) {
            std::printf("   t=%5.2fs  %s  -- there\n", t, report(now).c_str());
            return;
        }
        if (i % 5 == 0) {
            std::printf("   t=%5.2fs  %s\n", t, report(now).c_str());
        }
    }
    std::printf(
        "   not there: a target past a limit is clamped to it (srv/turret, or another client's "
        "limits)\n");
}

/// Switch the laser and read the range it measures.
void laser(vrsdk::VirtualRobot& robot, bool on) {
    std::printf("\n-- laser %s --\n", on ? "on" : "off");
    for (int i = 0; i < LASER_SAMPLES; ++i) {
        robot.set_turret_laser(on);
        robot.rate(HZ);
    }
    const vrsdk::TurretActuators now = robot.turret_actuators();
    std::printf("   LASER [%d]: on=%s, range %.2f m  (0 = off, or nothing within range)\n",
                VRSDK_TURRET_ACT_LASER, now.laser_on ? "true" : "false", now.laser_range);
}

/// Send nothing for a second and watch the head: whatever moves it now is the
/// target already latched, clamped by whatever limits the service just set.
void watch(vrsdk::VirtualRobot& robot) {
    for (int i = 0; i < WATCH_SAMPLES; ++i) {
        robot.rate(HZ);
        if (i % 5 == 0) {
            std::printf("   %s\n", report(robot.turret_actuators()).c_str());
        }
    }
}

/// The turret's sys_id if one was given; empty means find it.
///
/// A sys_id is a `uint32_t` on the wire, so anything that does not fit one is
/// refused rather than wrapped: truncating `4294967297` to `1` would attach to
/// somebody else's robot and look like it worked. `strtoul` reports the overflow
/// through `errno` but wraps a leading '-' silently, so both are checked, and the
/// range check catches the rest on a 64-bit `unsigned long`.
std::optional<std::uint32_t> sys_id_from_args(int argc, char** argv) {
    const char* program = argc > 0 ? argv[0] : "ex41_hello_turret";
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
        // ===== setup: find the turret, or attach to the one named =====
        vrsdk::check_version();
        const std::optional<std::uint32_t> sys_id = sys_id_from_args(argc, argv);
        std::optional<vrsdk::VirtualRobot> attached;
        if (sys_id) {
            try {
                attached = attach(*sys_id);
            } catch (const vrsdk::Error& e) {
                std::fprintf(stderr, "no turret at sys_id %u: [%d] %s\n\n%s\n", *sys_id, e.code(),
                             e.what(), USAGE);
                return 1;
            }
        } else {
            try {
                attached = find_turret();
            } catch (const vrsdk::Error& e) {
                std::fprintf(stderr, "could not look: [%d] %s\n\n%s\n", e.code(), e.what(), USAGE);
                return 1;
            }
            if (!attached) {
                std::fprintf(stderr, "no robot on the wire has a turret's layout.\n\n%s\n", USAGE);
                return 1;
            }
        }
        vrsdk::VirtualRobot& robot = *attached;
        const vrsdk::State s = robot.states();
        std::printf("attached to sys_id=%u '%s', reporting in \"%s\"\n", robot.sys_id(),
                    s.name.c_str(), s.coord_frame_id.c_str());

        // ===== as found =====
        const vrsdk::TurretActuators found = robot.turret_actuators();
        std::printf("found: %s\n", report(found).c_str());

        // ===== a rate: SET_TURRET_RATES =====
        std::printf("\n-- heading rate %g rad/s for %.1f s --\n", RATE, RATE_SAMPLES / HZ);
        for (int i = 0; i < RATE_SAMPLES; ++i) {
            robot.set_turret_rates(RATE, 0.0);
            robot.rate(HZ);
            if (i % 10 == 9) {
                std::printf("   %s\n", report(robot.turret_actuators()).c_str());
            }
        }
        // A rate latches: without this the pylon would keep turning to its limit.
        robot.set_turret_rates(0.0, 0.0);

        // ===== a target: SET_TURRET_JOINTS =====
        const double heading = deg(AIM_HEADING_DEG);
        const double pitch = deg(AIM_PITCH_DEG);
        aim(robot, heading, pitch);

        // ===== the laser =====
        laser(robot, true);
        laser(robot, false);

        // ===== srv/turret: narrow the heading limits, then restore them =====
        // Nothing is sent between the two calls but the calls themselves: the
        // head moves on the target latched above and whatever the limits now
        // allow.
        {
            auto config = vrsdk::turret_config();
            config.has_heading_min = true;
            config.heading_min = deg(-NARROW_DEG);
            config.has_heading_max = true;
            config.heading_max = deg(NARROW_DEG);
            robot.configure_turret(config);
        }
        std::printf(
            "\n-- srv/turret: heading limits +/-%g deg; the %g deg target is clamped to them --\n",
            NARROW_DEG, AIM_HEADING_DEG);
        watch(robot);

        {
            auto config = vrsdk::turret_config();
            config.has_heading_min = true;
            config.heading_min = deg(-SHIPPED_HEADING_DEG);
            config.has_heading_max = true;
            config.heading_max = deg(SHIPPED_HEADING_DEG);
            robot.configure_turret(config);
        }
        std::printf(
            "\n-- srv/turret: the shipped +/-%g deg back. The target was latched all along, so "
            "the head walks back out to it --\n",
            SHIPPED_HEADING_DEG);
        watch(robot);

        // ===== hand it back =====
        aim(robot, found.heading, found.pitch);
        robot.set_turret_laser(found.laser_on);
        std::printf(
            "pose and laser back as found. Scene-authored: left holding that pose, never "
            "deleted.\n");
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
