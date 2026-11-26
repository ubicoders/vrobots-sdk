// ex44 -- robot_properties: what a robot is, and what each actuator index means.
//
//     target/cpp-build/Release/ex44_robot_properties          # the warehouse forklift, sys_id 0
//     target/cpp-build/Release/ex44_robot_properties <id>     # any robot
//
// Every robot publishes `VRobotProperties` on `vrobots/{id}/z/properties`: its
// mass, centre of gravity, inertia, and the KIND OF EVERY ACTUATOR ENTRY. These
// are the values the simulation runs *with*, not per-step output, so they ride
// their own topic -- published when the robot's wire opens, again on any
// parameter change, and as a 1 Hz keepalive -- and `properties()` is a one-shot
// read: subscribe, take the first sample, undeclare.
//
// THE LAYOUT IS THE TYPE CHECK
//
//   `actuator.pwm`, `normalized` and `measured` in the state are index-parallel
//   arrays, and nothing in a state message says what index `i` *is* -- a wheel
//   speed in rad/s, a lift height in m, a held mass in kg. `actuator_types` does,
//   index for index, and that makes it the cheap answer to "what am I attached
//   to?": `connect()` takes the robot type on trust, while the layout is the
//   robot's own word. (ex30's capability probe, a service query timed until it
//   fails, was the only answer before this topic existed.) Once a handle has read
//   it, the typed getters -- `forklift_actuators()` and its siblings -- refuse a
//   robot whose layout contradicts theirs, which is how ex37-ex41 make sure.
//
//   As observed on the simulator:
//
//   | robot                  | `actuator_types`                        |
//   |------------------------|-----------------------------------------|
//   | forklift               | wheel, servo, linear, linear            |
//   | pallet robot           | wheel x4, linear, servo                 |
//   | rail arm               | linear, servo x6, vacuum, contact       |
//   | table arm              | servo x6, gripper, contact, hold        |
//   | turret                 | servo, servo, laser                     |
//   | truck                  | wheel x4, servo                         |
//   | multirotor, half-drone | rotor per rotor                         |
//   | msd, cart-pole         | unspecified: no kind exists for a force |
//
//   `cg` and `inertia` are on body axes in the frame the message names, the
//   robot's own. What the properties do NOT carry is how a robot is tuned: no
//   lift speed, no wheel geometry, no joint range. No config service reads
//   anything back.
//
// THE NEW CONFIG SERVICES
//
//   Each new robot type has one, and every one takes only the fields that are
//   set and acks a RECEIPT, not a result (ex22): the state stream is the
//   confirmation. This example uses the forklift's, on the lift speed, and
//   measures the carriage to prove it landed. The others are the same shape:
//
//   - `configure_forklift(const vrsdk_forklift_config_t&)` -- `srv/forklift`:
//     drive, steering, the mast's ranges and speeds, the pulse band, the reset
//     pulses.
//   - `configure_palletrobot(const vrsdk_palletrobot_config_t&)` --
//     `srv/palletrobot`: the pulse band, the virtual mecanum geometry, the
//     drive, the pad.
//   - `configure_arm(const vrsdk_arm_config_t&)` -- `srv/arm`, one service for
//     both arms: the protective stop, and the tool -- `suction_*` on the rail
//     arm, `gripper_*` on the table arm, each arm ignoring the other's.
//   - `configure_turret(const vrsdk_turret_config_t&)` -- `srv/turret`: joint
//     limits and rates, the laser's range, the stereo pair (ex41).
//   - `configure_lidars(const std::vector<LidarConfigEntry>&)` -- `srv/lidars`,
//     on every robot that carries one: mount, segments and layer mask, by lidar
//     name. Pro only: on Community it refuses in its reply, one of the two
//     services that ever say no (`srv/skin` is the other, ex23).
//
//   None has a getter, so "restore" means sending the shipped value back, which
//   is what this example does. A setting another client had changed comes back
//   as the shipped one. Everything set survives `reset()`; nothing survives a
//   simulator restart.

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <vrobots_sdk.hpp>

constexpr const char* USAGE =
    "Reads any robot; the default is the warehouse scene's forklift, sys_id 0, the\n"
    "only robot the srv/forklift half below runs on. For another robot, pass its id:\n"
    "\n"
    "    ex44_robot_properties 1\n"
    "\n"
    "The scene has to be loaded and in Play mode, and on the Community tier only the\n"
    "focused robot is on the wire at all. List what is publishing with:\n"
    "\n"
    "    vrobots topic list";

constexpr std::uint32_t SYS_ID = 0;  // the warehouse scene's forklift

// The shipped lift band, for lift_m(). This example changes only the speed.
constexpr double LIFT_MIN_M = -0.215;
constexpr double LIFT_MAX_M = 1.11;

constexpr double SHIPPED_LIFT_SPEED = 0.5;  // m/s, what restore sends back
constexpr double SLOW_LIFT_SPEED = 0.25;    // m/s, half of it
constexpr double NEUTRAL_US = 1500.0;
constexpr double HZ = 25.0;
constexpr int MOVE_SAMPLES = 150;  // 6 s; the slow move takes under 3
constexpr double MOVING_M = 0.02;  // a carriage this far from its start has started

namespace {

/// Where a lift pulse puts the carriage: linear across the shipped band (ex37).
double lift_m(double us) { return LIFT_MIN_M + (us - 1100.0) / 800.0 * (LIFT_MAX_M - LIFT_MIN_M); }

/// What a `measured` entry of this kind is in -- the reason the layout exists.
const char* unit(vrsdk::ActuatorKind kind) {
    switch (kind) {
        case vrsdk::ActuatorKind::Rotor:
            return "rad/s, rotor speed";
        case vrsdk::ActuatorKind::Servo:
            return "rad, a joint or steering angle";
        case vrsdk::ActuatorKind::Wheel:
            return "rad/s, wheel speed";
        case vrsdk::ActuatorKind::Surface:
            return "rad, surface deflection";
        case vrsdk::ActuatorKind::Linear:
            return "m, along a prismatic axis";
        case vrsdk::ActuatorKind::Vacuum:
            return "kg held by the suction pad, 0 = nothing";
        case vrsdk::ActuatorKind::Laser:
            return "m to the hit, 0 = off or none";
        case vrsdk::ActuatorKind::Gripper:
            return "m between the pads";
        case vrsdk::ActuatorKind::Contact:
            return "an ArmContact code: 0 none, 1 tool, 2 stop";
        case vrsdk::ActuatorKind::Hold:
            return "kg held by the tool, 0 = nothing";
        case vrsdk::ActuatorKind::Unspecified:
            return "no kind fits, e.g. a force in N";
        default:
            return "a kind newer than this SDK";
    }
}

/// One echoed pulse width, or `fallback` when the report is shorter.
double echo_us(const vrsdk_actuator_t& a, std::uint32_t index, std::uint32_t fallback) {
    return index < a.pwm_count ? a.pwm[index] : fallback;
}

/// Numbers as a list, `[a, b, c]`.
std::string list_of(const std::vector<double>& values) {
    std::string out = "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        char buffer[32];
        std::snprintf(buffer, sizeof buffer, "%s%.3f", i ? ", " : "", values[i]);
        out += buffer;
    }
    return out + "]";
}

/// Drive the lift to `lift_us` -- drive centred, forks at `width_us` -- and
/// measure how fast the carriage travelled, m/s.
///
/// Timed over the middle of the move, from the first sample that has left the
/// start to the first that has arrived, so the command's own latency does not
/// count against the speed. Height and time come from one snapshot, decoded in
/// hand, so they belong to the same sample.
double lift_speed(vrsdk::VirtualRobot& robot, double lift_us, double width_us) {
    const double want = lift_m(lift_us);
    const double start = robot.forklift_actuators().lift_height;
    std::optional<std::pair<double, double>> left_at;  // (elapsed, height) once it has started
    for (int i = 0; i < MOVE_SAMPLES; ++i) {
        robot.set_forklift(NEUTRAL_US, NEUTRAL_US, lift_us, width_us);
        robot.rate(HZ);
        const vrsdk::State s = robot.states();
        vrsdk::ForkliftActuators a;
        try {
            a = vrsdk::ForkliftActuators::from_actuator(s.actuator());
        } catch (const vrsdk::Error& e) {
            if (e.code() != VRSDK_ERR_DECODE) {
                throw;
            }
            continue;
        }
        if (!left_at) {
            if (std::fabs(a.lift_height - start) > MOVING_M) {
                left_at = std::make_pair(s.elapsed, a.lift_height);
            }
        } else {
            const auto [t_from, h_from] = *left_at;
            if (std::fabs(a.lift_height - want) < MOVING_M) {
                const double metres = std::fabs(a.lift_height - h_from);
                const double seconds = s.elapsed - t_from;
                std::printf("   %+.3f m -> %+.3f m: %.3f m in %.2f s mid-move = %.2f m/s\n", start,
                            a.lift_height, metres, seconds, metres / seconds);
                // Let it cover the last couple of centimetres before the next move.
                for (int j = 0; j < 5; ++j) {
                    robot.set_forklift(NEUTRAL_US, NEUTRAL_US, lift_us, width_us);
                    robot.rate(HZ);
                }
                return metres / seconds;
            }
        }
    }
    std::printf("   the carriage did not arrive in %.0f s\n", MOVE_SAMPLES / HZ);
    return 0.0;
}

/// The robot's sys_id: the one command-line argument, or `SYS_ID`.
///
/// A sys_id is a `uint32_t` on the wire, so anything that does not fit one is
/// refused rather than wrapped: truncating `4294967297` to `1` would attach to
/// somebody else's robot and look like it worked. `strtoul` reports the overflow
/// through `errno` but wraps a leading '-' silently, so both are checked, and the
/// range check catches the rest on a 64-bit `unsigned long`.
std::uint32_t sys_id_from_args(int argc, char** argv) {
    const char* program = argc > 0 ? argv[0] : "ex44_robot_properties";
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
        // The type is a claim: properties() below reads what the robot really is.
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Forklift, sys_id);
        try {
            robot.connect();
        } catch (const vrsdk::Error& e) {
            std::fprintf(stderr, "no robot at sys_id %u: [%d] %s\n\n%s\n", sys_id, e.code(),
                         e.what(), USAGE);
            return 1;
        }

        // ===== z/properties =====
        vrsdk::RobotProperties p;
        try {
            p = robot.properties();
        } catch (const vrsdk::Error& e) {
            std::fprintf(stderr, "no properties from sys_id %u: [%d] %s\n", sys_id, e.code(),
                         e.what());
            return 1;
        }
        const vrsdk::State s = robot.states();
        std::printf(
            "sys_id %u '%s': %.1f kg, centre of gravity (%+.3f, %+.3f, %+.3f) m, in \"%s\"\n",
            robot.sys_id(), s.name.c_str(), p.mass, p.cg[0], p.cg[1], p.cg[2],
            p.coord_frame_id.c_str());
        std::printf("inertia about the centre of gravity, kg*m^2:\n");
        for (const vrsdk::Vec3& row : p.inertia) {
            std::printf("   [%12.1f %12.1f %12.1f]\n", row[0], row[1], row[2]);
        }
        if (p.moment_arms.empty()) {
            std::printf("moment arms: none published for this type\n");
        } else {
            std::printf("moment arms: %s m\n", list_of(p.moment_arms).c_str());
        }

        // ===== the actuator layout, index by index =====
        // Index-parallel to the state's actuator arrays: kind i is what measured[i] is.
        std::printf("\n%zu actuator entries; the state's measured value beside each:\n",
                    p.actuator_types.size());
        for (std::size_t i = 0; i < p.actuator_types.size(); ++i) {
            const vrsdk::ActuatorKind kind = p.actuator_types[i];
            const double measured = i < s.actuator().measured_count
                                        ? s.actuator().measured[i]
                                        : std::numeric_limits<double>::quiet_NaN();
            std::printf("   [%zu] %-12s %+10.3f  %s\n", i, vrsdk::to_string(kind), measured,
                        unit(kind));
        }

        // ===== srv/forklift: only on a forklift =====
        if (p.actuator_types != vrsdk::forklift_actuator::KINDS) {
            std::printf(
                "\nnot a forklift's layout, so there is no srv/forklift to call here; the header "
                "lists the other robots' configure_* calls. Nothing was sent.\n");
            return 0;
        }

        // The forks as found: the echo is the latch (report order, pwm[DRIVE] is
        // the throttle). The lift moves half the band away, whichever way there is
        // room.
        const double found_lift_us = echo_us(s.actuator(), VRSDK_FORKLIFT_ACT_LIFT, 1100);
        const double found_width_us = echo_us(s.actuator(), VRSDK_FORKLIFT_ACT_FORK_WIDTH, 1301);
        const double lift_to_us =
            found_lift_us < 1500.0 ? found_lift_us + 400.0 : found_lift_us - 400.0;

        std::printf("\n-- srv/forklift: lift_speed %g m/s (shipped: %g) --\n", SLOW_LIFT_SPEED,
                    SHIPPED_LIFT_SPEED);
        {
            auto config = vrsdk::forklift_config();
            config.has_lift_speed = true;
            config.lift_speed = SLOW_LIFT_SPEED;
            robot.configure_forklift(config);
        }
        std::printf("acked -- a receipt. The carriage is the proof:\n");
        const double slow = lift_speed(robot, lift_to_us, found_width_us);

        // ===== restore it =====
        std::printf("\n-- srv/forklift: lift_speed back to the shipped %g m/s --\n",
                    SHIPPED_LIFT_SPEED);
        {
            auto config = vrsdk::forklift_config();
            config.has_lift_speed = true;
            config.lift_speed = SHIPPED_LIFT_SPEED;
            robot.configure_forklift(config);
        }
        const double shipped = lift_speed(robot, found_lift_us, found_width_us);

        std::printf(
            "\nmeasured %.2f m/s after asking for %g, %.2f m/s after asking for %g. The forks are "
            "back at %g / %g us, the drive centred. Scene-authored: never deleted.\n",
            slow, SLOW_LIFT_SPEED, shipped, SHIPPED_LIFT_SPEED, found_lift_us, found_width_us);
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
