// ex29 -- hello_cartpole: balance the classic underactuated problem.
//
//     target/cpp-build/Release/ex29_hello_cartpole <sys_id>
//
// THIS ONE TAKES AN ARGUMENT, AND EVERY EXAMPLE FROM HERE ON DOES. A cart-pole
// is NOT IN THE SPAWN CATALOG -- the sandbox scene registers `multirotor`,
// `truck` and `msd`, and asking for anything else is refused with a message that
// names the keys that scene *does* know (the run below does it once, on purpose,
// because a failed create is also how you enumerate a catalog). So the only way
// in is to attach to the one the scene authored, by its `sys_id` -- and THOSE IDS
// ARE ALLOCATED AT LOAD TIME AND KEEP INCREMENTING ACROSS SCENE LOADS, so no
// constant in this file could stay true. Find the live one with
// `vrobots topic list`.
//
// THE PLANT
//
//   A cart on a rail with a pole hinged on top. One actuator -- a force on the
//   cart, `SET_INVPEN`, newtons along the rail's +x, clamped to the config's
//   `max_force` -- and two things to control with it. That is what
//   "underactuated" means and why it is the textbook problem.
//
//   It publishes in the identity "unity" frame, and the published kinematics is
//   the CART'S. The pole rides the actuator channels, which are exactly three:
//
//   | channel                | meaning                                          |
//   |------------------------|--------------------------------------------------|
//   | `kin.lin_pos[0]`       | cart position along the rail, m -- IN WORLD COORDS |
//   | `kin.lin_vel[0]`       | cart speed along the rail, m/s                   |
//   | `actuator.measured[0]` | the force actually applied, N (after the clamp)   |
//   | `actuator.measured[1]` | pole angle theta, RADIANS                        |
//   | `actuator.measured[2]` | pole rate theta', rad/s                          |
//
//   `theta` is the signed angle from world +y to the hinge-to-bob direction,
//   right-handed about +z, wrapped into [-pi, pi]: 0 IS BALANCED UPRIGHT, +/-pi
//   IS HANGING -- both, because that is the wrap seam, so a fallen pole may print
//   either sign. A positive `theta` leans the bob toward -x.
//
// THE RAIL CENTRE IS NOT THE WORLD ORIGIN, AND IT IS NOT ON THE WIRE
//
//   This is the one that will cost you an afternoon. The cart slides along WORLD
//   X -- the simulator pins the cart's rotation to identity and freezes y, z and
//   every axis of rotation, so the rail really is the world x axis and
//   `lin_vel[0]` really is speed along it. But the rail is centred on WHEREVER
//   THE SCENE PARKED THE RIG, and the travel limits (`travel_half_range`, 4 m
//   each side) are measured from *that*, not from the world origin. Measured
//   live: this scene's cart-pole sits at x = -14.9.
//
//   The simulator knows the difference internally. IT DOES NOT PUBLISH IT:
//   `actuator.measured` is those three channels and nothing else, so there is no
//   rail-relative cart position anywhere on the wire. A controller that regulates
//   `lin_pos[0]` toward zero is therefore ordering the cart 15 m to the world
//   origin, past a dead stop it cannot cross, and at Kx = 0.5 that is a constant
//   7 N of destabilising bias against a 20 N actuator. The pole is on the floor
//   within a second, and nothing in the printout says why.
//
//   So CAPTURE THE ORIGIN YOURSELF. `reset()` teleports the cart to the pose
//   captured at its first physics step, which *is* the rail centre -- so one
//   reset, one settle, one read of `lin_pos[0]`, and every position term after
//   that is relative to a number you measured rather than one you assumed.
//
// `srv/cartpole` OWNS THE WHOLE PLANT, CART MASS INCLUDED
//
//   Every mass, length and limit is here, and `set_physical_params` (ex22) CANNOT
//   set the cart's mass: the robot re-stamps it from `cart_mass` on every
//   parameter apply, so a mass sent there is overwritten a step later.
//   Out-of-range values are not refused either, they are silently replaced by the
//   simulator's defaults -- which is why `configure_cartpole` refuses the
//   non-positive ones itself.
//
//   One field behaves unlike anything else in the API:
//   `initial_pole_angle_deg` IS IN DEGREES WHILE THE STATE REPORTS RADIANS, and
//   setting it RE-SEATS THE POLE AT REST IMMEDIATELY rather than at the next
//   reset. It is the episode's initial condition, so sending it mid-swing stops
//   the swing dead. This example stands the pole up with it before the loop
//   starts, because the controller below is a *balance* loop with no swing-up in
//   it -- from the simulator's own home angle of -45 degrees it cannot catch the
//   pole at all. (Simulated offline against the linearised plant: these gains
//   recover from about 15 degrees and no more.)
//
//   Its recovery is `reset()`, which does both halves at once -- the cart returns
//   to the rail centre *and* the pole is re-hung at rest at whatever home angle
//   was last configured.
//
// THE CONTROL LAW
//
//     d = x - x_rail_centre                            NOT x
//     F = -Kth*theta - Kthd*theta' + Kx*d + Kv*x'      clamped to +/-max_force
//
//   The angle terms are the obvious half: drive the cart UNDER the falling pole.
//   The cart terms look backwards -- a cart right of centre is pushed further
//   right -- and they are not: pushing +x tips the pole toward -x, and the angle
//   loop then chases it back through the centre. Steering a cart-pole by
//   deliberately tipping it the wrong way first is the non-minimum-phase
//   behaviour that makes this problem interesting.
//
//   Note the asymmetry in what needs correcting: the POSITION is world and must
//   have the rail centre subtracted, while the VELOCITY does not -- a twist is a
//   body quantity and the cart's body is pinned to identity, so `lin_vel[0]` is
//   already along the rail. Pose is world, twist is body; the two halves of `kin`
//   do not live in the same place.
//
//   One control cycle per RECEIVED state sample (ex09's pacing), so `dt` is the
//   state period and not whatever the machine felt like.
//
// THIS ROBOT BELONGS TO THE SCENE
//
//   It was not created here, so it is never removed here. The force latch is
//   released on the way out; a Ctrl-C is not, and leaves the last force applied.

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>

#include <vrobots_sdk.hpp>

constexpr const char* USAGE =
    "The cart-pole is scene-authored, and sys ids are handed out at scene load and\n"
    "keep incrementing, so there is no id this file could hard-code. Pass the live\n"
    "one:\n"
    "\n"
    "    ex29_hello_cartpole 7\n"
    "\n"
    "List what is publishing with:\n"
    "\n"
    "    vrobots topic list";

constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;

// The plant, pinned so the gains below mean something.
constexpr double CART_MASS_KG = 1.0;
constexpr double POLE_LENGTH_M = 1.2;
constexpr double BOB_MASS_KG = 0.2;
constexpr double ROD_MASS_KG = 0.1;
constexpr double MAX_FORCE_N = 20.0;
constexpr double SEED_DEG = -3.0;  // where the pole is stood up, DEGREES

// Balance gains. Deliberately not aggressive: at the 25 Hz state rate the
// measurement is a frame or two old, and a hot rate gain turns that delay into an
// oscillation that grows.
constexpr double K_THETA = 25.0;      // N per rad
constexpr double K_THETA_DOT = 8.0;   // N per rad/s
constexpr double K_X = 0.5;           // N per m
constexpr double K_V = 0.8;           // N per m/s

constexpr double FALLEN_RAD = 0.35;  // ~20 deg: past here, re-seat rather than flail
constexpr int MAX_RESEATS = 3;
constexpr int RUN_SAMPLES = 750;  // ~30 s at the 25 Hz state rate
constexpr double SAMPLE_TIMEOUT = 0.5;

namespace {

/// One actuator channel, or 0 when this robot does not publish it.
double channel(const vrsdk_actuator_t& a, std::uint32_t index) {
    return index < a.measured_count ? a.measured[index] : 0.0;
}

/// Let a reset or a re-seat land: services apply in phase 0 of the next physics
/// step, and the teleport needs a sample or two to reach the state stream.
void settle(vrsdk::VirtualRobot& robot) {
    for (int i = 0; i < 15; ++i) {
        robot.rate(25.0);
    }
}

/// The three actuator channels, formatted.
std::string measured_of(const vrsdk_actuator_t& a) {
    std::string out = "[";
    for (std::uint32_t i = 0; i < a.measured_count; ++i) {
        char buffer[32];
        std::snprintf(buffer, sizeof buffer, "%s%+.3f", i ? "," : "", a.measured[i]);
        out += buffer;
    }
    return out + "]";
}

/// One state line, in the units a person reads. Both cart positions, because the
/// gap between them is the whole trap.
void print_state(vrsdk::VirtualRobot& robot, const char* label, double rail_centre) {
    const vrsdk::State s = robot.states();
    std::printf("%s t=%7.2fs  theta=%+7.2f deg  rail=%+6.2f m (world x=%+7.2f)  measured=%s\n",
                label, s.elapsed, channel(s.actuator(), 1) * RAD_TO_DEG,
                s.kin().lin_pos[0] - rail_centre, s.kin().lin_pos[0],
                measured_of(s.actuator()).c_str());
}

/// The scene-authored cart-pole's sys_id, from the one command-line argument.
///
/// A sys_id is a `uint32_t` on the wire, so anything that does not fit one is
/// refused rather than wrapped: truncating `4294967297` to `1` would attach to
/// somebody else's robot and look like it worked. `strtoul` reports the overflow
/// through `errno` but wraps a leading '-' silently, so both are checked, and the
/// range check catches the rest on a 64-bit `unsigned long`.
std::uint32_t sys_id_from_args(int argc, char** argv) {
    const char* program = argc > 0 ? argv[0] : "ex29_hello_cartpole";
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

        // A failed create enumerates the catalog. (If some scene DOES know the
        // key, this spawns one -- so clean it up rather than litter.)
        try {
            vrsdk::VirtualRobot spawned = vrsdk::VirtualRobot::create(vrsdk::RobotType::CartPole);
            spawned.connect();
            std::printf(
                "this scene DOES create cart-poles (sys_id %u); removing it and attaching to %u "
                "as asked\n\n",
                spawned.sys_id(), sys_id);
            spawned.remove();
        } catch (const vrsdk::Error& e) {
            if (e.code() == VRSDK_ERR_SERVICE) {
                std::printf("create refused, and the refusal is the catalog: %s\n\n", e.what());
            } else {
                // The probe is a demonstration, not a prerequisite -- a manager
                // that does not answer says nothing about the robot we are about
                // to attach to.
                std::printf("could not probe the catalog: [%d] %s\n\n", e.code(), e.what());
            }
        }

        vrsdk::VirtualRobot robot(vrsdk::RobotType::CartPole, sys_id);
        robot.connect();
        std::printf("attached to sys_id=%u (CartPole), service key = vrobots/%u/z/srv/cartpole\n",
                    robot.sys_id(), robot.sys_id());

        // ===== pin the plant, and stand the pole up =====
        // Cart mass is owned HERE. Sending it to srv/params would be overwritten
        // a step later.
        {
            auto config = vrsdk::cartpole_config();
            config.has_cart_mass = true;
            config.cart_mass = CART_MASS_KG;
            config.has_travel_half_range = true;
            config.travel_half_range = 4.0;
            config.has_pole_rod_mass = true;
            config.pole_rod_mass = ROD_MASS_KG;
            config.has_bob_mass = true;
            config.bob_mass = BOB_MASS_KG;
            config.has_pole_length = true;
            config.pole_length = POLE_LENGTH_M;
            config.has_pole_angular_damping = true;
            config.pole_angular_damping = 0.01;
            config.has_max_force = true;
            config.max_force = MAX_FORCE_N;
            config.has_initial_pole_angle_deg = true;
            config.initial_pole_angle_deg = SEED_DEG;
            robot.configure_cartpole(config);
        }
        std::printf(
            "plant set; the pole is re-seated at %.0f deg AT REST, immediately -- not at the next "
            "reset\n",
            SEED_DEG);

        // ===== find the rail centre =====
        // The one number this plant needs and does not publish. reset() puts the
        // cart back at the pose captured on its first physics step, which IS the
        // centre the travel limits are measured from -- so measure it there
        // rather than assuming the world origin, which this rig is nowhere near.
        robot.reset();
        settle(robot);
        const double rail_centre = robot.states().kin().lin_pos[0];
        std::printf(
            "rail centre measured at x = %+.2f m (world). Every position term below is relative "
            "to THAT, not to 0.\n",
            rail_centre);
        print_state(robot, "seated", rail_centre);

        // ===== the balance loop =====
        int reseats = 0;
        double worst = 0.0;
        for (int i = 0; i < RUN_SAMPLES; ++i) {
            // One cycle per received sample. A timeout is a status, not a fault
            // (ex19): the last force latches, so holding is the right response.
            try {
                robot.wait_new_state(SAMPLE_TIMEOUT);
            } catch (const vrsdk::Error& e) {
                if (e.code() != VRSDK_ERR_TIMEOUT) {
                    throw;
                }
                std::printf("no new state -- holding the last force (it latches)\n");
                continue;
            }

            const vrsdk::State s = robot.states();
            // Position is a WORLD quantity, so the rail centre comes off it.
            // Velocity is a BODY quantity and the cart's body is pinned to
            // identity, so it is already along the rail and needs nothing done to
            // it.
            const double x = s.kin().lin_pos[0] - rail_centre;
            const double v = s.kin().lin_vel[0];
            const double theta = channel(s.actuator(), 1);
            const double theta_dot = channel(s.actuator(), 2);
            worst = std::fmax(worst, std::fabs(theta));

            // Past the catch envelope this loop is not a swing-up controller, it
            // is just a cart running at a wall. Restart the episode instead:
            // reset() re-centres the cart on the rail AND re-hangs the pole at
            // rest at the home angle configured above, which is both halves in
            // one call.
            if (std::fabs(theta) > FALLEN_RAD) {
                if (reseats >= MAX_RESEATS) {
                    std::printf("\nfallen past %.0f deg %d times -- stopping.\n",
                                FALLEN_RAD * RAD_TO_DEG, MAX_RESEATS);
                    break;
                }
                ++reseats;
                std::printf(
                    "\ntheta=%+.1f deg is past the catch envelope; resetting the episode (#%d)\n",
                    theta * RAD_TO_DEG, reseats);
                robot.set_cartpole_force(0.0);
                robot.reset();
                settle(robot);
                continue;
            }

            const double raw = -K_THETA * theta - K_THETA_DOT * theta_dot + K_X * x + K_V * v;
            const double force = std::fmin(std::fmax(raw, -MAX_FORCE_N), MAX_FORCE_N);
            robot.set_cartpole_force(force);

            if (i % 25 == 0) {
                std::printf(
                    "t=%7.2fs  theta=%+7.2f deg  theta'=%+6.2f rad/s  rail=%+6.2f m (world "
                    "x=%+7.2f)  x'=%+6.2f m/s  F=%+6.2f N  applied=%+6.2f N\n",
                    s.elapsed, theta * RAD_TO_DEG, theta_dot, x, s.kin().lin_pos[0], v, force,
                    channel(s.actuator(), 0));
            }
        }

        // ===== hand it back =====
        // A command latches: without this the cart keeps pushing forever.
        robot.set_cartpole_force(0.0);
        print_state(robot, "final ", rail_centre);
        std::printf(
            "worst excursion %.2f deg, %d re-seat(s). The robot belongs to the scene, so it is "
            "left running -- ex29 never removes it.\n",
            worst * RAD_TO_DEG, reseats);
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
