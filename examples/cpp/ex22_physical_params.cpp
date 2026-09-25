// ex22 -- physical_params: change the mass under a running controller.
//
//     target/cpp-build/Release/ex22_physical_params 1   # the scene's multirotor
//     target/cpp-build/Release/ex22_physical_params     # create one instead
//
// `srv/params` carries two numbers, mass and the principal moments of inertia,
// and it is the ONLY channel for either. That matters more than it sounds:
//
//   NEITHER FIGURE APPEARS IN THE STATE MESSAGE. Mass properties are
//   quasi-static configuration, not state, so there is nothing to read back and
//   nothing to diff. The confirmation is entirely BEHAVIOURAL -- the same pulse
//   width has to produce a different acceleration, or the change did not land.
//
// So this example flies a fixed collective twice, once at 1.0 kg and once at
// 2.0 kg, and compares the climb rate. Same command, same rotors, same air: a
// heavier aircraft climbs slower, and that is the whole receipt.
//
// Which is also the reason the service exists. It works MID-FLIGHT, so changing
// the mass under a running loop is the standard way to test a controller against
// a payload it was not tuned for.
//
// TWO SILENT FAILURES THE SDK REFUSES ON YOUR BEHALF
//
//   The simulator does not validate these; it keeps the prefab's value and acks
//   `ok`, so a bad request looks exactly like a good one from out here:
//
//   | you send                                   | the sim does                | you see |
//   |--------------------------------------------|-----------------------------|---------|
//   | `mass <= 0`                                | keeps the body's mass       | nothing |
//   | a moi not strictly positive on ALL three   | keeps Unity's own tensor    | nothing |
//
//   `set_physical_params` therefore refuses both before anything is published,
//   as VRSDK_ERR_INVALID_ARGUMENT naming the field. A half-filled inertia triple
//   -- two axes set, one left at zero -- is the classic way to think you changed
//   the inertia and not have; the third block below shows it being caught.
//
// FRAMES
//
//   Moments of inertia are read in YOUR header frame (the default here is
//   "unity") and permuted into the robot's. They are positive quantities, so
//   unlike a force or a rate the conversion never flips a sign -- it only
//   reorders the triple.
//
// NOT THE CART-POLE
//
//   A cart-pole re-stamps its cart's mass from `srv/cartpole` on every parameter
//   apply, so a mass sent here is overwritten a step later on that one robot
//   type. Use `configure_cartpole` there (ex29).
//
// WHICH MULTIROTOR, AND WHY ATTACHING COSTS YOU THE UNDO
//
//   With no argument this CREATES a multirotor and removes it again. Give it a
//   `sys_id` and it attaches to the scene's own -- and then NEVER REMOVES IT,
//   which has a consequence worth stating plainly:
//
//     The mass and inertia set below STAY SET, for that robot, for every other
//     client, until the scene is reloaded. `reset()` does not undo them (ex21:
//     it is a state reset, not a factory reset), and there is NO GETTER --
//     neither figure is in the state message, so the SDK cannot read the old
//     value first and put it back. Write down what you started with, or reload
//     the scene.
//
//   Prefer the argument for now anyway. AS OF SIM v3.0.0 A CLIENT-CREATED
//   MULTIROTOR SPAWNS WITH A RIGIDBODY THAT NEVER INTEGRATES: it hangs where it
//   spawned and ignores every pulse width and even a direct body force, while
//   its actuator echo answers normally. Both climb runs then read 0.00 m/s and
//   the comparison shows nothing -- not because the service failed, but because
//   nothing in that robot moves. Trucks and MSDs created the same way have live
//   physics; the scene's multirotor flies.

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>

#include <vrobots_sdk.hpp>

constexpr const char* USAGE =
    "With no argument this CREATES a multirotor and removes it again. With a sys_id it\n"
    "attaches to the scene's own, never removes it, and LEAVES THE MASS CHANGED until\n"
    "the scene is reloaded:\n"
    "\n"
    "    ex22_physical_params 1\n"
    "\n"
    "Prefer the argument until the created-multirotor physics bug is fixed (see the\n"
    "file header) -- on a created one both climb runs read 0.00 m/s. List what is\n"
    "publishing with:\n"
    "\n"
    "    vrobots topic list";

// High enough that BOTH masses still climb -- a run that ends up sitting on the
// ground measures nothing, and "0.00 m/s" would not distinguish a heavy aircraft
// from a request that never landed.
constexpr double COLLECTIVE_US = 1800.0;
constexpr double LIGHT_KG = 1.0;
constexpr double HEAVY_KG = 2.0;
constexpr double MOI[3] = {0.02, 0.02, 0.04};  // kg.m^2, in OUR header frame
constexpr double HZ = 25.0;
constexpr int SETTLE_SAMPLES = 25;   // ~1 s for the reset and the new mass to bite
constexpr int MEASURE_SAMPLES = 50;  // ~2 s of climb per run

namespace {

/// Reset to home, hold a fixed collective, and return the mean climb rate (m/s).
///
/// Height comes from `kin.lin_pos[2]`, negated because the multirotor publishes
/// in "frd" and the third component is DOWN.
///
/// NOT FROM `env.agl`, which looks like the obvious field and is a trap: as of
/// sim v3.0.0 it is a hard-coded zero for *every* robot. The simulator would need
/// a downward raycast to fill it and deliberately publishes 0 rather than
/// guessing, on the grounds that `env` is the truth block and an invented height
/// is worse than an absent one. It is a placeholder, not a measurement.
double climb_run(vrsdk::VirtualRobot& robot, double mass_kg) {
    std::printf("-- %.1f kg --\n", mass_kg);

    // Start each run from the same place, so the two are comparable. The reset
    // does not undo the mass: configuration survives a state reset (ex21).
    robot.reset();
    for (int i = 0; i < SETTLE_SAMPLES; ++i) {
        robot.set_mr_pwm({COLLECTIVE_US, COLLECTIVE_US, COLLECTIVE_US, COLLECTIVE_US});
        robot.rate(HZ);
    }

    const vrsdk::State start = robot.states();
    for (int i = 0; i < MEASURE_SAMPLES; ++i) {
        robot.set_mr_pwm({COLLECTIVE_US, COLLECTIVE_US, COLLECTIVE_US, COLLECTIVE_US});
        if (i % 25 == 0) {
            const vrsdk::State s = robot.states();
            const std::vector<std::uint32_t> echo = s.pwm();
            std::printf("   t=%6.2fs alt=%7.2f m  climb=%+6.2f m/s  echo=[", s.elapsed,
                        -s.kin().lin_pos[2],   // "frd": the third component is DOWN
                        -s.kin().lin_vel[2]);
            for (std::size_t c = 0; c < echo.size(); ++c) {
                std::printf("%s%u", c ? "," : "", echo[c]);
            }
            std::printf("]\n");
        }
        robot.rate(HZ);
    }
    const vrsdk::State end = robot.states();

    const double seconds = end.elapsed - start.elapsed;
    if (seconds <= 0.0) {
        return 0.0;
    }
    // "frd": z counts DOWN, so a climb is a DECREASE. The summary and the live
    // `climb=` column above must agree, or one of them is lying.
    return (start.kin().lin_pos[2] - end.kin().lin_pos[2]) / seconds;
}

/// Print a refusal the way a caller should read one: code, then the sim-side
/// behaviour it is standing in for.
template <typename Call> void show_refusal(const char* what, Call call) {
    try {
        call();
        std::printf("  %-48s UNEXPECTED: accepted\n", what);
    } catch (const vrsdk::Error& e) {
        std::printf("  %-48s [%d] %s\n", what, e.code(), e.what());
    }
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
    const char* program = argc > 0 ? argv[0] : "ex22_physical_params";
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
        std::printf("%s sys_id=%u, service key = vrobots/%u/z/srv/params\n",
                    target ? "attached to" : "created", robot.sys_id(), robot.sys_id());
        if (target) {
            std::printf(
                "NOTE: this robot belongs to the scene. The mass and inertia set below stay set "
                "until the scene is reloaded -- there is no getter to restore them from.\n\n");
        } else {
            std::printf("\n");
        }

        // ===== run 1: light =====
        {
            auto params = vrsdk::physical_params();
            params.has_mass = true;
            params.mass = LIGHT_KG;
            params.has_moi = true;
            params.moi[0] = MOI[0];
            params.moi[1] = MOI[1];
            params.moi[2] = MOI[2];
            robot.set_physical_params(params);
        }
        const double light = climb_run(robot, LIGHT_KG);

        // ===== run 2: heavy, same command =====
        {
            auto params = vrsdk::physical_params();
            params.has_mass = true;
            params.mass = HEAVY_KG;
            robot.set_physical_params(params);
        }
        const double heavy = climb_run(robot, HEAVY_KG);

        std::printf("\n%.0f us on every rotor, %.1f s of climb, twice:\n  %.1f kg -> %+6.2f m/s\n  "
                    "%.1f kg -> %+6.2f m/s\n",
                    COLLECTIVE_US, static_cast<double>(MEASURE_SAMPLES) / HZ, LIGHT_KG, light,
                    HEAVY_KG, heavy);
        std::printf(
            "The difference IS the receipt -- there is no mass field in the state message to read "
            "back.\n");

        // ===== the two refusals =====
        std::printf("\n-- what the SDK refuses before anything reaches the wire --\n");
        show_refusal("mass = 0.0", [&] {
            auto params = vrsdk::physical_params();
            params.has_mass = true;
            params.mass = 0.0;
            robot.set_physical_params(params);
        });
        show_refusal("moi = [0.02, 0.0, 0.04] (one axis left at zero)", [&] {
            auto params = vrsdk::physical_params();
            params.has_moi = true;
            params.moi[0] = 0.02;
            params.moi[1] = 0.0;
            params.moi[2] = 0.04;
            robot.set_physical_params(params);
        });
        show_refusal("nothing set at all",
                     [&] { robot.set_physical_params(vrsdk::physical_params()); });
        std::printf(
            "All three are acked `ok` by the simulator and silently ignored, which is why they "
            "are caught here instead.\n");

        if (!target) {
            robot.remove();
            std::printf("\ndeleted sys_id=%u\n", robot.sys_id());
        } else {
            std::printf(
                "\nsys_id=%u belongs to the scene: left running at %.1f kg, and it stays there "
                "until the scene is reloaded.\n",
                robot.sys_id(), HEAVY_KG);
        }
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
