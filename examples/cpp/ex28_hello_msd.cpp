// ex28 -- hello_msd: the smallest thing in the simulator that can be controlled.
//
//     target/cpp-build/Release/ex28_hello_msd
//
// A mass-spring-damper is one mass on one axis:
//
//     m*x'' + c*x' + k*x = F
//
// and the SDK owns every letter of it. `m` is `set_physical_params` (ex22), `k`
// and `c` are `configure_msd`, and `F` is `set_msd_force` -- newtons along the
// plant's +x, clamped to its `max_force` (100 N by default). Which makes this
// the one robot where you can *predict* the answer before you run it, and the
// printout below does:
//
//     settles at   F / k                metres
//     period       2*pi*sqrt(m/k)       seconds
//     damping      c / (2*sqrt(k*m))    -- < 1 rings, ~1 slides home, > 1 crawls
//
// AN MSD IS CREATABLE. The sandbox catalog is multirotor, truck, msd, so
// `VirtualRobot::create(RobotType::Msd)` spawns one -- unlike the cart-pole, the
// half-drone and the Global Hawk (ex29-ex33), which exist only where a scene
// authored them.
//
// READING IT
//
//   It publishes in the identity "unity" frame, so `kin.lin_pos[0]` is the
//   position you watch in the editor and `kin.lin_vel[0]` is x'. The other two
//   components never move. The actuator block carries the plant's own
//   arithmetic:
//
//   | channel                | meaning                                        |
//   |------------------------|------------------------------------------------|
//   | `actuator.measured[0]` | the TOTAL force on the mass, `F - k*x - c*x'`   |
//   | `actuator.measured[1]` | displacement from equilibrium, metres          |
//
//   Note that measured[0] is not the force you sent -- it is what the spring and
//   damper left of it, which is why it crosses zero at every peak.
//
// `configure_msd` CLAMPS WHERE THE SDK REFUSES
//
//   A negative k or c is COMMITTED AS ZERO by the simulator and acked `ok`: a
//   spring that quietly vanished, not an error. So `configure_msd` refuses
//   negatives before they are sent (demonstrated at the end), and for everything
//   else the committed value may still differ from the asked-for one -- the
//   state stream is the only place that says which.
//
// Commands latch: the step force below stays applied until the next command
// replaces it, which is why releasing it takes an explicit `set_msd_force(0.0)`.

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include <vrobots_sdk.hpp>

constexpr double MASS_KG = 1.0;  // pinned so the predictions below are arithmetic
constexpr double STEP_N = 20.0;
constexpr double HZ = 25.0;             // the state rate
constexpr int STEP_SAMPLES = 125;       // ~5 s pushing
constexpr int RELEASE_SAMPLES = 125;    // ~5 s ringing back down

namespace {

/// One step-and-release run on a given plant.
struct Response {
    std::string label;
    double k = 0.0;
    double c = 0.0;
    double settled = 0.0;
    double period = 0.0;
};

/// One actuator channel, or 0 when this robot does not publish it.
double channel(const vrsdk_actuator_t& a, std::uint32_t index) {
    return index < a.measured_count ? a.measured[index] : 0.0;
}

/// Retune the plant, push it with a step, then release and watch it ring down.
Response step_response(vrsdk::VirtualRobot& robot, const char* label, double k, double c,
                       bool retune) {
    std::printf("-- %s --\n", label);
    if (retune) {
        auto config = vrsdk::msd_config();
        config.has_spring_k = true;
        config.spring_k = k;
        config.has_damping_c = true;
        config.damping_c = c;
        robot.configure_msd(config);
    }
    // Home, at rest, with the force latch cleared -- otherwise the previous run's
    // step is still pushing.
    robot.set_msd_force(0.0);
    robot.reset();

    int crossings = 0;
    double previous_sign = 0.0;
    double settled = 0.0;

    for (int i = 0; i < STEP_SAMPLES; ++i) {
        robot.set_msd_force(STEP_N);
        const vrsdk::State s = robot.states();
        const double velocity = s.kin().lin_vel[0];

        // Each velocity sign change is half a cycle. Ignore the crawl either side
        // of zero, or numerical dither counts as oscillation.
        if (std::fabs(velocity) > 1e-3) {
            const double sign = velocity > 0.0 ? 1.0 : -1.0;
            if (previous_sign != 0.0 && sign != previous_sign) {
                ++crossings;
            }
            previous_sign = sign;
        }

        settled = channel(s.actuator(), 1);
        if (i % 25 == 0) {
            std::printf(
                "   t=%6.2fs x=%+7.3f m  x'=%+7.3f m/s  disp=%+7.3f m  net F=%+8.2f N\n",
                s.elapsed, s.kin().lin_pos[0], velocity, settled, channel(s.actuator(), 0));
        }
        robot.rate(HZ);
    }

    // Release. The force LATCHES, so this zero is not optional.
    std::printf("   release (set_msd_force(0.0)) -- watch it ring back to equilibrium\n");
    for (int i = 0; i < RELEASE_SAMPLES; ++i) {
        robot.set_msd_force(0.0);
        if (i % 50 == 0) {
            const vrsdk::State s = robot.states();
            std::printf("   t=%6.2fs x=%+7.3f m  x'=%+7.3f m/s  disp=%+7.3f m\n", s.elapsed,
                        s.kin().lin_pos[0], s.kin().lin_vel[0], channel(s.actuator(), 1));
        }
        robot.rate(HZ);
    }

    const double seconds = static_cast<double>(STEP_SAMPLES) / HZ;
    Response out;
    out.label = label;
    out.k = k;
    out.c = c;
    out.settled = settled;
    // Damped past ringing: no crossings to time.
    out.period = crossings > 0 ? 2.0 * seconds / crossings : std::numeric_limits<double>::infinity();
    return out;
}

}  // namespace

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot = vrsdk::VirtualRobot::create(vrsdk::RobotType::Msd);
        robot.connect();
        std::printf("created sys_id=%u (Msd), service key = vrobots/%u/z/srv/msd\n",
                    robot.sys_id(), robot.sys_id());

        {
            auto params = vrsdk::physical_params();
            params.has_mass = true;
            params.mass = MASS_KG;
            robot.set_physical_params(params);
        }

        const vrsdk::State first = robot.states();
        std::printf(
            "frame=\"%s\"  pos=(%+.3f,%+.3f,%+.3f) -- only the first component ever moves\n\n",
            first.coord_frame_id.c_str(), first.kin().lin_pos[0], first.kin().lin_pos[1],
            first.kin().lin_pos[2]);

        // ===== the plant as it spawns: k = 20, c = 1 =====
        const Response soft = step_response(robot, "as spawned (k=20, c=1)", 20.0, 1.0, false);

        // ===== four times stiffer: same push, a quarter of the travel, twice as fast
        const Response stiff = step_response(robot, "k=80, c=1", 80.0, 1.0, true);

        // ===== and damped, so it stops arguing about it =====
        const Response damped = step_response(robot, "k=80, c=16", 80.0, 16.0, true);

        std::printf("\n%.0f N step, %.0f kg, three plants:\n", STEP_N, MASS_KG);
        std::printf("  %-22s %9s %9s %9s %9s %7s\n", "", "x_final", "F/k", "period", "2pi*sqrt",
                    "zeta");
        for (const Response* run : {&soft, &stiff, &damped}) {
            const double tau = 2.0 * 3.14159265358979323846 * std::sqrt(MASS_KG / run->k);
            std::printf("  %-22s %9.3f %9.3f %9.2f %9.2f %7.2f\n", run->label.c_str(),
                        run->settled, STEP_N / run->k, run->period, tau,
                        run->c / (2.0 * std::sqrt(run->k * MASS_KG)));
        }
        std::printf(
            "Stiffer means smaller and faster; damped means it arrives once instead of four "
            "times.\n");

        // ===== the value the SDK will not send =====
        try {
            auto config = vrsdk::msd_config();
            config.has_spring_k = true;
            config.spring_k = -5.0;
            robot.configure_msd(config);
            std::printf("\nUNEXPECTED: a negative spring constant was accepted\n");
        } catch (const vrsdk::Error& e) {
            std::printf("\nspring_k = -5.0 -> [%d] %s\n", e.code(), e.what());
        }
        std::printf("(the simulator would commit that as 0 and ack `ok`: no spring, no error)\n");

        robot.remove();
        std::printf("deleted sys_id=%u\n", robot.sys_id());
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
