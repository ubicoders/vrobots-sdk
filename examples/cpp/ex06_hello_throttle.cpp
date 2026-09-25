// ex06 -- hello_throttle: the *other* multirotor actuator command, and how to
// tell a command that is ignored from one that landed.
//
//     target/cpp-build/Release/ex06_hello_throttle
//
// `SET_MR_THROTTLE` is normalised per-rotor throttle: four values on 0..1
// instead of four pulse widths. It is the command you would reach for to hover
// without thinking in microseconds -- and it is defined on the wire but **no
// robot type acts on it yet**, so this example is really a lesson in how that
// looks from the client side.
//
// There is no reply and no error. A robot that receives an id it does not
// implement silently ignores it, because the id space is shared across robot
// types and "not mine" is correct behaviour, not a fault. So the only evidence
// you ever get is the state stream, and the loop below prints all of it:
//
//   * `actuator.pwm`        -- the pulse widths the robot latched. Unchanged.
//   * `actuator.normalized` -- the normalised command it latched, if it has one.
//   * `actuator.measured`   -- what the devices actually did (rotor rad/s).
//
// Watch the echo across runs and you get latching for free: if you ran ex02
// first, the pulse widths *it* latched are still there, published by a process
// that has already exited. A command is a setpoint the robot holds, not an
// event, and the echo reports the holder rather than the sender.
//
// Run ex02 side by side: identical loop, an id the sim implements, and the echo
// moves. Today the way to fly a multirotor is `set_mr_pwm`.
//
// In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr double THROTTLE = 0.6;     // normalised 0..1, the same for all rotors
constexpr double HZ = 25.0;

/// Print a fixed-size actuator array, whatever its element type.
template <typename T> static void print_array(const char* label, const T* values, std::uint32_t n) {
    std::printf("%s=[", label);
    for (std::uint32_t i = 0; i < n; ++i) {
        std::printf(i ? ",%.3f" : "%.3f", static_cast<double>(values[i]));
    }
    std::printf("] ");
}

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();

        // Exactly four values: the wrapper refuses any other count before it
        // publishes, because the sim's quadrotor has four rotors.
        const std::vector<double> throttle = {THROTTLE, THROTTLE, THROTTLE, THROTTLE};

        // ===== loop =====
        for (;;) {
            // Published exactly like set_mr_pwm: one put on vrobots/<id>/z/cmd,
            // no reply, latched until the next one arrives.
            robot.set_mr_throttle(throttle);

            const vrsdk::State s = robot.states();
            // The state frame is the robot's, not yours -- "frd" here, so
            // lin_pos[2] is DOWN and altitude is its negation.
            const double alt = -s.kin().lin_pos[2];
            const vrsdk_actuator_t& a = s.actuator();

            std::printf("sent %.2f x4 -> alt=%.2f m  ", THROTTLE, alt);
            std::printf("pwm=[");
            for (std::uint32_t i = 0; i < a.pwm_count; ++i) {
                std::printf(i ? ",%u" : "%u", a.pwm[i]);
            }
            std::printf("] ");
            print_array("normalized", a.normalized, a.normalized_count);
            print_array("measured", a.measured, a.measured_count);
            std::printf("\n");

            robot.rate(HZ);
        }
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
