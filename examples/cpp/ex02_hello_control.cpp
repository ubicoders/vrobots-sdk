// ex02 -- hello_control: close the loop. Read state, send actuator commands.
//
//     cmake --build target/cpp-build --config Release
//     target/cpp-build/Release/ex02_hello_control
//
// `set_mr_pwm` is the lowest actuation level there is: **you are the flight
// controller**. No attitude stabilisation, no rate damping, nothing between
// these pulse widths and the thrust curves. 1100 is idle (a flying drone
// falls), 2000 is full, and hover is wherever total thrust crosses weight.
//
// There is no reply to a command. The proof it landed is the state stream:
// `actuator.pwm` echoes back what the robot latched, which is what the loop
// below prints beside the position. `PWM_US` is barely off idle; edit it to
// 1700 to watch the drone climb.
//
// In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr double PWM_US = 1501.0;    // microseconds per rotor, 1100-2000 band
constexpr double HZ = 100.0;

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();
        std::printf("connected to sys_id %u\n", robot.sys_id());

        // ===== loop =====
        for (;;) {
            const vrsdk::State s = robot.states();
            const double* p = s.kin().lin_pos;

            // Do some COOL control here and publish -- PID/EKF is user code,
            // NOT the SDK.
            const std::vector<double> cool_control_result = {PWM_US, PWM_US, PWM_US, PWM_US};
            robot.set_mr_pwm(cool_control_result);

            // The echo: what the robot actually latched, from the state stream.
            const std::vector<std::uint32_t> echo = s.pwm();
            std::printf("State t=%.3f pos=(%.3f,%.2f,%.2f)  pwm_echo=[", s.elapsed, p[0], p[1],
                        p[2]);
            for (std::size_t i = 0; i < echo.size(); ++i) {
                std::printf("%s%u", i ? "," : "", echo[i]);
            }
            // `measured` is what the devices did -- rotor rad/s here, so it is
            // the difference between "the command arrived" and "the rotors
            // spun".
            std::printf("]  rotor0=%.1f rad/s\n",
                        s.actuator().measured_count > 0 ? s.actuator().measured[0] : 0.0);

            robot.rate(HZ);
        }
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
