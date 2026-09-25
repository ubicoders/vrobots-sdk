// ex05 -- hello_car: drive the truck.
//
//     target/cpp-build/Release/ex05_hello_car
//
// Same loop shape as ex02, different actuator. `SET_CAR` channels are pulse
// widths on the 1100-2000 us band:
//
//   | channel  | 1100         | 1500              | 1900         |
//   |----------|--------------|-------------------|--------------|
//   | steer    | full left    | centre            | full right   |
//   | throttle | full reverse | stop (idle brake) | full forward |
//   | brake    | released     | --                | full         |
//
// Brake is **bottom-anchored** -- 1100 is released, not 1500 -- and omitting it
// sends the two-channel form, which brakes nothing.
//
// In the test scene **sys_id 0 is the truck and sys_id 1 is the multirotor**;
// run `vrobots topic list` to see what is actually there.

#include <cmath>
#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 0;      // the truck in the test scene
constexpr double STEER_US = 1400.0;      // left of centre
constexpr double THROTTLE_US = 1650.0;   // light forward
constexpr double BRAKE_US = 1100.0;      // released
constexpr double HZ = 50.0;

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Truck, SYS_ID);
        robot.connect();
        std::printf("connected to sys_id %u\n", robot.sys_id());

        // ===== loop =====
        for (;;) {
            const vrsdk::State s = robot.states();
            const double* p = s.kin().lin_pos;

            // A gentle left arc: steering left of centre, light forward
            // throttle, brake released.
            robot.set_car(STEER_US, THROTTLE_US, BRAKE_US);

            // Speed from the body-frame twist, so it is visible that the truck
            // really is moving rather than that the command was merely accepted.
            const double* v = s.kin().lin_vel;
            const double speed = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

            const std::vector<std::uint32_t> echo = s.pwm();
            std::printf("State t=%.3f pos=(%.3f,%.2f,%.2f) speed=%.2f m/s  pwm_echo=[", s.elapsed,
                        p[0], p[1], p[2], speed);
            for (std::size_t i = 0; i < echo.size(); ++i) {
                std::printf("%s%u", i ? "," : "", echo[i]);
            }
            std::printf("]\n");

            robot.rate(HZ);
        }
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
