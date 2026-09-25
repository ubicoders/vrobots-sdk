// ex07 -- body_wrench: push the robot around with a force and a torque.
//
//     target/cpp-build/Release/ex07_body_wrench
//
// The wrench group is the disturbance-injection channel: a wind gust, a dropped
// payload, a contact push. Three typed wrappers cover it, and they are the
// clearest example in the SDK of the wire's shape leaking into an API:
//
//   | call                    | wire payload                                |
//   |-------------------------|---------------------------------------------|
//   | `set_body_force(f)`     | `vec3` = force                              |
//   | `set_body_torque(t)`    | `vec3` = torque                             |
//   | `set_body_ft(f, t)`     | `vec3` = force, **`vec3_arr[0]`** = torque  |
//
// `SET_BODY_FT` is asymmetric because the schema is; the wrapper hides it, and
// ex08 shows what filling it by hand looks like.
//
// **Vectors carry a frame.** Every command this SDK sends is stamped with the
// `coord_frame_id` in the connect options -- "unity" by default, which is
// left-handed X-right / Y-up / Z-forward. The robot converts your vector into
// its own axes using the physically correct rule for the command (a force
// converts differently from a torque, which is a pseudovector and carries the
// handedness sign). An *untagged* vector is taken at face value and silently
// flips sign between opposite-handed conventions, which is why the SDK always
// tags. Note the two frames in the printout: you send "unity", the state comes
// back "frd".
//
// Honest caveat, the same one as ex06: **no robot type acts on these yet.**
// They are on the wire, and silently ignored. `state.wrench` -- printed below
// -- is the total force and torque the *simulator* has on the body, so it is
// where the effect will appear the day the sim implements them. Until then it
// shows the robot's own actuators and nothing of yours.
//
// In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

#include <array>
#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr std::array<double, 3> GUST_N = {5.0, 0.0, 0.0};      // newtons, OUR frame
constexpr std::array<double, 3> TWIST_NM = {0.0, 0.0, 0.2};    // newton-metres, same frame
constexpr double HZ = 2.0;

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();
        // The C++ surface has no accessor for the options in effect (Rust has
        // `robot.options()`, Python has `robot.options`), so the send frame is
        // the documented default unless you passed a
        // `vrsdk_connect_options_t` of your own.
        std::printf("sending vectors tagged \"unity\" (the SDK default)\n");

        // ===== loop =====
        for (std::uint64_t step = 0;; ++step) {
            // One verb per iteration, so each printed line names exactly what
            // went out on the wire.
            const char* sent = nullptr;
            switch (step % 3) {
                case 0:
                    robot.set_body_force(GUST_N);
                    sent = "set_body_force(5, 0, 0)";
                    break;
                case 1:
                    robot.set_body_torque(TWIST_NM);
                    sent = "set_body_torque(0, 0, 0.2)";
                    break;
                default:
                    robot.set_body_ft(GUST_N, TWIST_NM);
                    sent = "set_body_ft((5,0,0), (0,0,0.2))";
                    break;
            }

            const vrsdk::State s = robot.states();
            const double* f = s.raw.wrench.force;
            const double* t = s.raw.wrench.torque;
            std::printf("%s\n", sent);
            std::printf(
                "    state.wrench force=(%+.2f,%+.2f,%+.2f) N  torque=(%+.2f,%+.2f,%+.2f) N.m  "
                "in \"%s\"\n",
                f[0], f[1], f[2], t[0], t[1], t[2], s.coord_frame_id.c_str());

            robot.rate(HZ);
        }
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
