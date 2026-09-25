// ex08 -- generic_cmd: `send_cmd`, the escape hatch for the whole command space.
//
//     target/cpp-build/Release/ex08_generic_cmd
//
// Every typed wrapper -- `set_mr_pwm`, `set_car`, `set_body_force` -- is one
// line over `send_cmd(cmd_id, &args)`. `Command` is a union by convention: one
// message, and **`cmd_id` decides which payload fields mean anything**. Fill
// `vrsdk_cmd_args_t` from `vrsdk_cmd_args_default` (which NULLs every pointer),
// set only what the command reads, and the rest never reaches the wire.
//
// This example sends two commands per iteration to the truck, on purpose:
//
//   1. `SET_CAR` **by hand** -- int_arr = [steer_us, throttle_us, brake_us],
//      the exact bytes `set_car()` would have built. The truck implements it,
//      so `actuator.pwm` echoes the numbers back: **that echo is the proof the
//      escape hatch really reaches the robot.**
//   2. `ADD_BODY_FORCE` -- vec3, an id no robot type acts on yet, and one with
//      no typed wrapper at all (there is `set_body_force`, but no *add*). It is
//      published successfully and silently ignored.
//
// Neither call throws. That is the lesson: a command has no reply, so "it did
// not throw" means "published", never "acted on". Wrong id, wrong sys_id and
// wrong array length are all indistinguishable from the client side -- the
// state stream simply does not change. Check the echo, always.
//
// Use it for ids the SDK has no wrapper for; prefer the wrappers where they
// exist, because they validate (a pulse width outside 1100-2000 throws before
// anything is sent, and `send_cmd` will happily publish it).
//
// Note the ids below are spelled out as constants. The C++ surface does not
// export the `VROBOTS_CMDS` enum (Rust has `vrobots_sdk::cmd`, Python has
// `vrsdk.cmd`); the numbers are fixed by the schema.
//
// In the test scene **sys_id 0 is the truck and sys_id 1 is the multirotor**.

#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 0;  // the truck in the test scene
// VROBOTS_CMDS ids, from the schema.
constexpr std::uint32_t SET_CAR = 304;
constexpr std::uint32_t ADD_BODY_FORCE = 203;
constexpr std::int32_t STEER_US = 1500;     // centre
constexpr std::int32_t THROTTLE_US = 1600;  // light forward
constexpr std::int32_t BRAKE_US = 1100;     // released
constexpr double HZ = 5.0;

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Truck, SYS_ID);
        robot.connect();

        const std::int32_t channels[3] = {STEER_US, THROTTLE_US, BRAKE_US};
        const double gust[3] = {0.0, 0.0, 25.0};  // newtons, in our header frame

        // ===== loop =====
        for (;;) {
            // (1) An implemented id, built by hand. int_arr is the field SET_CAR
            //     reads; everything else stays NULL and off the wire.
            vrsdk_cmd_args_t drive;
            vrsdk_cmd_args_default(&drive);
            drive.int_arr = channels;
            drive.int_arr_len = 3;
            robot.send_cmd(SET_CAR, &drive);

            // (2) An id nothing acts on, whose payload rides vec3. Same call,
            //     no exception, no effect. Every payload field is a borrowed
            //     pointer: NULL means "absent" and nothing is retained after
            //     the call returns, so a stack array is fine.
            vrsdk_cmd_args_t force;
            vrsdk_cmd_args_default(&force);
            force.vec3 = gust;  // exactly 3 doubles
            robot.send_cmd(ADD_BODY_FORCE, &force);

            const vrsdk::State s = robot.states();
            const std::vector<std::uint32_t> echo = s.pwm();
            std::printf("sent SET_CAR(%u) + ADD_BODY_FORCE(%u) -> echo=[", SET_CAR,
                        ADD_BODY_FORCE);
            for (std::size_t i = 0; i < echo.size(); ++i) {
                std::printf("%s%u", i ? "," : "", echo[i]);
            }
            const double* f = s.raw.wrench.force;
            std::printf("]\n");
            std::printf(
                "      SET_CAR landed (the echo is the receipt); ADD_BODY_FORCE was ignored "
                "-- wrench=(%+.2f,%+.2f,%+.2f) N unchanged\n",
                f[0], f[1], f[2]);

            robot.rate(HZ);
        }
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
