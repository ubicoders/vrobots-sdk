// ex01 -- hello_states: read the robot's state at your own rate.
//
// Build (from the repo root) against an unpacked C bundle from the Releases
// page (see examples/cpp/README.md), then run with the sim in Play mode:
//
//     cmake -S examples/cpp -B target/cpp-build -DCMAKE_BUILD_TYPE=Release
//     cmake --build target/cpp-build --config Release
//     target/cpp-build/Release/ex01_hello_states        (Windows)
//     target/cpp-build/ex01_hello_states                (Linux)
//
// Note the shape: `main` does setup, then owns a plain infinite loop. There is
// no base class, no runner and no `update()` callback -- the SDK never calls
// your code. `states()` is always the latest snapshot, so there is no "did new
// data arrive?" flag to check either.
//
// No command-line arguments here or in any other example: the settings live in
// the constants at the top, and a permutation worth showing is its own file.
//
// In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**;
// run `vrobots topic list` to see what is actually publishing.

#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr double HZ = 50.0;

/// Everything the SDK waits on, retries or drops shows up here. Registering it
/// before connect() is the point: connect is the noisiest moment, and a hang
/// with no log is the hardest thing to debug.
static void on_log(vrsdk::LogLevel level, const char* target, const char* message) {
    std::printf("[%-5s %s] %s\n", vrsdk::to_string(level), target, message);
}

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();  // header and library must be the same release
        vrsdk::set_log_callback(on_log);

        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();  // blocks until the first state snapshot arrives
        std::printf("connected to sys_id %u\n", robot.sys_id());

        // ===== loop =====
        for (;;) {
            const vrsdk::State s = robot.states();  // latest snapshot, never torn
            const double* p = s.kin().lin_pos;
            std::printf("State t=%.3f pos=(%.3f,%.2f,%.2f)\n", s.elapsed, p[0], p[1], p[2]);
            robot.rate(HZ);  // drift-compensated pacing, Hz
        }
    } catch (const vrsdk::Error& e) {
        // `code()` is the SDK's stable number -- the same one Python's
        // VrError.code and the CLI's `error [N]` report.
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
