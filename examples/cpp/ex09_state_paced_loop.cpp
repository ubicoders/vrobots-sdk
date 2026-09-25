// ex09 -- state_paced_loop: run once per sample instead of once per tick.
//
//     target/cpp-build/Release/ex09_state_paced_loop
//
// ex01 uses `rate(50.0)`: *your* clock drives the loop, and `states()` hands
// back whatever the latest snapshot is -- sometimes the same one twice,
// sometimes skipping one. That is the right default for a controller, which
// wants to emit an output on a fixed schedule whatever the sensor did.
//
// `wait_new_state(timeout)` inverts it: **the data drives the loop.** It blocks
// until a snapshot newer than the current one arrives, so the body runs exactly
// once per published sample -- no duplicates, no skips, and no need to guess a
// rate that divides 25 Hz. Reach for it when you are logging, differentiating
// or filtering, where processing a sample twice is a bug.
//
// The two things to get right:
//
//   * **A timeout is not a failure.** `VRSDK_ERR_TIMEOUT` means "no new sample
//     in time", which is how a paused or stopped sim announces itself; the
//     session is fine and the next call may well succeed. Catch that one code
//     and carry on -- letting it escape is what turns a paused sim into a
//     crashed program. Every other code is real, which is why the catch below
//     rethrows anything else.
//   * **`seq` is the ground truth for drops.** The SDK stores the newest sample
//     it received; if two arrive between wakeups you see the second one and
//     `seq` jumps. `stats().seq_gaps` counts that for you.
//
// In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;    // the multirotor in the test scene
constexpr double TIMEOUT_S = 0.2;      // 5x the 25 Hz period

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();

        std::uint64_t last_seq = 0;
        std::int64_t last_t_ns = 0;

        // ===== loop =====
        for (;;) {
            try {
                robot.wait_new_state(TIMEOUT_S);
            } catch (const vrsdk::Error& e) {
                if (e.code() != VRSDK_ERR_TIMEOUT) {
                    throw;  // a real failure
                }
                // Not a broken session: no sample arrived in time. The sim is
                // paused, stopped, or the machine is very busy. states() still
                // returns the last snapshot it had.
                const vrsdk::State s = robot.states();
                std::printf("no new state in %.1fs; still holding seq=%llu at t=%.3f\n", TIMEOUT_S,
                            static_cast<unsigned long long>(s.seq), s.elapsed);
                continue;
            }

            // Exactly one new sample is waiting -- read it and do the work.
            const vrsdk::State s = robot.states();
            const double dt_ms =
                last_t_ns == 0 ? 0.0 : static_cast<double>(s.t_ns - last_t_ns) / 1e6;
            const std::uint64_t skipped = s.seq > last_seq + 1 ? s.seq - last_seq - 1 : 0;
            last_seq = s.seq;
            last_t_ns = s.t_ns;

            const double* p = s.kin().lin_pos;
            std::printf("seq=%llu dt=%6.1f ms pos=(%.3f,%.2f,%.2f)",
                        static_cast<unsigned long long>(s.seq), dt_ms, p[0], p[1], p[2]);
            if (skipped > 0) {
                std::printf("  <- %llu sample(s) skipped", static_cast<unsigned long long>(skipped));
            }
            std::printf("\n");
        }
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
