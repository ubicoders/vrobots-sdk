// ex19 -- robust_loop: a loop that survives the simulator going away.
//
//     target/cpp-build/Release/ex19_robust_loop
//
// **Try this while it runs:** stop the simulator (close it, or leave Play
// mode), wait a few seconds, and start it again. The loop must not exit, must
// not spin, and must pick the robot back up on its own. That is the whole
// example.
//
// Three behaviours make that work, and each is a deliberate design decision
// rather than an accident:
//
// **1. `states()` never throws and never blocks.** When the sim stops it keeps
// returning the last snapshot it had -- forever, unchanged. That is the
// observer contract: a control loop must not throw from a data read. The cost
// is that a dead sim looks exactly like a stationary robot, so **a stall is not
// detectable from `states()` alone**. Watch `elapsed` (or `seq`) stop
// advancing.
//
// **2. `wait_new_state()` is the detector.** It throws `VRSDK_ERR_TIMEOUT` when
// nothing new arrived, which is a *status*, not a fault -- the session is
// healthy and the next call may succeed. The right handling is to catch that
// one code, note it, keep the last known state, and try again. Letting it
// escape `main` is the bug this example exists to prevent.
//
// **3. Nothing tears down.** The zenoh session, the subscriber and the command
// publisher all outlive the sim's absence. When the simulator returns, samples
// resume on the same session with no reconnect logic here -- discovery is
// zenoh's job.
//
// The restart is visible in two places, and neither is an error:
//
//   * **`seq` restarts from 0.** The SDK recognises that as a new publisher
//     rather than as thousands of lost samples: it logs "state seq went
//     backwards: the publisher restarted", and **`seq_gaps` stays where it
//     was.** A gap count that jumped by thousands after a restart would make
//     the counter useless for what it is for, which is spotting real drops.
//   * **`elapsed` does not reset.** It is measured from the epoch fixed at this
//     handle's first sample, so it keeps counting through the outage and comes
//     back having jumped forward by however long the sim was away.
//
// Commands sent into the void also succeed: publishing to a topic nobody is
// subscribed to is not an error in zenoh, so `set_mr_pwm` throws nothing the
// whole time the sim is down. There is no reply to a command, ever -- so the
// echo in the state stream is the only thing that can tell you a robot is
// listening.
//
// In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

#include <chrono>
#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr double TIMEOUT_S = 0.5;    // ~12x the 25 Hz period
constexpr double PWM_US = 1501.0;
constexpr std::uint64_t REPORT_EVERY = 25;  // one status line a second at 25 Hz

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();
        std::printf(
            "connected. Now stop and restart the simulator -- this loop should ride it out.\n\n");

        const std::vector<double> hold = {PWM_US, PWM_US, PWM_US, PWM_US};
        bool healthy = true;
        std::uint64_t samples = 0;
        std::chrono::steady_clock::time_point down_since{};

        // ===== loop =====
        for (;;) {
            bool timed_out = false;
            try {
                robot.wait_new_state(TIMEOUT_S);
            } catch (const vrsdk::Error& e) {
                if (e.code() != VRSDK_ERR_TIMEOUT) {
                    throw;  // a real failure: the session itself is gone
                }
                timed_out = true;
            }

            if (timed_out) {
                if (healthy) {
                    std::printf("\nSTALLED: no new state in %.1fs. Not an error -- holding.\n",
                                TIMEOUT_S);
                    healthy = false;
                    down_since = std::chrono::steady_clock::now();
                }
                // states() still answers, with the LAST snapshot. Note that
                // `elapsed` is frozen: that, not an exception, is how a dead
                // sim looks from a data read.
                const vrsdk::State s = robot.states();
                const double down =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - down_since)
                        .count();
                std::printf(
                    "    down %5.1fs -- stale snapshot still readable: seq=%llu t=%.2fs "
                    "(frozen)\n",
                    down, static_cast<unsigned long long>(s.seq), s.elapsed);

                // Publishing into an empty topic is not an error in zenoh, so
                // this keeps succeeding. A command has no reply; only the echo
                // in the state stream ever proves anything landed.
                robot.set_mr_pwm(hold);
                continue;
            }

            if (!healthy) {
                const double outage =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - down_since)
                        .count();
                const vrsdk_state_stats_t st = robot.stats();
                const vrsdk::State s = robot.states();
                std::printf(
                    "RECOVERED after %.1f s -- seq restarted at %llu (elapsed jumped to %.2fs, "
                    "it never resets); received=%llu seq_gaps=%llu missed_samples=%llu -- a "
                    "restart is not a gap\n",
                    outage, static_cast<unsigned long long>(s.seq), s.elapsed,
                    static_cast<unsigned long long>(st.received),
                    static_cast<unsigned long long>(st.seq_gaps),
                    static_cast<unsigned long long>(st.missed_samples));
                healthy = true;
            }
            ++samples;

            const vrsdk::State s = robot.states();
            if (samples % REPORT_EVERY == 0) {
                const double* p = s.kin().lin_pos;
                const vrsdk_state_stats_t st = robot.stats();
                std::printf(
                    "ok  seq=%llu t=%.2fs pos=(%.2f,%.2f,%.2f) received=%llu gaps=%llu "
                    "decode_errors=%llu\n",
                    static_cast<unsigned long long>(s.seq), s.elapsed, p[0], p[1], p[2],
                    static_cast<unsigned long long>(st.received),
                    static_cast<unsigned long long>(st.seq_gaps),
                    static_cast<unsigned long long>(st.decode_errors));
            }

            // Command as normal while the link is up.
            robot.set_mr_pwm(hold);
        }
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "unrecoverable: error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
