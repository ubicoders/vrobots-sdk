// ex20 -- logging_tour: where the SDK's diagnostics come from.
//
//     target/cpp-build/Release/ex20_logging_tour
//
// The SDK reports through **two channels, and they mean different things**:
//
//   * **Thrown errors** are things *you* did or asked for that could not be
//     done: a pulse width outside the band, a camera that does not exist, a
//     service that said no. They are `vrsdk::Error` exceptions carrying a
//     stable `code()`, and they never appear only in a log.
//   * **Log events** are things the *SDK* did on your behalf: opening a
//     session, waiting for the first sample, retrying a service, dropping a
//     malformed payload, reconfiguring a camera that was already mounted (which
//     ends its old stream). **Nothing in the SDK waits, retries or drops
//     silently**, and this is where that shows up.
//     Ignore it and a hang has no explanation.
//
// C++ has no logging framework to plug into, so the SDK does the only portable
// thing: it hands each event to a **C function pointer you register**, with a
// level, a target (the Rust module that emitted it) and a message. What happens
// next is entirely yours -- printf, spdlog, syslog, a ring buffer.
//
// Three rules for the handler, all consequences of where it is called from:
//
//   1. **It is called from SDK threads** -- the zenoh runtime and each camera's
//      reader -- possibly concurrently. It must be thread-safe.
//   2. **It must not block for long**, because it runs on the thread that is
//      trying to deliver your data.
//   3. **It must not call back into the SDK.** No `states()` from inside a log
//      handler.
//
// Registration is process-wide, and `nullptr` unregisters. `set_log_level` sets
// the floor (default Info); `zenoh` and `iceoryx2` stay at Warn regardless,
// because zenoh at Debug is a firehose that would bury the SDK's own events.
//
// Note the trampoline in the header: the handler is registered through an
// `extern "C"` shim rather than a lambda, because a capture-less lambda's
// function-pointer conversion has C++ language linkage, and handing that to a C
// callback slot is a mismatch every compiler accepts and none promises to.
//
// In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

#include <atomic>
#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr int SAMPLES = 10;

// Counters rather than a container: the handler runs on SDK threads, so
// anything it touches has to be safe without a lock the SDK knows nothing about.
static std::atomic<int> g_counts[6] = {};

/// The handler. Thread-safe, fast, and it never calls back into the SDK.
static void on_log(vrsdk::LogLevel level, const char* target, const char* message) {
    const int index = static_cast<int>(level);
    if (index >= 0 && index < 6) {
        g_counts[index].fetch_add(1, std::memory_order_relaxed);
    }
    std::printf("   [%-5s %s] %s\n", vrsdk::to_string(level), target, message);
}

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();

        // Register BEFORE connecting: connect is the noisiest and most
        // diagnostic moment, and a hang with no log is the hardest thing to
        // debug.
        vrsdk::set_log_callback(on_log);
        vrsdk::set_log_level(vrsdk::LogLevel::Debug);
        std::printf("log callback registered at level %s\n\n",
                    vrsdk::to_string(vrsdk::LogLevel::Debug));

        // ===== the noisy moment =====
        std::printf("-- connect: watch the DEBUG lines for the session, the probe and the first "
                    "sample\n");
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();

        // ===== channel 1: errors are thrown =====
        std::printf("\n-- an error is thrown, not logged:\n");
        try {
            // 0.7 is a normalised throttle, not a pulse width. The SDK refuses
            // it before publishing rather than clamping, because a clamped 0.7
            // would look like a valid idle command.
            robot.set_mr_pwm({0.7, 0.7, 0.7, 0.7});
            std::printf("   unexpected: 0.7 us was accepted\n");
        } catch (const vrsdk::Error& e) {
            std::printf("   [%d] %s -- %s\n", e.code(), e.name(), e.what());
        }

        // ===== channel 2: events for what the SDK does =====
        std::printf("\n-- %d states at the sim's own rate (quieter now):\n", SAMPLES);
        vrsdk::set_log_level(vrsdk::LogLevel::Warn);  // turn the volume back down
        for (int i = 0; i < SAMPLES; ++i) {
            robot.wait_new_state(0.5);
        }
        const vrsdk::State s = robot.states();
        std::printf("   seq=%llu t=%.3f (no log lines above: Warn hides the routine ones)\n",
                    static_cast<unsigned long long>(s.seq), s.elapsed);

        // ===== the third channel: counted, never thrown =====
        // A malformed payload is logged as a warning, counted in stats(), and
        // stored in last_error() -- but never thrown from states(), because one
        // bad frame must not end a flight.
        const vrsdk_state_stats_t st = robot.stats();
        const std::optional<vrsdk::Error> err = robot.last_error();
        std::printf("\n-- counted rather than thrown: received=%llu decode_errors=%llu ",
                    static_cast<unsigned long long>(st.received),
                    static_cast<unsigned long long>(st.decode_errors));
        std::printf("last_error=%s\n", err ? err->what() : "none");

        std::printf("\nevents seen by the handler: trace=%d debug=%d info=%d warn=%d error=%d\n",
                    g_counts[0].load(), g_counts[1].load(), g_counts[2].load(),
                    g_counts[3].load(), g_counts[4].load());

        // Unregister before the handler's storage goes away. Registration is
        // process-wide, so a dangling function pointer would outlive main.
        vrsdk::set_log_callback(nullptr);
        std::printf("callback unregistered; the SDK falls silent.\n");
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
