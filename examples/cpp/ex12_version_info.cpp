// ex12 -- version_info: what this build speaks, and how well the link is working.
//
//     target/cpp-build/Release/ex12_version_info
//
// Everything here is diagnostics -- the four things to print in a bug report,
// and the reason each one exists.
//
// **1. Version identity.** A schema or IPC version mismatch does not present as
// an error. It presents as garbage field values, or as topics that look absent.
// iceoryx2 in particular compares major.minor.patch on every shared-memory
// open, so a version one patch off **does not error** -- it silently delivers
// nothing, which reads exactly like "the sim isn't publishing". So the pins
// have to be printable from the binary you are actually running.
//
// C++ has one more version to check than the other surfaces: `check_version()`
// asserts that this header and the linked library are the same release. The
// snapshot structs are shared between them **by layout**, so a mismatched pair
// reads every field after the first difference at the wrong offset, with
// nothing to signal it. That is why every example here calls it first.
//
// **2. Subscriber statistics.** `received`, `decode_errors`, `seq_gaps`,
// `missed_samples` -- counted continuously, never fatal. A decode error does
// not tear the session down and does not throw: one malformed payload must not
// end a flight. `seq_gaps` is the number the sim's own publisher can prove.
//
// **3. `last_error()`.** The error that was counted instead of thrown. Empty
// means nothing has failed to decode; non-empty beside a non-zero
// `decode_errors` is the actual reason, and it is almost always schema drift.
//
// The state snapshot also carries the sim's own `schema_version`. Comparing it
// with ours is the single most useful check in this file.

#include <chrono>
#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr int SAMPLES = 50;          // ~2 s at the 25 Hz state rate
constexpr double HZ = 25.0;

int main() {
    try {
        // ===== what this build is =====
        vrsdk::check_version();  // header vs library -- the C++-only check
        const vrsdk::VersionInfo v = vrsdk::version_info();
        std::printf("vrobots_sdk %s\n", v.sdk_version.c_str());
        std::printf("  vrobots_msgs  %s (schema_version %u)\n", v.msgs_commit.c_str(),
                    v.schema_version);
        std::printf("  flatbuffers   %s\n", v.flatbuffers.c_str());
        std::printf("  zenoh         %s\n", v.zenoh.c_str());
        std::printf("  iceoryx2      %s\n", v.iceoryx2.c_str());
        std::printf("  src_id        %u\n", v.src_id);
        std::printf("  header        %s (must equal the library above)\n", VROBOTS_SDK_VERSION);

        // ===== what the other end is =====
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();
        const vrsdk::State first = robot.states();
        std::printf(
            "\nsim says: schema_version=%u (ours %u), frame=\"%s\", its header src_id=%u "
            "(ours %u)\n",
            first.raw.schema_version, v.schema_version, first.coord_frame_id.c_str(),
            first.raw.src_id, v.src_id);
        if (first.raw.schema_version != v.schema_version) {
            std::printf(
                "  MISMATCH -- fields may decode as garbage. Rebuild the SDK against the sim's "
                "vrobots_msgs commit.\n");
        }

        // ===== how well it is arriving =====
        std::printf("\nwatching for %d loop iterations at %.0f Hz ...\n", SAMPLES, HZ);
        const auto started = std::chrono::steady_clock::now();
        for (int i = 0; i < SAMPLES; ++i) {
            robot.rate(HZ);
        }
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

        const vrsdk_state_stats_t st = robot.stats();
        const vrsdk::State last = robot.states();
        std::printf(
            "\nstats after %.1f s: received=%llu decode_errors=%llu seq_gaps=%llu "
            "missed_samples=%llu last_seq=%llu\n",
            elapsed, static_cast<unsigned long long>(st.received),
            static_cast<unsigned long long>(st.decode_errors),
            static_cast<unsigned long long>(st.seq_gaps),
            static_cast<unsigned long long>(st.missed_samples),
            static_cast<unsigned long long>(st.last_seq));
        std::printf("  effective rate %.1f Hz over the window; sim clock advanced %.2f s\n",
                    static_cast<double>(st.received) / (elapsed > 0.0 ? elapsed : 1.0),
                    last.elapsed - first.elapsed);

        // Counted, not thrown. This is where a decode failure went.
        if (const std::optional<vrsdk::Error> e = robot.last_error()) {
            std::printf("  last_error: [%d] %s\n", e->code(), e->what());
        } else {
            std::printf("  last_error: none -- every payload decoded\n");
        }
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
