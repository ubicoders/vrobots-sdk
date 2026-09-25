// ex11 -- topic_discovery: what is on the wire right now, from code.
//
//     target/cpp-build/Release/ex11_topic_discovery
//
// The first thing to run when nothing seems to be happening, and the library
// half of `vrobots topic list`. No robot, no connect: discovery answers "is the
// sim publishing at all, and under which ids?" before you have a handle to ask
// with.
//
// The two transports answer that question in completely different ways, and the
// `observed` flag is where that difference surfaces:
//
//   * **zenoh has no registry.** Listing means *listening* for WINDOW_S, so a
//     topic that publishes nothing during the window does not appear at all --
//     and `hz`, `samples` and `bytes` are real measurements. A short window on a
//     slow topic is the classic false negative: `vrobots/*/z/frames` publishes
//     at 1 Hz, so a 0.5 s window loses it.
//   * **iceoryx2 has one.** Camera streams come from *reading* it, so they
//     appear instantly and unmeasured -- all three counters are 0. `live` is
//     what matters there: false marks a stale record whose owning process died
//     (`sb topic prune` clears those). And because it is shared memory, the
//     iceoryx2 half only ever sees **this host** -- a remote sim lists states
//     and services but no cameras, which is not a discovery failure.
//
// An empty list is a legitimate answer, not an error: the sim is not in Play
// mode, is on another machine (pass a `vrsdk_connect_options_t` with a router
// endpoint), or the window was too short for zenoh's discovery.

#include <cstdio>
#include <map>
#include <optional>
#include <string>
#include <vector>
#include <vrobots_sdk.hpp>

constexpr double WINDOW_S = 1.5;

int main() {
    try {
        vrsdk::check_version();

        std::printf("listening for %.1fs ...\n", WINDOW_S);
        const std::vector<vrsdk::TopicInfo> topics = vrsdk::list_topics(WINDOW_S);

        if (topics.empty()) {
            std::printf(
                "no vrobots topics. The sim is not in Play mode, is on another host, or %.1fs "
                "was too short for zenoh discovery.\n",
                WINDOW_S);
            return 0;
        }

        std::printf("\n%-4s %7s %9s  topic\n", "wire", "Hz", "bytes");
        for (const vrsdk::TopicInfo& t : topics) {
            // `observed` decides whether the numbers mean anything at all.
            if (t.observed) {
                std::printf("[%s] %7.1f %9llu  %s\n", t.transport, t.hz,
                            static_cast<unsigned long long>(t.bytes), t.key.c_str());
            } else {
                std::printf("[%s] %7s %9s  %s\n", t.transport, t.live ? "-" : "stale", "-",
                            t.key.c_str());
            }
        }

        // The reason to do this in code rather than in the CLI: the result is
        // data. Grouping it by sys_id is how a program answers "which robots
        // exist, and does the one I want have a camera?".
        std::map<std::optional<std::uint32_t>, std::vector<std::string>> by_robot;
        for (const vrsdk::TopicInfo& t : topics) {
            by_robot[t.sys_id].push_back(t.key);
        }

        std::printf("\nby robot:\n");
        for (const auto& [sys_id, keys] : by_robot) {
            if (sys_id) {
                std::printf("  sys_id %u: %zu topic(s)\n", *sys_id, keys.size());
            } else {
                // `manager` and `scene` sit where an id would, so they can never
                // collide with one -- and they parse as "no id".
                std::printf("  swarm-wide (manager/scene): %zu topic(s)\n", keys.size());
            }
            for (const std::string& key : keys) {
                std::printf("      %s\n", key.c_str());
            }
        }
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
