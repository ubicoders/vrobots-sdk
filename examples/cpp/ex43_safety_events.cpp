// ex43 -- safety_events: every robot's incidents, one at a time, and the silence between them.
//
//     target/cpp-build/Release/ex43_safety_events
//
// A robot that comes close to a person, touches one or hits one publishes a
// `SafetyEvent` on its own `vrobots/{id}/z/event/safety`, and an arm's
// protective stop publishes one when a link meets an object or another robot.
// This example listens to all of them at once, on `vrobots/*/z/event/safety`,
// with no robot handle at all: `subscribe_all_safety_events()` opens a zenoh
// session of its own and closes it when the stream is destroyed.
//
// SILENCE IS THE NORMAL STATE
//
//   Every other stream in the SDK is periodic -- a state at 25 Hz, a scan at 20
//   -- and a quiet one is a broken one. This one is the opposite: ONE SAMPLE PER
//   INCIDENT, NOTHING IN BETWEEN. A minute without an event is a minute in which
//   nobody got hurt, so `next_timeout()` coming back with nothing is an answer,
//   not an error, and there is no rate to watch.
//
//   The flip side is that silence cannot tell "nothing happened" from "nobody is
//   publishing". So this example first asks which robots are on the wire at all
//   (ex11's `list_topics`), and refuses to listen to a simulator that is not
//   there.
//
// EVENTS QUEUE
//
//   A state is superseded by the next one, so the state stream keeps only the
//   latest. An incident is superseded by nothing -- a contact that turns into a
//   strike is two records -- so the stream QUEUES every event and hands each one
//   out once, oldest first: `next_timeout()` waits for one, `try_next()` never
//   waits, and `while (auto e = events.try_next())` takes all of them, the read
//   for a control loop. A robot handle has a stream of its own too,
//   `subscribe_safety_events()`.
//
// WHAT AN EVENT SAYS
//
//   | field                         | meaning                                                    |
//   |-------------------------------|------------------------------------------------------------|
//   | `sys_id`, `robot_name`, `seq` | whose incident it is -- never who was hit -- and that      |
//   |                               | robot's event counter                                      |
//   | `tier`                        | `near_miss` (close, moving in, never touched), `contact`,  |
//   |                               | `strike` (fast, or a large impulse)                        |
//   | `other_kind`, `other_name`    | `person`, `robot` or `object`, and its name in the scene   |
//   | `closing_speed`               | the robot's speed toward the other party, m/s, + closing   |
//   | `impulse`                     | N*s; 0 for a near miss, and for an arm link, which is      |
//   |                               | posed, not pushed                                          |
//   | `robot_at_fault`              | the robot was the one moving in                            |
//   | `position`                    | where, a WORLD position in the robot's own frame           |
//   |                               | (`coord_frame_id`)                                         |
//
//   Only incidents after the subscriber is live arrive; zenoh replays nothing,
//   and a fresh session needs a moment of discovery before the first one can.
//   On the Community tier only the focused robot is on the wire, and another
//   robot's incident is dropped inside the simulator, not queued for later.
//
// MAKING ONE HAPPEN
//
//   In the warehouse scene people walk between the racks. Drive the forklift
//   toward one with the keyboard (arrows, focused in the simulator) while this
//   runs, or jog the rail arm into a box stack -- its protective stop reports
//   the link that met it.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include <vrobots_sdk.hpp>

constexpr double WINDOW_S = 1.5;  // ex11's listening window
constexpr double LISTEN_S = 30.0;
constexpr double QUIET_NOTE_S = 5.0;  // say so after this much silence

namespace {

/// Seconds since `start`, on the steady clock.
double seconds_since(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

/// Robot ids as a list, `[0, 1, 2]`.
std::string ids_of(const std::vector<std::uint32_t>& ids) {
    std::string out = "[";
    for (std::size_t i = 0; i < ids.size(); ++i) {
        out += (i ? ", " : "") + std::to_string(ids[i]);
    }
    return out + "]";
}

/// The sys_ids publishing a state topic: the robots on the wire right now.
std::vector<std::uint32_t> robots_on_the_wire() {
    const std::vector<vrsdk::TopicInfo> topics = vrsdk::list_topics(WINDOW_S);
    std::vector<std::uint32_t> robots;
    for (const vrsdk::TopicInfo& t : topics) {
        if (!t.sys_id) {
            continue;
        }
        const std::uint32_t sys_id = *t.sys_id;
        if (t.key == "vrobots/" + std::to_string(sys_id) + "/z/state" &&
            std::find(robots.begin(), robots.end(), sys_id) == robots.end()) {
            robots.push_back(sys_id);
        }
    }
    return robots;
}

/// One event, as it arrived.
void print_event(double t, const vrsdk::SafetyEvent& e) {
    std::printf("t=%5.1fs  robot %u '%s' event #%llu: %s with %s '%s'\n", t, e.sys_id,
                e.robot_name.c_str(), static_cast<unsigned long long>(e.seq),
                vrsdk::to_string(e.tier), vrsdk::to_string(e.other_kind), e.other_name.c_str());
    std::printf(
        "          closing %+.2f m/s, impulse %.1f N*s, robot at fault: %s, at (%+.2f, %+.2f, "
        "%+.2f) m in \"%s\"\n",
        e.closing_speed, e.impulse, e.robot_at_fault ? "true" : "false", e.position[0],
        e.position[1], e.position[2], e.coord_frame_id.c_str());
}

}  // namespace

int main() {
    try {
        // ===== setup: is anybody there? =====
        vrsdk::check_version();
        std::printf("listening for %gs for the robots on the wire ...\n", WINDOW_S);
        std::vector<std::uint32_t> robots;
        try {
            robots = robots_on_the_wire();
        } catch (const vrsdk::Error& e) {
            std::fprintf(stderr, "could not look: [%d] %s\n", e.code(), e.what());
            return 1;
        }
        if (robots.empty()) {
            std::fprintf(stderr,
                         "no robot is publishing, so no event could ever arrive. Is the simulator "
                         "running, with a scene in Play mode? `vrobots topic list` shows what is "
                         "on the wire.\n");
            return 1;
        }
        std::printf("robots on the wire: %s -- an event can come from any of them\n",
                    ids_of(robots).c_str());

        // ===== subscribe, swarm-wide =====
        vrsdk::SafetyEventStream events;
        try {
            events = vrsdk::subscribe_all_safety_events();
        } catch (const vrsdk::Error& e) {
            std::fprintf(stderr, "could not subscribe: [%d] %s\n", e.code(), e.what());
            return 1;
        }
        std::printf("subscribed to %s for %gs\n\n", events.key().c_str(), LISTEN_S);

        // ===== listen =====
        const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        int near_misses = 0;
        int contacts = 0;
        int strikes = 0;
        int unspecified = 0;
        while (seconds_since(start) < LISTEN_S) {
            // Wait for the next event, but no longer than QUIET_NOTE_S, so that a
            // quiet stretch is reported as what it is.
            const double wait =
                std::fmin(QUIET_NOTE_S, std::fmax(0.0, LISTEN_S - seconds_since(start)));
            const std::optional<vrsdk::SafetyEvent> event = events.next_timeout(wait);
            if (!event) {
                std::printf("t=%5.1fs  quiet: no incident, which is the normal state\n",
                            seconds_since(start));
                continue;
            }
            print_event(seconds_since(start), *event);
            switch (event->tier) {
                case vrsdk::SafetyTier::NearMiss:
                    ++near_misses;
                    break;
                case vrsdk::SafetyTier::Contact:
                    ++contacts;
                    break;
                case vrsdk::SafetyTier::Strike:
                    ++strikes;
                    break;
                // Unspecified, or a tier newer than this SDK: unknown severity, not harmless.
                default:
                    ++unspecified;
                    break;
            }
        }

        // ===== what the stream counted =====
        // On an aperiodic topic a lost event looks exactly like no event, so
        // these counters are the only way to know the record is complete.
        const vrsdk_safety_stats_t stats = events.stats();
        std::printf("\n%llu event(s) in %gs: %d near miss, %d contact, %d strike, %d unspecified\n",
                    static_cast<unsigned long long>(stats.received), LISTEN_S, near_misses,
                    contacts, strikes, unspecified);
        std::printf(
            "dropped %llu (queue full, nobody reading), missed %llu in %llu seq gap(s) (lost on "
            "the way), decode errors %llu, still queued %zu\n",
            static_cast<unsigned long long>(stats.dropped),
            static_cast<unsigned long long>(stats.missed_events),
            static_cast<unsigned long long>(stats.seq_gaps),
            static_cast<unsigned long long>(stats.decode_errors), events.pending());
        // Destroying the stream undeclares the subscriber and closes its own session.
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
