// ex23 -- skins: the one service that ever says no.
//
//     target/cpp-build/Release/ex23_skins
//
// `srv/skin` takes a catalog key and dresses the robot in it. The catalogs are
// **per robot type**, matched case-insensitively:
//
//   | type       | keys                                            |
//   |------------|-------------------------------------------------|
//   | multirotor | blue desert gold green mono pink snow white     |
//   | truck      | black blue camouflage gray red                  |
//
// Every other robot type ships no catalog at all, so every request to one is a
// no-op.
//
// IT IS THE ONLY SERVICE THAT REPORTS A REFUSAL -- AND ONLY ONE KIND
//
//   Skins are tier-gated inside the simulator. A tier refusal comes back as an
//   honest `ok = false` **with a reason**, which the SDK throws as a
//   `vrsdk::Error` with `code() == VRSDK_ERR_SERVICE` carrying the simulator's
//   own message. That is the single place in this entire API surface where a
//   service says no, and it is worth knowing precisely because of what happens
//   with everything else:
//
//   | request                            | reply     | what actually happened   |
//   |------------------------------------|-----------|--------------------------|
//   | a key your tier allows             | ok        | the skin changed         |
//   | any key, tier too low              | ok=false  | **thrown -- do not retry, the answer will not change** |
//   | `gold` on a TRUCK (a multirotor key)| ok       | nothing; logged in the sim |
//   | `chartreuse` (in no catalog at all) | ok       | nothing; logged in the sim |
//
//   The last two rows are the lesson. A wrong *key* is not an error, it is a
//   receipt for a request that was dropped -- so a typo looks exactly like
//   success, and the confirmation is the robot in front of you. Both are
//   demonstrated below, after the five real keys.
//
// ON A TRUCK A SKIN IS NOT ONLY COSMETIC
//
//   The wheel colliders travel with the skin prefab, so a swap **rebinds the
//   physics wheels**. This example keeps the truck rolling across every change
//   so that shows up on the wire: `actuator.measured[0..3]` are the four wheel
//   speeds (FL, FR, RL, RR, rad/s) and `[4]` is the steering servo. They must
//   keep turning across each swap -- a wheel channel that flatlines is a rebind
//   that did not take.
//
// An empty key is refused client-side: sim-side it is a payload-less read-back
// probe, not a skin.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <vrobots_sdk.hpp>

// Every key the truck catalog knows.
constexpr const char* TRUCK_SKINS[] = {"black", "blue", "camouflage", "gray", "red"};
// A perfectly valid key -- for a multirotor. On a truck it is acked and dropped.
constexpr const char* WRONG_TYPE_SKIN = "gold";
// In nobody's catalog.
constexpr const char* UNKNOWN_SKIN = "chartreuse";

constexpr double STEER_US = 1500.0;     // straight ahead
constexpr double THROTTLE_US = 1600.0;  // slow forward, so the wheels are always turning
constexpr double BRAKE_US = 1100.0;     // released (brake is bottom-anchored, ex05)
constexpr double HZ = 25.0;
constexpr int HOLD_SAMPLES = 30;  // ~1.2 s per skin

namespace {

/// The first `n` actuator channels, formatted so two rows compare by eye.
std::string channels(const vrsdk_actuator_t& a, std::uint32_t first, std::uint32_t last) {
    std::string out = "[";
    for (std::uint32_t i = first; i < last && i < a.measured_count; ++i) {
        char buffer[32];
        std::snprintf(buffer, sizeof buffer, "%s%+7.3f", i > first ? "," : "", a.measured[i]);
        out += buffer;
    }
    return out + "]";
}

/// Ask for one skin and keep driving through it. `false` means the tier refused
/// -- an answer, not a failure to talk to.
bool wear(vrsdk::VirtualRobot& robot, const std::string& skin) {
    try {
        robot.set_skin(skin);
        std::printf("set_skin(\"%s\") -> ok\n", skin.c_str());
    } catch (const vrsdk::Error& e) {
        // The sim's own words. This is the ONLY service that ever gets here, and
        // only for a tier refusal -- anything else is a real failure.
        if (e.code() != VRSDK_ERR_SERVICE) {
            throw;
        }
        std::printf("set_skin(\"%s\") -> REFUSED by the sim: %s\n", skin.c_str(), e.what());
        return false;
    }

    for (int i = 0; i < HOLD_SAMPLES; ++i) {
        robot.set_car(STEER_US, THROTTLE_US, BRAKE_US);
        if (i % 15 == 0) {
            const vrsdk::State s = robot.states();
            const double* v = s.kin().lin_vel;
            const double speed = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            // 0..3 are FL, FR, RL, RR in rad/s; they must keep turning across
            // the swap, because the colliders were just rebound.
            std::printf("    t=%6.2fs speed=%5.2f m/s wheels=%s steer_servo=%s\n", s.elapsed,
                        speed, channels(s.actuator(), 0, 4).c_str(),
                        channels(s.actuator(), 4, 5).c_str());
        }
        robot.rate(HZ);
    }
    return true;
}

}  // namespace

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot = vrsdk::VirtualRobot::create(vrsdk::RobotType::Truck);
        robot.connect();
        // The C++ surface has no topic-name builder; the shape is fixed by the
        // wire, so compose it here.
        std::printf("created sys_id=%u, service key = vrobots/%u/z/srv/skin\n\n", robot.sys_id(),
                    robot.sys_id());

        // ===== the real catalog =====
        for (const char* skin : TRUCK_SKINS) {
            if (!wear(robot, skin)) {
                // A tier refusal is final. Retrying it is the one thing this
                // service makes unambiguous, so stop rather than walk the list.
                std::printf("\nStopping: the tier gate does not open on a retry.\n");
                robot.remove();
                return 0;
            }
        }

        // ===== the two that look like success and are not =====
        std::printf("\n-- keys that are acked `ok` and dropped inside the simulator --\n");
        wear(robot, WRONG_TYPE_SKIN);  // a multirotor key, on a truck
        wear(robot, UNKNOWN_SKIN);     // no catalog has it
        std::printf(
            "Both returned without throwing. The truck is still wearing \"%s\" -- the ack was a "
            "receipt for a request the robot then refused with a log line no client can see.\n",
            TRUCK_SKINS[sizeof TRUCK_SKINS / sizeof TRUCK_SKINS[0] - 1]);

        // ===== and the one the SDK will not even send =====
        try {
            robot.set_skin("   ");
            std::printf("\nUNEXPECTED: an empty key was accepted\n");
        } catch (const vrsdk::Error& e) {
            std::printf("\nempty key -> [%d] %s\n", e.code(), e.what());
        }

        robot.remove();
        std::printf("\ndeleted sys_id=%u\n", robot.sys_id());
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
