// ex25 -- frames: change the axes the robot reports in, and read the scene's.
//
//     target/cpp-build/Release/ex25_frames
//
// ex18 showed two robots in one scene disagreeing about which way is up -- the
// truck publishing "fru", the multirotor "frd". This is the service that decides
// that, and the query that reads the level above it.
//
// **Frames are presentation, never physics.** The truck below does not move one
// millimetre differently after the change; the numbers describing it are
// permuted, and one of them changes sign. Registered ids:
//
//   | id      | axes                          | handed |
//   |---------|-------------------------------|--------|
//   | `unity` | +x right, +y up, +z forward   | left   |
//   | `frd`   | +x forward, +y right, +z down | right  |
//   | `fru`   | +x forward, +y right, +z up   | left   |
//   | `cv`    | +x right, +y down, +z forward | right  |
//
// plus whatever the scene registered at runtime -- which is why `coord_frame_id`
// (a string) is authoritative and `axis_convention` (an enum tag) is the
// convenience beside it.
//
// THREE LEVELS, MOST SPECIFIC WINS
//
//     device override   (set_frames, per device)      <- most specific
//     robot override    (set_frames, robot_frame_id)
//     robot default     (truck: fru, multirotor: frd, globalhawk: frd -- regardless of the scene)
//     scene frame       (scene_frame(); every launch starts at fru)
//
// `set_frames` takes two independent halves. An empty `robot_frame_id` leaves
// the robot's level alone; `vrsdk::INHERIT_FRAME` as an id **clears** that
// level's override so the one below wins again -- which is a different thing
// from "", meaning "do not touch".
//
// TWO NAMES FOR THE SAME DEVICE
//
//   The device the frames service matches is `gps`; the block it moves is called
//   `gnss` in the state message. Use the `vrsdk::device` constants rather than a
//   literal -- an unrecognised name is skipped ENTRY BY ENTRY, with a log line
//   inside the simulator and an `ok` ack, so a typo is invisible from out here.
//   The call below includes one deliberate miss (`camera/front`, on a truck that
//   has no such camera) to show that the other entries still apply.
//
// WHERE THE CONFIRMATION IS
//
//   Two places, and neither is the ack: the `coord_frame_id` stamped on every
//   subsequent state header, and the robot's `z/frames` topic, which republishes
//   the full definition of each frame -- basis matrix included -- on change and
//   then at 1 Hz.

#include <cstdio>
#include <string>
#include <vector>

#include <vrobots_sdk.hpp>

constexpr double HZ = 25.0;
constexpr int SETTLE_SAMPLES = 12;  // ~0.5 s: the change lands on the next physics step

namespace {

/// Let the change land: services apply in phase 0 of the next physics step.
void settle(vrsdk::VirtualRobot& robot) {
    for (int i = 0; i < SETTLE_SAMPLES; ++i) {
        robot.rate(HZ);
    }
}

/// The robot's frame, one device's frame, and a vector that shows the difference.
///
/// The per-device `coord_frame_id` fields are fixed NUL-terminated C buffers, so
/// they print with `%s` directly -- only the robot-level one is a `std::string`.
void report(vrsdk::VirtualRobot& robot, const char* label) {
    const vrsdk::State s = robot.states();
    const double* p = s.kin().lin_pos;
    std::printf("%s robot=%-6s pos=(%+7.3f,%+7.3f,%+7.3f)  gyro frame=\"%s\"  gnss frame=\"%s\"\n",
                label, s.coord_frame_id.c_str(), p[0], p[1], p[2],
                s.sensors().gyroscope.coord_frame_id, s.sensors().gnss.coord_frame_id);
}

/// Print a client-side refusal: code first, then the sim behaviour it prevents.
template <typename Call> void show_refusal(const char* what, Call call) {
    try {
        call();
        std::printf("  %-32s UNEXPECTED: accepted\n", what);
    } catch (const vrsdk::Error& e) {
        std::printf("  %-32s [%d] %s\n", what, e.code(), e.what());
    }
}

}  // namespace

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot = vrsdk::VirtualRobot::create(vrsdk::RobotType::Truck);
        robot.connect();
        const std::uint32_t sys_id = robot.sys_id();
        // The C++ surface has no topic-name builder; the shapes are fixed by the
        // wire, so compose them here.
        std::printf(
            "created sys_id=%u\n  service : vrobots/%u/z/srv/frames\n  scene   : "
            "vrobots/scene/z/srv/frame\n  z/frames: vrobots/%u/z/frames\n\n",
            sys_id, sys_id, sys_id);

        // ===== level 1: the scene =====
        // A payload-less GET that reads and changes nothing. Scene scope, not
        // robot scope -- the answer is the same for every robot loaded, and this
        // robot's session is used only because that is where the wire is.
        const vrsdk::SceneFrame scene = robot.scene_frame();
        std::printf("scene frame: \"%s\" (axis_convention %d)\n", scene.coord_frame_id.c_str(),
                    scene.axis_convention);

        // ===== what the truck reports today =====
        report(robot, "default   ");

        // ===== a robot override plus device overrides =====
        std::printf("\nset_frames(\"frd\", [gyroscope->fru, gps->inherit, camera/front->cv])\n");
        robot.set_frames("frd",
                         {
                             // Keep the gyro reading the way it was, while the
                             // robot moves to frd.
                             {vrsdk::device::GYROSCOPE, "fru"},
                             // Clear any override this device had: fall back to
                             // the robot's level.
                             {vrsdk::device::GPS, vrsdk::INHERIT_FRAME},
                             // Deliberate miss: this truck has no camera called
                             // "front". The entry is skipped with a log line and
                             // an `ok` ack; the others still apply.
                             {vrsdk::device::camera("front"), "cv"},
                         });
        settle(robot);
        report(robot, "overridden");
        std::printf(
            "  ^ same motion, different numbers: fru and frd differ only in the sign of the "
            "third component.\n");

        // ===== put it back =====
        std::printf("\nset_frames(INHERIT_FRAME, [gyroscope->inherit])\n");
        robot.set_frames(vrsdk::INHERIT_FRAME,
                         {{vrsdk::device::GYROSCOPE, vrsdk::INHERIT_FRAME}});
        settle(robot);
        report(robot, "cleared   ");
        std::printf(
            "  ^ back to the truck's own default. (Here that is fru and the scene is fru too, so "
            "this one run cannot tell you which level answered -- the robot default outranks the "
            "scene either way.)\n");

        // ===== what the SDK will not send =====
        std::printf("\n-- refused before anything reaches the wire --\n");
        show_refusal("nothing set", [&] { robot.set_frames(std::string(), {}); });
        show_refusal("an entry with an empty device", [&] { robot.set_frames({{"", "frd"}}); });
        show_refusal("an entry with an empty frame id",
                     [&] { robot.set_frames({{vrsdk::device::GYROSCOPE, ""}}); });
        std::printf(
            "(use INHERIT_FRAME to clear an override; \"\" would be skipped sim-side)\n");

        robot.remove();
        std::printf("\ndeleted sys_id=%u\n", sys_id);
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
