// ex16 -- two_cameras: two streams, one loop, independent freshness.
//
//     target/cpp-build/Release/ex16_two_cameras
//
// The stereo pair every vrobot already has: **`front_left` and `front_right`,
// both at 720p rgba8**. Each `open_camera` call returns its own `CameraStream`,
// and each stream has its own reader thread, its own sequence numbers and its
// own freshness -- so the two `fresh()` calls below are genuinely independent.
// There is no combined "wait for both", by design: the cameras are separate
// iceoryx2 services and they render on their own schedules. If you need them
// paired, pair them yourself on `t_ns`, which is what the skew figure below
// does.
//
// Neither call changes anything in the simulator, so two of these programs can
// run at once on the same pair without either noticing, and neither has any
// cleanup to do. That is the ordinary case for reading images; ex17 is the one
// example that adds a camera of its own.
//
// The constraint to know before you do add one: **resolution is one knob for
// the whole robot**, shared by every camera on it -- which is why both streams
// here are RESOLUTION and could not be anything else. Asking for a second
// resolution while this handle holds a stream at another throws
// `VRSDK_ERR_INVALID_ARGUMENT`: the change would restart every other stream
// under a new service name and silently break the handles you already hold, so
// the SDK will not do it behind your back. Format is per camera; only
// resolution is shared.
//
// `CameraStream` is move-only, which is what keeps ownership unambiguous when a
// program holds several: there is exactly one owner of each reader thread.
//
// In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

#include <array>
#include <cstdio>
#include <string>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr const char* LEFT = "front_left";  // the pair every vrobot ships with
constexpr const char* RIGHT = "front_right";
constexpr const char* RESOLUTION = "720p";  // the same for both -- robot-wide
constexpr const char* FORMAT = "rgba8";     // Unity's native readback, NOT rgb8
constexpr std::uint64_t FRAMES = 60;  // per camera
constexpr double HZ = 100.0;

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();

        // Two subscriptions, no mutation: both cameras are already on the robot.
        vrsdk::CameraStream left = robot.open_camera(LEFT, RESOLUTION, FORMAT);
        vrsdk::CameraStream right = robot.open_camera(RIGHT, RESOLUTION, FORMAT);
        std::printf("left : %s\n", left.service_name().c_str());
        std::printf("right: %s\n", right.service_name().c_str());
        std::printf("mounted by this handle: %zu  <- neither is ours\n",
                    robot.mounted_cameras().size());

        std::uint64_t n_left = 0;
        std::uint64_t n_right = 0;
        std::int64_t last_left_ns = 0;

        // ===== loop =====
        while (n_left < FRAMES || n_right < FRAMES) {
            // Two consumers, each draining its own stream. Neither call can
            // consume the other's frame.
            if (const std::optional<vrsdk::Frame> f = left.fresh()) {
                ++n_left;
                last_left_ns = f->t_ns();
                if (n_left % 20 == 1) {
                    std::printf("L frame %llu: seq=%llu t=%.3f\n",
                                static_cast<unsigned long long>(n_left),
                                static_cast<unsigned long long>(f->seq()), f->elapsed());
                }
            }
            if (const std::optional<vrsdk::Frame> f = right.fresh()) {
                ++n_right;
                if (n_right % 20 == 1) {
                    // The only honest way to relate two frames: subtract their
                    // capture stamps. The SDK never pairs them for you.
                    const double skew_ms =
                        last_left_ns == 0 ? 0.0
                                          : static_cast<double>(f->t_ns() - last_left_ns) / 1e6;
                    std::printf("R frame %llu: seq=%llu t=%.3f  skew_vs_last_left=%+.1f ms\n",
                                static_cast<unsigned long long>(n_right),
                                static_cast<unsigned long long>(f->seq()), f->elapsed(), skew_ms);
                }
            }
            robot.rate(HZ);
        }

        const vrsdk_camera_stats_t ls = left.stats();
        const vrsdk_camera_stats_t rs = right.stats();
        std::printf("\nleft : %llu read, received=%llu seq_gaps=%llu missed=%llu\n",
                    static_cast<unsigned long long>(n_left),
                    static_cast<unsigned long long>(ls.received),
                    static_cast<unsigned long long>(ls.seq_gaps),
                    static_cast<unsigned long long>(ls.missed_frames));
        std::printf("right: %llu read, received=%llu seq_gaps=%llu missed=%llu\n",
                    static_cast<unsigned long long>(n_right),
                    static_cast<unsigned long long>(rs.received),
                    static_cast<unsigned long long>(rs.seq_gaps),
                    static_cast<unsigned long long>(rs.missed_frames));

        // No cleanup, for either stream: this handle created neither camera.
        // Both keep rendering and publishing for everyone else after exit.
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
