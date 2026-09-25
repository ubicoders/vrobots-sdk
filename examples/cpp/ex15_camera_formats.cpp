// ex15 -- camera_formats: what the pixel format and the resolution cost you.
//
//     target/cpp-build/Release/ex15_camera_formats
//
// Every vrobot ships `front_left` and `front_right` at **720p rgba8**, which is
// Unity's native readback and the widest stream the SDK ever hands you. This
// example opens one of them, reads the geometry off the frames, and prices the
// alternatives against what it is actually receiving.
//
// Two rules decide what a stream costs, and they are not symmetrical:
//
//   * **Resolution is one knob per robot**, shared by every camera on it.
//     Changing it restarts *every* stream on that robot under a new service
//     name. Asking for a second resolution while this handle holds a stream at
//     another throws `VRSDK_ERR_INVALID_ARGUMENT` rather than silently breaking
//     the handle you already have.
//   * **Format is per camera**, and changing it renames that one stream -- the
//     iceoryx2 service name embeds `<resolution>_<format>`, so
//     `front_left/720p_rgba8` and `front_left/360p_mono8` are different
//     services and nothing negotiates between them.
//
// That naming is why `open_camera` needs all three strings to match exactly,
// and why a typo and a camera that does not exist are the same event: a timeout
// (ex13).
//
// The prices, at the resolutions the sim offers:
//
//   | resolution | mono8 (1 B) | rgb8 (3 B) | rgba8 (4 B)   |
//   |------------|-------------|------------|---------------|
//   | 360p       | 230 400     | 691 200    | 921 600       |
//   | 720p       | 921 600     | 2 764 800  | **3 686 400** |
//   | 1080p      | 2 073 600   | 6 220 800  | 8 294 400     |
//
// Sixteen times between the corners. At 60 fps that is 13 MB/s against
// 200 MB/s -- memory bandwidth rather than network, since it all rides the same
// shared memory, but it is the difference between free and not. Anything that
// only needs luminance and geometry (optical flow, fiducials, horizon
// detection) wants the cheap end.
//
// **Getting the cheap end means creating a camera**, because the scene's pair
// is rgba8 and nothing reconfigures a camera you do not own. That is
// `mount_camera`, and ex17 is the one example that uses it -- pass "360p" and
// "mono8" there and the same code below reads one byte per pixel instead, with
// no RGB-vs-BGR question at all (`frame.row(y)[x]` is the intensity). Note what
// the robot-wide rule means if you do: the scene's pair is not unmounted, but it
// comes back at 360p under new names, and open_camera(.., "720p", ..) here
// starts timing out until the sim restarts.
//
// In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

#include <array>
#include <cstdio>
#include <string>
#include <utility>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;           // the multirotor in the test scene
constexpr const char* CAMERA = "front_left";  // every vrobot ships front_left and front_right
constexpr const char* RESOLUTION = "720p";    // 360p | 720p | 1080p -- robot-wide
constexpr const char* FORMAT = "rgba8";       // mono8 | rgb8 | rgba8 -- per camera
constexpr std::uint64_t FRAMES = 60;
constexpr double HZ = 100.0;

/// Bytes per pixel, in the order the table above prints them.
constexpr std::array<std::pair<const char*, std::size_t>, 3> FORMATS = {
    std::pair{"mono8", std::size_t{1}}, std::pair{"rgb8", std::size_t{3}},
    std::pair{"rgba8", std::size_t{4}}};

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();

        // The service name IS <camera>/<resolution>_<format>. Nothing is mounted
        // here: this attaches to the stream the robot already publishes.
        vrsdk::CameraStream cam = robot.open_camera(CAMERA, RESOLUTION, FORMAT);
        std::printf("camera stream: %s\n", cam.service_name().c_str());

        std::uint64_t seen = 0;
        std::uint64_t total_bytes = 0;
        std::int64_t first_ns = 0;
        std::int64_t last_ns = 0;

        // ===== loop =====
        while (seen < FRAMES) {
            if (const std::optional<vrsdk::Frame> frame = cam.fresh()) {
                ++seen;
                total_bytes += frame->data.size();
                last_ns = frame->t_ns();
                if (seen == 1) {
                    first_ns = frame->t_ns();
                    const std::size_t px =
                        static_cast<std::size_t>(frame->width()) * frame->height();
                    std::printf(
                        "\nframe %ux%u = %zu px, %s at %u B/px, step=%u B/row, %zu B/frame\n",
                        frame->width(), frame->height(), px, FORMAT, frame->bytes_per_pixel(),
                        frame->step(), frame->data.size());
                    // What the same picture would cost in the other two formats.
                    for (const auto& [name, bpp] : FORMATS) {
                        const char* mark = std::string(name) == FORMAT ? "  <- this stream" : "";
                        std::printf("  %5s: %9zu B/frame%s\n", name, px * bpp, mark);
                    }
                } else if (seen % 20 == 0) {
                    std::printf("frame %llu: seq=%llu t=%.3f\n",
                                static_cast<unsigned long long>(seen),
                                static_cast<unsigned long long>(frame->seq()), frame->elapsed());
                }
            }
            robot.rate(HZ);
        }

        // What it actually cost, measured off the capture stamps rather than the
        // wall clock: this is the stream's own rate, not the loop's.
        const double span_s = static_cast<double>(last_ns - first_ns) / 1e9;
        if (span_s > 0.0) {
            const double fps = static_cast<double>(seen - 1) / span_s;
            std::printf("\n%llu frames over %.2fs = %.1f fps, %.1f MB/s at %s\n",
                        static_cast<unsigned long long>(seen), span_s, fps,
                        static_cast<double>(total_bytes) / span_s / 1e6, FORMAT);
            // mono8 is 1 B/px instead of 4, and 360p is a quarter of the pixels.
            const double ratio = 1.0 / 4.0 / 4.0;
            std::printf(
                "the same frames as 360p mono8 would be %.1f MB/s (see ex17 to mount one)\n",
                static_cast<double>(total_bytes) * ratio / span_s / 1e6);
        }

        // Nothing to unmount: this handle never created a camera, and the
        // robot-wide resolution knob was never touched, so the scene is exactly
        // as it was.
        const vrsdk_camera_stats_t st = cam.stats();
        std::printf("received=%llu decode_errors=%llu seq_gaps=%llu\n",
                    static_cast<unsigned long long>(st.received),
                    static_cast<unsigned long long>(st.decode_errors),
                    static_cast<unsigned long long>(st.seq_gaps));
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
