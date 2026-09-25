// ex03 -- hello_image: read camera frames alongside states.
//
//     target/cpp-build/Release/ex03_hello_image
//
// **Every vrobot ships with `front_left` and `front_right` mounted at 720p
// rgba8**, so an example never has to create a camera to get pixels.
// `open_camera` subscribes to one of those streams and touches the simulator not
// at all -- no mount, no unmount, nothing to leave behind. `mount_camera` adds a
// camera of *your* choosing and is used in exactly one example, ex17, where the
// mount pose and lens are the subject. `vrobots topic list` shows the streams
// that exist right now.
//
// Two things this example is really about:
//
//   * **Images are a separate stream.** They arrive at the render rate, states
//     at 25 Hz, and no frame belongs to any state. `fresh()` returns a value
//     only when a new frame has arrived since you last asked, so the image half
//     of the loop runs once per frame while the state half runs every
//     iteration. Compare `t_ns` explicitly when fusing.
//   * **Frames are top-down RGBA, not BGR.** The wire is bottom-up (Unity's
//     render order) and the SDK flips while copying, so row 0 is the top of the
//     picture -- but channels are the renderer's own order and are never
//     swapped. `cv::Mat` users need cv::cvtColor(..., cv::COLOR_RGBA2BGR).
//     The sky-ness figures printed below are the orientation check.
//
// Nothing in this run mutates the simulator, which is why the loop can be
// bounded by FRAMES or stopped with Ctrl-C and it makes no difference to the
// scene either way. ex14 saves a frame to disk.
//
// In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr const char* CAMERA = "front_left";  // every vrobot ships front_left and front_right
constexpr const char* RESOLUTION = "720p";
constexpr const char* FORMAT = "rgba8";  // Unity's native readback -- four channels, NOT rgb8
constexpr std::uint64_t FRAMES = 120;    // then exit
constexpr double HZ = 100.0;

/// Mean `blue - red` across one row: strongly positive for sky, negative for
/// most ground. 0 for mono8, which has no channels to compare.
static double blueness(const vrsdk::Frame& frame, std::uint32_t row) {
    const std::uint32_t bpp = frame.bytes_per_pixel();
    const std::uint8_t* pixels = frame.row(row);
    if (bpp < 3 || pixels == nullptr) {
        return 0.0;
    }
    double sum = 0.0;
    // Channel order is R,G,B(,A) -- the SDK normalises rows and stride, never
    // channel order, so this is the renderer's own layout.
    for (std::uint32_t x = 0; x < frame.width(); ++x) {
        sum += static_cast<double>(pixels[x * bpp + 2]) - static_cast<double>(pixels[x * bpp]);
    }
    return frame.width() > 0 ? sum / frame.width() : 0.0;
}

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();

        // open_camera SUBSCRIBES to a camera the robot already has, without
        // mutating the sim. The name, resolution and format must match the
        // publisher exactly -- on iceoryx2 those three strings are the stream
        // identity -- so a mismatch surfaces as a timeout (ex13 shows that path).
        vrsdk::CameraStream cam = robot.open_camera(CAMERA, RESOLUTION, FORMAT);
        std::printf("camera stream: %s\n", cam.service_name().c_str());

        std::uint64_t seen = 0;

        // ===== loop =====
        while (seen < FRAMES) {
            const vrsdk::State s = robot.states();

            // A value only if new since the last read.
            if (auto frame = cam.fresh()) {
                ++seen;
                std::printf("Image %s t=%.3f size=(%ux%u) seq=%llu lag_vs_state=%.1f ms\n",
                            frame->camera_name.c_str(), frame->elapsed(), frame->width(),
                            frame->height(),
                            static_cast<unsigned long long>(frame->seq()),
                            static_cast<double>(s.t_ns - frame->t_ns()) / 1e6);

                // The orientation check. Row 0 is the top of the image, so
                // outdoors it is sky -- and the way to recognise sky is that it
                // is BLUE, not that it is bright: the desert floor is brighter.
                const double top = blueness(*frame, 0);
                const double bottom = blueness(*frame, frame->height() - 1);
                const char* verdict = frame->bytes_per_pixel() == 1
                                          ? "mono8: no colour to judge by"
                                          : (top > bottom + 20.0
                                                 ? "top-down: sky above ground"
                                                 : "no sky/ground split here -- check where the "
                                                   "camera points");
                std::printf("      sky-ness (B-R) top=%+.0f bottom=%+.0f (%s), fov_y=%.1f deg\n",
                            top, bottom, verdict, frame->info.intrinsics.fov_y * 57.2957795);
            } else {
                std::printf("State t=%.3f (no new frame)\n", s.elapsed);
            }

            robot.rate(HZ);
        }

        // Nothing to unmount: this handle never created a camera. Letting the
        // stream go ends this subscription only -- front_left keeps rendering and
        // publishing for everyone else.
        const vrsdk_camera_stats_t st = cam.stats();
        std::printf("%llu frame(s), received=%llu decode_errors=%llu seq_gaps=%llu\n",
                    static_cast<unsigned long long>(seen),
                    static_cast<unsigned long long>(st.received),
                    static_cast<unsigned long long>(st.decode_errors),
                    static_cast<unsigned long long>(st.seq_gaps));
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
