// ex34 -- camera_view: show the robot's own camera in an OpenCV window.
//
//     target/cpp-build/Release/ex34_camera_view
//
// The one example that needs a library outside the SDK, which is why CMake only
// builds it when `find_package(OpenCV)` succeeds: nobody has to install OpenCV
// to build the other thirty-five.
//
// Like every camera example here it uses a camera the robot already has:
// **every vrobot ships with `front_left` and `front_right` mounted at 720p
// rgba8**, and `open_camera` attaches to one of those streams without changing
// anything in the simulator. There is no mount and no unmount -- the camera is
// not ours, so there is nothing to clean up. ex17 is the one example that adds a
// camera of its own.
//
// Everything else here is ex03 with a window bolted on, and the window is the
// whole lesson. **`frame.data` is RGBA, and OpenCV is BGR.** The SDK normalises
// geometry and nothing else: rows are already top-down and `step == width *
// bytes_per_pixel` with no padding, so a `cv::Mat` header maps straight onto the
// bytes with no copy at all -- but channels are the renderer's own order and are
// never swapped. The conversion happens once, here, at the call site that wants
// BGR.
//
// That Mat BORROWS the frame's pixels: it is valid exactly as long as `frame` is
// alive, and `cv::cvtColor` writes into its own destination, so nothing here
// outlives the loop iteration that made it.
//
// The loop is frame-paced rather than clock-paced: `wait_new_frame` blocks until
// the next render, so `imshow` runs exactly once per frame instead of redrawing
// one it has already shown. A timeout is a status, not a failure -- the
// simulator is paused, or the camera stopped -- so the body still pumps
// `cv::waitKey` to keep the window responsive and goes round again.
//
// FORMAT is `rgba8`, Unity's native readback, so the Mat is CV_8UC4 and the
// conversion is cv::COLOR_RGBA2BGR. rgb8 would be CV_8UC3 and cv::COLOR_RGB2BGR;
// mono8 would be CV_8UC1 and need no conversion at all (ex15).
//
// Quit with **q** or **Esc** in the window, or Ctrl-C -- either is safe now that
// this example leaves the simulator untouched.
//
// In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

#include <cstdio>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr const char* CAMERA = "front_left";  // every vrobot ships front_left and front_right
constexpr const char* RESOLUTION = "720p";
constexpr const char* FORMAT = "rgba8";  // four channels, so the Mat is CV_8UC4
constexpr const char* WINDOW = "vrobots camera";
constexpr double TIMEOUT_S = 0.5;

/// One millisecond of GUI pumping, which is also how a keypress is read.
/// `cv::imshow` alone draws nothing until `cv::waitKey` runs.
static bool quit_requested() {
    const int key = cv::waitKey(1) & 0xFF;
    return key == 'q' || key == 27;  // 27 is Esc
}

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();

        // open_camera SUBSCRIBES to a camera the robot already has and mutates
        // nothing. The three strings must match the publisher exactly -- on
        // iceoryx2 they *are* the stream identity -- so a mismatch is a timeout
        // rather than an error from the far end (ex13 shows that path).
        vrsdk::CameraStream cam = robot.open_camera(CAMERA, RESOLUTION, FORMAT);
        std::printf("showing %s -- press q or Esc to quit\n", cam.service_name().c_str());

        cv::namedWindow(WINDOW, cv::WINDOW_AUTOSIZE);

        // ===== loop =====
        // Frame-paced: wait_new_frame blocks until the next render, so imshow
        // runs once per frame rather than redrawing one it has already shown.
        std::uint64_t seen = 0;
        for (;;) {
            try {
                cam.wait_new_frame(TIMEOUT_S);
            } catch (const vrsdk::Error& e) {
                if (e.code() != VRSDK_ERR_TIMEOUT) {
                    throw;
                }
                // A status, not a failure: the sim is paused, or the camera
                // stopped. Still pump the GUI so the window stays responsive.
                if (quit_requested()) {
                    break;
                }
                continue;
            }

            const std::optional<vrsdk::Frame> frame = cam.fresh();
            if (!frame) {
                continue;
            }
            ++seen;

            // A header over the frame's own bytes -- no copy. Row-major,
            // top-down and tightly packed is exactly what cv::Mat wants.
            const cv::Mat rgba(static_cast<int>(frame->height()), static_cast<int>(frame->width()),
                               CV_8UC4, const_cast<std::uint8_t*>(frame->data.data()),
                               static_cast<std::size_t>(frame->step()));

            // The SDK never does this for you: RGBA is what Unity rendered, BGR
            // is what OpenCV displays.
            cv::Mat bgr;
            cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);

            cv::imshow(WINDOW, bgr);
            if (quit_requested()) {
                break;
            }
        }

        // ===== cleanup =====
        // Only ours: the window. Letting the stream go ends this subscription
        // and nothing else -- front_left keeps rendering for everyone.
        cv::destroyAllWindows();
        const vrsdk_camera_stats_t st = cam.stats();
        std::printf(
            "showed %llu frame(s), received=%llu decode_errors=%llu seq_gaps=%llu\n",
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
