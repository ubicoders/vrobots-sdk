// ex14 -- camera_save: grab one frame, write it to disk, exit.
//
//     target/cpp-build/Release/ex14_camera_save
//
// The shortest complete camera program there is: open, wait for one frame,
// write it, exit. No loop to speak of -- which makes it the right place to show
// what a `vrsdk::Frame` actually holds.
//
// It uses the camera the robot already has: **every vrobot ships with
// `front_left` and `front_right` at 720p rgba8**, and `open_camera` attaches to
// one of them without changing anything in the simulator. Nothing is mounted, so
// there is nothing to unmount and no cleanup step to forget.
//
// `frame.data` is a `std::vector<std::uint8_t>` of `height * step` bytes,
// row-major **top-down**, tightly packed (`step == width * bytes_per_pixel`,
// never padded). It is **owned**: the pixels are copied out of the SDK's frame
// at construction, so the value outlives the stream and the robot. A 720p RGBA
// frame is 3.7 MB, so move it rather than copy it when passing it around.
//
// The SDK normalises geometry and nothing else: the wire is bottom-up and the
// SDK flips while copying, but **channels are the renderer's own order and are
// never swapped**. `rgba8` is R,G,B,A. Anything expecting BGR -- OpenCV, most
// notably -- converts at its own call site, once, in the code that needs it.
//
// The output is a binary PPM (P6): a nine-byte header and then the pixels,
// which is the whole format. It needs no image library, and every viewer reads
// it. The Python twin of this example writes a PNG through OpenCV instead,
// because in Python that dependency is normally already there.
//
// In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

#include <cstdio>
#include <string>
#include <vector>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr const char* CAMERA = "front_left";  // every vrobot ships front_left and front_right
constexpr const char* RESOLUTION = "720p";
constexpr const char* FORMAT = "rgba8";  // Unity's native readback -- four channels, NOT rgb8
constexpr const char* OUTPUT = "frame.ppm";
constexpr double TIMEOUT_S = 2.0;

/// Write the frame as a binary PPM (P6) -- the simplest image format there is.
/// Mono8 is expanded to grey RGB; rgba8 drops alpha.
static bool write_ppm(const vrsdk::Frame& frame, const std::string& path) {
    std::FILE* out = std::fopen(path.c_str(), "wb");
    if (out == nullptr) {
        return false;
    }
    std::fprintf(out, "P6\n%u %u\n255\n", frame.width(), frame.height());
    const std::uint32_t bpp = frame.bytes_per_pixel();
    std::vector<std::uint8_t> rgb;
    rgb.reserve(static_cast<std::size_t>(frame.width()) * frame.height() * 3);
    // Row-major and top-down already, so a straight walk is the right order.
    for (std::size_t i = 0; i + bpp <= frame.data.size(); i += bpp) {
        if (bpp == 1) {
            rgb.insert(rgb.end(), {frame.data[i], frame.data[i], frame.data[i]});
        } else {
            rgb.insert(rgb.end(), {frame.data[i], frame.data[i + 1], frame.data[i + 2]});
        }
    }
    const bool ok = std::fwrite(rgb.data(), 1, rgb.size(), out) == rgb.size();
    std::fclose(out);
    return ok;
}

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();
        // open_camera SUBSCRIBES to a camera the robot already has; it creates
        // nothing and so leaves nothing behind.
        vrsdk::CameraStream cam = robot.open_camera(CAMERA, RESOLUTION, FORMAT);
        std::printf("attached to %s\n", cam.service_name().c_str());

        // ===== the one frame =====
        // open_camera already waited for the stream to exist, but the next
        // frame still has to be rendered. Block for it rather than polling.
        cam.wait_new_frame(TIMEOUT_S);
        const std::optional<vrsdk::Frame> frame = cam.fresh();
        if (!frame) {
            std::fprintf(stderr, "wait_new_frame returned but no frame was waiting\n");
            return 1;
        }

        std::printf("frame seq=%llu t=%.3fs %ux%u (%u B/px, step=%u, %zu bytes)\n",
                    static_cast<unsigned long long>(frame->seq()), frame->elapsed(),
                    frame->width(), frame->height(), frame->bytes_per_pixel(), frame->step(),
                    frame->data.size());
        const vrsdk_intrinsics_t& i = frame->info.intrinsics;
        std::printf(
            "intrinsics fx=%.1f fy=%.1f cx=%.1f cy=%.1f fov_y=%.1f deg  clip %.2f..%.0f m\n", i.fx,
            i.fy, i.cx, i.cy, i.fov_y * 57.2957795, i.near_clip, i.far_clip);

        if (write_ppm(*frame, OUTPUT)) {
            std::printf("wrote %s\n", OUTPUT);
        } else {
            std::fprintf(stderr, "could not write %s\n", OUTPUT);
        }

        // Nothing to unmount: this handle never created a camera. front_left
        // keeps publishing for everyone else exactly as it did before the run.
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
