// ex13 -- open_camera: attach to a camera that already exists, changing nothing.
//
//     target/cpp-build/Release/ex13_open_camera
//
// `open_camera` is what every camera example here uses, and this file is the
// one that shows it failing. It opens the iceoryx2 subscriber and touches the
// simulator not at all. Two processes can open the same stream; neither
// disturbs the other, and neither has to own the camera. `mount_camera` (ex17)
// is the other side: it **creates** a camera on the robot, which is a mutation
// and needs a name nobody is using.
//
// The price is that **the name, resolution and format must match the publisher
// exactly**, because on iceoryx2 those three strings *are* the stream identity
// and there is no type negotiation behind them. A mismatch is not an error at
// the far end -- it is simply a service that does not exist -- so it throws
// `VRSDK_ERR_TIMEOUT` after `camera_timeout` (5 s by default). That is the loud
// failure this example is built to show you: a typo in FORMAT and a camera that
// was never mounted are the same event.
//
// The constants below are the default every vrobot ships with: `front_left` and
// `front_right` at 720p rgba8. `vrobots topic list` prints the streams that
// actually exist right now; the [i] lines are exactly these names. The one
// thing that moves them is a **resolution** change, which is robot-wide: after a
// program mounts a camera at 360p (ex17 is the only one here that can) the pair
// is still mounted but publishing as `front_left/360p_rgba8`, so open that name
// instead or restart the sim.
//
// In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr const char* CAMERA = "front_left";  // every vrobot ships this pair
constexpr const char* RESOLUTION = "720p";
constexpr const char* FORMAT = "rgba8";  // Unity's native readback -- note: NOT rgb8
constexpr std::uint64_t FRAMES = 60;
constexpr double TIMEOUT_S = 0.5;

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();

        vrsdk::CameraStream cam;
        try {
            cam = robot.open_camera(CAMERA, RESOLUTION, FORMAT);
        } catch (const vrsdk::Error& e) {
            if (e.code() != VRSDK_ERR_TIMEOUT) {
                throw;
            }
            // The whole point of the example: nothing is mounted under that
            // exact identity, and there is no way for the SDK to tell you which
            // of the three strings is wrong.
            std::printf("no publisher for %s/%s_%s: %s\n", CAMERA, RESOLUTION, FORMAT, e.what());
            std::printf(
                "run `vrobots topic list` -- the [i] lines are the streams that do exist. A "
                "camera another process mounted then unmounted is gone.\n");
            return 0;
        }
        std::printf("attached to %s (nothing in the sim changed)\n", cam.service_name().c_str());
        std::printf("camera name on the robot: %s\n", cam.name().c_str());

        // ===== loop =====
        // Frame-paced, not clock-paced: wait_new_frame blocks until the next
        // one, so the body runs exactly once per rendered frame. Same idea as
        // ex09's wait_new_state, and the same rule about timeouts.
        std::uint64_t seen = 0;
        while (seen < FRAMES) {
            try {
                cam.wait_new_frame(TIMEOUT_S);
            } catch (const vrsdk::Error& e) {
                if (e.code() != VRSDK_ERR_TIMEOUT) {
                    throw;
                }
                std::printf("no frame in %.1fs -- the camera stopped, or the sim is paused\n",
                            TIMEOUT_S);
                continue;
            }
            const std::optional<vrsdk::Frame> frame = cam.fresh();
            if (!frame) {
                continue;
            }
            ++seen;
            if (seen % 10 == 1) {
                const double* m = frame->info.mount.position;
                std::printf("frame %llu: seq=%llu %ux%u %zu bytes, mount=(%+.2f,%+.2f,%+.2f) m\n",
                            static_cast<unsigned long long>(seen),
                            static_cast<unsigned long long>(frame->seq()), frame->width(),
                            frame->height(), frame->data.size(), m[0], m[1], m[2]);
            }
        }

        // There is nothing to clean up, and that is the lesson. This handle
        // never mounted anything, so it cannot unmount anything either -- the
        // SDK refuses locally rather than sending a remove for a camera that
        // belongs to the scene.
        try {
            robot.unmount_camera(CAMERA);
            std::printf("unexpected: unmounted a camera this handle never mounted\n");
        } catch (const vrsdk::Error& e) {
            std::printf("\nunmount_camera refused, correctly: [%d] %s\n", e.code(), e.what());
        }

        // Destroying the stream ends this subscription only. The camera keeps
        // rendering and publishing for everyone else.
        const vrsdk_camera_stats_t st = cam.stats();
        std::printf(
            "read %llu frame(s): received=%llu decode_errors=%llu seq_gaps=%llu "
            "missed_frames=%llu\n",
            static_cast<unsigned long long>(seen), static_cast<unsigned long long>(st.received),
            static_cast<unsigned long long>(st.decode_errors),
            static_cast<unsigned long long>(st.seq_gaps),
            static_cast<unsigned long long>(st.missed_frames));
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
