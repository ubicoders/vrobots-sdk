// ex17 -- camera_pose: point the camera somewhere else, and read the pose back.
//
//     target/cpp-build/Release/ex17_camera_pose
//
// **This is the one example that adds a camera.** Every other camera example
// opens `front_left` or `front_right`, the pair every vrobot already ships at
// 720p rgba8, because reading images never needs more than that and opening a
// stream mutates nothing. `mount_camera` is for the case those defaults cannot
// serve: a camera somewhere else on the robot, pointing somewhere else, through
// a different lens -- or in a different format, since the scene's pair is rgba8
// (ex15).
//
// `mount_camera(name, resolution, format)` uses the defaults: at the robot
// origin, looking along its forward axis, 600 px focal length. The fourth
// argument is a `vrsdk_camera_options_t`, filled from
// `vrsdk_camera_options_default` and then overridden field by field -- start
// from the defaults rather than zeroing the struct, because a zeroed one asks
// for a zero focal length and zero clip planes.
//
//   * `mount_position`  -- metres from the robot origin, **in your header
//     frame** ("unity" by default: left-handed, X right, Y up, Z forward).
//   * `mount_euler_deg` -- Unity-local euler angles in **degrees**. A 180 in
//     the roll slot turns the camera upside down, which is what the sky-ness
//     figures below detect.
//   * `fx` / `fy`       -- focal length in pixels at the current resolution.
//     The lens is specified as intrinsics, not as a field of view; the
//     principal point is fixed at the image centre and the render is an ideal
//     pinhole with no distortion. `fx != fy` renders anamorphic. A *smaller*
//     focal length is a *wider* angle: fov_y = 2 * atan(height / (2 * fy)).
//   * `near_clip` / `far_clip` -- metres.
//
// **Intrinsics and the mount pose ride with every frame** (`frame.info.
// intrinsics`, `frame.info.mount`), so a gimballed or re-mounted camera can
// never desync from its images -- there is no separate camera-info topic to
// join by timestamp.
//
// **The read-back is not the numbers you sent, and that is the point.** You
// express the mount in *your* frame; the robot converts it into *its* frame and
// reports it back tagged with that frame. Live, against this scene:
//
//     requested  [0.10, 0.20, 0.30] m  "unity"
//     read back  (-0.20, +0.30, -0.10) m  "frd"    -- permuted and signed
//
// Components move and change sign. Never assume your triple survives intact:
// `frame.info.mount` is the authority on where the camera actually is, and
// comparing it against your request is the only way to confirm what the sim
// did.
//
// **Timing.** The service acks immediately, but the camera has to be rebuilt
// and re-rendered before it publishes. For a *new* camera the stream does not
// exist until that is done, so its very first frame already carries the
// requested pose. Re-mounting an existing name is the case to watch: the
// format/pose change ends the old stream and starts a new one, and anything
// still holding the old handle is reading a dead service. The loop below prints
// a line whenever the pose differs from the previous frame's, so any settling
// would be visible.
//
// In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

#include <cmath>
#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr const char* CAMERA = "tilt";
constexpr const char* RESOLUTION = "720p";
constexpr const char* FORMAT = "rgb8";
constexpr double MOUNT_POSITION[3] = {0.10, 0.20, 0.30};   // metres, OUR frame ("unity")
constexpr double MOUNT_EULER_DEG[3] = {0.0, 0.0, 180.0};   // upside down
constexpr double FOCAL_PX = 400.0;  // wider than the 600 px default
constexpr std::uint64_t FRAMES = 40;
constexpr double HZ = 100.0;
constexpr double RAD2DEG = 57.2957795130823;

/// Mean `blue - red` across one row: positive for sky, negative for ground.
static double blueness(const vrsdk::Frame& frame, std::uint32_t row) {
    const std::uint32_t bpp = frame.bytes_per_pixel();
    const std::uint8_t* pixels = frame.row(row);
    if (bpp < 3 || pixels == nullptr || frame.width() == 0) {
        return 0.0;
    }
    double sum = 0.0;
    for (std::uint32_t x = 0; x < frame.width(); ++x) {
        sum += static_cast<double>(pixels[x * bpp + 2]) - static_cast<double>(pixels[x * bpp]);
    }
    return sum / frame.width();
}

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();

        // Start from the documented defaults, then override.
        vrsdk_camera_options_t options;
        vrsdk_camera_options_default(&options);
        for (int i = 0; i < 3; ++i) {
            options.mount_position[i] = MOUNT_POSITION[i];
            options.mount_euler_deg[i] = MOUNT_EULER_DEG[i];
        }
        options.fx = FOCAL_PX;
        options.fy = FOCAL_PX;
        options.near_clip = 0.2;
        options.far_clip = 500.0;
        std::printf("requested: position=(%.2f,%.2f,%.2f) m, euler=(%.1f,%.1f,%.1f) deg, fx=fy=%.0f\n",
                    MOUNT_POSITION[0], MOUNT_POSITION[1], MOUNT_POSITION[2], MOUNT_EULER_DEG[0],
                    MOUNT_EULER_DEG[1], MOUNT_EULER_DEG[2], FOCAL_PX);

        vrsdk::CameraStream cam = robot.mount_camera(CAMERA, RESOLUTION, FORMAT, &options);
        std::printf("camera stream: %s\n", cam.service_name().c_str());

        std::uint64_t seen = 0;
        double settled[3] = {1e9, 1e9, 1e9};

        // ===== loop =====
        while (seen < FRAMES) {
            if (const std::optional<vrsdk::Frame> frame = cam.fresh()) {
                ++seen;
                const vrsdk_mount_pose_t& m = frame->info.mount;
                const vrsdk_intrinsics_t& in = frame->info.intrinsics;

                // Degrees on the way in, radians on the way out: the wire is SI.
                const double euler_deg[3] = {m.euler_rad[0] * RAD2DEG, m.euler_rad[1] * RAD2DEG,
                                             m.euler_rad[2] * RAD2DEG};
                const bool changed = euler_deg[0] != settled[0] || euler_deg[1] != settled[1] ||
                                     euler_deg[2] != settled[2];
                if (changed || seen % 20 == 0) {
                    std::printf(
                        "frame %llu seq=%llu: mount pos=(%+.2f,%+.2f,%+.2f) m  "
                        "euler=(%+.1f,%+.1f,%+.1f) deg  frame=\"%.*s\"\n",
                        static_cast<unsigned long long>(seen),
                        static_cast<unsigned long long>(frame->seq()), m.position[0],
                        m.position[1], m.position[2], euler_deg[0], euler_deg[1], euler_deg[2],
                        static_cast<int>(sizeof m.coord_frame_id - 1), m.coord_frame_id);
                    const double default_fov =
                        2.0 * std::atan(frame->height() / 2.0 / 600.0) * RAD2DEG;
                    std::printf(
                        "         lens fx=%.0f fy=%.0f -> fov_y=%.1f deg (600 px would be %.1f), "
                        "clip %.2f..%.0f m\n",
                        in.fx, in.fy, in.fov_y * RAD2DEG, default_fov, in.near_clip, in.far_clip);
                    settled[0] = euler_deg[0];
                    settled[1] = euler_deg[1];
                    settled[2] = euler_deg[2];
                }

                // The pose read-back says what the sim was told. This says what
                // it rendered: with the camera rolled 180, the sky lands in the
                // BOTTOM rows of a buffer whose row 0 is still, always, the top.
                if (seen == FRAMES) {
                    const double top = blueness(*frame, 0);
                    const double bottom = blueness(*frame, frame->height() - 1);
                    const char* verdict =
                        bottom > top + 20.0
                            ? "sky is at the BOTTOM: the camera really is upside down"
                            : (top > bottom + 20.0 ? "sky is at the top: the roll did not take "
                                                     "effect"
                                                   : "no sky/ground split -- check where the "
                                                     "camera points");
                    std::printf("\nsky-ness (B-R) top=%+.0f bottom=%+.0f -> %s\n", top, bottom,
                                verdict);
                }
            }
            robot.rate(HZ);
        }

        // Leave the sim as we found it. Mounting is the half of the API that
        // has a cleanup step, and this is it: unmount_camera removes exactly the
        // name it is given, so `front_left` and `front_right` are untouched
        // throughout. (RESOLUTION matches theirs, so the robot-wide knob never
        // moved either.)
        robot.unmount_camera(CAMERA);
        std::printf("unmounted %s\n", CAMERA);
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
