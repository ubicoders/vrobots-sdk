//! ex17 -- camera_pose: point the camera somewhere else, and read the pose back.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex17_camera_pose
//! ```
//!
//! **This is the one example that adds a camera.** Every other camera example
//! opens `front_left` or `front_right`, the pair every vrobot already ships at
//! 720p rgba8, because reading images never needs more than that and opening a
//! stream mutates nothing. `mount_camera` is for the case those defaults cannot
//! serve: a camera somewhere else on the robot, pointing somewhere else, through
//! a different lens -- or in a different format, since the scene's pair is rgba8
//! (ex15).
//!
//! `mount_camera` uses the defaults: at the robot origin, looking along its
//! forward axis, 600 px focal length. `mount_camera_with` takes a
//! `CameraOptions` and configures the mount and the lens:
//!
//! - `mount_position` -- metres from the robot origin, **in your header frame**
//!   (`"unity"` by default: left-handed, X right, Y up, Z forward).
//! - `mount_euler_deg` -- Unity-local euler angles in **degrees**. A 180 in the
//!   roll slot turns the camera upside down, which is what the sky-ness figures
//!   below detect.
//! - `fx` / `fy` -- focal length in pixels at the current resolution. The lens is
//!   specified as intrinsics, not as a field of view; the principal point is
//!   fixed at the image centre and the render is an ideal pinhole with no
//!   distortion. `fx != fy` renders anamorphic. A *smaller* focal length is a
//!   *wider* angle: `fov_y = 2 * atan(height / (2 * fy))`.
//! - `near_clip` / `far_clip` -- metres.
//!
//! **Intrinsics and the mount pose ride with every frame** (`frame.intrinsics`,
//! `frame.mount`), so a gimballed or re-mounted camera can never desync from its
//! images -- there is no separate camera-info topic to join by timestamp.
//!
//! **The read-back is not the numbers you sent, and that is the point.** You
//! express the mount in *your* frame; the robot converts it into *its* frame and
//! reports it back tagged with that frame. Live, against this scene:
//!
//! ```text
//! requested  [0.10, 0.20, 0.30] m  "unity"
//! read back  (-0.20, +0.30, -0.10) m  "frd"    -- permuted and signed
//! ```
//!
//! Components move and change sign. Never assume your triple survives intact:
//! `frame.mount` is the authority on where the camera actually is, and comparing
//! it against your request is the only way to confirm what the sim did.
//!
//! **Timing.** The service acks immediately, but the camera has to be rebuilt and
//! re-rendered before it publishes. For a *new* camera the stream does not exist
//! until that is done, so its very first frame already carries the requested
//! pose. Re-mounting an existing name is the case to watch: the format/pose
//! change ends the old stream and starts a new one (the SDK logs "reconfiguring
//! an already-mounted camera"), and anything still holding the old handle is
//! reading a dead service. The loop below prints a line whenever the pose
//! differs from the previous frame's, so any settling would be visible.
//!
//! In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

use vrobots_sdk::{CameraOptions, RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const CAMERA: &str = "tilt";
const RESOLUTION: &str = "720p";
const FORMAT: &str = "rgb8";
const MOUNT_POSITION: [f64; 3] = [0.10, 0.20, 0.30]; // metres, in OUR frame ("unity")
const MOUNT_EULER_DEG: [f64; 3] = [0.0, 0.0, 180.0]; // upside down
const FOCAL_PX: f64 = 400.0; // wider than the 600 px default
const FRAMES: u64 = 40;
const HZ: f64 = 100.0;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    let options = CameraOptions::default()
        .with_mount_position(MOUNT_POSITION)
        .with_mount_euler_deg(MOUNT_EULER_DEG)
        .with_focal_length(FOCAL_PX)
        .with_clip(0.2, 500.0);
    println!("requested: {options:?}");

    let cam = robot.mount_camera_with(CAMERA, RESOLUTION, FORMAT, &options)?;
    println!("camera stream: {}", cam.service_name());

    let mut seen = 0u64;
    let mut settled: Option<[f64; 3]> = None;

    // ===== loop =====
    while seen < FRAMES {
        if let Some(frame) = cam.fresh() {
            seen += 1;

            // Degrees on the way in, radians on the way out: the wire is SI.
            let euler_deg = [
                frame.mount.euler_rad[0].to_degrees(),
                frame.mount.euler_rad[1].to_degrees(),
                frame.mount.euler_rad[2].to_degrees(),
            ];
            let changed = settled != Some(euler_deg);
            if changed || seen.is_multiple_of(20) {
                println!(
                    "frame {seen} seq={}: mount pos=({:+.2},{:+.2},{:+.2}) m  \
                     euler=({:+.1},{:+.1},{:+.1}) deg  frame={:?} axes={}",
                    frame.seq,
                    frame.mount.position[0],
                    frame.mount.position[1],
                    frame.mount.position[2],
                    euler_deg[0],
                    euler_deg[1],
                    euler_deg[2],
                    frame.mount.coord_frame_id,
                    frame.mount.axis_convention.0
                );
                println!(
                    "         lens fx={:.0} fy={:.0} -> fov_y={:.1} deg (600 px would be {:.1}), \
                     clip {:.2}..{:.0} m",
                    frame.intrinsics.fx,
                    frame.intrinsics.fy,
                    frame.intrinsics.fov_y.to_degrees(),
                    2.0 * ((f64::from(frame.height) / 2.0) / 600.0)
                        .atan()
                        .to_degrees(),
                    frame.intrinsics.near_clip,
                    frame.intrinsics.far_clip
                );
                settled = Some(euler_deg);
            }

            // The pose read-back says what the sim was told. This says what it
            // rendered: with the camera rolled 180, the sky lands in the BOTTOM
            // rows of a buffer whose row 0 is still, always, the top.
            if seen == FRAMES {
                let (top, bottom) = (blueness(&frame, 0), blueness(&frame, frame.height - 1));
                println!(
                    "\nsky-ness (B-R) top={top:+.0} bottom={bottom:+.0} -> {}",
                    if bottom > top + 20.0 {
                        "sky is at the BOTTOM: the camera really is upside down"
                    } else if top > bottom + 20.0 {
                        "sky is at the top: the roll did not take effect"
                    } else {
                        "no sky/ground split -- check where the camera points"
                    }
                );
            }
        }
        robot.rate(HZ);
    }

    // Leave the sim as we found it. Mounting is the half of the API that has a
    // cleanup step, and this is it: unmount_camera removes exactly the name it is
    // given, so `front_left` and `front_right` are untouched throughout.
    // (RESOLUTION matches theirs, so the robot-wide knob never moved either.)
    robot.unmount_camera(CAMERA)?;
    println!("unmounted {CAMERA}");
    Ok(())
}

/// Mean `blue - red` across one row: strongly positive for sky, negative for
/// most ground.
fn blueness(frame: &vrobots_sdk::Frame, row: u32) -> f64 {
    let bpp = frame.bytes_per_pixel() as usize;
    let Some(pixels) = frame.row(row) else {
        return 0.0;
    };
    if bpp < 3 || pixels.is_empty() {
        return 0.0;
    }
    let sum: f64 = pixels
        .chunks_exact(bpp)
        .map(|p| f64::from(p[2]) - f64::from(p[0]))
        .sum();
    sum / (pixels.len() / bpp) as f64
}
