//! ex03 -- hello_image: read camera frames alongside states.
//!
//! Run with the sim in Play mode:
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex03_hello_image
//! ```
//!
//! **Every vrobot ships with `front_left` and `front_right` mounted at 720p
//! rgba8**, so an example never has to create a camera to get pixels.
//! `open_camera` subscribes to one of those streams and touches the simulator not
//! at all -- no mount, no unmount, nothing to leave behind. `mount_camera` adds a
//! camera of *your* choosing and is used in exactly one example, ex17, where the
//! mount pose and lens are the subject.
//!
//! Two things this example is really about:
//!
//! - **Images are a separate stream.** They arrive at the render rate, states at
//!   25 Hz, and no frame belongs to any state. `fresh()` returns `Some` only when
//!   a new frame has arrived since you last asked, so the image half of the loop
//!   runs once per frame while the state half runs every iteration.
//! - **Frames are top-down.** The wire is bottom-up (Unity's render order); the
//!   SDK flips while copying, so `frame.data` is row-major top-down and row 0 is
//!   the top of the picture. The sky-ness figures printed below are the check:
//!   outdoors, the top row is sky and the bottom row is ground.
//!
//! Nothing in this run mutates the simulator, which is why the loop can be
//! bounded by `FRAMES` or stopped with Ctrl-C and it makes no difference to the
//! scene either way.
//!
//! In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**;
//! `ex11_topic_discovery` lists what is actually publishing.

use vrobots_sdk::{Frame, RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const CAMERA: &str = "front_left"; // every vrobot ships front_left and front_right
const RESOLUTION: &str = "720p";
const FORMAT: &str = "rgba8"; // Unity's native readback -- four channels, NOT rgb8
const FRAMES: u64 = 120; // then exit
const HZ: f64 = 100.0;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    // open_camera SUBSCRIBES to a camera the robot already has, without mutating
    // the sim. The name, resolution and format must match the publisher exactly
    // -- on iceoryx2 those three strings are the stream identity -- so a mismatch
    // surfaces as VrError::Timeout (ex13 shows that path).
    let cam = robot.open_camera(CAMERA, RESOLUTION, FORMAT)?;
    println!("camera stream: {}", cam.service_name());

    let mut seen = 0u64;

    // ===== loop =====
    while seen < FRAMES {
        let s = robot.states();

        // Images are a separate stream with their own timestamps -- never assume
        // they match the state's. Compare t_ns explicitly when fusing.
        if let Some(frame) = cam.fresh() {
            // Some only if new since the last read
            seen += 1;
            println!(
                "Image {} t={:.3} size=({}x{}) seq={} lag_vs_state={:.1} ms",
                frame.camera_name,
                frame.elapsed,
                frame.width,
                frame.height,
                frame.seq,
                (s.t_ns - frame.t_ns) as f64 / 1e6
            );

            // The orientation check. Row 0 is the top of the image, so outdoors
            // it is sky -- and the way to recognise sky is that it is BLUE, not
            // that it is bright. Measured on this scene: the sky rows run
            // B - R = +98 and the pale ground runs -25, so brightness alone
            // reports the picture upside down (the desert floor is the brighter
            // of the two).
            let (top, bottom) = (blueness(&frame, 0), blueness(&frame, frame.height - 1));
            println!(
                "      sky-ness (B-R) top={top:+.0} bottom={bottom:+.0} ({}), fov_y={:.1} deg",
                match (top, bottom) {
                    _ if frame.bytes_per_pixel() == 1 => "mono8: no colour to judge by",
                    (t, b) if t > b + 20.0 => "top-down: sky above ground",
                    _ => "no sky/ground split here -- check where the camera points",
                },
                frame.intrinsics.fov_y.to_degrees()
            );
            // frame.data dereferences to a &[u8] of row-major, top-down RGBA:
            // feed it to OpenCV (ex34), or write it out as ex14 does.
        } else {
            println!("State t={:.3} (no new frame)", s.elapsed);
        }

        robot.rate(HZ);
    }

    // Nothing to unmount: this handle never created a camera. Dropping the stream
    // ends this subscription only -- front_left keeps rendering and publishing for
    // everyone else.
    let stats = cam.stats();
    println!(
        "{seen} frame(s), received={} decode_errors={} seq_gaps={}",
        stats.received, stats.decode_errors, stats.seq_gaps
    );
    Ok(())
}

/// Mean `blue - red` across one row: strongly positive for sky, negative for
/// most ground. `0.0` for mono8, which has no channels to compare.
fn blueness(frame: &Frame, row: u32) -> f64 {
    let bpp = frame.bytes_per_pixel() as usize;
    if bpp < 3 {
        return 0.0;
    }
    let Some(pixels) = frame.row(row) else {
        return 0.0;
    };
    let mut sum = 0.0;
    let mut count = 0.0;
    // Channel order is R,G,B(,A) -- the SDK normalises rows and stride, never
    // channel order, so this is the renderer's own layout.
    for pixel in pixels.chunks_exact(bpp) {
        sum += f64::from(pixel[2]) - f64::from(pixel[0]);
        count += 1.0;
    }
    if count == 0.0 { 0.0 } else { sum / count }
}
