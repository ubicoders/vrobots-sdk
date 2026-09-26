//! ex15 -- camera_formats: what the pixel format and the resolution cost you.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex15_camera_formats
//! ```
//!
//! Every vrobot ships `front_left` and `front_right` at **720p rgba8**, which is
//! Unity's native readback and the widest stream the SDK ever hands you. This
//! example opens one of them, reads the geometry off the frames, and prices the
//! alternatives against what it is actually receiving.
//!
//! Two rules decide what a stream costs, and they are not symmetrical:
//!
//! - **Resolution is one knob per robot**, shared by every camera on it. Changing
//!   it restarts *every* stream on that robot under a new service name. Asking
//!   for a second resolution while this handle holds a stream at another is
//!   refused with `VrError::InvalidArgument` rather than silently breaking the
//!   handle you already have.
//! - **Format is per camera**, and changing it renames that one stream -- the
//!   iceoryx2 service name embeds `<resolution>_<format>`, so
//!   `front_left/720p_rgba8` and `front_left/360p_mono8` are different services
//!   and nothing negotiates between them.
//!
//! That naming is why `open_camera` needs all three strings to match exactly, and
//! why a typo and a camera that does not exist are the same event: a timeout
//! (ex13).
//!
//! The prices, at the resolutions the sim offers:
//!
//! | resolution | mono8 (1 B) | rgb8 (3 B) | rgba8 (4 B) |
//! |---|---|---|---|
//! | 360p | 230 400 | 691 200 | 921 600 |
//! | 720p | 921 600 | 2 764 800 | **3 686 400** |
//! | 1080p | 2 073 600 | 6 220 800 | 8 294 400 |
//!
//! Sixteen times between the corners. At 60 fps that is 13 MB/s against
//! 200 MB/s -- memory bandwidth rather than network, since it all rides the same
//! shared memory, but it is the difference between free and not. Anything that
//! only needs luminance and geometry (optical flow, fiducials, horizon detection)
//! wants the cheap end.
//!
//! **Getting the cheap end means creating a camera**, because the scene's pair is
//! rgba8 and nothing reconfigures a camera you do not own. That is `mount_camera`,
//! and ex17 is the one example that uses it -- pass `"360p"` and `"mono8"` there
//! and the same code below reads one byte per pixel instead, with no
//! RGB-vs-BGR question at all. Note what the robot-wide rule means if you do: the
//! scene's pair is not unmounted, but it comes back at 360p under new names, and
//! `open_camera(.., "720p", ..)` here starts timing out until the sim restarts.
//!
//! In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const CAMERA: &str = "front_left"; // every vrobot ships front_left and front_right
const RESOLUTION: &str = "720p"; // 360p | 720p | 1080p -- robot-wide
const FORMAT: &str = "rgba8"; // mono8 | rgb8 | rgba8 -- per camera
const FRAMES: u64 = 60;
const HZ: f64 = 100.0;

/// Bytes per pixel, in the order the table above prints them.
const FORMATS: [(&str, usize); 3] = [("mono8", 1), ("rgb8", 3), ("rgba8", 4)];

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    // The service name IS <camera>/<resolution>_<format>. Nothing is mounted
    // here: this attaches to the stream the robot already publishes.
    let cam = robot.open_camera(CAMERA, RESOLUTION, FORMAT)?;
    println!("camera stream: {}", cam.service_name());
    let spec = cam.spec();
    println!(
        "spec: name={} resolution={} format={} ({} bytes/frame)",
        spec.name,
        spec.resolution,
        spec.format,
        spec.data_size()
    );

    let mut seen = 0u64;
    let mut total_bytes = 0u64;
    let (mut first_ns, mut last_ns) = (0i64, 0i64);

    // ===== loop =====
    while seen < FRAMES {
        if let Some(frame) = cam.fresh() {
            seen += 1;
            total_bytes += frame.data.len() as u64;
            last_ns = frame.t_ns;
            if seen == 1 {
                first_ns = frame.t_ns;
                let px = frame.width as usize * frame.height as usize;
                println!(
                    "\nframe {}x{} = {px} px, {} at {} B/px, step={} B/row, {} B/frame",
                    frame.width,
                    frame.height,
                    frame.format,
                    frame.bytes_per_pixel(),
                    frame.step,
                    frame.data.len()
                );
                // What the same picture would cost in the other two formats.
                for (name, bpp) in FORMATS {
                    let mark = if name == frame.format.as_str() {
                        "  <- this stream"
                    } else {
                        ""
                    };
                    println!("  {name:>5}: {:>9} B/frame{mark}", px * bpp);
                }
            } else if seen.is_multiple_of(20) {
                println!("frame {seen}: seq={} t={:.3}", frame.seq, frame.elapsed);
            }
        }
        robot.rate(HZ);
    }

    // What it actually cost, measured off the capture stamps rather than the
    // wall clock: this is the stream's own rate, not the loop's.
    let span_s = (last_ns - first_ns) as f64 / 1e9;
    if span_s > 0.0 {
        let fps = (seen - 1) as f64 / span_s;
        println!(
            "\n{seen} frames over {span_s:.2}s = {fps:.1} fps, {:.1} MB/s at {FORMAT}",
            total_bytes as f64 / span_s / 1e6
        );
        // mono8 is 1 B/px instead of 4, and 360p is a quarter of the pixels.
        let ratio = 1.0 / f64::from(cam.spec().format.bytes_per_pixel()) / 4.0;
        println!(
            "the same frames as 360p mono8 would be {:.1} MB/s (see ex17 to mount one)",
            total_bytes as f64 * ratio / span_s / 1e6
        );
    }

    // Nothing to unmount: this handle never created a camera, and the robot-wide
    // resolution knob was never touched, so the scene is exactly as it was.
    let stats = cam.stats();
    println!(
        "received={} decode_errors={} seq_gaps={}",
        stats.received, stats.decode_errors, stats.seq_gaps
    );
    Ok(())
}
