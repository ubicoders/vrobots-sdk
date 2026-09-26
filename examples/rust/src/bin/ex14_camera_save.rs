//! ex14 -- camera_save: grab one frame, write it to disk, exit.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex14_camera_save
//! ```
//!
//! The shortest complete camera program there is: open, wait for one frame,
//! write it, exit. No loop to speak of -- which makes it the right place to show
//! what a `Frame` actually holds.
//!
//! It uses the camera the robot already has: **every vrobot ships with
//! `front_left` and `front_right` at 720p rgba8**, and `open_camera` attaches to
//! one of them without changing anything in the simulator. Nothing is mounted, so
//! there is nothing to unmount and no cleanup step to forget.
//!
//! `frame.data` is `height * step` bytes, row-major **top-down**, tightly packed
//! (`step == width * bytes_per_pixel`, never padded). The SDK normalises geometry
//! and nothing else: the wire is bottom-up and the SDK flips while copying, but
//! **channels are the renderer's own order and are never swapped**. `rgba8` is
//! R,G,B,A. Anything expecting BGR -- OpenCV, most notably -- converts at its own
//! call site, once, in the code that needs it.
//!
//! The output is a binary PPM (P6): a nine-byte header and then the pixels, which
//! is the whole format. It needs no image library, and every viewer reads it. The
//! Python twin of this example writes a PNG through OpenCV instead, because in
//! Python that dependency is normally already there.
//!
//! In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

use std::io::Write;
use std::time::Duration;

use vrobots_sdk::{Frame, RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const CAMERA: &str = "front_left"; // every vrobot ships front_left and front_right
const RESOLUTION: &str = "720p";
const FORMAT: &str = "rgba8"; // Unity's native readback -- four channels, NOT rgb8
const OUTPUT: &str = "frame.ppm";
const TIMEOUT: Duration = Duration::from_secs(2);

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;
    // open_camera SUBSCRIBES to a camera the robot already has; it creates
    // nothing and so leaves nothing behind.
    let cam = robot.open_camera(CAMERA, RESOLUTION, FORMAT)?;
    println!("attached to {}", cam.service_name());

    // ===== the one frame =====
    // open_camera already waited for the stream to exist, but the next frame
    // still has to be rendered. Block for it rather than polling.
    cam.wait_new_frame(TIMEOUT)?;
    let frame = cam
        .fresh()
        .expect("wait_new_frame returned Ok, so one is waiting");

    println!(
        "frame seq={} t={:.3}s {}x{} {} ({} B/px, step={}, {} bytes)",
        frame.seq,
        frame.elapsed,
        frame.width,
        frame.height,
        frame.format,
        frame.bytes_per_pixel(),
        frame.step,
        frame.data.len()
    );
    println!(
        "intrinsics fx={:.1} fy={:.1} cx={:.1} cy={:.1} fov_y={:.1} deg  clip {:.2}..{:.0} m",
        frame.intrinsics.fx,
        frame.intrinsics.fy,
        frame.intrinsics.cx,
        frame.intrinsics.cy,
        frame.intrinsics.fov_y.to_degrees(),
        frame.intrinsics.near_clip,
        frame.intrinsics.far_clip
    );

    match write_ppm(&frame, OUTPUT) {
        Ok(()) => println!("wrote {OUTPUT}"),
        Err(e) => eprintln!("could not write {OUTPUT}: {e}"),
    }

    // Nothing to unmount: this handle never created a camera. front_left keeps
    // publishing for everyone else exactly as it did before the run.
    Ok(())
}

/// Write the frame as a binary PPM (P6) -- the simplest image format there is.
/// Mono8 is expanded to grey RGB; rgba8 drops alpha.
fn write_ppm(frame: &Frame, path: &str) -> std::io::Result<()> {
    let mut out = std::io::BufWriter::new(std::fs::File::create(path)?);
    write!(out, "P6\n{} {}\n255\n", frame.width, frame.height)?;

    let bpp = frame.bytes_per_pixel() as usize;
    let mut rgb = Vec::with_capacity(frame.width as usize * frame.height as usize * 3);
    // Row-major and top-down already, so a straight walk is the right order.
    for pixel in frame.data.chunks_exact(bpp) {
        match bpp {
            1 => rgb.extend_from_slice(&[pixel[0], pixel[0], pixel[0]]),
            _ => rgb.extend_from_slice(&pixel[..3]),
        }
    }
    out.write_all(&rgb)?;
    out.flush()
}
