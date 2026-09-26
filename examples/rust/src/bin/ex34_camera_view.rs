//! ex34 -- camera_view: show the robot's own camera in an OpenCV window.
//!
//! ```text
//! cargo run -p vrobots-examples --features opencv --bin ex34_camera_view
//! ```
//!
//! The one example that needs a dependency outside the SDK, which is why it is
//! behind the `opencv` feature: `cargo build --workspace` skips it unless you ask
//! for it, so nobody has to install OpenCV to build the other thirty-five.
//!
//! Like every camera example here it uses a camera the robot already has: **every
//! vrobot ships with `front_left` and `front_right` mounted at 720p rgba8**, and
//! `open_camera` attaches to one of those streams without changing anything in
//! the simulator. There is no mount and no unmount -- the camera is not ours, so
//! there is nothing to clean up. ex17 is the one example that adds a camera of
//! its own.
//!
//! Everything else here is ex03 with a window bolted on, and the window is the
//! whole lesson. **`frame.data` is RGBA, and OpenCV is BGR.** The SDK normalises geometry
//! and nothing else: rows are already top-down and `step == width *
//! bytes_per_pixel` with no padding, so a `Mat` header maps straight onto the
//! bytes, but channels are the renderer's own order and are never swapped. The
//! conversion happens once, here, at the call site that wants BGR.
//!
//! The loop is frame-paced rather than clock-paced: `wait_new_frame` blocks until
//! the next render, so `imshow` runs exactly once per frame instead of spinning on
//! one it has already drawn. A timeout is a status, not a failure -- the simulator
//! is paused, or the camera stopped -- so the body still pumps `wait_key` to keep
//! the window responsive and goes round again.
//!
//! `FORMAT` is `rgba8`, Unity's native readback, so the Mat is `CV_8UC4` and the
//! conversion is `COLOR_RGBA2BGR`. rgb8 would be `CV_8UC3` and `COLOR_RGB2BGR`;
//! mono8 would be `CV_8UC1` and needs no conversion at all (ex15).
//!
//! Quit with **q** or **Esc** in the window, or Ctrl-C -- either is safe now that
//! this example leaves the simulator untouched.
//!
//! In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

use std::error::Error;
use std::time::Duration;

use opencv::core::{CV_8UC4, Mat, Scalar};
use opencv::prelude::*;
use opencv::{highgui, imgproc};
use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const CAMERA: &str = "front_left"; // every vrobot ships front_left and front_right
const RESOLUTION: &str = "720p";
const FORMAT: &str = "rgba8"; // four channels, so the Mat is CV_8UC4
const WINDOW: &str = "vrobots camera";
const TIMEOUT: Duration = Duration::from_millis(500);

const KEY_Q: i32 = 113;
const KEY_ESC: i32 = 27;

fn main() -> Result<(), Box<dyn Error>> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    // open_camera SUBSCRIBES to a camera the robot already has and mutates
    // nothing. The three strings must match the publisher exactly -- on iceoryx2
    // they *are* the stream identity -- so a mismatch is a Timeout rather than an
    // error from the far end (ex13 shows that path).
    let cam = robot.open_camera(CAMERA, RESOLUTION, FORMAT)?;
    println!("showing {} -- press q or Esc to quit", cam.service_name());

    highgui::named_window(WINDOW, highgui::WINDOW_AUTOSIZE)?;

    // ===== loop =====
    // Frame-paced: wait_new_frame blocks until the next render, so imshow runs
    // once per frame rather than redrawing one it has already shown.
    let mut seen = 0u64;
    loop {
        if let Err(VrError::Timeout(_)) = cam.wait_new_frame(TIMEOUT) {
            // A status, not a failure: the sim is paused, or the camera stopped.
            // Still pump the GUI so the window stays responsive.
            if quit_requested()? {
                break;
            }
            continue;
        }
        let Some(frame) = cam.fresh() else { continue };
        seen += 1;

        // The pixels are already row-major, top-down and tightly packed, so this
        // is a straight copy into a Mat of the same shape.
        let mut rgba = Mat::new_rows_cols_with_default(
            frame.height as i32,
            frame.width as i32,
            CV_8UC4,
            Scalar::all(0.0),
        )?;
        rgba.data_bytes_mut()?.copy_from_slice(&frame.data);

        // The SDK never does this for you: RGBA is what Unity rendered, BGR is
        // what OpenCV displays.
        let mut bgr = Mat::default();
        imgproc::cvt_color_def(&rgba, &mut bgr, imgproc::COLOR_RGBA2BGR)?;

        highgui::imshow(WINDOW, &bgr)?;
        if quit_requested()? {
            break;
        }
    }

    // ===== cleanup =====
    // Only ours: the window. Dropping the stream ends this subscription and
    // nothing else -- front_left keeps rendering for everyone.
    highgui::destroy_all_windows()?;
    let stats = cam.stats();
    println!(
        "showed {seen} frame(s), received={} decode_errors={} seq_gaps={}",
        stats.received, stats.decode_errors, stats.seq_gaps
    );
    Ok(())
}

/// One millisecond of GUI pumping, which is also how a keypress is read.
/// `imshow` alone draws nothing until `wait_key` runs.
fn quit_requested() -> Result<bool, opencv::Error> {
    let key = highgui::wait_key(1)?;
    Ok(key == KEY_Q || key == KEY_ESC)
}
