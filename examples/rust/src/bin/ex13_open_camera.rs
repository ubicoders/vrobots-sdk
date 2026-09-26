//! ex13 -- open_camera: attach to a camera that already exists, changing nothing.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex13_open_camera
//! ```
//!
//! `open_camera` is what every camera example here uses, and this file is the one
//! that shows it failing. It opens the iceoryx2 subscriber and touches the
//! simulator not at all. Two processes can open the same stream; neither disturbs
//! the other, and neither has to own the camera. `mount_camera` (ex17) is the
//! other side: it **creates** a camera on the robot, which is a mutation and
//! needs a name nobody is using.
//!
//! The price is that **the name, resolution and format must match the publisher
//! exactly**, because on iceoryx2 those three strings *are* the stream identity
//! and there is no type negotiation behind them. A mismatch is not an error at
//! the far end -- it is simply a service that does not exist -- so it fails as
//! `VrError::Timeout` after `camera_timeout` (5 s by default). That is the loud
//! failure this example is built to show you: a typo in `FORMAT` and a camera
//! that was never mounted are the same event.
//!
//! The constants below are the default every vrobot ships with: `front_left` and
//! `front_right` at 720p rgba8. `ex11_topic_discovery` prints the streams that
//! actually exist right now; its `[i]` lines are exactly these names. The one
//! thing that moves them is a **resolution** change, which is robot-wide: after a
//! program mounts a camera at 360p (ex17 is the only one here that can) the pair
//! is still mounted but publishing as `front_left/360p_rgba8`, so open that name
//! instead or restart the sim.
//!
//! In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

use std::time::Duration;

use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const CAMERA: &str = "front_left"; // every vrobot ships front_left and front_right
const RESOLUTION: &str = "720p";
const FORMAT: &str = "rgba8"; // Unity's native readback -- note: NOT rgb8
const FRAMES: u64 = 60;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    let cam = match robot.open_camera(CAMERA, RESOLUTION, FORMAT) {
        Ok(cam) => cam,
        Err(VrError::Timeout(detail)) => {
            // The whole point of the example: nothing is mounted under that
            // exact identity, and there is no way for the SDK to tell you which
            // of the three strings is wrong.
            eprintln!("no publisher for {CAMERA}/{RESOLUTION}_{FORMAT}: {detail}");
            eprintln!(
                "run `cargo run -p vrobots-examples --bin ex11_topic_discovery`: its \
                 [i] lines are the streams that do exist. A camera another process \
                 mounted then unmounted is gone."
            );
            return Ok(());
        }
        Err(other) => return Err(other),
    };
    println!(
        "attached to {} (nothing in the sim changed)",
        cam.service_name()
    );

    let spec = cam.spec();
    println!(
        "spec: name={} resolution={} format={} ({} bytes/frame)",
        spec.name,
        spec.resolution,
        spec.format,
        spec.data_size()
    );

    // ===== loop =====
    // Frame-paced, not clock-paced: wait_new_frame blocks until the next one, so
    // the body runs exactly once per rendered frame. Same idea as ex09's
    // wait_new_state, and the same rule about timeouts.
    let mut seen = 0u64;
    while seen < FRAMES {
        if let Err(VrError::Timeout(_)) = cam.wait_new_frame(Duration::from_millis(500)) {
            println!("no frame in 500 ms -- the camera stopped, or the sim is paused");
            continue;
        }
        let Some(frame) = cam.fresh() else { continue };
        seen += 1;
        if seen % 10 == 1 {
            println!(
                "frame {seen}: seq={} {}x{} {} bytes, mount=({:+.2},{:+.2},{:+.2}) m",
                frame.seq,
                frame.width,
                frame.height,
                frame.data.len(),
                frame.mount.position[0],
                frame.mount.position[1],
                frame.mount.position[2]
            );
        }
    }

    // There is nothing to clean up, and that is the lesson. This handle never
    // mounted anything, so it cannot unmount anything either -- the SDK refuses
    // locally rather than sending a remove for a camera that belongs to the
    // scene.
    match robot.unmount_camera(CAMERA) {
        Ok(()) => println!("unexpected: unmounted a camera this handle never mounted"),
        Err(e) => println!("\nunmount_camera refused, correctly: [{}] {e}", e.code()),
    }

    // Dropping the stream ends this subscription only. The camera keeps
    // rendering and publishing for everyone else.
    let stats = cam.stats();
    println!(
        "read {seen} frame(s): received={} decode_errors={} seq_gaps={} missed_frames={}",
        stats.received, stats.decode_errors, stats.seq_gaps, stats.missed_frames
    );
    Ok(())
}
