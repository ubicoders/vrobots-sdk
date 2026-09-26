//! ex16 -- two_cameras: two streams, one loop, independent freshness.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex16_two_cameras
//! ```
//!
//! The stereo pair every vrobot already has: **`front_left` and `front_right`,
//! both at 720p rgba8**. Each `open_camera` call returns its own `CameraStream`,
//! and each stream has its own reader thread, its own sequence numbers and its
//! own freshness -- so the two `fresh()` calls below are genuinely independent.
//! There is no combined "wait for both", by design: the cameras are separate
//! iceoryx2 services and they render on their own schedules. If you need them
//! paired, pair them yourself on `t_ns`, which is what the skew figure below
//! does.
//!
//! Neither call changes anything in the simulator, so two of these programs can
//! run at once on the same pair without either noticing, and neither has any
//! cleanup to do. That is the ordinary case for reading images; ex17 is the one
//! example that adds a camera of its own.
//!
//! The constraint to know before you do add one: **resolution is one knob for the
//! whole robot**, shared by every camera on it -- which is why both streams here
//! are `RESOLUTION` and could not be anything else. Asking for a second
//! resolution while this handle holds a stream at another is refused with
//! `VrError::InvalidArgument`: the change would restart every other stream under
//! a new service name and silently break the handles you already hold, so the SDK
//! will not do it behind your back. Format is per camera; only resolution is
//! shared.
//!
//! In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const LEFT: &str = "front_left"; // the pair every vrobot ships with
const RIGHT: &str = "front_right";
const RESOLUTION: &str = "720p"; // the same for both -- robot-wide
const FORMAT: &str = "rgba8"; // Unity's native readback -- four channels, NOT rgb8
const FRAMES: u64 = 60; // per camera
const HZ: f64 = 100.0;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    // Two subscriptions, no mutation: both cameras are already on the robot.
    let left = robot.open_camera(LEFT, RESOLUTION, FORMAT)?;
    let right = robot.open_camera(RIGHT, RESOLUTION, FORMAT)?;
    println!("left : {}", left.service_name());
    println!("right: {}", right.service_name());
    println!(
        "mounted by this handle: {:?}  <- neither is ours",
        robot.mounted_cameras()
    );

    let (mut n_left, mut n_right) = (0u64, 0u64);
    let mut last_left_ns = 0i64;

    // ===== loop =====
    while n_left < FRAMES || n_right < FRAMES {
        // Two consumers, each draining its own stream. Neither call can consume
        // the other's frame.
        if let Some(f) = left.fresh() {
            n_left += 1;
            last_left_ns = f.t_ns;
            if n_left % 20 == 1 {
                println!("L frame {n_left}: seq={} t={:.3}", f.seq, f.elapsed);
            }
        }
        if let Some(f) = right.fresh() {
            n_right += 1;
            if n_right % 20 == 1 {
                // The only honest way to relate two frames: subtract their
                // capture stamps. The SDK never pairs them for you.
                let skew_ms = if last_left_ns == 0 {
                    f64::NAN
                } else {
                    (f.t_ns - last_left_ns) as f64 / 1e6
                };
                println!(
                    "R frame {n_right}: seq={} t={:.3}  skew_vs_last_left={skew_ms:+.1} ms",
                    f.seq, f.elapsed
                );
            }
        }
        robot.rate(HZ);
    }

    let (ls, rs) = (left.stats(), right.stats());
    println!(
        "\nleft : {n_left} read, received={} seq_gaps={} missed={}",
        ls.received, ls.seq_gaps, ls.missed_frames
    );
    println!(
        "right: {n_right} read, received={} seq_gaps={} missed={}",
        rs.received, rs.seq_gaps, rs.missed_frames
    );

    // No cleanup, for either stream: this handle created neither camera. Both
    // keep rendering and publishing for everyone else after the process exits.
    Ok(())
}
