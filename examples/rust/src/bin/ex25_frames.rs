//! ex25 -- frames: change the axes the robot reports in, and read the scene's.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex25_frames
//! ```
//!
//! ex18 showed two robots in one scene disagreeing about which way is up -- the
//! truck publishing `"fru"`, the multirotor `"frd"`. This is the service that
//! decides that, and the query that reads the level above it.
//!
//! **Frames are presentation, never physics.** The truck below does not move one
//! millimetre differently after the change; the numbers describing it are
//! permuted, and one of them changes sign. Registered ids:
//!
//! | id | axes | handed |
//! |---|---|---|
//! | `unity` | +x right, +y up, +z forward | left |
//! | `frd` | +x forward, +y right, +z down | right |
//! | `fru` | +x forward, +y right, +z up | left |
//! | `cv` | +x right, +y down, +z forward | right |
//!
//! plus whatever the scene registered at runtime -- which is why
//! `coord_frame_id` (a string) is authoritative and `axis_convention` (an enum
//! tag) is the convenience beside it.
//!
//! # Three levels, most specific wins
//!
//! ```text
//! device override   (srv/frames, per device)      <- most specific
//! robot override    (srv/frames, robot_frame_id)
//! robot default     (truck: fru, multirotor: frd, globalhawk: frd -- regardless of the scene)
//! scene frame       (scene_frame(); every launch starts at fru)
//! ```
//!
//! `set_frames` takes two independent halves. `None` for `robot_frame_id` leaves
//! the robot's level alone; [`INHERIT_FRAME`] as an id **clears** that level's
//! override so the one below wins again -- which is a different thing from `""`,
//! meaning "do not touch".
//!
//! # Two names for the same device
//!
//! The device the frames service matches is **`gps`**; the block it moves is
//! called **`gnss`** in the state message. Use the [`device`] constants rather
//! than a literal -- an unrecognised name is skipped **entry by entry**, with a
//! log line inside the simulator and an `ok` ack, so a typo is invisible from
//! out here. The call below includes one deliberate miss (`camera/front`, on a
//! truck that has no such camera) to show that the other entries still apply.
//!
//! # Where the confirmation is
//!
//! Two places, and neither is the ack: the `coord_frame_id` stamped on every
//! subsequent state header, and the robot's `z/frames` topic, which republishes
//! the full definition of each frame -- basis matrix included -- on change and
//! then at 1 Hz.
//!
//! [`INHERIT_FRAME`]: vrobots_sdk::INHERIT_FRAME
//! [`device`]: vrobots_sdk::device

use vrobots_sdk::{DeviceFrame, INHERIT_FRAME, RobotType, State, VirtualRobot, VrError, device};

const HZ: f64 = 25.0;
const SETTLE_SAMPLES: u32 = 12; // ~0.5 s: the change lands on the next physics step

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Truck, None)?;
    let sys_id = robot.sys_id();
    println!(
        "created sys_id={sys_id}\n  service : {}\n  scene   : {}\n  z/frames: {}\n",
        vrobots_sdk::topics::srv_frames(sys_id),
        vrobots_sdk::topics::scene_srv_frame(),
        vrobots_sdk::topics::frames(sys_id)
    );

    // ===== level 1: the scene =====
    // A payload-less GET that reads and changes nothing. Scene scope, not robot
    // scope -- the answer is the same for every robot loaded, and this robot's
    // session is used only because that is where the wire is.
    let scene = robot.scene_frame()?;
    println!(
        "scene frame: {:?} (axis_convention {}, {:?})",
        scene.coord_frame_id,
        scene.axis_convention.0,
        scene.axis_convention.name()
    );

    // ===== what the truck reports today =====
    report(&robot, "default   ");

    // ===== a robot override plus device overrides =====
    println!(
        "\nset_frames(Some(\"frd\"), [gyroscope->fru, gps->inherit, \
         camera/front->cv])"
    );
    robot.set_frames(
        Some("frd"),
        &[
            // Keep the gyro reading the way it was, while the robot moves to frd.
            DeviceFrame::new(device::GYROSCOPE, "fru"),
            // Clear any override this device had: fall back to the robot's level.
            DeviceFrame::new(device::GPS, INHERIT_FRAME),
            // Deliberate miss: this truck has no camera called "front". The entry
            // is skipped with a log line and an `ok` ack; the others still apply.
            DeviceFrame::new(device::camera("front"), "cv"),
        ],
    )?;
    settle(&robot);
    report(&robot, "overridden");
    println!(
        "  ^ same motion, different numbers: fru and frd differ only in the sign \
         of the third component."
    );

    // ===== put it back =====
    println!("\nset_frames(Some(INHERIT_FRAME), [gyroscope->inherit])");
    robot.set_frames(
        Some(INHERIT_FRAME),
        &[DeviceFrame::new(device::GYROSCOPE, INHERIT_FRAME)],
    )?;
    settle(&robot);
    report(&robot, "cleared   ");
    println!(
        "  ^ back to the truck's own default. (Here that is fru and the scene is \
         fru too, so this one run cannot tell you which level answered -- the \
         robot default outranks the scene either way.)"
    );

    // ===== what the SDK will not send =====
    println!("\n-- refused before anything reaches the wire --");
    show_refusal("nothing set", robot.set_frames(None, &[]));
    show_refusal(
        "an entry with an empty device",
        robot.set_frames(None, &[DeviceFrame::new("", "frd")]),
    );
    show_refusal(
        "an entry with an empty frame id",
        robot.set_frames(None, &[DeviceFrame::new(device::GYROSCOPE, "")]),
    );
    println!("(use INHERIT_FRAME to clear an override; \"\" would be skipped sim-side)");

    robot.delete()?;
    println!("\ndeleted sys_id={sys_id}");
    Ok(())
}

/// Let the change land: services apply in phase 0 of the next physics step.
fn settle(robot: &VirtualRobot) {
    for _ in 0..SETTLE_SAMPLES {
        robot.rate(HZ);
    }
}

/// The robot's frame, one device's frame, and a vector that shows the difference.
fn report(robot: &VirtualRobot, label: &str) {
    let s: &State = &robot.states();
    let [x, y, z] = s.kin.lin_pos;
    println!(
        "{label} robot={:<6?} ({:<5}) pos=({x:+7.3},{y:+7.3},{z:+7.3})  \
         gyro frame={:?}  gnss frame={:?}",
        s.coord_frame_id,
        s.axis_convention.name(),
        s.sensors.gyroscope.coord_frame_id,
        s.sensors.gnss.coord_frame_id
    );
}

/// Print a client-side refusal: code first, then the sim behaviour it prevents.
fn show_refusal(what: &str, result: Result<(), VrError>) {
    match result {
        Ok(()) => println!("  {what:<32} UNEXPECTED: accepted"),
        Err(e) => println!("  {what:<32} [{}] {}", e.code(), e.detail()),
    }
}
