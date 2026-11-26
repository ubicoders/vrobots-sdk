//! ex42 -- lidar: one revolution, read as ranges plus the layout that places them.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex42_lidar                             # warehouse: forklift 0, pallet robot 1
//! cargo run -p vrobots-examples --bin ex42_lidar -- <forklift> <palletrobot>  # other ids
//! ```
//!
//! Two lidars, one API. The forklift carries a 3-D unit named `lidar` (VLP-16
//! class: 16 rings x 900 columns, 0.4-100 m, 20 Hz) and the pallet robot two 2-D
//! units, `lidar_front_left` and `lidar_rear_right` (RPLIDAR A2 class: one ring
//! of 360, 0.15-12 m, 10 Hz). They ride different wires --
//!
//! | | 3-D | 2-D |
//! |---|---|---|
//! | open with | `open_lidar_3d(name)` | `open_lidar_2d(name)` |
//! | topic | `vrobots/{id}/i/lidar/{name}`, iceoryx2 | `vrobots/{id}/z/lidar/{name}`, zenoh |
//! | reaches | this host only | this host or the network |
//!
//! -- and decode into the same `LidarScan`, read through the same
//! `LidarStream`, so `show()` below takes either. Opening one subscribes and
//! changes nothing in the simulator: like a camera (ex13), there is nothing to
//! clean up.
//!
//! # Ranges, not points
//!
//! The wire carries one range per beam, plus the layout that places it. Beam
//! `ring * columns + column` points along
//!
//! ```text
//! azimuth   = azimuth_start   + column * azimuth_step    column 0 forward, counter-clockwise from above
//! elevation = elevation_start + ring   * elevation_step  ring 0 the lowest
//! point     = (r cos(el) cos(az), r cos(el) sin(az), r sin(el))    FLU: x forward, y left, z up
//! ```
//!
//! **`+inf` means no return** -- nothing within `max_range` -- never 0, so the
//! nearest-return search below skips infinities, not zeros. `beam_point()` is
//! that formula; `LidarScan::points()` applies it to every beam at once and,
//! under a `srv/frames` override of device `lidar/<name>`, re-expresses the
//! points in that frame, while `beam_point()` stays in FLU. Either way a point
//! is in the **lidar's** frame, origin at its optical centre: the mount in
//! `params`, given in the robot's frame, is what takes it onto the body.
//!
//! The 2-D units publish their full 360 degrees, so on the pallet robot the
//! inward quarter of each ring looks straight through the robot's own body (its
//! colliders are skipped). Masking that quarter is the consumer's call.
//!
//! # Lidar is Pro
//!
//! On the Community tier the simulator publishes no lidar topic at all, so the
//! open finds nothing and fails with `VrError::Timeout`: after 5 s for the 3-D
//! (a registry lookup, `camera_timeout`) and 15 s for the 2-D (zenoh has no
//! registry, so the open waits for a first scan, `probe_timeout`). The SDK
//! cannot tell "no such lidar" from "not on this plan" and says both. That is an
//! answer, not a crash: this example prints it and moves on. On Community only
//! the focused robot is on the wire at all, so a robot may not attach either --
//! that too is printed, and its half skipped.

use std::time::Duration;

use vrobots_sdk::{LidarScan, LidarStream, RobotType, VirtualRobot, VrError};

const USAGE: &str = "\
The warehouse scene pins its forklift at sys_id 0 and its pallet robot at 1,
which are the defaults. For others, pass both ids:

    cargo run -p vrobots-examples --bin ex42_lidar -- 0 1

The scene has to be loaded and in Play mode. List what is publishing -- the
lidars included -- with:

    cargo run -p vrobots-examples --bin ex11_topic_discovery";

const FORKLIFT_ID: u32 = 0; // the warehouse scene's forklift
const PALLETROBOT_ID: u32 = 1; // and its pallet robot
const LIDAR_3D: &str = "lidar"; // the forklift's one 3-D unit
const LIDAR_2D: &str = "lidar_front_left"; // the pallet robot's front-left 2-D unit
const SCAN_TIMEOUT: Duration = Duration::from_secs(1); // a 2-D revolution is 100 ms
const RATE_SCANS: u32 = 10; // scans timed to measure the rate
const BEARINGS_DEG: [f64; 4] = [0.0, 90.0, 180.0, 270.0]; // front, left, back, right

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let (forklift_id, palletrobot_id) = sys_ids_from_args();

    // ===== 3-D: the forklift's lidar, on iceoryx2 =====
    println!("\n== 3-D: '{LIDAR_3D}' on sys_id {forklift_id} ==");
    if let Some(forklift) = attach(RobotType::Forklift, forklift_id) {
        match forklift.open_lidar_3d(LIDAR_3D) {
            Ok(lidar) => show(&lidar),
            // Absent: the answer, not a crash. On Community it is the tier.
            Err(VrError::Timeout(detail)) => println!("not there: {detail}"),
            Err(e) => println!("could not open it: [{}] {e}", e.code()),
        }
    }

    // ===== 2-D: the pallet robot's front-left lidar, on zenoh =====
    println!("\n== 2-D: '{LIDAR_2D}' on sys_id {palletrobot_id} ==");
    if let Some(palletrobot) = attach(RobotType::PalletRobot, palletrobot_id) {
        println!(
            "zenoh has no registry: the open waits up to {:?} for a first scan",
            palletrobot.options().probe_timeout
        );
        match palletrobot.open_lidar_2d(LIDAR_2D) {
            Ok(lidar) => show(&lidar),
            Err(VrError::Timeout(detail)) => println!("not there: {detail}"),
            Err(e) => println!("could not open it: [{}] {e}", e.code()),
        }
    }

    // Each stream ends with its handle; both lidars keep scanning for everyone else.
    Ok(())
}

/// Attach to a robot for its lidar, or say why not and skip that half.
///
/// No type check here (ex37-ex41 make one): a lidar is opened by name, and a
/// name the robot does not carry is the same clean "not there" as a missing
/// topic.
fn attach(robot_type: RobotType, sys_id: u32) -> Option<VirtualRobot> {
    match VirtualRobot::connect(robot_type, Some(sys_id)) {
        Ok(robot) => Some(robot),
        Err(e) => {
            println!("no {robot_type:?} at sys_id {sys_id}: [{}] {e}", e.code());
            println!(
                "(is the warehouse scene loaded and in Play mode? On the Community tier only \
                 the focused robot is on the wire at all.)"
            );
            None
        }
    }
}

/// One scan, taken apart: the layout, the returns, the nearest one, four
/// bearings; then the rate the stream runs at.
fn show(lidar: &LidarStream) {
    // Scan-paced, like ex13's frames: block until the next revolution lands.
    if let Err(e) = lidar.wait_new_scan(SCAN_TIMEOUT) {
        println!("no scan: [{}] {e}", e.code());
        return;
    }
    let Some(scan) = lidar.latest() else {
        return;
    };
    let p = &scan.params;
    println!(
        "{} on {} [{}], scan seq {} at t={:.2} s",
        p.name,
        lidar.topic(),
        lidar.transport().tag(),
        scan.seq,
        scan.elapsed
    );

    // ===== the layout: where each range points =====
    println!(
        "  layout    {} ring(s) x {} columns = {} ranges, {:.0} Hz, segments {}",
        p.rings,
        p.columns,
        scan.ranges.len(),
        p.scan_rate,
        p.segments
    );
    println!(
        "  column c  azimuth {:+.1} + c x {:.2} deg (0 forward, counter-clockwise from above)",
        p.azimuth_start.to_degrees(),
        p.azimuth_step.to_degrees()
    );
    println!(
        "  ring r    elevation {:+.1} + r x {:.1} deg (ring 0 the lowest)",
        p.elevation_start.to_degrees(),
        p.elevation_step.to_degrees()
    );
    println!(
        "  range     {:.2} to {:.1} m; no return reads +inf, never 0",
        p.min_range, p.max_range
    );
    println!(
        "  mount     ({:+.2}, {:+.2}, {:+.2}) m, euler ({:+.0}, {:+.0}, {:+.0}) deg, in {:?} -- the \
         robot's frame; the scan's own frame is {:?}",
        p.mount_position[0],
        p.mount_position[1],
        p.mount_position[2],
        p.mount_euler_rad[0].to_degrees(),
        p.mount_euler_rad[1].to_degrees(),
        p.mount_euler_rad[2].to_degrees(),
        p.mount_coord_frame_id,
        scan.coord_frame_id
    );

    // ===== the returns, and the nearest one =====
    let mut hits = 0;
    let mut nearest_index = 0;
    let mut nearest_range = f32::INFINITY;
    for (i, &r) in scan.ranges.iter().enumerate() {
        if r.is_finite() {
            hits += 1;
            if r < nearest_range {
                nearest_range = r;
                nearest_index = i;
            }
        }
    }
    println!("  returns   {hits} of {} beams", scan.ranges.len());
    if hits > 0 {
        let columns = p.columns as usize;
        let ring = nearest_index / columns;
        let column = nearest_index % columns;
        let [x, y, z] = beam_point(&scan, nearest_index);
        println!(
            "  nearest   {nearest_range:.2} m on ring {ring}, column {column} (az {:+.1}, el {:+.1} \
             deg) -> ({x:+.2}, {y:+.2}, {z:+.2}) m FLU",
            (p.azimuth_start + column as f64 * p.azimuth_step).to_degrees(),
            (p.elevation_start + ring as f64 * p.elevation_step).to_degrees()
        );
    }

    // ===== four bearings on the lowest ring =====
    println!(
        "  ring 0, elevation {:+.1} deg, at four bearings:",
        p.elevation_start.to_degrees()
    );
    for bearing in BEARINGS_DEG {
        // The column nearest that azimuth; ring 0 is the first `columns` ranges.
        let column = ((bearing.to_radians() - p.azimuth_start) / p.azimuth_step).round() as usize
            % p.columns as usize;
        let r = scan.ranges[column];
        if r.is_finite() {
            let [x, y, z] = beam_point(&scan, column);
            println!(
                "     az {bearing:+6.1} deg  {r:6.2} m  -> ({x:+6.2}, {y:+6.2}, {z:+6.2}) m FLU"
            );
        } else {
            println!("     az {bearing:+6.1} deg     inf    -> no return");
        }
    }

    // ===== the stream =====
    // seq counts revolutions and t_ns stamps each one's completion on the sim
    // clock, so the rate comes out of the scans, not out of this process's
    // scheduling.
    let mut last = scan.clone();
    for _ in 0..RATE_SCANS {
        if let Err(e) = lidar.wait_new_scan(SCAN_TIMEOUT) {
            println!("  the stream stopped: [{}] {e}", e.code());
            return;
        }
        if let Some(next) = lidar.latest() {
            last = next;
        }
    }
    let revolutions = last.seq.saturating_sub(scan.seq);
    let seconds = (last.t_ns - scan.t_ns) as f64 * 1e-9;
    let stats = lidar.stats();
    println!(
        "  stream    {revolutions} more scans in {seconds:.2} s of sim time = {:.1} Hz; received \
         {}, seq gaps {}, decode errors {}",
        revolutions as f64 / seconds,
        stats.received,
        stats.seq_gaps,
        stats.decode_errors
    );
}

/// The point beam `index` hit, metres, in FLU: the wire's own formula.
///
/// `index` is `ring * columns + column`. Only for a finite range: `+inf` is no
/// return, and has no point.
fn beam_point(scan: &LidarScan, index: usize) -> [f64; 3] {
    let p = &scan.params;
    let columns = p.columns as usize;
    let ring = (index / columns) as f64;
    let column = (index % columns) as f64;
    let az = p.azimuth_start + column * p.azimuth_step;
    let el = p.elevation_start + ring * p.elevation_step;
    let r = f64::from(scan.ranges[index]);
    [
        r * el.cos() * az.cos(),
        r * el.cos() * az.sin(),
        r * el.sin(),
    ]
}

/// The forklift's and the pallet robot's sys_ids: the two command-line
/// arguments, or [`FORKLIFT_ID`] and [`PALLETROBOT_ID`].
fn sys_ids_from_args() -> (u32, u32) {
    let mut args = std::env::args();
    let program = args.next().unwrap_or_else(|| "ex42_lidar".to_string());
    let forklift = sys_id_or(args.next(), FORKLIFT_ID, &program);
    let palletrobot = sys_id_or(args.next(), PALLETROBOT_ID, &program);
    (forklift, palletrobot)
}

/// One sys_id argument, or `default` when it was not given.
fn sys_id_or(arg: Option<String>, default: u32, program: &str) -> u32 {
    let Some(arg) = arg else {
        return default;
    };
    match arg.parse::<u32>() {
        Ok(sys_id) => sys_id,
        Err(_) => {
            eprintln!("usage: {program} [forklift_sys_id] [palletrobot_sys_id]\n\n{USAGE}");
            std::process::exit(2);
        }
    }
}
