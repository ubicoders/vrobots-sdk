//! ex18 -- multi_robot: two robots, two handles, one loop.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex18_multi_robot
//! ```
//!
//! "One program is one embedded system bound to one robot" is the shape the SDK
//! is built around -- but nothing enforces it. A `VirtualRobot` is just a handle:
//! construct as many as you like, each with its own subscriber, its own snapshot
//! and its own command topic. Here one process drives the truck while watching
//! the multirotor.
//!
//! What that buys you, and what it costs:
//!
//! - **The handles are independent.** Every robot listens only on its own `cmd`
//!   topic, so the sys_id in the topic *is* the routing. There is no way for a
//!   command addressed to the truck to reach the multirotor -- send the wrong id
//!   and the symptom is silence, not a wrong robot moving.
//! - **`rate()` belongs to one handle.** It paces the calling loop, so call it on
//!   exactly one of them (below: the truck) and let the other's snapshot be read
//!   at that rate. Calling it on both would sleep twice per iteration and halve
//!   the loop rate.
//! - **The clocks are shared, the epochs are not.** `t_ns` is sim capture time
//!   for both, so it is directly comparable between robots. `elapsed` is measured
//!   from *each robot's own first sample*, so the two differ by whenever each
//!   `connect()` happened -- compare `t_ns` when relating two robots, never
//!   `elapsed`.
//! - **The two robots do not agree about axes**, and this is the trap. Measured
//!   live in this scene: the truck publishes `"fru"` and the multirotor
//!   publishes `"frd"`. Same third component, opposite sign -- up for one, down
//!   for the other. So the separation computed below is *wrong* in the strict
//!   sense, and a program that mixes the two positions without converting has a
//!   sign error it will not see. `coord_frame_id` is on every snapshot for
//!   exactly this reason; read it rather than assuming, especially in the one
//!   kind of program that holds two robots at once.
//!
//! Two handles is also two zenoh sessions and two subscriber threads. That is
//! fine for a handful of robots; a swarm of fifty wants one subscriber on
//! `vrobots/*/z/state`, which is a different program.
//!
//! In the test scene **sys_id 0 is the truck and sys_id 1 is the multirotor**.

use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const TRUCK_ID: u32 = 0;
const DRONE_ID: u32 = 1;
const STEER_US: f64 = 1500.0; // straight ahead
const THROTTLE_US: f64 = 1650.0; // light forward
const HZ: f64 = 10.0;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    // Two connects, two sessions. Each blocks until *its* robot's first state
    // snapshot arrives, so both are live by the time the loop starts.
    let truck = VirtualRobot::connect(RobotType::Truck, Some(TRUCK_ID))?;
    let drone = VirtualRobot::connect(RobotType::Multirotor, Some(DRONE_ID))?;
    println!(
        "truck sys_id={} ({:?}), drone sys_id={} ({:?})",
        truck.sys_id(),
        truck.robot_type(),
        drone.sys_id(),
        drone.robot_type()
    );

    // ===== loop =====
    loop {
        // One robot commanded ...
        truck.set_car(STEER_US, THROTTLE_US, Some(1100.0))?;
        let t = truck.states();

        // ... the other only observed. Nothing pairs the two snapshots: they are
        // whatever each subscriber last received.
        let d = drone.states();

        let [tx, ty, tz] = t.kin.lin_pos;
        let [dx, dy, dz] = d.kin.lin_pos;
        // t_ns is the shared clock -- this difference is real. (elapsed is not:
        // each robot counts from its own first sample.)
        let skew_ms = (t.t_ns - d.t_ns) as f64 / 1e6;
        let separation = ((tx - dx).powi(2) + (ty - dy).powi(2) + (tz - dz).powi(2)).sqrt();

        // Each snapshot names its own frame, and here they differ: the truck is
        // "fru" (third component UP) and the drone is "frd" (third component
        // DOWN). Print the tag beside every position rather than assuming one.
        println!(
            "truck[{}] pos=({tx:.2},{ty:.2},{tz:.2}) [{:?}] echo={:?}  |  \
             drone[{}] pos=({dx:.2},{dy:.2},{dz:.2}) [{:?}] alt={:.2} m",
            t.sys_id,
            t.coord_frame_id,
            t.actuator.pwm,
            d.sys_id,
            d.coord_frame_id,
            -dz // "frd": altitude is minus the down component
        );
        println!(
            "    naive separation={separation:.2} m{}  snapshot skew={skew_ms:+.1} ms  \
             (elapsed: truck {:.2}s vs drone {:.2}s -- different epochs)",
            if t.coord_frame_id == d.coord_frame_id {
                ""
            } else {
                " (WRONG: mixed frames, convert first)"
            },
            t.elapsed,
            d.elapsed
        );

        // Paced once, on one handle.
        truck.rate(HZ);
    }
}
