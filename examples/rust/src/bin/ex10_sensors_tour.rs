//! ex10 -- sensors_tour: everything in one state snapshot, printed once a second.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex10_sensors_tour
//! ```
//!
//! The other examples read two or three fields. This one walks the whole
//! `State`, because the snapshot is organised around a distinction that is easy
//! to miss and expensive to get wrong -- **three epistemic categories, kept
//! apart on purpose**:
//!
//! | block | what it is |
//! |---|---|
//! | `kin`, `wrench`, `env` | simulator **truth** -- unavailable on a real robot |
//! | `sensors` | the **measured**, noisy, robot-observable view of that instant |
//! | `estimate` | what the robot's own filter **believes** |
//!
//! So `estimate.kin - kin` *is* the estimator error, and `sensors.gyroscope -
//! kin.ang_vel` *is* the gyro's noise realisation. Characterising a sensor is
//! always a diff between two published blocks; nothing has to be inferred.
//!
//! Details the printout makes visible:
//!
//! - **The accelerometer reads specific force**, not coordinate acceleration: at
//!   rest it is +1 g, in free fall 0. Subtracting gravity is your job and needs
//!   an attitude.
//! - **Sensors run at their own rates.** Each carries its own `timestamp` and
//!   `valid` flag, so a ~5 Hz GNSS fix repeated across state samples is only
//!   detectable by its stamp -- watch `gnss.timestamp` sit still while the state
//!   `t` advances.
//! - **The barometer's `altitude` drifts with the weather** (it is pressure
//!   altitude against `qnh`). `env.agl` LOOKS like the truthful height but is a
//!   hard-coded 0 in sim v3.0.0 (the downward raycast it needs ships with the
//!   scanning sensors) -- until then, derive height from `kin.lin_pos[2]`
//!   (negated: frd counts down).
//! - **Optical flow is optional** and deliberately a poor sensor: `valid` goes
//!   false over featureless ground. Robots mount it only if asked
//!   (`srv/sensors`), so `valid=false` here usually means "not mounted".
//! - **Magnetic field is in gauss**, not tesla (1 T = 10 000 G).
//! - **Every vector names its frame.** `coord_frame_id` on the snapshot is the
//!   robot's, not yours -- `"frd"` in this scene, so `lin_pos[2]` is DOWN and
//!   altitude is its negation.
//! - **`estimate` is only as real as the robot's filter.** The test scene runs
//!   none, so it arrives `valid=false` with zero fields and an empty frame id.
//!   That is what "no estimator" looks like on the wire, not a decode failure.
//!
//! In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

use vrobots_sdk::{RobotType, VirtualRobot, VrError};

const SYS_ID: u32 = 1; // the multirotor in the test scene
const HZ: f64 = 1.0; // slow: this is a page of text per iteration

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    // ===== loop =====
    loop {
        let s = robot.states();

        println!(
            "\n=== {} sys_id={} seq={} t={:.3}s schema={} frame={:?} ({}) ===",
            s.name,
            s.sys_id,
            s.seq,
            s.elapsed,
            s.schema_version,
            s.coord_frame_id,
            s.axis_convention.name()
        );

        // -- truth ----------------------------------------------------------
        let k = &s.kin;
        println!("TRUTH  kinematics");
        println!("  lin_pos   {} m       (world)", v3(k.lin_pos));
        println!(
            "  quat      [{:+.3},{:+.3},{:+.3},{:+.3}] (world, xyzw)",
            k.quat[0], k.quat[1], k.quat[2], k.quat[3]
        );
        println!("  lin_vel   {} m/s     (body)", v3(k.lin_vel));
        println!(
            "  ang_vel   {} rad/s   (body -- what a gyro measures)",
            v3(k.ang_vel)
        );
        println!("  lin_acc   {} m/s^2   (body)", v3(k.lin_acc));
        println!("  ang_acc   {} rad/s^2 (body)", v3(k.ang_acc));
        println!(
            "  wrench    F={} N  T={} N.m",
            v3(s.wrench.force),
            v3(s.wrench.torque)
        );

        // -- measured -------------------------------------------------------
        let n = &s.sensors;
        println!("MEASURED  sensors");
        println!(
            "  accel     {} m/s^2  {}   [specific force: +1 g at rest]",
            v3(n.accelerometer.linear_acceleration),
            stamp(n.accelerometer.valid, n.accelerometer.timestamp)
        );
        println!(
            "  gyro      {} rad/s  {}",
            v3(n.gyroscope.angular_velocity),
            stamp(n.gyroscope.valid, n.gyroscope.timestamp)
        );
        println!(
            "  mag       {} gauss  {}",
            v3(n.magnetometer.magnetic_field),
            stamp(n.magnetometer.valid, n.magnetometer.timestamp)
        );
        println!(
            "  baro      {:.1} Pa  alt={:.2} m (qnh {:.1} hPa)  {}",
            n.barometer.pressure,
            n.barometer.altitude,
            n.barometer.qnh,
            stamp(n.barometer.valid, n.barometer.timestamp)
        );
        println!(
            "  gnss      lat={:.6} lon={:.6} alt={:.2} m  vel={} m/s (NED)",
            n.gnss.geo_point.latitude,
            n.gnss.geo_point.longitude,
            n.gnss.geo_point.altitude,
            v3(n.gnss.velocity)
        );
        println!(
            "            fix={} eph={:.2} epv={:.2} m  {}   [slowest device, ~5 Hz]",
            n.gnss.fix_type,
            n.gnss.eph,
            n.gnss.epv,
            stamp(n.gnss.valid, n.gnss.timestamp)
        );
        println!(
            "  flow      {} m/s  {}   [optional; mount it via srv/sensors]",
            v3(n.optical_flow.velocity),
            stamp(n.optical_flow.valid, n.optical_flow.timestamp)
        );

        // -- believed -------------------------------------------------------
        let e = &s.estimate;
        println!(
            "BELIEVED  estimate  {}  frame={:?}",
            stamp(e.valid, e.timestamp),
            e.coord_frame_id
        );
        println!(
            "  lin_pos   {} m       (estimate.kin - kin IS the error)",
            v3(e.kin.lin_pos)
        );
        println!("  lin_vel   {} m/s", v3(e.kin.lin_vel));

        // -- environment and actuators ---------------------------------------
        let env = &s.env;
        println!("WORLD  environment");
        println!(
            "  gravity   {} m/s^2   air {:.1} Pa {:.3} kg/m^3 {:.1} C",
            v3(env.gravity),
            env.air_pressure,
            env.air_density,
            env.temperature
        );
        println!(
            "  agl       {:.2} m    home lat={:.6} lon={:.6}   [agl is hard-coded 0 in sim v3.0.0 -- use -lin_pos[2]]",
            env.agl, env.geo_point.latitude, env.geo_point.longitude
        );
        println!("ACTUATORS  command in, motion out");
        println!(
            "  pwm        {:?} us      (echo of the last command)",
            s.actuator.pwm
        );
        println!("  normalized {:?}", s.actuator.normalized);
        println!(
            "  measured   {:?}   (rotor rad/s -- what the devices did)",
            s.actuator.measured
        );

        robot.rate(HZ);
    }
}

/// A 3-vector, aligned so a column of them reads as a column.
fn v3(v: [f64; 3]) -> String {
    format!("({:+8.3},{:+8.3},{:+8.3})", v[0], v[1], v[2])
}

/// A sensor's own validity and clock -- the two fields that reveal its rate.
fn stamp(valid: bool, timestamp: f64) -> String {
    format!(
        "[{} t={timestamp:.3}]",
        if valid { "valid" } else { "INVALID" }
    )
}
