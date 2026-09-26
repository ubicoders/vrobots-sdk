//! ex26 -- drive_config: retune the truck's drivetrain under itself.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex26_drive_config
//! ```
//!
//! `srv/drive` is the truck's own service -- ask a multirotor for it and the GET
//! finds no responder (ex30 makes that the point of an example). Every value is
//! read **live** by the steering servo, the drive motor and the dynamics, so a
//! change bites from the next physics step with no rebuild and no dropout.
//!
//! There is nothing to read back, so this example measures instead: drive the
//! same full-left circle four times and compare the steady turn radius,
//! `speed / yaw_rate`. A steering limit that halves must roughly double the
//! radius, or the request did not land.
//!
//! # What the simulator quietly substitutes
//!
//! None of these reach the ack, which is `ok` either way:
//!
//! | field | out of range | what you get |
//! |---|---|---|
//! | `max_steer_deg` | anything | **hard-clamped to 0-60 degrees** |
//! | `no_load_wheel_rpm` | `<= 0` | the default, 200 |
//! | `pwm_band` | not strictly increasing | the whole band replaced by 1100 / 1500 / 1900 |
//! | `drive_mode` | not 2 or 4 | ignored -- so the SDK refuses it first |
//!
//! Run 3 below asks for 90 degrees and gets 60. The only way to know is the
//! circle it drives.
//!
//! # Two pulse-width bands, and they are not the same band
//!
//! The truck's factory band is **1100 / 1500 / 1900 us** with a deadband either
//! side of neutral, while [`set_car`] validates against the wider **1100-2000**
//! the actuators are specified on. So 1950 is accepted by the SDK and is past
//! full throttle for this truck. `pwm_band` moves all four numbers as one group
//! -- there is no way to change only the neutral point.
//!
//! # Reading the drivetrain in the state stream
//!
//! `actuator.pwm` echoes the three channels you sent. `actuator.measured[0..3]`
//! are the four wheel speeds in rad/s (FL, FR, RL, RR) -- an undriven wheel
//! still reports, because the road turns it, so `drive_mode` 2 versus 4 shows up
//! as *which* wheels lead under power, not as two silent channels.
//! `actuator.measured[4]` is the steering servo, and it is the channel that
//! answers `max_steer_deg`.
//!
//! There is also a selector form of this service (`?mode=2|4`) for clients that
//! cannot attach a payload. The SDK can, so it does not use it.
//!
//! [`set_car`]: vrobots_sdk::VirtualRobot::set_car

use vrobots_sdk::{DriveConfig, PwmBand, RobotType, VirtualRobot, VrError};

const STEER_US: f64 = 1100.0; // full left
const THROTTLE_US: f64 = 1700.0; // steady forward
const BRAKE_US: f64 = 1100.0; // released
const HZ: f64 = 25.0;
const SPIN_UP_SAMPLES: u32 = 75; // ~3 s to reach a steady circle
const MEASURE_SAMPLES: u32 = 40; // ~1.6 s averaged

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Truck, None)?;
    println!(
        "created sys_id={}, service key = {}\n",
        robot.sys_id(),
        vrobots_sdk::topics::srv_drive(robot.sys_id())
    );

    // ===== run 1: as spawned =====
    let stock = circle(&robot, "as spawned")?;

    // ===== run 2: half the steering =====
    robot.configure_drive(&DriveConfig::default().with_max_steer_deg(15.0))?;
    let narrow = circle(&robot, "max_steer_deg = 15")?;

    // ===== run 3: more than the simulator allows =====
    robot.configure_drive(&DriveConfig::default().with_max_steer_deg(90.0))?;
    let clamped = circle(&robot, "max_steer_deg = 90 -> clamped to 60")?;

    // ===== run 4: rear-wheel drive, softer motor, factory band restated =====
    robot.configure_drive(
        &DriveConfig::default()
            .with_drive_mode(2) // rear axle only (4 = all wheels)
            .with_max_steer_deg(30.0)
            .with_steer_rate_dps(120.0) // 0 would be an ideal, instant servo
            .with_max_motor_torque_nm(40.0)
            .with_no_load_wheel_rpm(200.0)
            .with_idle_brake_torque_nm(5.0)
            .with_max_brake_torque_nm(150.0)
            // All four numbers move together, and this IS the factory band.
            .with_pwm_band(PwmBand::new(1100, 1500, 1900, 30)),
    )?;
    let rwd = circle(&robot, "drive_mode = 2, 40 N.m, 30 deg")?;

    println!("\nsteady turn radius (speed / yaw rate), same command every time:");
    for run in [&stock, &narrow, &clamped, &rwd] {
        println!(
            "  {:<38} r={:6.2} m   speed={:5.2} m/s  yaw={:+6.3} rad/s  servo={:+7.3}",
            run.label, run.radius, run.speed, run.yaw_rate, run.servo
        );
    }
    println!(
        "Runs 2 and 3 are the whole lesson: a limit that halves widens the circle, \
         and the 90 that came back as 60 is indistinguishable from a 60 that was \
         asked for."
    );

    // ===== what the SDK refuses =====
    println!("\n-- refused before anything reaches the wire --");
    show_refusal(
        "drive_mode = 3",
        robot.configure_drive(&DriveConfig::default().with_drive_mode(3)),
    );
    show_refusal(
        "nothing set",
        robot.configure_drive(&DriveConfig::default()),
    );

    robot.delete()?;
    println!("\ndeleted sys_id={}", robot.sys_id());
    Ok(())
}

/// One steady-state circle.
struct Circle {
    label: String,
    speed: f64,
    yaw_rate: f64,
    radius: f64,
    servo: f64,
}

/// Reset, drive a full-left circle, and average the steady part of it.
///
/// The truck publishes in `"fru"`, so the third component of a body rate is yaw
/// about UP -- `ang_vel[2]`.
fn circle(robot: &VirtualRobot, label: &str) -> Result<Circle, VrError> {
    println!("-- {label} --");
    robot.reset()?;

    for _ in 0..SPIN_UP_SAMPLES {
        robot.set_car(STEER_US, THROTTLE_US, Some(BRAKE_US))?;
        robot.rate(HZ);
    }

    let (mut speed, mut yaw_rate, mut servo) = (0.0, 0.0, 0.0);
    for i in 0..MEASURE_SAMPLES {
        robot.set_car(STEER_US, THROTTLE_US, Some(BRAKE_US))?;
        let s = robot.states();
        let [vx, vy, vz] = s.kin.lin_vel;
        speed += (vx * vx + vy * vy + vz * vz).sqrt();
        yaw_rate += s.kin.ang_vel[2];
        servo += s.actuator.measured.get(4).copied().unwrap_or(0.0);
        if i == 0 {
            println!(
                "   echo={:?} wheels={:?}",
                s.actuator.pwm,
                &s.actuator.measured[..s.actuator.measured.len().min(4)]
            );
        }
        robot.rate(HZ);
    }

    let n = f64::from(MEASURE_SAMPLES);
    let (speed, yaw_rate, servo) = (speed / n, yaw_rate / n, servo / n);
    Ok(Circle {
        label: label.to_string(),
        speed,
        yaw_rate,
        radius: if yaw_rate.abs() > 1e-6 {
            speed / yaw_rate.abs()
        } else {
            f64::INFINITY
        },
        servo,
    })
}

/// Print a client-side refusal: code first, then the sim behaviour it prevents.
fn show_refusal(what: &str, result: Result<(), VrError>) {
    match result {
        Ok(()) => println!("  {what:<16} UNEXPECTED: accepted"),
        Err(e) => println!("  {what:<16} [{}] {}", e.code(), e.detail()),
    }
}
