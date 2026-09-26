//! ex28 -- hello_msd: the smallest thing in the simulator that can be controlled.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex28_hello_msd
//! ```
//!
//! A mass-spring-damper is one mass on one axis:
//!
//! ```text
//! m*x'' + c*x' + k*x = F
//! ```
//!
//! and the SDK owns every letter of it. `m` is
//! [`set_physical_params`](vrobots_sdk::VirtualRobot::set_physical_params) (ex22),
//! `k` and `c` are [`configure_msd`](vrobots_sdk::VirtualRobot::configure_msd),
//! and `F` is [`set_msd_force`](vrobots_sdk::VirtualRobot::set_msd_force) --
//! newtons along the plant's `+x`, clamped to its `max_force` (100 N by default).
//! Which makes this the one robot where you can *predict* the answer before you
//! run it, and the printout below does:
//!
//! ```text
//! settles at   F / k          metres
//! period       2*pi*sqrt(m/k) seconds
//! damping      c / (2*sqrt(k*m))   -- < 1 rings, ~1 slides home, > 1 crawls
//! ```
//!
//! **An Msd is creatable.** The sandbox catalog is `multirotor`, `truck`, `msd`,
//! so `connect(RobotType::Msd, None)` spawns one -- unlike the cart-pole, the
//! half-drone and the Global Hawk (ex29-ex33), which exist only where a scene
//! authored them.
//!
//! # Reading it
//!
//! It publishes in the identity `"unity"` frame, so `kin.lin_pos[0]` is the
//! position you watch in the editor and `kin.lin_vel[0]` is `x'`. The other two
//! components never move. The actuator block carries the plant's own arithmetic:
//!
//! | channel | meaning |
//! |---|---|
//! | `actuator.measured[0]` | the **total** force on the mass, `F - k*x - c*x'` |
//! | `actuator.measured[1]` | displacement from equilibrium, metres |
//!
//! Note that `measured[0]` is not the force you sent -- it is what the spring and
//! damper left of it, which is why it crosses zero at every peak.
//!
//! # `configure_msd` clamps where the SDK refuses
//!
//! A negative `k` or `c` is **committed as zero** by the simulator and acked
//! `ok`: a spring that quietly vanished, not an error. So
//! [`configure_msd`](vrobots_sdk::VirtualRobot::configure_msd) refuses negatives
//! before they are sent (demonstrated at the end), and for everything else the
//! committed value may still differ from the asked-for one -- the state stream is
//! the only place that says which.
//!
//! Commands latch: the step force below stays applied until the next command
//! replaces it, which is why releasing it takes an explicit `set_msd_force(0.0)`.

use vrobots_sdk::{MsdConfig, PhysicalParams, RobotType, VirtualRobot, VrError};

const MASS_KG: f64 = 1.0; // pinned so the predictions below are arithmetic
const STEP_N: f64 = 20.0;
const HZ: f64 = 25.0; // the state rate
const STEP_SAMPLES: u32 = 125; // ~5 s pushing
const RELEASE_SAMPLES: u32 = 125; // ~5 s ringing back down

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Msd, None)?;
    println!(
        "created sys_id={} ({:?}), service key = {}",
        robot.sys_id(),
        robot.robot_type(),
        vrobots_sdk::topics::srv_msd(robot.sys_id())
    );
    robot.set_physical_params(&PhysicalParams::default().with_mass(MASS_KG))?;

    let s = robot.states();
    println!(
        "frame={:?}  pos=({:+.3},{:+.3},{:+.3}) -- only the first component ever \
         moves\n",
        s.coord_frame_id, s.kin.lin_pos[0], s.kin.lin_pos[1], s.kin.lin_pos[2]
    );

    // ===== the plant as it spawns: k = 20, c = 1 =====
    let soft = step_response(&robot, "as spawned (k=20, c=1)", 20.0, 1.0, false)?;

    // ===== four times stiffer: same push, a quarter of the travel, twice as fast
    let stiff = step_response(&robot, "k=80, c=1", 80.0, 1.0, true)?;

    // ===== and damped, so it stops arguing about it =====
    let damped = step_response(&robot, "k=80, c=16", 80.0, 16.0, true)?;

    println!("\n{STEP_N} N step, {MASS_KG} kg, three plants:");
    println!(
        "  {:<22} {:>9} {:>9} {:>9} {:>9} {:>7}",
        "", "x_final", "F/k", "period", "2pi*sqrt", "zeta"
    );
    for run in [&soft, &stiff, &damped] {
        println!(
            "  {:<22} {:>9.3} {:>9.3} {:>9.2} {:>9.2} {:>7.2}",
            run.label,
            run.settled,
            STEP_N / run.k,
            run.period,
            std::f64::consts::TAU * (MASS_KG / run.k).sqrt(),
            run.c / (2.0 * (run.k * MASS_KG).sqrt())
        );
    }
    println!(
        "Stiffer means smaller and faster; damped means it arrives once instead of \
         four times."
    );

    // ===== the value the SDK will not send =====
    match robot.configure_msd(&MsdConfig::default().with_spring_k(-5.0)) {
        Ok(()) => println!("\nUNEXPECTED: a negative spring constant was accepted"),
        Err(e) => println!("\nspring_k = -5.0 -> [{}] {}", e.code(), e.detail()),
    }
    println!("(the simulator would commit that as 0 and ack `ok`: no spring, no error)");

    robot.delete()?;
    println!("deleted sys_id={}", robot.sys_id());
    Ok(())
}

/// One step-and-release run on a given plant.
struct Response {
    label: String,
    k: f64,
    c: f64,
    settled: f64,
    period: f64,
}

/// Retune the plant, push it with a step, then release and watch it ring down.
fn step_response(
    robot: &VirtualRobot,
    label: &str,
    k: f64,
    c: f64,
    retune: bool,
) -> Result<Response, VrError> {
    println!("-- {label} --");
    if retune {
        robot.configure_msd(&MsdConfig::default().with_spring_k(k).with_damping_c(c))?;
    }
    // Home, at rest, with the force latch cleared -- otherwise the previous run's
    // step is still pushing.
    robot.set_msd_force(0.0)?;
    robot.reset()?;

    let mut crossings = 0u32;
    let mut previous_sign = 0.0f64;
    let mut settled = 0.0;

    for i in 0..STEP_SAMPLES {
        robot.set_msd_force(STEP_N)?;
        let s = robot.states();
        let velocity = s.kin.lin_vel[0];

        // Each velocity sign change is half a cycle. Ignore the crawl either side
        // of zero, or numerical dither counts as oscillation.
        if velocity.abs() > 1e-3 {
            let sign = velocity.signum();
            if previous_sign != 0.0 && sign != previous_sign {
                crossings += 1;
            }
            previous_sign = sign;
        }

        settled = s.actuator.measured.get(1).copied().unwrap_or(0.0);
        if i % 25 == 0 {
            println!(
                "   t={:6.2}s x={:+7.3} m  x'={:+7.3} m/s  disp={:+7.3} m  \
                 net F={:+8.2} N",
                s.elapsed,
                s.kin.lin_pos[0],
                velocity,
                settled,
                s.actuator.measured.first().copied().unwrap_or(0.0)
            );
        }
        robot.rate(HZ);
    }

    // Release. The force LATCHES, so this zero is not optional.
    println!("   release (set_msd_force(0.0)) -- watch it ring back to equilibrium");
    for i in 0..RELEASE_SAMPLES {
        robot.set_msd_force(0.0)?;
        if i % 50 == 0 {
            let s = robot.states();
            println!(
                "   t={:6.2}s x={:+7.3} m  x'={:+7.3} m/s  disp={:+7.3} m",
                s.elapsed,
                s.kin.lin_pos[0],
                s.kin.lin_vel[0],
                s.actuator.measured.get(1).copied().unwrap_or(0.0)
            );
        }
        robot.rate(HZ);
    }

    let seconds = f64::from(STEP_SAMPLES) / HZ;
    Ok(Response {
        label: label.to_string(),
        k,
        c,
        settled,
        period: if crossings > 0 {
            2.0 * seconds / f64::from(crossings)
        } else {
            f64::INFINITY // damped past ringing: no crossings to time
        },
    })
}
