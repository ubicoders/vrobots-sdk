//! ex08 -- generic_cmd: `send_cmd`, the escape hatch for the whole command space.
//!
//! ```text
//! cargo run -p vrobots-examples --bin ex08_generic_cmd
//! ```
//!
//! Every typed wrapper -- `set_mr_pwm`, `set_car`, `set_body_force` -- is one line
//! over `send_cmd(cmd_id, &CmdArgs)`. `Command` is a union by convention: one
//! message, and **`cmd_id` decides which payload fields mean anything**. Fill the
//! ones that command reads, leave the rest empty, and they never reach the wire.
//!
//! This example sends two commands per iteration to the truck, on purpose:
//!
//! 1. `SET_CAR` **by hand** -- `int_arr = [steer_us, throttle_us, brake_us]`, the
//!    exact bytes `set_car()` would have built. The truck implements it, so
//!    `actuator.pwm` echoes the numbers back: **that echo is the proof the escape
//!    hatch really reaches the robot.**
//! 2. `ADD_BODY_FORCE` -- `vec3`, an id no robot type acts on yet, and one with no
//!    typed wrapper at all (there is `set_body_force`, but no *add*). It is
//!    published successfully and silently ignored.
//!
//! Both calls return `Ok`. That is the lesson: a command has no reply, so
//! `Ok(())` means "published", never "acted on". Wrong id, wrong sys_id and wrong
//! array length are all indistinguishable from the client side -- the state
//! stream simply does not change. Check the echo, always.
//!
//! Use it for ids the SDK has no wrapper for; prefer the wrappers where they
//! exist, because they validate (a pulse width outside 1100-2000 is refused
//! before anything is sent, and `send_cmd` will happily publish it).
//!
//! In the test scene **sys_id 0 is the truck and sys_id 1 is the multirotor**.

use vrobots_sdk::{CmdArgs, RobotType, VirtualRobot, VrError, cmd};

const SYS_ID: u32 = 0; // the truck in the test scene
const STEER_US: i32 = 1500; // centre
const THROTTLE_US: i32 = 1600; // light forward
const BRAKE_US: i32 = 1100; // released
const GUST_N: [f64; 3] = [0.0, 0.0, 25.0]; // newtons, in our header frame
const HZ: f64 = 5.0;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Truck, Some(SYS_ID))?;

    // ===== loop =====
    loop {
        // (1) An implemented id, built by hand. CmdArgs::ints fills int_arr,
        //     which is the field SET_CAR reads; everything else stays off the
        //     wire.
        let drive = CmdArgs::ints(&[STEER_US, THROTTLE_US, BRAKE_US]);
        robot.send_cmd(cmd::SET_CAR, &drive)?;

        // (2) An id nothing acts on, whose payload rides vec3. Same call, same
        //     Ok(()), no effect. `cmd::name` turns an id back into its schema
        //     name, which is what makes a log line readable.
        let gust = CmdArgs::default().with_vec3(GUST_N);
        robot.send_cmd(cmd::ADD_BODY_FORCE, &gust)?;

        let s = robot.states();
        println!(
            "sent {}({}) + {}({}) -> echo={:?}",
            cmd::name(cmd::SET_CAR),
            cmd::SET_CAR,
            cmd::name(cmd::ADD_BODY_FORCE),
            cmd::ADD_BODY_FORCE,
            s.actuator.pwm
        );
        println!(
            "      SET_CAR landed (the echo is the receipt); ADD_BODY_FORCE was \
             ignored -- wrench=({:+.2},{:+.2},{:+.2}) N unchanged",
            s.wrench.force[0], s.wrench.force[1], s.wrench.force[2]
        );

        robot.rate(HZ);
    }
}
