// ex26 -- drive_config: retune the truck's drivetrain under itself.
//
//     target/cpp-build/Release/ex26_drive_config
//
// `srv/drive` is the truck's own service -- ask a multirotor for it and the GET
// finds no responder (ex30 makes that the point of an example). Every value is
// read LIVE by the steering servo, the drive motor and the dynamics, so a change
// bites from the next physics step with no rebuild and no dropout.
//
// There is nothing to read back, so this example measures instead: drive the
// same full-left circle four times and compare the steady turn radius,
// `speed / yaw_rate`. A steering limit that halves must roughly double the
// radius, or the request did not land.
//
// WHAT THE SIMULATOR QUIETLY SUBSTITUTES
//
//   None of these reach the ack, which is `ok` either way:
//
//   | field               | out of range            | what you get                    |
//   |---------------------|-------------------------|---------------------------------|
//   | `max_steer_deg`     | anything                | HARD-CLAMPED to 0-60 degrees    |
//   | `no_load_wheel_rpm` | <= 0                    | the default, 200                |
//   | `pwm_band`          | not strictly increasing | the whole band replaced by 1100/1500/1900 |
//   | `drive_mode`        | not 2 or 4              | ignored -- so the SDK refuses it first |
//
//   Run 3 below asks for 90 degrees and gets 60. The only way to know is the
//   circle it drives.
//
// TWO PULSE-WIDTH BANDS, AND THEY ARE NOT THE SAME BAND
//
//   The truck's factory band is 1100 / 1500 / 1900 us with a deadband either
//   side of neutral, while `set_car` validates against the wider 1100-2000 the
//   actuators are specified on. So 1950 is accepted by the SDK and is past full
//   throttle for this truck. `pwm_band` moves all four numbers as one group --
//   there is no way to change only the neutral point.
//
// READING THE DRIVETRAIN IN THE STATE STREAM
//
//   `actuator.pwm` echoes the three channels you sent. `actuator.measured[0..3]`
//   are the four wheel speeds in rad/s (FL, FR, RL, RR) -- an undriven wheel
//   still reports, because the road turns it, so `drive_mode` 2 versus 4 shows
//   up as *which* wheels lead under power, not as two silent channels.
//   `actuator.measured[4]` is the steering servo, and it is the channel that
//   answers `max_steer_deg`.
//
// There is also a selector form of this service (`?mode=2|4`) for clients that
// cannot attach a payload. The SDK can, so it does not use it.

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include <vrobots_sdk.hpp>

constexpr double STEER_US = 1100.0;     // full left
constexpr double THROTTLE_US = 1700.0;  // steady forward
constexpr double BRAKE_US = 1100.0;     // released
constexpr double HZ = 25.0;
constexpr int SPIN_UP_SAMPLES = 75;  // ~3 s to reach a steady circle
constexpr int MEASURE_SAMPLES = 40;  // ~1.6 s averaged

namespace {

/// One steady-state circle.
struct Circle {
    std::string label;
    double speed = 0.0;
    double yaw_rate = 0.0;
    double radius = 0.0;
    double servo = 0.0;
};

/// Reset, drive a full-left circle, and average the steady part of it.
///
/// The truck publishes in "fru", so the third component of a body rate is yaw
/// about UP -- `ang_vel[2]`.
Circle circle(vrsdk::VirtualRobot& robot, const char* label) {
    std::printf("-- %s --\n", label);
    robot.reset();

    for (int i = 0; i < SPIN_UP_SAMPLES; ++i) {
        robot.set_car(STEER_US, THROTTLE_US, BRAKE_US);
        robot.rate(HZ);
    }

    double speed = 0.0;
    double yaw_rate = 0.0;
    double servo = 0.0;
    for (int i = 0; i < MEASURE_SAMPLES; ++i) {
        robot.set_car(STEER_US, THROTTLE_US, BRAKE_US);
        const vrsdk::State s = robot.states();
        const double* v = s.kin().lin_vel;
        speed += std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        yaw_rate += s.kin().ang_vel[2];
        const vrsdk_actuator_t& a = s.actuator();
        servo += a.measured_count > 4 ? a.measured[4] : 0.0;
        if (i == 0) {
            std::printf("   echo=[");
            for (std::uint32_t c = 0; c < a.pwm_count; ++c) {
                std::printf("%s%u", c ? "," : "", a.pwm[c]);
            }
            std::printf("] wheels=[");
            for (std::uint32_t c = 0; c < a.measured_count && c < 4; ++c) {
                std::printf("%s%+7.3f", c ? "," : "", a.measured[c]);
            }
            std::printf("]\n");
        }
        robot.rate(HZ);
    }

    const double n = static_cast<double>(MEASURE_SAMPLES);
    Circle out;
    out.label = label;
    out.speed = speed / n;
    out.yaw_rate = yaw_rate / n;
    out.servo = servo / n;
    out.radius = std::fabs(out.yaw_rate) > 1e-6 ? out.speed / std::fabs(out.yaw_rate)
                                                : std::numeric_limits<double>::infinity();
    return out;
}

/// Print a client-side refusal: code first, then the sim behaviour it prevents.
template <typename Call> void show_refusal(const char* what, Call call) {
    try {
        call();
        std::printf("  %-16s UNEXPECTED: accepted\n", what);
    } catch (const vrsdk::Error& e) {
        std::printf("  %-16s [%d] %s\n", what, e.code(), e.what());
    }
}

}  // namespace

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot = vrsdk::VirtualRobot::create(vrsdk::RobotType::Truck);
        robot.connect();
        std::printf("created sys_id=%u, service key = vrobots/%u/z/srv/drive\n\n", robot.sys_id(),
                    robot.sys_id());

        // ===== run 1: as spawned =====
        const Circle stock = circle(robot, "as spawned");

        // ===== run 2: half the steering =====
        {
            auto config = vrsdk::drive_config();
            config.has_max_steer_deg = true;
            config.max_steer_deg = 15.0;
            robot.configure_drive(config);
        }
        const Circle narrow = circle(robot, "max_steer_deg = 15");

        // ===== run 3: more than the simulator allows =====
        {
            auto config = vrsdk::drive_config();
            config.has_max_steer_deg = true;
            config.max_steer_deg = 90.0;
            robot.configure_drive(config);
        }
        const Circle clamped = circle(robot, "max_steer_deg = 90 -> clamped to 60");

        // ===== run 4: rear-wheel drive, softer motor, factory band restated ====
        {
            auto config = vrsdk::drive_config();
            config.has_drive_mode = true;
            config.drive_mode = 2;  // rear axle only (4 = all wheels)
            config.has_max_steer_deg = true;
            config.max_steer_deg = 30.0;
            config.has_steer_rate_dps = true;
            config.steer_rate_dps = 120.0;  // 0 would be an ideal, instant servo
            config.has_max_motor_torque_nm = true;
            config.max_motor_torque_nm = 40.0;
            config.has_no_load_wheel_rpm = true;
            config.no_load_wheel_rpm = 200.0;
            config.has_idle_brake_torque_nm = true;
            config.idle_brake_torque_nm = 5.0;
            config.has_max_brake_torque_nm = true;
            config.max_brake_torque_nm = 150.0;
            // All four numbers move together, and this IS the factory band.
            config.has_pwm_band = true;
            config.pwm_band = vrsdk_pwm_band_t{1100, 1500, 1900, 30};
            robot.configure_drive(config);
        }
        const Circle rwd = circle(robot, "drive_mode = 2, 40 N.m, 30 deg");

        std::printf("\nsteady turn radius (speed / yaw rate), same command every time:\n");
        for (const Circle* run : {&stock, &narrow, &clamped, &rwd}) {
            std::printf("  %-38s r=%6.2f m   speed=%5.2f m/s  yaw=%+6.3f rad/s  servo=%+7.3f\n",
                        run->label.c_str(), run->radius, run->speed, run->yaw_rate, run->servo);
        }
        std::printf(
            "Runs 2 and 3 are the whole lesson: a limit that halves widens the circle, and the 90 "
            "that came back as 60 is indistinguishable from a 60 that was asked for.\n");

        // ===== what the SDK refuses =====
        std::printf("\n-- refused before anything reaches the wire --\n");
        show_refusal("drive_mode = 3", [&] {
            auto config = vrsdk::drive_config();
            config.has_drive_mode = true;
            config.drive_mode = 3;
            robot.configure_drive(config);
        });
        show_refusal("nothing set", [&] { robot.configure_drive(vrsdk::drive_config()); });

        robot.remove();
        std::printf("\ndeleted sys_id=%u\n", robot.sys_id());
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
