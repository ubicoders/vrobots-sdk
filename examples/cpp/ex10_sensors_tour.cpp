// ex10 -- sensors_tour: everything in one state snapshot, printed once a second.
//
//     target/cpp-build/Release/ex10_sensors_tour
//
// The other examples read two or three fields. This one walks the whole
// `vrsdk_state_t`, because the snapshot is organised around a distinction that
// is easy to miss and expensive to get wrong -- **three epistemic categories,
// kept apart on purpose**:
//
//   | block                    | what it is                                   |
//   |--------------------------|----------------------------------------------|
//   | `kin`, `wrench`, `env`   | simulator **truth** -- absent on a real robot |
//   | `sensors`                | the **measured**, noisy, observable view      |
//   | `estimate`               | what the robot's own filter **believes**      |
//
// So `estimate.kin - kin` *is* the estimator error, and `sensors.gyroscope -
// kin.ang_vel` *is* the gyro's noise realisation. Characterising a sensor is
// always a diff between two published blocks; nothing has to be inferred.
//
// Details the printout makes visible:
//
//   * **The accelerometer reads specific force**, not coordinate acceleration:
//     at rest it is +1 g, in free fall 0. Subtracting gravity is your job and
//     needs an attitude.
//   * **Sensors run at their own rates.** Each carries its own `timestamp` and
//     `valid` flag, so a ~5 Hz GNSS fix repeated across state samples is only
//     detectable by its stamp -- watch `gnss.timestamp` sit still while the
//     state `t` advances.
//   * **The barometer's `altitude` drifts with the weather** (it is pressure
//     altitude against `qnh`). `env.agl` LOOKS like the truthful height but is a
//     hard-coded 0 in sim v3.0.0 (the downward raycast it needs ships with the
//     scanning sensors) -- until then, derive height from `kin.lin_pos[2]`
//     (negated: frd counts down).
//   * **Optical flow is optional** and deliberately a poor sensor: `valid` goes
//     false over featureless ground. Robots mount it only if asked
//     (`srv/sensors`), so `valid=false` here usually means "not mounted".
//   * **Magnetic field is in gauss**, not tesla (1 T = 10 000 G).
//   * **Every vector names its frame.** `coord_frame_id` on the snapshot is the
//     robot's, not yours -- "frd" in this scene, so `lin_pos[2]` is DOWN and
//     altitude is its negation.
//   * **`estimate` is only as real as the robot's filter.** The test scene runs
//     none, so it arrives `valid=false` with zero fields and an empty frame id.
//     That is what "no estimator" looks like on the wire, not a decode failure.
//
// In the test scene **sys_id 1 is the multirotor and sys_id 0 is the truck**.

#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr double HZ = 1.0;           // slow: this is a page of text per iteration

/// A 3-vector, aligned so a column of them reads as a column.
static void v3(const char* label, const double* v, const char* unit, const char* note) {
    std::printf("  %-9s (%+8.3f,%+8.3f,%+8.3f) %-8s %s\n", label, v[0], v[1], v[2], unit, note);
}

/// A sensor's own validity and clock -- the two fields that reveal its rate.
static void stamp(bool valid, double timestamp) {
    std::printf("[%s t=%.3f]", valid ? "valid" : "INVALID", timestamp);
}

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();

        // ===== loop =====
        for (;;) {
            const vrsdk::State s = robot.states();
            const vrsdk_state_t& r = s.raw;

            std::printf("\n=== %s sys_id=%u seq=%llu t=%.3fs schema=%u frame=\"%s\" ===\n",
                        s.name.c_str(), s.sys_id, static_cast<unsigned long long>(s.seq),
                        s.elapsed, r.schema_version, s.coord_frame_id.c_str());

            // -- truth --------------------------------------------------------
            std::printf("TRUTH  kinematics\n");
            v3("lin_pos", r.kin.lin_pos, "m", "(world)");
            std::printf("  %-9s [%+.3f,%+.3f,%+.3f,%+.3f] (world, xyzw)\n", "quat", r.kin.quat[0],
                        r.kin.quat[1], r.kin.quat[2], r.kin.quat[3]);
            v3("lin_vel", r.kin.lin_vel, "m/s", "(body)");
            v3("ang_vel", r.kin.ang_vel, "rad/s", "(body -- what a gyro measures)");
            v3("lin_acc", r.kin.lin_acc, "m/s^2", "(body)");
            v3("ang_acc", r.kin.ang_acc, "rad/s^2", "(body)");
            v3("force", r.wrench.force, "N", "(total on the body)");
            v3("torque", r.wrench.torque, "N.m", "");

            // -- measured -----------------------------------------------------
            const vrsdk_sensors_t& n = r.sensors;
            std::printf("MEASURED  sensors\n");
            std::printf("  accel     (%+8.3f,%+8.3f,%+8.3f) m/s^2  ",
                        n.accelerometer.linear_acceleration[0],
                        n.accelerometer.linear_acceleration[1],
                        n.accelerometer.linear_acceleration[2]);
            stamp(n.accelerometer.valid, n.accelerometer.timestamp);
            std::printf("   [specific force: +1 g at rest]\n");

            std::printf("  gyro      (%+8.3f,%+8.3f,%+8.3f) rad/s  ", n.gyroscope.angular_velocity[0],
                        n.gyroscope.angular_velocity[1], n.gyroscope.angular_velocity[2]);
            stamp(n.gyroscope.valid, n.gyroscope.timestamp);
            std::printf("\n");

            std::printf("  mag       (%+8.3f,%+8.3f,%+8.3f) gauss  ", n.magnetometer.magnetic_field[0],
                        n.magnetometer.magnetic_field[1], n.magnetometer.magnetic_field[2]);
            stamp(n.magnetometer.valid, n.magnetometer.timestamp);
            std::printf("\n");

            std::printf("  baro      %.1f Pa  alt=%.2f m (qnh %.1f hPa)  ", n.barometer.pressure,
                        n.barometer.altitude, n.barometer.qnh);
            stamp(n.barometer.valid, n.barometer.timestamp);
            std::printf("\n");

            std::printf("  gnss      lat=%.6f lon=%.6f alt=%.2f m  vel=(%+.3f,%+.3f,%+.3f) m/s (NED)\n",
                        n.gnss.geo_point.latitude, n.gnss.geo_point.longitude,
                        n.gnss.geo_point.altitude, n.gnss.velocity[0], n.gnss.velocity[1],
                        n.gnss.velocity[2]);
            std::printf("            fix=%u eph=%.2f epv=%.2f m  ", n.gnss.fix_type, n.gnss.eph,
                        n.gnss.epv);
            stamp(n.gnss.valid, n.gnss.timestamp);
            std::printf("   [slowest device, ~5 Hz]\n");

            std::printf("  flow      (%+8.3f,%+8.3f,%+8.3f) m/s  ", n.optical_flow.velocity[0],
                        n.optical_flow.velocity[1], n.optical_flow.velocity[2]);
            stamp(n.optical_flow.valid, n.optical_flow.timestamp);
            std::printf("   [optional; mount it via srv/sensors]\n");

            // -- believed -----------------------------------------------------
            std::printf("BELIEVED  estimate  ");
            stamp(r.estimate.valid, r.estimate.timestamp);
            // The fixed C char arrays are NUL-terminated; the precision bounds
            // the read even if a future field ever fills the buffer exactly.
            std::printf("  frame=\"%.*s\"\n",
                        static_cast<int>(sizeof r.estimate.coord_frame_id - 1),
                        r.estimate.coord_frame_id);
            v3("lin_pos", r.estimate.kin.lin_pos, "m", "(estimate.kin - kin IS the error)");
            v3("lin_vel", r.estimate.kin.lin_vel, "m/s", "");

            // -- environment and actuators -------------------------------------
            std::printf("WORLD  environment\n");
            std::printf("  gravity   (%+.3f,%+.3f,%+.3f) m/s^2   air %.1f Pa %.3f kg/m^3 %.1f C\n",
                        r.env.gravity[0], r.env.gravity[1], r.env.gravity[2], r.env.air_pressure,
                        r.env.air_density, r.env.temperature);
            std::printf(
                "  agl       %.2f m    home lat=%.6f lon=%.6f   [agl is hard-coded 0 in sim "
                "v3.0.0 -- use -lin_pos[2]]\n",
                r.env.agl, r.env.geo_point.latitude, r.env.geo_point.longitude);

            std::printf("ACTUATORS  command in, motion out\n");
            std::printf("  pwm        [");
            for (std::uint32_t i = 0; i < r.actuator.pwm_count; ++i) {
                std::printf(i ? ",%u" : "%u", r.actuator.pwm[i]);
            }
            std::printf("] us   (echo of the last command)\n");
            std::printf("  normalized [");
            for (std::uint32_t i = 0; i < r.actuator.normalized_count; ++i) {
                std::printf(i ? ",%.3f" : "%.3f", r.actuator.normalized[i]);
            }
            std::printf("]\n  measured   [");
            for (std::uint32_t i = 0; i < r.actuator.measured_count; ++i) {
                std::printf(i ? ",%.3f" : "%.3f", r.actuator.measured[i]);
            }
            std::printf("]   (rotor rad/s -- what the devices did)\n");

            robot.rate(HZ);
        }
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
