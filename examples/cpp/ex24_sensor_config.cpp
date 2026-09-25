// ex24 -- sensor_config: give the sensors a noise model worth filtering.
//
//     target/cpp-build/Release/ex24_sensor_config
//
// ex10 read the sensor block and told you to mount and tune it through
// `srv/sensors`. This is that call. It is also the one service whose effect you
// can *see* in the state stream rather than infer: degrade the gyro and the
// measured rates roughen while the truth in `kin` stays perfectly smooth.
//
// The robot sits still on the ground for the whole run -- nothing is commanded
// -- so truth is constant and the spread of each sensor reading IS its noise
// realisation.
//
// THE ONE THING THAT WILL BREAK A SENSOR BY ACCIDENT
//
//   `vrsdk_sensor_config_t` gates each block independently: `has_* = false`
//   means the simulator does not touch that setting. **Inside a
//   `vrsdk_imu_noise_t` block there are no flags at all.** The schema has none,
//   and the simulator writes all six vectors unconditionally -- so a field you
//   leave at zero is not "keep the current value", it is **zero**. A block built
//   from `{}` has `scale_factor = {0,0,0}`, which is not a noisy sensor, it is a
//   **dead** one that reads 0.000 forever.
//
//   Which is why `vrsdk::imu_noise()` exists and returns an *ideal* channel:
//   unit gain, no noise, no bias. Never write `vrsdk_imu_noise_t block{};` --
//   start from a whole sensor and spoil exactly what you mean to spoil:
//
//       auto gyro = vrsdk::imu_noise();          // a working gyro
//       gyro.white_std = {0.02, 0.02, 0.02};     // now a realistic one
//
//   `bias_tau_s` is the block's one exception: `<= 0` keeps the simulator's
//   current time constant (100 s), which is why the ideal block leaves it at 0.
//
//   The same rule applies to the whole request: build it with
//   `vrsdk::sensor_config()`, which pre-fills every block with a working
//   default, rather than with `{}`.
//
// REPORTED QUALITY IS NOT APPLIED NOISE
//
//   The two GNSS blocks are independent, and the split is deliberate:
//
//     * `gps_quality` (eph, epv, fix_type) is what the receiver CLAIMS. The
//       simulator always has a perfect fix, so setting `fix_type = 0` does not
//       invalidate the GNSS and raising `eph` does not scatter the position. It
//       feeds your filter's covariance, nothing else -- and it echoes back
//       *exactly*, which makes it the crispest confirmation in this example.
//     * `gps_noise` is the error actually applied, in NED, and unlike the IMU
//       blocks it is not re-expressed from your header frame: a geodetic
//       receiver's error ellipsoid is not a body quantity.
//
// WHAT IS DELIBERATELY NOT HERE
//
//   GNSS home coordinates, the magnetic field vector and sea-level pressure are
//   SCENE truths, not per-robot config -- two robots in one world must not
//   disagree about them. And a block naming a sensor this robot does not carry
//   is dropped with a log line and an `ok` ack, like every other silent refusal.
//
//   Mounting the optical flow is the exception that reports itself:
//   `sensors.optical_flow.valid` goes from false (ex10's "usually means not
//   mounted") to true.

#include <cmath>
#include <cstdio>
#include <string>

#include <vrobots_sdk.hpp>

constexpr int SAMPLES = 100;         // ~4 s at the 25 Hz state rate
constexpr int SETTLE_SAMPLES = 50;   // ~2 s to land and stop moving before measuring
constexpr double SAMPLE_TIMEOUT = 0.5;

constexpr double GYRO_WHITE = 0.02;    // rad/s, per axis
constexpr double ACCEL_WHITE = 0.4;    // m/s^2, per axis
constexpr double BARO_WHITE_PA = 25.0; // pascals
constexpr double REPORTED_EPH = 4.5;   // metres -- echoes back exactly

namespace {

/// Per-axis spread of one vector channel, accumulated sample by sample.
struct Spread {
    double n = 0.0;
    double sum[3] = {};
    double sum_sq[3] = {};

    void push(const double v[3]) {
        n += 1.0;
        for (int i = 0; i < 3; ++i) {
            sum[i] += v[i];
            sum_sq[i] += v[i] * v[i];
        }
    }

    /// Population standard deviation per axis. With the robot at rest, truth is
    /// constant, so this is the sensor's own noise.
    [[nodiscard]] std::string std_dev() const {
        std::string out = "[";
        for (int i = 0; i < 3; ++i) {
            double sigma = 0.0;
            if (n >= 2.0) {
                const double mean = sum[i] / n;
                sigma = std::sqrt(std::fmax(sum_sq[i] / n - mean * mean, 0.0));
            }
            char buffer[32];
            std::snprintf(buffer, sizeof buffer, "%s%.3f", i ? "," : "", sigma);
            out += buffer;
        }
        return out + "]";
    }
};

/// The same, for a scalar channel.
struct ScalarSpread {
    double n = 0.0;
    double sum = 0.0;
    double sum_sq = 0.0;

    void push(double v) {
        n += 1.0;
        sum += v;
        sum_sq += v * v;
    }

    [[nodiscard]] double std_dev() const {
        if (n < 2.0) {
            return 0.0;
        }
        const double mean = sum / n;
        return std::sqrt(std::fmax(sum_sq / n - mean * mean, 0.0));
    }
};

/// One measurement window.
struct Window {
    Spread gyro;
    Spread accel;
    ScalarSpread baro;
    double eph = 0.0;
    bool flow_valid = false;

    void print(const char* label) const {
        std::printf(
            "%-11s gyro sigma=%s rad/s  accel sigma=%s m/s^2  baro sigma=%.2f Pa  eph=%.2f m  "
            "flow_valid=%s\n",
            label, gyro.std_dev().c_str(), accel.std_dev().c_str(), baro.std_dev(), eph,
            flow_valid ? "true" : "false");
    }
};

/// Collect one window of samples, one control iteration per state message.
Window measure(vrsdk::VirtualRobot& robot) {
    Window w;
    vrsdk::State s;
    for (int i = 0; i < SAMPLES; ++i) {
        // One sample per iteration: states() alone would count the same snapshot
        // several times and understate the spread.
        robot.wait_new_state(SAMPLE_TIMEOUT);
        s = robot.states();
        w.gyro.push(s.sensors().gyroscope.angular_velocity);
        w.accel.push(s.sensors().accelerometer.linear_acceleration);
        w.baro.push(s.sensors().barometer.pressure);
    }
    w.eph = s.sensors().gnss.eph;
    w.flow_valid = s.sensors().optical_flow.valid;
    return w;
}

}  // namespace

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot = vrsdk::VirtualRobot::create(vrsdk::RobotType::Multirotor);
        robot.connect();
        std::printf("created sys_id=%u, service key = vrobots/%u/z/srv/sensors\n", robot.sys_id(),
                    robot.sys_id());
        std::printf("Nothing is commanded: the robot rests, so the spread IS the noise.\n\n");

        // A fresh spawn latches 1100 us -- idle, which is not enough to hold it
        // up -- so give it a moment to arrive and stop moving before measuring.
        // A transient in the window would be counted as sensor noise.
        for (int i = 0; i < SETTLE_SAMPLES; ++i) {
            robot.rate(25.0);
        }

        // ===== before =====
        const Window before = measure(robot);
        before.print("as spawned");

        // ===== configure =====
        // Every block starts from an IDEAL sensor -- see the header. The gyro
        // block below also carries a slow bias, which is what makes an estimator
        // earn its keep.
        auto config = vrsdk::sensor_config();

        config.has_gyro_noise = true;
        config.gyro_noise = vrsdk::imu_noise();
        for (int i = 0; i < 3; ++i) {
            config.gyro_noise.white_std[i] = GYRO_WHITE;
            config.gyro_noise.bias_instability[i] = 0.002;
        }
        config.gyro_noise.bias_tau_s = 60.0;

        config.has_accel_noise = true;
        config.accel_noise = vrsdk::imu_noise();
        for (int i = 0; i < 3; ++i) {
            config.accel_noise.white_std[i] = ACCEL_WHITE;
        }

        config.has_baro_pressure_noise_std = true;
        config.baro_pressure_noise_std = BARO_WHITE_PA;

        config.has_gps_quality = true;
        config.gps_quality = vrsdk::gps_quality();
        config.gps_quality.eph = REPORTED_EPH;
        config.gps_quality.epv = 9.0;

        config.has_gps_noise = true;
        config.gps_noise = vrsdk::gps_noise();
        config.gps_noise.position_std[0] = 2.0;  // NED metres
        config.gps_noise.position_std[1] = 2.0;
        config.gps_noise.position_std[2] = 3.0;
        config.gps_noise.velocity_std[0] = 0.2;  // NED m/s
        config.gps_noise.velocity_std[1] = 0.2;
        config.gps_noise.velocity_std[2] = 0.3;

        config.has_optical_flow_mounted = true;
        config.optical_flow_mounted = true;
        config.has_optical_flow_noise_std = true;
        for (int i = 0; i < 3; ++i) {
            config.optical_flow_noise_std[i] = 0.05;
        }

        robot.configure_sensors(config);
        std::printf("\nconfigure_sensors acked -- live from the next sensor sample.\n\n");

        // ===== after =====
        const Window after = measure(robot);
        after.print("configured");

        std::printf("\ngyro  white-noise floor %s -> %s rad/s (asked for %.3f per axis)\n",
                    before.gyro.std_dev().c_str(), after.gyro.std_dev().c_str(), GYRO_WHITE);
        std::printf("accel white-noise floor %s -> %s m/s^2 (asked for %.3f per axis)\n",
                    before.accel.std_dev().c_str(), after.accel.std_dev().c_str(), ACCEL_WHITE);
        std::printf("baro  pressure spread %.2f -> %.2f Pa (asked for %.1f)\n",
                    before.baro.std_dev(), after.baro.std_dev(), BARO_WHITE_PA);
        std::printf(
            "gnss  reported eph %.2f -> %.2f m -- an exact echo, because reported quality is a "
            "claim, not a model\n",
            before.eph, after.eph);
        std::printf(
            "flow  valid %s -> %s -- mounting the sensor is the one part of this service that "
            "announces itself\n",
            before.flow_valid ? "true" : "false", after.flow_valid ? "true" : "false");

        // ===== and the request that changes nothing =====
        try {
            robot.configure_sensors(vrsdk::sensor_config());
            std::printf("\nUNEXPECTED: an empty config was accepted\n");
        } catch (const vrsdk::Error& e) {
            std::printf("\nempty config -> [%d] %s\n", e.code(), e.what());
        }

        robot.remove();
        std::printf("deleted sys_id=%u\n", robot.sys_id());
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
