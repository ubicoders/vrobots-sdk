// ex42 -- lidar: one revolution, read as ranges plus the layout that places them.
//
//     target/cpp-build/Release/ex42_lidar                             # warehouse: forklift 0, pallet robot 1
//     target/cpp-build/Release/ex42_lidar <forklift> <palletrobot>    # other ids
//
// Two lidars, one API. The forklift carries a 3-D unit named `lidar` (VLP-16
// class: 16 rings x 900 columns, 0.4-100 m, 20 Hz) and the pallet robot two 2-D
// units, `lidar_front_left` and `lidar_rear_right` (RPLIDAR A2 class: one ring
// of 360, 0.15-12 m, 10 Hz). They ride different wires --
//
//   |           | 3-D                                     | 2-D                                  |
//   |-----------|-----------------------------------------|--------------------------------------|
//   | open with | `open_lidar_3d(name)`                   | `open_lidar_2d(name)`                |
//   | topic     | `vrobots/{id}/i/lidar/{name}`, iceoryx2 | `vrobots/{id}/z/lidar/{name}`, zenoh |
//   | reaches   | this host only                          | this host or the network             |
//
// -- and decode into the same `LidarScan`, read through the same `LidarStream`,
// so `show()` below takes either. Opening one subscribes and changes nothing in
// the simulator: like a camera (ex13), there is nothing to clean up.
//
// RANGES, NOT POINTS
//
//   The wire carries one range per beam, plus the layout that places it. Beam
//   `ring * columns + column` points along
//
//     azimuth   = azimuth_start   + column * azimuth_step    column 0 forward, counter-clockwise from above
//     elevation = elevation_start + ring   * elevation_step  ring 0 the lowest
//     point     = (r cos(el) cos(az), r cos(el) sin(az), r sin(el))    FLU: x forward, y left, z up
//
//   `+inf` MEANS NO RETURN -- nothing within `max_range` -- never 0, so the
//   nearest-return search below skips infinities, not zeros. `beam_point()` is
//   that formula; `LidarScan::points()` applies it to every beam at once and,
//   under a `srv/frames` override of device `lidar/<name>`, re-expresses the
//   points in that frame, while `beam_point()` stays in FLU. Either way a point
//   is in the LIDAR'S frame, origin at its optical centre: the mount in
//   `params()`, given in the robot's frame, is what takes it onto the body.
//
//   The 2-D units publish their full 360 degrees, so on the pallet robot the
//   inward quarter of each ring looks straight through the robot's own body (its
//   colliders are skipped). Masking that quarter is the consumer's call.
//
// LIDAR IS PRO
//
//   On the Community tier the simulator publishes no lidar topic at all, so the
//   open finds nothing and throws `VRSDK_ERR_TIMEOUT`: after 5 s for the 3-D (a
//   registry lookup, `camera_timeout_s`) and 15 s for the 2-D (zenoh has no
//   registry, so the open waits for a first scan, `probe_timeout_s`). The SDK
//   cannot tell "no such lidar" from "not on this plan" and says both. That is an
//   answer, not a crash: this example prints it and moves on. On Community only
//   the focused robot is on the wire at all, so a robot may not attach either --
//   that too is printed, and its half skipped.

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include <vrobots_sdk.hpp>

constexpr const char* USAGE =
    "The warehouse scene pins its forklift at sys_id 0 and its pallet robot at 1,\n"
    "which are the defaults. For others, pass both ids:\n"
    "\n"
    "    ex42_lidar 0 1\n"
    "\n"
    "The scene has to be loaded and in Play mode. List what is publishing -- the\n"
    "lidars included -- with:\n"
    "\n"
    "    vrobots topic list";

constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double DEG_TO_RAD = PI / 180.0;

constexpr std::uint32_t FORKLIFT_ID = 0;              // the warehouse scene's forklift
constexpr std::uint32_t PALLETROBOT_ID = 1;           // and its pallet robot
constexpr const char* LIDAR_3D = "lidar";             // the forklift's one 3-D unit
constexpr const char* LIDAR_2D = "lidar_front_left";  // the pallet robot's front-left 2-D unit
constexpr double SCAN_TIMEOUT_S = 1.0;                // a 2-D revolution is 100 ms
constexpr int RATE_SCANS = 10;                        // scans timed to measure the rate
constexpr double BEARINGS_DEG[4] = {0.0, 90.0, 180.0, 270.0};  // front, left, back, right

namespace {

/// The point beam `index` hit, metres, in FLU: the wire's own formula.
///
/// `index` is `ring * columns + column`. Only for a finite range: `+inf` is no
/// return, and has no point.
vrsdk::Vec3 beam_point(const vrsdk::LidarScan& scan, std::size_t index) {
    const vrsdk_lidar_params_t& p = scan.params();
    const std::size_t columns = p.columns;
    const double ring = static_cast<double>(index / columns);
    const double column = static_cast<double>(index % columns);
    const double az = p.azimuth_start + column * p.azimuth_step;
    const double el = p.elevation_start + ring * p.elevation_step;
    const double r = scan.ranges[index];
    return {r * std::cos(el) * std::cos(az), r * std::cos(el) * std::sin(az), r * std::sin(el)};
}

/// Attach to a robot for its lidar, or say why not and skip that half.
///
/// No type check here (ex37-ex41 make one): a lidar is opened by name, and a
/// name the robot does not carry is the same clean "not there" as a missing
/// topic.
std::optional<vrsdk::VirtualRobot> attach(vrsdk::RobotType robot_type, std::uint32_t sys_id) {
    try {
        vrsdk::VirtualRobot robot(robot_type, sys_id);
        robot.connect();
        return robot;
    } catch (const vrsdk::Error& e) {
        std::printf("no %s at sys_id %u: [%d] %s\n", vrsdk::to_string(robot_type), sys_id,
                    e.code(), e.what());
        std::printf(
            "(is the warehouse scene loaded and in Play mode? On the Community tier only the "
            "focused robot is on the wire at all.)\n");
        return std::nullopt;
    }
}

/// One scan, taken apart: the layout, the returns, the nearest one, four
/// bearings; then the rate the stream runs at.
void show(vrsdk::LidarStream& lidar) {
    // Scan-paced, like ex13's frames: block until the next revolution lands.
    try {
        lidar.wait_new_scan(SCAN_TIMEOUT_S);
    } catch (const vrsdk::Error& e) {
        std::printf("no scan: [%d] %s\n", e.code(), e.what());
        return;
    }
    const std::optional<vrsdk::LidarScan> scan = lidar.latest();
    if (!scan) {
        return;
    }
    const vrsdk_lidar_params_t& p = scan->params();
    std::printf("%s on %s [%s], scan seq %llu at t=%.2f s\n", scan->name.c_str(),
                lidar.topic().c_str(), lidar.transport(),
                static_cast<unsigned long long>(scan->seq()), scan->elapsed());

    // ===== the layout: where each range points =====
    std::printf("  layout    %u ring(s) x %u columns = %zu ranges, %.0f Hz, segments %u\n", p.rings,
                p.columns, scan->ranges.size(), p.scan_rate, p.segments);
    std::printf(
        "  column c  azimuth %+.1f + c x %.2f deg (0 forward, counter-clockwise from above)\n",
        p.azimuth_start * RAD_TO_DEG, p.azimuth_step * RAD_TO_DEG);
    std::printf("  ring r    elevation %+.1f + r x %.1f deg (ring 0 the lowest)\n",
                p.elevation_start * RAD_TO_DEG, p.elevation_step * RAD_TO_DEG);
    std::printf("  range     %.2f to %.1f m; no return reads +inf, never 0\n", p.min_range,
                p.max_range);
    std::printf(
        "  mount     (%+.2f, %+.2f, %+.2f) m, euler (%+.0f, %+.0f, %+.0f) deg, in \"%.*s\" -- the "
        "robot's frame; the scan's own frame is \"%s\"\n",
        p.mount_position[0], p.mount_position[1], p.mount_position[2],
        p.mount_euler_rad[0] * RAD_TO_DEG, p.mount_euler_rad[1] * RAD_TO_DEG,
        p.mount_euler_rad[2] * RAD_TO_DEG, static_cast<int>(sizeof p.mount_coord_frame_id - 1),
        p.mount_coord_frame_id, scan->coord_frame_id.c_str());

    // ===== the returns, and the nearest one =====
    std::size_t hits = 0;
    std::size_t nearest_index = 0;
    float nearest_range = std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
        const float r = scan->ranges[i];
        if (std::isfinite(r)) {
            ++hits;
            if (r < nearest_range) {
                nearest_range = r;
                nearest_index = i;
            }
        }
    }
    std::printf("  returns   %zu of %zu beams\n", hits, scan->ranges.size());
    if (hits > 0) {
        const std::size_t columns = p.columns;
        const std::size_t ring = nearest_index / columns;
        const std::size_t column = nearest_index % columns;
        const auto [x, y, z] = beam_point(*scan, nearest_index);
        std::printf(
            "  nearest   %.2f m on ring %zu, column %zu (az %+.1f, el %+.1f deg) -> (%+.2f, %+.2f, "
            "%+.2f) m FLU\n",
            nearest_range, ring, column,
            (p.azimuth_start + static_cast<double>(column) * p.azimuth_step) * RAD_TO_DEG,
            (p.elevation_start + static_cast<double>(ring) * p.elevation_step) * RAD_TO_DEG, x, y,
            z);
    }

    // ===== four bearings on the lowest ring =====
    std::printf("  ring 0, elevation %+.1f deg, at four bearings:\n",
                p.elevation_start * RAD_TO_DEG);
    for (const double bearing : BEARINGS_DEG) {
        // The column nearest that azimuth; ring 0 is the first `columns` ranges.
        const long steps = std::lround((bearing * DEG_TO_RAD - p.azimuth_start) / p.azimuth_step);
        const std::size_t column = static_cast<std::size_t>(steps > 0 ? steps : 0) % p.columns;
        const float r = scan->ranges[column];
        if (std::isfinite(r)) {
            const auto [x, y, z] = beam_point(*scan, column);
            std::printf("     az %+6.1f deg  %6.2f m  -> (%+6.2f, %+6.2f, %+6.2f) m FLU\n", bearing,
                        r, x, y, z);
        } else {
            std::printf("     az %+6.1f deg     inf    -> no return\n", bearing);
        }
    }

    // ===== the stream =====
    // seq counts revolutions and t_ns stamps each one's completion on the sim
    // clock, so the rate comes out of the scans, not out of this process's
    // scheduling.
    vrsdk::LidarScan last = *scan;
    for (int i = 0; i < RATE_SCANS; ++i) {
        try {
            lidar.wait_new_scan(SCAN_TIMEOUT_S);
        } catch (const vrsdk::Error& e) {
            std::printf("  the stream stopped: [%d] %s\n", e.code(), e.what());
            return;
        }
        if (std::optional<vrsdk::LidarScan> next = lidar.latest()) {
            last = std::move(*next);
        }
    }
    const std::uint64_t revolutions = last.seq() > scan->seq() ? last.seq() - scan->seq() : 0;
    const double seconds = static_cast<double>(last.t_ns() - scan->t_ns()) * 1e-9;
    const vrsdk_lidar_stats_t stats = lidar.stats();
    std::printf(
        "  stream    %llu more scans in %.2f s of sim time = %.1f Hz; received %llu, seq gaps "
        "%llu, decode errors %llu\n",
        static_cast<unsigned long long>(revolutions), seconds,
        static_cast<double>(revolutions) / seconds, static_cast<unsigned long long>(stats.received),
        static_cast<unsigned long long>(stats.seq_gaps),
        static_cast<unsigned long long>(stats.decode_errors));
}

/// One sys_id argument, or `fallback` when it was not given.
///
/// A sys_id is a `uint32_t` on the wire, so anything that does not fit one is
/// refused rather than wrapped: truncating `4294967297` to `1` would attach to
/// somebody else's robot and look like it worked. `strtoul` reports the overflow
/// through `errno` but wraps a leading '-' silently, so both are checked, and the
/// range check catches the rest on a 64-bit `unsigned long`.
std::uint32_t sys_id_or(const char* arg, std::uint32_t fallback, const char* program) {
    if (arg == nullptr) {
        return fallback;
    }
    char* end = nullptr;
    errno = 0;
    const unsigned long parsed = std::strtoul(arg, &end, 10);
    if (end == arg || *end != '\0' || arg[0] == '-' || errno == ERANGE ||
        parsed > std::numeric_limits<std::uint32_t>::max()) {
        std::fprintf(stderr, "usage: %s [forklift_sys_id] [palletrobot_sys_id]\n\n%s\n", program,
                     USAGE);
        std::exit(2);
    }
    return static_cast<std::uint32_t>(parsed);
}

/// The forklift's and the pallet robot's sys_ids: the two command-line
/// arguments, or `FORKLIFT_ID` and `PALLETROBOT_ID`.
std::pair<std::uint32_t, std::uint32_t> sys_ids_from_args(int argc, char** argv) {
    const char* program = argc > 0 ? argv[0] : "ex42_lidar";
    const std::uint32_t forklift = sys_id_or(argc > 1 ? argv[1] : nullptr, FORKLIFT_ID, program);
    const std::uint32_t palletrobot =
        sys_id_or(argc > 2 ? argv[2] : nullptr, PALLETROBOT_ID, program);
    return {forklift, palletrobot};
}

}  // namespace

int main(int argc, char** argv) {
    try {
        // ===== setup =====
        vrsdk::check_version();
        const auto [forklift_id, palletrobot_id] = sys_ids_from_args(argc, argv);

        // ===== 3-D: the forklift's lidar, on iceoryx2 =====
        std::printf("\n== 3-D: '%s' on sys_id %u ==\n", LIDAR_3D, forklift_id);
        if (std::optional<vrsdk::VirtualRobot> forklift =
                attach(vrsdk::RobotType::Forklift, forklift_id)) {
            vrsdk::LidarStream lidar;
            try {
                lidar = forklift->open_lidar_3d(LIDAR_3D);
            } catch (const vrsdk::Error& e) {
                // Absent: the answer, not a crash. On Community it is the tier.
                if (e.code() == VRSDK_ERR_TIMEOUT) {
                    std::printf("not there: %s\n", e.detail().c_str());
                } else {
                    std::printf("could not open it: [%d] %s\n", e.code(), e.what());
                }
            }
            if (lidar) {
                show(lidar);
            }
        }

        // ===== 2-D: the pallet robot's front-left lidar, on zenoh =====
        std::printf("\n== 2-D: '%s' on sys_id %u ==\n", LIDAR_2D, palletrobot_id);
        if (std::optional<vrsdk::VirtualRobot> palletrobot =
                attach(vrsdk::RobotType::PalletRobot, palletrobot_id)) {
            // Nothing reads back the options a handle runs with; this one was
            // built with the defaults, so the default is the one in effect.
            vrsdk_connect_options_t options{};
            vrsdk_options_default(&options);
            std::printf("zenoh has no registry: the open waits up to %gs for a first scan\n",
                        options.probe_timeout_s);
            vrsdk::LidarStream lidar;
            try {
                lidar = palletrobot->open_lidar_2d(LIDAR_2D);
            } catch (const vrsdk::Error& e) {
                if (e.code() == VRSDK_ERR_TIMEOUT) {
                    std::printf("not there: %s\n", e.detail().c_str());
                } else {
                    std::printf("could not open it: [%d] %s\n", e.code(), e.what());
                }
            }
            if (lidar) {
                show(lidar);
            }
        }

        // Each stream ends with its handle; both lidars keep scanning for everyone else.
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
