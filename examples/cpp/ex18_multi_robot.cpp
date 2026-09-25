// ex18 -- multi_robot: two robots, two handles, one loop.
//
//     target/cpp-build/Release/ex18_multi_robot
//
// "One program is one embedded system bound to one robot" is the shape the SDK
// is built around -- but nothing enforces it. A `VirtualRobot` is just a
// handle: construct as many as you like, each with its own subscriber, its own
// snapshot and its own command topic. Here one process drives the truck while
// watching the multirotor.
//
// What that buys you, and what it costs:
//
//   * **The handles are independent.** Every robot listens only on its own
//     `cmd` topic, so the sys_id in the topic *is* the routing. There is no way
//     for a command addressed to the truck to reach the multirotor -- send the
//     wrong id and the symptom is silence, not a wrong robot moving.
//   * **`rate()` belongs to one handle.** It paces the calling loop, so call it
//     on exactly one of them (below: the truck) and let the other's snapshot be
//     read at that rate. Calling it on both would sleep twice per iteration and
//     halve the loop rate.
//   * **The clocks are shared, the epochs are not.** `t_ns` is sim capture time
//     for both, so it is directly comparable between robots. `elapsed` is
//     measured from *each robot's own first sample*, so the two differ by
//     whenever each `connect()` happened -- compare `t_ns` when relating two
//     robots, never `elapsed`.
//   * **The two robots do not agree about axes**, and this is the trap.
//     Measured live in this scene: the truck publishes "fru" and the multirotor
//     publishes "frd". Same third component, opposite sign -- up for one, down
//     for the other. So the separation computed below is *wrong* in the strict
//     sense, and a program that mixes the two positions without converting has
//     a sign error it will not see. `coord_frame_id` is on every snapshot for
//     exactly this reason; read it rather than assuming, especially in the one
//     kind of program that holds two robots at once.
//
// Two handles is also two zenoh sessions and two subscriber threads. That is
// fine for a handful of robots; a swarm of fifty wants one subscriber on
// `vrobots/*/z/state`, which is a different program. `VirtualRobot` is
// move-only, so a container of them is a `std::vector<vrsdk::VirtualRobot>`
// built with `emplace_back` or moves -- never copies.
//
// In the test scene **sys_id 0 is the truck and sys_id 1 is the multirotor**.

#include <cmath>
#include <cstdio>
#include <vrobots_sdk.hpp>

constexpr std::uint32_t TRUCK_ID = 0;
constexpr std::uint32_t DRONE_ID = 1;
constexpr double STEER_US = 1500.0;     // straight ahead
constexpr double THROTTLE_US = 1650.0;  // light forward
constexpr double HZ = 10.0;

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        // Two connects, two sessions. Each blocks until *its* robot's first
        // state snapshot arrives, so both are live by the time the loop starts.
        vrsdk::VirtualRobot truck(vrsdk::RobotType::Truck, TRUCK_ID);
        truck.connect();
        vrsdk::VirtualRobot drone(vrsdk::RobotType::Multirotor, DRONE_ID);
        drone.connect();
        std::printf("truck sys_id=%u, drone sys_id=%u\n", truck.sys_id(), drone.sys_id());

        // ===== loop =====
        for (;;) {
            // One robot commanded ...
            truck.set_car(STEER_US, THROTTLE_US, 1100.0);
            const vrsdk::State t = truck.states();

            // ... the other only observed. Nothing pairs the two snapshots:
            // they are whatever each subscriber last received.
            const vrsdk::State d = drone.states();

            const double* tp = t.kin().lin_pos;
            const double* dp = d.kin().lin_pos;
            // t_ns is the shared clock -- this difference is real. (elapsed is
            // not: each robot counts from its own first sample.)
            const double skew_ms = static_cast<double>(t.t_ns - d.t_ns) / 1e6;
            const double separation =
                std::sqrt((tp[0] - dp[0]) * (tp[0] - dp[0]) + (tp[1] - dp[1]) * (tp[1] - dp[1]) +
                          (tp[2] - dp[2]) * (tp[2] - dp[2]));

            // Each snapshot names its own frame, and here they differ: the
            // truck is "fru" (third component UP) and the drone is "frd" (third
            // component DOWN). Print the tag beside every position.
            std::printf(
                "truck[%u] pos=(%.2f,%.2f,%.2f) [%s]  |  drone[%u] pos=(%.2f,%.2f,%.2f) [%s] "
                "alt=%.2f m\n",
                t.sys_id, tp[0], tp[1], tp[2], t.coord_frame_id.c_str(), d.sys_id, dp[0], dp[1],
                dp[2], d.coord_frame_id.c_str(),
                -dp[2]);  // "frd": altitude is minus the down component
            std::printf(
                "    naive separation=%.2f m%s  snapshot skew=%+.1f ms  (elapsed: truck %.2fs vs "
                "drone %.2fs -- different epochs)\n",
                separation,
                t.coord_frame_id == d.coord_frame_id ? "" : " (WRONG: mixed frames, convert first)",
                skew_ms, t.elapsed, d.elapsed);

            // Paced once, on one handle.
            truck.rate(HZ);
        }
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
