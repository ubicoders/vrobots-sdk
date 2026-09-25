// ex04 -- hello_service: create a robot, and learn what "explicit lifecycle"
// means.
//
//     target/cpp-build/Release/ex04_hello_service
//
// Lifecycle and configuration are one-shot request/response, so there is no
// loop here. Three things worth internalising:
//
//   * **Robots outlive the process.** Destroying the handle closes the session
//     and leaves the robot flying; only `remove()` deletes one. This example
//     removes what it created so that running it twice does not litter the
//     scene -- comment the `remove()` out and the robot stays, which is the
//     whole point. (It is spelled `remove()` rather than `delete()` because
//     `delete` is a C++ keyword.)
//   * **Create is the one non-idempotent service.** The SDK sends it exactly
//     once and never retries it, because every retry that reaches the manager
//     reserves another id and spawns another robot.
//   * **The ack is a receipt, not a result.** `connect()` returns only once the
//     new robot's state topic has published, and `remove()` returns only once it
//     has fallen silent. Absence is the proof, both ways.
//
// Comment out the remove, then attach to the id it prints with
// ex01_hello_states (edit SYS_ID) -- that is the create-then-attach lifecycle
// in two commands.

#include <cstdio>
#include <vrobots_sdk.hpp>

int main() {
    try {
        vrsdk::check_version();

        // Create a NEW robot in the sim: `create` means "no sys_id", so the
        // manager assigns one and the reply carries it. (A constructor would be
        // ambiguous with the attach form -- see the header.)
        vrsdk::VirtualRobot robot = vrsdk::VirtualRobot::create(vrsdk::RobotType::Multirotor);
        robot.connect();
        const std::uint32_t sys_id = robot.sys_id();
        std::printf("created sys_id = %u\n", sys_id);

        // The create reply is a receipt; the robot *exists* once its state topic
        // publishes, which connect() already waited for -- so this is real data.
        const vrsdk::State s = robot.states();
        std::printf("first state: t=%.3f seq=%llu name=\"%s\"\n", s.elapsed,
                    static_cast<unsigned long long>(s.seq), s.name.c_str());
        // The C++ surface has no topic-name builder (Rust has
        // `vrobots_sdk::topics`, Python has `vrsdk.topics`), and the shape is
        // fixed by the wire, so compose it here.
        std::printf("its state topic: vrobots/%u/z/state\n", sys_id);

        // Deletion is explicit and never implicit. The manager's ack is only a
        // receipt, so remove() also waits for the robot's state topic to fall
        // silent -- that is the real confirmation.
        robot.remove();
        std::printf("deleted sys_id = %u (removed=%s)\n", sys_id,
                    robot.removed() ? "true" : "false");

        // The handle is spent. Commands do not silently do nothing -- they say
        // why.
        try {
            robot.set_mr_pwm({1500.0, 1500.0, 1500.0, 1500.0});
            std::printf("unexpected: a deleted robot accepted a command\n");
        } catch (const vrsdk::Error& e) {
            std::printf("the handle is spent, as expected: [%d] %s\n", e.code(), e.what());
        }
        return 0;
    } catch (const vrsdk::Error& e) {
        std::fprintf(stderr, "error [%d] %s\n", e.code(), e.what());
        return 1;
    }
}
