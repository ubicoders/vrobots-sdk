// A sim-less smoke test for the C++ bindings.
//
// Proves the parts that fail silently if the build is wrong, without needing
// Unity: the library links and its exported symbols resolve, the header and the
// library are the same release (the plain-data structs are shared by layout),
// the option initializers agree with the SDK's documented defaults, the error
// path produces the right exception with the right code, and RAII releases
// every handle. `ctest` runs it; CI builds and runs it as the C++ half of the
// bindings check.
//
// What it deliberately does NOT do is talk to a simulator: `list_topics` runs
// with a short window and an empty result is a pass, because "no sim running"
// is the normal state on a build machine.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vrobots_sdk.hpp>

static int failures = 0;

static void check(bool condition, const char* what) {
    std::printf("  %-58s %s\n", what, condition ? "ok" : "FAIL");
    if (!condition) {
        ++failures;
    }
}

/// The log callback has to fire from a C++ function pointer, so count the hits.
static int log_hits = 0;
static std::string last_log;

static void on_log(vrsdk::LogLevel level, const char* target, const char* message) {
    ++log_hits;
    last_log = std::string(vrsdk::to_string(level)) + "|" + target + "|" + message;
}

int main() {
    std::printf("vrobots_sdk C++ smoke test\n\n");

    // -- the library links, and it is the release this header describes -----
    std::printf("version\n");
    check(vrsdk::version() == VROBOTS_SDK_VERSION, "vrsdk_version() == VROBOTS_SDK_VERSION");
    try {
        vrsdk::check_version();
        check(true, "check_version() accepts a matched pair");
    } catch (const vrsdk::Error& e) {
        check(false, e.what());
    }

    const vrsdk::VersionInfo info = vrsdk::version_info();
    check(info.sdk_version == VROBOTS_SDK_VERSION, "version_info().sdk_version matches");
    check(!info.zenoh.empty() && info.zenoh.find('.') != std::string::npos,
          "version_info() carries the zenoh pin");
    check(info.src_id == 122, "version_info().src_id is this SDK's 122");
    std::printf("    msgs=%s schema=%u zenoh=%s iceoryx2=%s flatbuffers=%s\n",
                info.msgs_commit.c_str(), info.schema_version, info.zenoh.c_str(),
                info.iceoryx2.c_str(), info.flatbuffers.c_str());

    // -- the option initializers -------------------------------------------
    std::printf("\noptions\n");
    vrsdk_connect_options_t options;
    vrsdk_options_default(&options);
    check(options.src_id == 122, "vrsdk_options_default sets src_id 122");
    check(options.probe_timeout_s > 0.0 && options.connect_timeout_s > 0.0,
          "vrsdk_options_default sets non-zero timeouts");
    check(options.router_endpoint == nullptr, "vrsdk_options_default leaves the router unset");
    check(options.start_active, "vrsdk_options_default starts a created robot active");

    vrsdk_camera_options_t camera_options;
    vrsdk_camera_options_default(&camera_options);
    check(camera_options.fx == 600.0 && camera_options.fy == 600.0,
          "vrsdk_camera_options_default sets the 600 px focal length");
    check(camera_options.far_clip > camera_options.near_clip,
          "vrsdk_camera_options_default clips are ordered");

    vrsdk_cmd_args_t args;
    vrsdk_cmd_args_default(&args);
    check(args.int_arr == nullptr && args.int_arr_len == 0,
          "vrsdk_cmd_args_default is an empty payload");

    // -- error mapping ------------------------------------------------------
    std::printf("\nerrors\n");
    check(std::string(vrsdk_error_name(VRSDK_ERR_TIMEOUT)) == "timeout",
          "VRSDK_ERR_TIMEOUT is named \"timeout\"");
    check(std::string(vrsdk_error_name(VRSDK_ERR_PANIC)) == "panic",
          "VRSDK_ERR_PANIC is named \"panic\"");

    // A robot handle built with a zeroed options struct must be refused at
    // construction -- before anything blocks -- with an argument error.
    vrsdk_connect_options_t zeroed;
    std::memset(&zeroed, 0, sizeof zeroed);
    try {
        vrsdk::VirtualRobot bad(vrsdk::RobotType::Multirotor, 0, &zeroed);
        check(false, "a zeroed options struct must throw");
    } catch (const vrsdk::Error& e) {
        check(e.code() == VRSDK_ERR_INVALID_ARGUMENT,
              "a zeroed options struct throws VRSDK_ERR_INVALID_ARGUMENT");
        check(std::string(e.name()) == "invalid_argument", "Error::name() is the stable kind");
        check(std::string(e.what()).find("connect_timeout_s") != std::string::npos,
              "the message names the offending field");
    }

    // Reading state before connect() must say why, not hand back zeros that
    // look like a robot sitting at the origin.
    try {
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Truck, 0);
        check(!robot.connected(), "a fresh handle is not connected");
        check(robot.sys_id() == 0, "an attach handle knows its id before connecting");
        (void)robot.states();
        check(false, "states() before connect() must throw");
    } catch (const vrsdk::Error& e) {
        check(e.code() == VRSDK_ERR_SESSION, "states() before connect() throws VRSDK_ERR_SESSION");
        check(std::string(e.what()).find("connect") != std::string::npos,
              "the message names the fix");
    }

    // A Rust panic must surface as a code, never unwind into C++. If the SDK's
    // catch_unwind were ever removed this call would abort the process.
    check(vrsdk_panic_for_test() == VRSDK_ERR_PANIC, "a Rust panic becomes VRSDK_ERR_PANIC");
    check(std::strstr(vrsdk_last_error_message(), "vrsdk_panic_for_test") != nullptr,
          "the panic message survives into vrsdk_last_error_message()");

    // -- the log bridge -----------------------------------------------------
    std::printf("\nlogging\n");
    vrsdk::set_log_callback(on_log);
    vrsdk::set_log_level(vrsdk::LogLevel::Debug);
    // A failing SDK call logs; so does a panic. Either way something must reach
    // the C++ handler now that one is registered.
    (void)vrsdk_panic_for_test();
    check(log_hits > 0, "the log callback receives SDK events");
    if (log_hits > 0) {
        std::printf("    last: %s\n", last_log.c_str());
    }
    vrsdk::set_log_level(vrsdk::LogLevel::Info);
    vrsdk::set_log_callback(nullptr);
    const int hits_before = log_hits;
    (void)vrsdk_panic_for_test();
    check(log_hits == hits_before, "passing nullptr unregisters the callback");

    // -- discovery ----------------------------------------------------------
    // An empty list is a pass: this test must run on a machine with no sim.
    std::printf("\ndiscovery\n");
    try {
        const std::vector<vrsdk::TopicInfo> topics = vrsdk::list_topics(0.4);
        check(true, "list_topics() with a short window returns without throwing");
        std::printf("    %zu topic(s) visible%s\n", topics.size(),
                    topics.empty() ? " (no simulator running -- expected here)" : "");
        for (const vrsdk::TopicInfo& t : topics) {
            std::printf("      [%s] %s\n", t.transport, t.key.c_str());
        }
    } catch (const vrsdk::Error& e) {
        if (e.code() == VRSDK_ERR_SESSION) {
            // A sandboxed CI runner may have no usable network stack for zenoh
            // to open a session on. That is an environment condition, not a
            // binding bug -- report it and carry on.
            std::printf("  %-58s %s\n", "list_topics() skipped: zenoh could not open a session",
                        "skip");
            std::printf("    %s\n", e.what());
        } else {
            check(false, e.what());
        }
    }
    // A non-positive window must be refused rather than silently reporting
    // "(no topics)" for a healthy sim.
    try {
        (void)vrsdk::list_topics(0.0);
        check(false, "a zero-second discovery window must throw");
    } catch (const vrsdk::Error& e) {
        check(e.code() == VRSDK_ERR_INVALID_ARGUMENT,
              "a zero-second window throws VRSDK_ERR_INVALID_ARGUMENT");
    }

    // -- RAII ---------------------------------------------------------------
    std::printf("\nRAII\n");
    {
        vrsdk::VirtualRobot a(vrsdk::RobotType::Multirotor, 1);
        vrsdk::VirtualRobot b = std::move(a);
        check(b.sys_id() == 1, "a moved-to robot keeps the handle");
        try {
            (void)a.sys_id();
            check(false, "a moved-from robot must throw");
        } catch (const vrsdk::Error& e) {
            check(e.code() == VRSDK_ERR_INVALID_HANDLE,
                  "a moved-from robot throws VRSDK_ERR_INVALID_HANDLE");
        }
    }
    check(true, "handles released without a crash at scope exit");

    {
        // An empty CameraStream must behave like a moved-from one, not crash.
        vrsdk::CameraStream empty;
        check(!static_cast<bool>(empty), "a default CameraStream is empty");
        check(!empty.is_running(), "an empty CameraStream is not running");
        try {
            (void)empty.fresh();
            check(false, "fresh() on an empty stream must throw");
        } catch (const vrsdk::Error& e) {
            check(e.code() == VRSDK_ERR_INVALID_HANDLE,
                  "fresh() on an empty stream throws VRSDK_ERR_INVALID_HANDLE");
        }
    }

    std::printf("\n%s\n", failures == 0 ? "all checks passed" : "SMOKE TEST FAILED");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
