# Logging

You turn on the SDK's `tracing` events, choose which targets are loud, and learn which failures are returned to you and which are only ever logged.

```sh
cargo run -p vrobots-examples --bin ex20_logging_tour
./target/cpp-build/ex20_logging_tour
python examples/python/ex20_logging_tour.py
RUST_LOG=vrobots_sdk=debug cargo run -p vrobots-examples --bin ex20_logging_tour
```

If a `connect` hangs, `RUST_LOG=vrobots_sdk=debug` is the answer. The rest of this
page is why that works and what else the setting is good for.

`RUST_LOG` reaches the Rust core, so the other two surfaces turn the volume up in their
own idiom instead. Python routes the same events into the standard `logging` module, so
`vrsdk.init_logging("debug")` (or raising the `vrobots_sdk` logger's level yourself) is
the equivalent. C++ registers a handler with `vrsdk::set_log_callback` and then calls
`vrsdk::set_log_level(vrsdk::LogLevel::Debug)`.

## Turning it on

One function, and the only thing about it worth memorising is that it can decline to
act. From `crates/vrobots-sdk/src/lib.rs`:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
pub fn init_logging(filter: &str)
```

{{#endtab }}
{{#tab name="C++" }}

`cpp/include/vrobots_sdk.hpp`:

```cpp
using LogHandler = void (*)(LogLevel level, const char* target, const char* message);

inline void set_log_callback(LogHandler handler)

inline void set_log_level(LogLevel level)
```

{{#endtab }}
{{#tab name="Python" }}

`crates/vrobots-sdk-py/python/vrsdk/__init__.py`:

```python
def init_logging(
    level: _Union[str, int] = "info",
    *,
    format: str = _LOG_FORMAT,  # noqa: A002 - mirrors logging.basicConfig
) -> None:
```

{{#endtab }}
{{#endtabs }}

This is where the three surfaces diverge most. Rust installs a `tracing` subscriber.
Python has no subscriber to install: importing `vrsdk` already bridges the core's events
into the standard `logging` module, so `init_logging` is `logging.basicConfig` plus
raising the `vrobots_sdk` logger's level, and an application that configures `logging`
itself can skip it entirely. C++ has no logging framework to plug into, so you register a
function pointer with `set_log_callback` and set the floor with `set_log_level`;
registration is process-wide and `nullptr` unregisters.

It returns nothing and cannot fail. It installs a `tracing_subscriber` fmt layer with an `EnvFilter`, and **does nothing
if a subscriber is already installed**. That matters twice: a library must never
force a global subscriber on its consumer, and calling it a second time from
somewhere else in your program cannot fight the first call.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex20_logging_tour.rs`:

```rust
    vrobots_sdk::init_logging(FILTER);
    println!("log filter: {FILTER:?} (RUST_LOG overrides it)\n");

    // Calling it twice is harmless: the second call finds a subscriber already
    // installed and returns. Same reason it is safe to call from a library
    // consumer's main.
    vrobots_sdk::init_logging("error");
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex20_logging_tour.cpp`:

```cpp
        // Register BEFORE connecting: connect is the noisiest and most
        // diagnostic moment, and a hang with no log is the hardest thing to
        // debug.
        vrsdk::set_log_callback(on_log);
        vrsdk::set_log_level(vrsdk::LogLevel::Debug);
        std::printf("log callback registered at level %s\n\n",
                    vrsdk::to_string(vrsdk::LogLevel::Debug));
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex20_logging_tour.py`:

```python
    for name in QUIET:
        logging.getLogger(name).setLevel(logging.WARNING)

    # (2) Then the one-liner. It is logging.basicConfig(level=...) plus raising
    #     the vrobots_sdk logger (the part people forget) plus dropping the
    #     extension's cache of what each Python logger accepts. Calling it
    #     *after* the levels above is what makes them take effect immediately.
    vrsdk.init_logging(LEVEL)
    print(f"init_logging({LEVEL!r}) -- SDK records now reach the root handler\n")
```

{{#endtab }}
{{#endtabs }}

Only Rust's call is idempotent by declining to act. C++ and Python are both last-write-wins:
a second `set_log_callback` replaces the handler, and a second `init_logging` re-raises the
level. Both examples configure logging before `connect`, which is the noisiest and most
diagnostic moment.

The second call has no effect at all, and the filter stays at the first one.

Three ways to decide where events go:

| Way | When to use it |
|---|---|
| `init_logging("info")` | Binaries and examples. One line, and it yields to anything already installed. |
| `RUST_LOG` in the environment | Changing the volume of a program you do not want to edit. It overrides the argument entirely. |
| Your own subscriber, installed before the SDK is used | Production. Skip `init_logging` and the events flow into whatever you built: JSON, OpenTelemetry, a file. |

## The filters worth knowing

The filter syntax is `RUST_LOG`'s, per target, comma separated.

| Setting | Use |
|---|---|
| `vrobots_sdk=debug` | A `connect` that hangs. Shows the session opening, the probe and the wait for the first sample. |
| `vrobots_sdk=debug,zenoh=info` | Adds zenoh's own view of discovery and peering. `zenoh=debug` is a firehose. |
| `vrobots_sdk=trace` | Every publish, one line per command. |
| `off` | Silence. |
| `warn` | The `vrobots` CLI's default. |

## iceoryx2 does not go through `tracing`

iceoryx2 has its own logger, it writes to **stderr**, and no `RUST_LOG` setting
touches it. The SDK turns it down to `Error` when it creates its node, because
iceoryx2 is chatty about conditions the SDK already handles: a service that is not
published yet, a stale registry record.

`IOX2_LOG_LEVEL` is the environment variable that turns it back up, and the SDK
honours it rather than overriding it. Set it when a camera stream will not pair and
you need to see the version and QoS negotiation that failed.

> **Gotcha.** `RUST_LOG=trace` produces nothing at all from iceoryx2, and
> `IOX2_LOG_LEVEL` produces nothing from the SDK. They are two separate systems
> writing to two separate places. Set both when debugging a camera.

## Two channels, and they mean different things

The distinction the SDK draws is between what you asked for and what it did on your
behalf.

| Channel | Carries | How you see it |
|---|---|---|
| Returned errors | Things you did or asked for that could not be done: a pulse width outside the band, a camera that does not exist, a service with no responder. | A `Result<_, VrError>` value. Never only in a log. |
| `tracing` events | Things the SDK did for you: opening a session, waiting for a first sample, retrying a service, dropping a malformed payload, reconfiguring a camera that was already mounted. | Log lines, when a subscriber is installed. |

Nothing in the SDK waits, retries or drops silently, and the events are where that
shows up. Ignore them and a hang has no explanation.

The example demonstrates the first channel with an argument that is refused rather
than clamped:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    match robot.set_mr_pwm([0.7; 4]) {
        // 0.7 is a normalised throttle, not a pulse width. The SDK refuses it
        // before publishing rather than clamping, because a clamped 0.7 would
        // look like a valid idle command.
        Ok(()) => println!("   unexpected: 0.7 us was accepted"),
        Err(e) => println!("   [{}] {} -- {}", e.code(), e.kind(), e.detail()),
    }
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex20_logging_tour.cpp`:

```cpp
        std::printf("\n-- an error is thrown, not logged:\n");
        try {
            // 0.7 is a normalised throttle, not a pulse width. The SDK refuses
            // it before publishing rather than clamping, because a clamped 0.7
            // would look like a valid idle command.
            robot.set_mr_pwm({0.7, 0.7, 0.7, 0.7});
            std::printf("   unexpected: 0.7 us was accepted\n");
        } catch (const vrsdk::Error& e) {
            std::printf("   [%d] %s -- %s\n", e.code(), e.name(), e.what());
        }
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex20_logging_tour.py`:

```python
    print("\n-- an error is raised, not logged:")
    try:
        # 0.7 is a normalised throttle, not a pulse width. The SDK refuses it
        # before publishing rather than clamping, because a clamped 0.7 would
        # look like a valid idle command.
        mr.set_mr_pwm(0.7, 0.7, 0.7, 0.7)
        print("   unexpected: 0.7 us was accepted")
    except vrsdk.VrError as e:
        print(f"   [{e.code} {e.kind}] {e.detail}")
        print(f"   err.name({e.code}) == {vrsdk.err.name(e.code)!r}")
```

{{#endtab }}
{{#endtabs }}

The channel is the same; the delivery is not. Rust returns the refusal as a `Result` you
match on, while C++ throws `vrsdk::Error` and Python raises `vrsdk.VrError`, so both wrap
the call in a `try`. The four pulse widths are an array in Rust and C++ and four positional
arguments in Python.

```text
-- an error is returned, not logged:
   [2] invalid_argument -- pwm[0] = 0.7 is outside the 1100-2000 us pulse-width band (neutral is 1500; values look like microseconds, not normalised units)
```

Nothing about that appears in the log at any filter level. It is a value, and the
only place a value can go is your `match`.

## The third case: counted, never raised

A malformed state payload is neither of the two. It is logged as a warning, counted
in `stats()`, stored in `last_error()`, and **not** returned from `states()`, because
one bad frame must not end a flight.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    let stats = robot.stats();
    println!(
        "\n-- counted rather than raised: received={} decode_errors={} last_error={}",
        stats.received,
        stats.decode_errors,
        match robot.last_error() {
            Some(e) => format!("[{}] {e}", e.code()),
            None => "none".to_string(),
        }
    );
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex20_logging_tour.cpp`:

```cpp
        const vrsdk_state_stats_t st = robot.stats();
        const std::optional<vrsdk::Error> err = robot.last_error();
        std::printf("\n-- counted rather than thrown: received=%llu decode_errors=%llu ",
                    static_cast<unsigned long long>(st.received),
                    static_cast<unsigned long long>(st.decode_errors));
        std::printf("last_error=%s\n", err ? err->what() : "none");
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex20_logging_tour.py`:

```python
    st = mr.stats
    err = mr.last_error
    print(
        f"\n-- counted rather than raised: received={st.received} "
        f"decode_errors={st.decode_errors} last_error="
        + ("none" if err is None else f"[{err.code}] {err.detail}")
    )
```

{{#endtab }}
{{#endtabs }}

`stats` and `last_error` are methods in Rust and C++ and properties in Python. The absent
error is `Option` in Rust, `std::optional<vrsdk::Error>` in C++ and `None` in Python, and
in none of the three does reading it raise.

```text
-- counted rather than raised: received=10 decode_errors=0 last_error=none
```

A non-zero `decode_errors` beside a non-empty `last_error()` is almost always schema
drift between this build and the simulator's, which is
[Versions and pins](03-version-and-pins.md).

**Next:** [Measuring rates](05-rates.md)

**See also:** [Stream health](../ch03-reading-state/08-health.md), [Appendix C: Error reference](../appendix-c-errors.md)
