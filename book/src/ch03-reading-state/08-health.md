# Stream health

The counters that say whether your loop is really seeing every sample, and how to survive the simulator going away and coming back.

```sh
cargo run -p vrobots-examples --bin ex12_version_info
./target/cpp-build/ex12_version_info
python examples/python/ex12_version_info.py
cargo run -p vrobots-examples --bin ex19_robust_loop
./target/cpp-build/ex19_robust_loop
python examples/python/ex19_robust_loop.py
```

## Why a rate is not enough

Measuring how many samples per second your loop processed answers a weaker question
than it appears to. Twenty-five samples a second with three gaps and twenty samples a
second with none are different experiments, and a rate alone cannot tell them apart: a
publisher that slowed down and a network that dropped every fifth sample produce the
same average.

The SDK counts the difference for you, continuously, from the sequence numbers the
publisher stamps. `stats()` returns a copy of those counters.

| Field | Type | Counts | Notes |
|---|---|---|---|
| `received` | `u64` | samples that arrived and decoded | divide by a wall interval for the effective rate |
| `decode_errors` | `u64` | samples that arrived and did not decode | never fatal; see `last_error` |
| `seq_gaps` | `u64` | how many times `seq` jumped by more than one | the number of drop *events* |
| `missed_samples` | `u64` | total samples implied missing by those jumps | the size of those events |
| `last_seq` | `u64` | `seq` of the most recently decoded sample | |

`seq_gaps` and `missed_samples` answer different questions. Ten gaps of one sample each
is jitter in the transport; one gap of ten samples is something that stopped. Both
report `missed_samples = 10`.

## Decode errors are counted, not raised

A malformed payload does not tear down the session, does not stop the subscriber and
does not raise anywhere in your loop. One bad sample must not end a flight, so it
increments `decode_errors` and the loop keeps running. The error itself is kept for
inspection.

| Method | Returns | Meaning |
|---|---|---|
| `stats()` | `StateStats` | how many, and of what kind |
| `last_error()` | `Option<VrError>` | the error that was counted instead of raised |

`last_error()` returning `None` means every payload so far decoded. A non-empty value
beside a non-zero `decode_errors` is the actual reason, and it is almost always schema
drift between your build and the simulator's.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex12_version_info.rs`:

```rust
let stats = robot.stats();
let last = robot.states();
println!(
    "\nstats after {:.1} s: received={} decode_errors={} seq_gaps={} \
     missed_samples={} last_seq={}",
    elapsed.as_secs_f64(),
    stats.received,
    stats.decode_errors,
    stats.seq_gaps,
    stats.missed_samples,
    stats.last_seq
);
println!(
    "  effective rate {:.1} Hz over the window; sim clock advanced {:.2} s",
    stats.received as f64 / elapsed.as_secs_f64(),
    last.elapsed - first.elapsed
);

// Counted, not raised. This is where a decode failure went.
match robot.last_error() {
    None => println!("  last_error: none -- every payload decoded"),
    Some(e) => println!("  last_error: [{}] {e}", e.code()),
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex12_version_info.cpp`:

```cpp
const vrsdk_state_stats_t st = robot.stats();
const vrsdk::State last = robot.states();
std::printf(
    "\nstats after %.1f s: received=%llu decode_errors=%llu seq_gaps=%llu "
    "missed_samples=%llu last_seq=%llu\n",
    elapsed, static_cast<unsigned long long>(st.received),
    static_cast<unsigned long long>(st.decode_errors),
    static_cast<unsigned long long>(st.seq_gaps),
    static_cast<unsigned long long>(st.missed_samples),
    static_cast<unsigned long long>(st.last_seq));
std::printf("  effective rate %.1f Hz over the window; sim clock advanced %.2f s\n",
            static_cast<double>(st.received) / (elapsed > 0.0 ? elapsed : 1.0),
            last.elapsed - first.elapsed);

// Counted, not thrown. This is where a decode failure went.
if (const std::optional<vrsdk::Error> e = robot.last_error()) {
    std::printf("  last_error: [%d] %s\n", e->code(), e->what());
} else {
    std::printf("  last_error: none -- every payload decoded\n");
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex12_version_info.py`:

```python
st = mr.stats
last = mr.states
print(
    f"\nstats after {elapsed:.1f} s: received={st.received} "
    f"decode_errors={st.decode_errors} seq_gaps={st.seq_gaps} "
    f"missed_samples={st.missed_samples} last_seq={st.last_seq}"
)
print(
    f"  effective rate {st.received / elapsed:.1f} Hz over the window; "
    f"sim clock advanced {last.elapsed - first.elapsed:.2f} s"
)

# Counted, not raised. This is where a decode failure went.
err = mr.last_error
if err is None:
    print("  last_error: none -- every payload decoded")
else:
    print(f"  last_error: [{err.code} {err.kind}] {err.detail}")
```

{{#endtab }}
{{#endtabs }}

The five counters carry the same names everywhere. The absence is spelled three ways for the
same reason: it is an optional, not an error. Rust returns `Option<VrError>`, C++ returns
`std::optional<vrsdk::Error>`, and Python returns `None` or a `VrError` **without raising
it**, which is the whole point of the field. In Python both `stats` and `last_error` are
properties.

On a healthy link the effective rate lands near the stream's 25 Hz, the gap counters
stay at zero, and the last line reports nothing:

<!-- VERIFY: printout reconstructed from the format strings; the values are illustrative, not captured from a run. -->

```text
stats after 2.0 s: received=50 decode_errors=0 seq_gaps=0 missed_samples=0 last_seq=361
  effective rate 25.0 Hz over the window; sim clock advanced 2.00 s
  last_error: none -- every payload decoded
```

The same example prints the simulator's `schema_version` beside the SDK's, which is the
check worth making before you trust any of the numbers above. A version mismatch does
not present as an error; it presents as fields that decode to plausible nonsense.
[Versions and pins](../ch08-tooling/03-version-and-pins.md) covers that comparison.

## Noticing that the simulator stopped

`states()` cannot tell you. It keeps returning the last snapshot it had, unchanged,
forever, and a dead simulator therefore looks exactly like a perfectly stationary robot.
That is the observer contract, and the cost of it is that a stall is not detectable from
a data read alone.

Two things do detect it: `elapsed` (or `seq`) ceasing to advance, and a `Timeout` from
`wait_new_state`. The second is the deliberate one.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex19_robust_loop.rs`:

```rust
Err(VrError::Timeout(_)) => {
    if healthy {
        println!("\nSTALLED: no new state in {TIMEOUT:?}. Not an error -- holding.");
        healthy = false;
        down_since = Some(Instant::now());
    }
    // states() still answers, with the LAST snapshot. Note that
    // `elapsed` is frozen: that, not an exception, is how a dead sim
    // looks from a data read.
    let s = robot.states();
    let down = down_since.map_or(0.0, |t| t.elapsed().as_secs_f64());
    println!(
        "    down {down:5.1}s -- stale snapshot still readable: seq={} \
         t={:.2}s (frozen)",
        s.seq, s.elapsed
    );

    // Publishing into an empty topic is not an error in zenoh, so
    // this keeps returning Ok. A command has no reply; only the echo
    // in the state stream ever proves anything landed.
    robot.set_mr_pwm([PWM_US; 4])?;
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex19_robust_loop.cpp`:

```cpp
if (timed_out) {
    if (healthy) {
        std::printf("\nSTALLED: no new state in %.1fs. Not an error -- holding.\n",
                    TIMEOUT_S);
        healthy = false;
        down_since = std::chrono::steady_clock::now();
    }
    // states() still answers, with the LAST snapshot. Note that
    // `elapsed` is frozen: that, not an exception, is how a dead
    // sim looks from a data read.
    const vrsdk::State s = robot.states();
    const double down =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - down_since)
            .count();
    std::printf(
        "    down %5.1fs -- stale snapshot still readable: seq=%llu t=%.2fs "
        "(frozen)\n",
        down, static_cast<unsigned long long>(s.seq), s.elapsed);

    // Publishing into an empty topic is not an error in zenoh, so
    // this keeps succeeding. A command has no reply; only the echo
    // in the state stream ever proves anything landed.
    robot.set_mr_pwm(hold);
    continue;
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex19_robust_loop.py`:

```python
if healthy:
    print(f"\nSTALLED: no new state in {TIMEOUT}s. Not an error -- holding.")
    healthy = False
    down_since = time.perf_counter()
# `states` still answers, with the LAST snapshot. Note that
# `elapsed` is frozen: that, not an exception, is how a dead sim
# looks from a data read.
s = mr.states
down = time.perf_counter() - down_since
print(
    f"    down {down:5.1f}s -- stale snapshot still readable: "
    f"seq={s.seq} t={s.elapsed:.2f}s (frozen)"
)
# Publishing into an empty topic is not an error in zenoh, so this
# keeps succeeding. A command has no reply; only the echo in the
# state stream ever proves anything landed.
mr.set_mr_pwm([PWM_US] * 4)
continue
```

{{#endtab }}
{{#endtabs }}

Each surface measures the outage with its own monotonic clock: `Instant`, `steady_clock` and
`time.perf_counter`. The SDK does not supply one, because `elapsed` is the simulator's clock
and it is exactly the thing that has stopped.

Close the simulator while that runs and the output changes character without the program
exiting, with `t` frozen at whatever it reached:

<!-- VERIFY: printout reconstructed from the format strings; the values are illustrative, not captured from a run. -->

```text
STALLED: no new state in 500ms. Not an error -- holding.
    down   0.5s -- stale snapshot still readable: seq=146 t=5.84s (frozen)
    down   1.0s -- stale snapshot still readable: seq=146 t=5.84s (frozen)
```

Commands sent during the outage keep returning `Ok`, because publishing to a topic
nobody is subscribed to is not an error in zenoh. Nothing in that path can tell you the
robot is gone, which is the practical reason the actuator echo is the only proof a
command landed.

## Surviving a restart

Start the simulator again and samples resume on the same session. There is no reconnect
logic in the example, because there is nothing to reconnect: the zenoh session, the
subscriber and the command publisher all outlive the simulator's absence, and discovery
is zenoh's job.

The restart is visible in two places, and neither is an error.

| What you see | Why | What not to do |
|---|---|---|
| `seq` restarts from 0 | it is a new publisher | do not count it as thousands of lost samples; the SDK logs `state seq went backwards: the publisher restarted` and leaves `seq_gaps` where it was |
| `elapsed` jumps forward | its epoch is fixed at this handle's first sample and never resets | do not read it as the simulator's run time; reconnect if you need that |

Leaving `seq_gaps` untouched across a restart is a deliberate decision. A counter that
jumped by several thousand every time someone left Play mode would be useless for the
thing it exists for, which is spotting real drops.

> **Gotcha.** A loop that treats `VrError::Timeout` as fatal turns every pause of the
> simulator into a crashed program, and a loop that ignores it entirely spins on stale
> data without noticing. Handle it as a state change: note the transition once, keep
> using the last known snapshot, and log the recovery when samples come back.

**Next:** [A tour of the whole snapshot](09-sensors-tour.md)

**See also:** [Pacing your loop](07-pacing.md), [Versions and pins](../ch08-tooling/03-version-and-pins.md), [Appendix C: Error reference](../appendix-c-errors.md)
