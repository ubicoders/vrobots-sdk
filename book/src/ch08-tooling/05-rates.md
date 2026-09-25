# Measuring rates

You watch one topic for a window and get its whole arrival distribution, which is the part an average hides.

```sh
cargo run -p vrobots-sdk --bin vrobots -- topic hz vrobots/1/z/state -w 5
```

When a control loop stutters, the mean rate is almost always fine. The two numbers
that explain it are the maximum interval and the gap count, and this page is about
reading them.

## The calls

Both take one exact key and one window, and both block for the whole window before
returning. From `crates/vrobots-sdk/src/hz.rs`:

```rust
pub fn measure_rate(key: &str, window: Duration) -> VrResult<RateReport>
pub fn measure_rate_with(key: &str, window: Duration, options: &ConnectOptions) -> VrResult<RateReport>
```

All three surfaces carry it: Python as `vrsdk.measure_rate(key, window=5.0)` returning a
`RateReport`, C++ as `vrsdk::measure_rate(key, window_s = 5.0)` returning
`vrsdk::RateReport`, both mirroring the CLI's 5-second default window. `vrobots topic hz`
remains the shell spelling, and Python can still run that CLI in-process through
`vrsdk.cli_main`. Do not confuse these calls with `robot.rate(hz)`, which paces your own
loop and also exists on all three.

Neither prints anything: the result is the `RateReport` below, and formatting it is
what `vrobots topic hz` does with it.

The transport is auto-detected from the key: an `/i/` segment in the third position
means iceoryx2, and everything else is treated as zenoh. That default is deliberate,
because a zenoh subscribe on a key nobody publishes reports zero samples while an
iceoryx2 open on a service that does not exist blocks until it times out.

A wildcard key and a zero window are both `InvalidArgument`, refused before a session
opens. A wildcard would interleave several `seq` streams and report a gap for every
sample.

Only the zenoh half honours `ConnectOptions::router_endpoint`. Camera streams are
shared memory, so measuring one on a remote simulator is not a thing that can work.

## `RateReport`

Plain owned scalars with no `Option`s, because the C and Python bindings mirror the
struct field for field. "Absent" is spelled as a `have_*` flag beside a zeroed value.

| Field | Type | Units | Notes |
|---|---|---|---|
| `key` | `String` | | The key that was watched. |
| `transport` | `Transport` | | Detected from the key. |
| `window_s` | `f64` | s | How long the subscriber stayed open. |
| `samples` | `u64` | | Payloads received. |
| `span_s` | `f64` | s | First arrival to last arrival. `0.0` with fewer than 2 samples. |
| `hz` | `f64` | Hz | `(samples - 1) / span_s`. `0.0` with fewer than 2 samples. |
| `mean_interval_ms` | `f64` | ms | Mean gap between arrivals. `0.0` with fewer than 2 samples. |
| `min_interval_ms` | `f64` | ms | Shortest gap. |
| `max_interval_ms` | `f64` | ms | Longest gap. The worst stall the loop actually saw. |
| `jitter_ms` | `f64` | ms | Population standard deviation of the gaps. |
| `bytes` | `u64` | B | Total payload bytes received. |
| `have_seq` | `bool` | | Whether any payload carried a `header.seq`. |
| `first_seq`, `last_seq` | `u64` | | First and last seq seen. `0` when `have_seq` is false. |
| `seq_gaps` | `u64` | | How many times seq jumped forward by more than one, i.e. how many separate drop events. |
| `missed` | `u64` | | Total samples missed across all gaps. |
| `seq_restarts` | `u64` | | How many times seq went backwards or repeated. |
| `have_latency` | `bool` | | Whether any payload carried a `header.timestamp_ns`. |
| `mean_latency_ms` | `f64` | ms | Publish stamp to arrival. |
| `min_latency_ms` | `f64` | ms | Negative means the clocks disagree, not that a message arrived before it was sent. |
| `max_latency_ms` | `f64` | ms | |
| `undecodable` | `u64` | | Payloads that did not parse at all. Counted, never fatal. |

Two methods: `mean_bytes()` is `bytes / samples`, and `bytes_per_second()` divides by
`window_s` rather than `span_s`, because a link budget is per wall second.

## The rate is over the span, not the window

`hz` is `(samples - 1) / span_s`, where the span runs from the first arrival to the
last. It is not `samples / window`.

Both parts of that matter. Dividing by the window would count zenoh's discovery
latency, several hundred milliseconds after a fresh session opens during which
nothing can arrive, as time the publisher was silent. And `n` samples give `n - 1`
intervals, so a single arrival has no rate at all: reporting "1 sample in 5 s =
0.2 Hz" would be inventing a period from one event, and the report says "too few to
measure an interval" instead.

The window is still reported beside the span, because the difference between them is
information. A publisher that dies one second into a five second window has a perfect
25 Hz over a 1.000 s span, and only the window tells you the other four seconds were
silence.

> **Note.** `TopicInfo::hz` from [Discovery from code](02-discovery-from-code.md) is
> the other formula, `samples / window`, across every key at once. That is a presence
> check, not a measurement. Use `measure_rate` when the number has to be right.

## Read the distribution, not the average

A control loop at 25 Hz wants a 40 ms period. These two reports have the same mean:

| | Healthy | Stuttering |
|---|---|---|
| `hz` | 25.00 | 24.80 |
| `mean_interval_ms` | 40.00 | 40.32 |
| `min_interval_ms` | 40.00 | 2.10 |
| `max_interval_ms` | 40.00 | 213.00 |
| `jitter_ms` | 0.00 | 31.40 |
| `seq_gaps` | 0 | 3 |
| `missed` | 0 | 11 |

<!-- VERIFY: the stuttering column above is an illustrative shape rather than a captured measurement. -->

The right-hand column is a loop that froze for 213 ms, five times its period, then
received a burst of queued samples 2 ms apart. The mean absorbed both. `max_interval`
is the number that would have broken a controller, and `seq_gaps` with `missed` says
those 11 samples were dropped rather than delayed: zenoh's default reliability is
best effort, and iceoryx2 drops when the subscriber queue is full, so a loop that
reads slower than the publisher writes produces exactly this.

## `seq_restarts` means one of two things

The counter increments when `seq` goes backwards or repeats, which is never a drop.

- On a camera stream it means the stream restarted, which happens when the resolution
  or the pixel format changed mid-window. See
  [Mount, open and unmount](../ch05-cameras/01-mount-open-unmount.md).
- On a zenoh topic it means two publishers are sharing one topic, which is the bug
  this counter exists to catch. Two processes writing the same `sys_id` interleave
  their sequence numbers, and everything downstream of that is wrong.

Restarts are counted separately from gaps on purpose. A restart that read as
thousands of missed samples would make `seq_gaps` useless for what it is for.

## Latency is only sometimes a latency

The three latency fields are arrival wall clock minus the publisher's
`header.timestamp_ns`, which subtracts two different clocks.

Same host, that difference is a real transport-plus-decode delay. Across machines it
is dominated by the offset between the two clocks, it can be tens of milliseconds
with no transport involved at all, and it can be negative when the publisher's clock
runs ahead. A negative `min_latency_ms` is clock skew being reported honestly, not a
message that arrived before it was sent.

The interval statistics do not have this problem. They are measured with a monotonic
`Instant`, so an NTP step in the middle of a window cannot corrupt them.

**Next:** [More than one robot](06-multi-robot.md)

**See also:** [The vrobots command](01-cli.md), [Pacing your loop](../ch03-reading-state/07-pacing.md), [Stream health](../ch03-reading-state/08-health.md)
