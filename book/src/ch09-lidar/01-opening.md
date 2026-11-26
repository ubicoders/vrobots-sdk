# Opening a lidar stream

Two open calls, three ways to read, and what a failed open is telling you.

```sh
cargo run -p vrobots-examples --bin ex42_lidar
./target/cpp-build/ex42_lidar
python examples/python/ex42_lidar.py
```

## Opening changes nothing

`open_lidar_3d` and `open_lidar_2d` subscribe, and that is all they do. The simulator is not
told, the lidar is not reconfigured, and there is nothing to clean up afterwards. Two
programs can open the same lidar and neither disturbs the other.

The only argument is the lidar's name on the robot: `lidar` for a 3-D unit,
`lidar_front_left` or `lidar_rear_right` for the pallet robot's 2-D units. A name is one
segment of the topic, so the SDK refuses an empty name, and any name with a character other
than a letter, a digit, `_` or `-`, before anything is sent. One stream reads one lidar, and
there is no wildcard form.

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`crates/vrobots-sdk/src/lidar.rs`:

```rust
pub fn open_lidar_3d(&self, name: &str) -> VrResult<LidarStream>
pub fn open_lidar_2d(&self, name: &str) -> VrResult<LidarStream>
```

{{#endtab }}
{{#tab name="C++" }}

`include/vrobots_sdk.hpp`:

```cpp
[[nodiscard]] LidarStream open_lidar_3d(const std::string& name)
[[nodiscard]] LidarStream open_lidar_2d(const std::string& name)
```

{{#endtab }}
{{#tab name="Python" }}

`vrsdk/_vrsdk.pyi`:

```python
def open_lidar_3d(self, name: str) -> LidarStream: ...
def open_lidar_2d(self, name: str) -> LidarStream: ...
```

{{#endtab }}
{{#endtabs }}

Both return the same stream type. Which call you make decides the transport, and
`transport` on the stream reports it afterwards: `"i"` for iceoryx2, `"z"` for zenoh. In
Rust `transport()` returns the `Transport` enum, and `transport().tag()` is that letter.

## A 3-D stream and a 2-D stream

The example attaches to the Warehouse forklift for its 3-D unit and to the pallet robot for
one of its 2-D units, and treats a missing topic as an answer to print rather than a failure
to propagate. Look at the branch on the timeout code.

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex42_lidar.rs`:

```rust
match forklift.open_lidar_3d(LIDAR_3D) {
    Ok(lidar) => show(&lidar),
    // Absent: the answer, not a crash. On Community it is the tier.
    Err(VrError::Timeout(detail)) => println!("not there: {detail}"),
    Err(e) => println!("could not open it: [{}] {e}", e.code()),
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex42_lidar.cpp`:

```cpp
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
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex42_lidar.py`:

```python
try:
    lidar = forklift.open_lidar_3d(LIDAR_3D)
except VrError as e:
    # Absent: the answer, not a crash. On Community it is the tier.
    if e.code == err.TIMEOUT:
        print(f"not there: {e.detail}")
    else:
        print(f"could not open it: [{e.code}] {e}")
else:
    show(lidar)
```

{{#endtab }}
{{#endtabs }}

The 2-D open has the same shape, with `open_lidar_2d(LIDAR_2D)` on the pallet robot's
handle. With `LIDAR_3D = "lidar"` and `LIDAR_2D = "lidar_front_left"`, and the Warehouse
scene's forklift at sys id 0 and pallet robot at sys id 1, the two streams report these
topics, which are what `vrobots topic list` shows for them:

```text
lidar on vrobots/0/i/lidar/lidar [i], scan seq 7278 at t=0.08 s
lidar_front_left on vrobots/1/z/lidar/lidar_front_left [z], scan seq 3647 at t=0.22 s
```

## Two open timeouts with two meanings

The two calls wait for different things, because the two transports can answer different
questions.

| | `open_lidar_3d` | `open_lidar_2d` |
|---|---|---|
| Waits for | the topic to be found | the first scan to arrive |
| Why | iceoryx2 has a registry, so a topic either exists or does not | zenoh has no registry, so a first scan is the only proof the topic exists |
| Budget | the camera timeout | the probe timeout |
| Default | 5 s | 15 s |
| Rust option | `ConnectOptions::camera_timeout` | `ConnectOptions::probe_timeout` |
| Python option | `camera_timeout` | `probe_timeout` |
| C++ option | `camera_timeout_s` | `probe_timeout_s` |
| On return | the stream may not hold a scan yet | the stream already holds one |

The consequence shows in the first read. After `open_lidar_2d` returns, `latest` has a scan.
After `open_lidar_3d` returns, wait for one with `wait_new_scan` before you read.

The 2-D budget is the long one because it includes zenoh discovery, which takes several
seconds after a simulator starts. On the Community plan that is fifteen seconds of waiting
for a topic that does not exist, so shorten `probe_timeout` in a program that expects to run
there. Measured on Community with the defaults, the 3-D open failed about 5 s after the call
and the 2-D open about 15 s after it.

## The missing topic error

A lidar topic that is not found is a timeout on every surface: `VrError::Timeout` in Rust,
`VRSDK_ERR_TIMEOUT` in C++ and `vrsdk.err.TIMEOUT` in Python. The message names the topic,
the budget, and every cause the SDK cannot rule out, the plan first:

```text
not there: lidar topic "vrobots/1/z/lidar/lidar_front_left" not found within 15s. Lidar needs the Pro tier: on Community the simulator publishes no lidar topic at all, so a plan without lidar and a lidar that does not exist look the same from here, and the SDK cannot tell which this is. Otherwise check the lidar name, that the sim is in Play mode, and the sys id. `vrobots topic list` shows the lidars that do exist
```

For a 3-D lidar the last check reads "that the sim runs on this host", since a simulator on
another machine presents as the same absence. A 2-D open has one more failure of its own:
if samples arrived on the topic and none of them decoded as a scan, the error is a decode
error rather than a timeout, because the topic is there and the plan is not the problem.

## Three ways to read

| Rust and C++ | Python | Returns | Blocks | Consumes freshness |
|---|---|---|---|---|
| `fresh()` | `read()` | the latest scan, only if it is new since the last call | no | yes |
| `latest()` | `latest` | the latest scan, new or not; nothing before the first one | no | no |
| `wait_new_scan(timeout)` | `wait_new_scan(timeout)` | nothing; returns when a newer scan has landed | yes, up to `timeout` | no |

Each scan is handed out once by `fresh`, so a loop that polls at 100 Hz against a 20 Hz
lidar gets a scan about 20 times a second and nothing the rest of the time. Python splits
the question from the answer, as it does for cameras: `lidar.fresh` is a boolean property
that asks, and `lidar.read()` returns the scan. Rust and C++ can ask the same question
with `has_fresh()`, which says whether a scan that `fresh()` has not handed out yet is
waiting. It claims nothing and copies nothing, so the scan is still there for the
`fresh()` that follows. `wait_new_scan` takes seconds in C++ and
Python, defaulting to 0.5 and 1.0, and a `Duration` in Rust.

The example paces on scans rather than on the clock, and reads with `latest`:

{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex42_lidar.rs`:

```rust
// Scan-paced, like ex13's frames: block until the next revolution lands.
if let Err(e) = lidar.wait_new_scan(SCAN_TIMEOUT) {
    println!("no scan: [{}] {e}", e.code());
    return;
}
let Some(scan) = lidar.latest() else {
    return;
};
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex42_lidar.cpp`:

```cpp
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
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex42_lidar.py`:

```python
# Scan-paced, like ex13's frames: block until the next revolution lands.
try:
    lidar.wait_new_scan(SCAN_TIMEOUT)
except VrError as e:
    print(f"no scan: [{e.code}] {e}")
    return
scan = lidar.latest
if scan is None:
    return
```

{{#endtab }}
{{#endtabs }}

That fragment prints nothing while scans arrive. When none arrives in time, the timeout
names the topic and the deadline. In Rust the error reads:

```text
timeout: no lidar scan on vrobots/0/i/lidar/lidar within 1s
```

C++ and Python carry the same text behind the numeric code the example prints in brackets.

<!-- VERIFY: the exact spelling of the deadline in the C++ and Python messages (the camera stream writes it as milliseconds in Python) has not been captured from a live run. -->


> **Gotcha.** A timeout from `wait_new_scan` never means the stream is broken. It is how a
> paused simulator presents, and also how a lidar that stopped publishing presents: a plan
> change or a robot that was removed. Handle it as a condition, and decide in your own code
> how many in a row mean the lidar is gone.

## Stream health

`stats` returns the stream's counters: a method in Rust and C++, a property in Python.

| Field | Notes |
|---|---|
| `received` | scans received and stored by the stream, read or not |
| `decode_errors` | payloads that did not decode; counted and dropped, never fatal |
| `seq_gaps` | times the sequence number jumped forward by more than one |
| `missed_scans` | total scans missed across all gaps |
| `last_seq` | the last sequence number seen |

A non-zero `seq_gaps` means scans were dropped on the way, not that they were late. When
`decode_errors` is non-zero, `last_error` holds the most recent failure. `received` counts
what the stream stored, so a loop that reads more slowly than the lidar scans sees fewer
scans than `received` reports, and that difference is your loop falling behind.

The example ends each lidar with a line built from those counters:

```text
  stream    10 more scans in 0.50 s of sim time = 19.8 Hz; received 11, seq gaps 0, decode errors 0
  stream    10 more scans in 1.00 s of sim time = 10.0 Hz; received 12, seq gaps 0, decode errors 0
```

The first line is the forklift's 3-D unit and the second the pallet robot's 2-D unit. Over
the Rust, C++ and Python runs the 3-D unit measured 19.3 to 19.8 Hz and the 2-D unit 9.9 to
10.0 Hz, with no sequence gaps.

`stop` ends the subscription, and dropping the stream does the same. Neither one stops the
lidar: it keeps scanning for every other reader. A 3-D stream outlives the robot handle it
was opened from. A 2-D stream uses that handle's connection and goes quiet once the handle
is closed.

**Next:** [Inside a scan](02-scan-layout.md)

**See also:** [Freshness](../ch05-cameras/04-freshness.md), [Stream health](../ch03-reading-state/08-health.md), [The vrobots command](../ch08-tooling/01-cli.md)
