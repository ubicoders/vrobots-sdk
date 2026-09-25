# Two cameras at once

Independent streams, independent freshness, and pairing a frame with a state snapshot.

```sh
cargo run -p vrobots-examples --bin ex16_two_cameras
./target/cpp-build/ex16_two_cameras
python examples/python/ex16_two_cameras.py
```

The stereo pair every vrobot already has: **`front_left` and `front_right`, both at 720p
rgba8**. Each `open_camera` call returns its own `CameraStream`, and each stream owns its
own reader thread, its own sequence numbers and its own freshness.

## Opening both

Both streams are at the same resolution, and could not be anything else: resolution is one
knob for the whole robot. Were you mounting instead, asking for a second resolution while
this handle holds a stream at another is refused with `VrError::InvalidArgument` rather than
restarting the first stream under a new name behind your back. Format is per camera; only
resolution is shared.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex16_two_cameras.rs`:

```rust
// Two subscriptions, no mutation: both cameras are already on the robot.
let left = robot.open_camera(LEFT, RESOLUTION, FORMAT)?;
let right = robot.open_camera(RIGHT, RESOLUTION, FORMAT)?;
println!("left : {}", left.service_name());
println!("right: {}", right.service_name());
println!(
    "mounted by this handle: {:?}  <- neither is ours",
    robot.mounted_cameras()
);
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex16_two_cameras.cpp`:

```cpp
        // Two subscriptions, no mutation: both cameras are already on the robot.
        vrsdk::CameraStream left = robot.open_camera(LEFT, RESOLUTION, FORMAT);
        vrsdk::CameraStream right = robot.open_camera(RIGHT, RESOLUTION, FORMAT);
        std::printf("left : %s\n", left.service_name().c_str());
        std::printf("right: %s\n", right.service_name().c_str());
        std::printf("mounted by this handle: %zu  <- neither is ours\n",
                    robot.mounted_cameras().size());
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex16_two_cameras.py`:

```python
    # Two subscriptions, no mutation: both cameras are already on the robot.
    left = mr.open_camera(LEFT, RESOLUTION, FORMAT)
    right = mr.open_camera(RIGHT, RESOLUTION, FORMAT)
    print(f"left : {left.service_name}")
    print(f"right: {right.service_name}")
    print(f"mounted by this handle: {mr.mounted_cameras()}  <- neither is ours")
```

{{#endtab }}
{{#endtabs }}

Only the printing differs. `service_name` is a property in Python and a method in the other
two, and C++ has no `CameraSpec` type, so `mounted_cameras()` there hands back a
`std::array<std::string, 3>` of name, resolution and format, printed here as a count.

Two service names, and an empty owned list -- which is the point:

```text
left : vrobots/1/i/cam/front_left/720p_rgba8
right: vrobots/1/i/cam/front_right/720p_rgba8
mounted by this handle: []  <- neither is ours
```

Neither call changed anything in the simulator, so two of these programs can run at once on
the same pair without either noticing, and a third can be mounting a camera of its own
alongside them.

## Two consumers in one loop

There is no combined "wait for both", by design: the cameras are separate iceoryx2 services
and they render on their own schedules. Two `fresh()` calls in one loop are genuinely
independent, and neither can consume the other's frame.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
while n_left < FRAMES || n_right < FRAMES {
    // Two consumers, each draining its own stream. Neither call can consume
    // the other's frame.
    if let Some(f) = left.fresh() {
        n_left += 1;
        last_left_ns = f.t_ns;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex16_two_cameras.cpp`:

```cpp
        while (n_left < FRAMES || n_right < FRAMES) {
            // Two consumers, each draining its own stream. Neither call can
            // consume the other's frame.
            if (const std::optional<vrsdk::Frame> f = left.fresh()) {
                ++n_left;
                last_left_ns = f->t_ns();
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex16_two_cameras.py`:

```python
    while n_left < FRAMES or n_right < FRAMES:
        # Two consumers, each draining its own stream. Neither call can consume
        # the other's frame.
        f = left.read()
        if f is not None:
            n_left += 1
            last_left_ns = f.t_ns
```

{{#endtab }}
{{#endtabs }}

Rust and C++ take the frame out of an `Option`, so the `if` both tests and binds. Python calls
`left.read()`, which returns the frame or `None`, and this is the per-stream read that gives
each `CameraStream` its own freshness; `t_ns` is a method in C++ and a field in the other two.

Neither branch prints on every pass: the example reports every twentieth frame per camera.
The loop is paced with `robot.rate(HZ)` at 100 Hz against streams that render at about 60
fps, so most iterations find one stream fresh and the other not.

## Skew between the two

The only honest way to relate two frames is to subtract their capture stamps. Both are on
the same clock, so the difference is a real interval.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
let skew_ms = if last_left_ns == 0 {
    f64::NAN
} else {
    (f.t_ns - last_left_ns) as f64 / 1e6
};
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex16_two_cameras.cpp`:

```cpp
                    const double skew_ms =
                        last_left_ns == 0 ? 0.0
                                          : static_cast<double>(f->t_ns() - last_left_ns) / 1e6;
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex16_two_cameras.py`:

```python
                skew_ms = (
                    float("nan") if last_left_ns == 0 else (f.t_ns - last_left_ns) / 1e6
                )
```

{{#endtab }}
{{#endtabs }}

The subtraction is the same in all three. Only the "no left frame yet" value differs: Rust and
Python report `NaN`, C++ reports `0.0`, so a C++ reader cannot tell that first row from a
genuinely zero skew.

Printed every twentieth frame, alongside the sequence numbers each stream keeps for itself:

```text
L frame 1: seq=8 t=2.104
R frame 1: seq=8 t=2.104  skew_vs_last_left=+0.0 ms
L frame 21: seq=28 t=2.437
R frame 21: seq=28 t=2.437  skew_vs_last_left=+0.2 ms
```

<!-- VERIFY: the seq, t and skew magnitudes above are illustrative and have not been measured against a live run of ex16. -->

Sequence numbers are per stream and start from that stream's own beginning, so `left` and
`right` agreeing on a number means nothing. Compare `t_ns`, never `seq`.

## Pairing a frame with a state

The same rule applies between a camera and the state stream, and it is the more common
case. **The SDK never pairs them for you.** Frames arrive at the render rate, states at 25
Hz, and no frame belongs to any state. What they share is the clock: `Frame::t_ns` and
`State::t_ns` are both nanoseconds on the simulator's unix clock, and `Frame::elapsed` and
`State::elapsed` count from the same epoch, the robot's first state sample.

So fusion code subtracts. From `examples/rust/src/bin/ex03_hello_image.rs`, which reports
the age of each frame against the state snapshot taken in the same iteration:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
let s = robot.states();

// Images are a separate stream with their own timestamps -- never assume
// they match the state's. Compare t_ns explicitly when fusing.
if let Some(frame) = cam.fresh() {
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex03_hello_image.cpp`:

```cpp
            const vrsdk::State s = robot.states();

            // A value only if new since the last read.
            if (auto frame = cam.fresh()) {
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex03_hello_image.py`:

```python
        s = mr.states

        # Images are a separate stream with their own timestamps -- never assume
        # they match the state's. Compare t_ns explicitly when fusing.
        if cam.fresh:
            frame = cam.frame  # metadata for the image we are about to read
```

{{#endtab }}
{{#endtabs }}

Python splits what the other two do in one move: `mr.states` and `cam.fresh` are properties,
and the frame comes from `cam.frame` after the freshness test rather than out of the test
itself.

`(s.t_ns - frame.t_ns) as f64 / 1e6` is that age in milliseconds. For anything that needs a
state at the instant of capture rather than the newest one, keep a short ring of recent
snapshots and pick the one whose `t_ns` is closest to the frame's.

## Nothing to clean up

There is no unmount at the end of this example, for either stream. This handle created
neither camera, so it has nothing to remove: letting the streams go ends two subscriptions,
and both cameras keep rendering and publishing for everyone else.

That is also why `unmount_camera(LEFT)` here would be refused rather than obeyed -- it
removes only what `mount_camera` added, and page
[Mount, open and unmount](01-mount-open-unmount.md) shows the refusal in full. A program
that does mount a pair of its own unmounts each by name, in either order, since each call
removes exactly the name it is given and cannot undo the other.

**Next:** [Saving a frame](07-saving-frames.md)

**See also:** [Freshness](04-freshness.md), [Formats and resolution](02-formats-and-resolution.md), [More than one robot](../ch08-tooling/06-multi-robot.md)
