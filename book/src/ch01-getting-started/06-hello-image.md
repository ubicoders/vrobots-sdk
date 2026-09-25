# Hello image

Open the camera the robot already has, and read frames as they arrive.

```sh
cargo run -p vrobots-examples --bin ex03_hello_image
./target/cpp-build/ex03_hello_image
python examples/python/ex03_hello_image.py
```

## Opening a camera

**Every vrobot ships with `front_left` and `front_right` mounted, at 720p rgba8.** Reading
images does not start with creating a camera: it starts with attaching to one of those.
`open_camera` opens the iceoryx2 subscriber and touches the simulator not at all. The name,
resolution and format are constants, as they are in every example.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex03_hello_image.rs`:

```rust
const SYS_ID: u32 = 1; // the multirotor in the test scene
const CAMERA: &str = "front_left"; // every vrobot ships front_left and front_right
const RESOLUTION: &str = "720p";
const FORMAT: &str = "rgba8"; // Unity's native readback -- four channels, NOT rgb8
const FRAMES: u64 = 120; // then exit
const HZ: f64 = 100.0;

fn main() -> Result<(), VrError> {
    // ===== setup =====
    vrobots_sdk::init_logging("info");
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;

    // open_camera SUBSCRIBES to a camera the robot already has, without mutating
    // the sim. The name, resolution and format must match the publisher exactly
    // -- on iceoryx2 those three strings are the stream identity -- so a mismatch
    // surfaces as VrError::Timeout (ex13 shows that path).
    let cam = robot.open_camera(CAMERA, RESOLUTION, FORMAT)?;
    println!("camera stream: {}", cam.service_name());

    let mut seen = 0u64;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex03_hello_image.cpp`:

```cpp
constexpr std::uint32_t SYS_ID = 1;  // the multirotor in the test scene
constexpr const char* CAMERA = "front_left";  // every vrobot ships front_left and front_right
constexpr const char* RESOLUTION = "720p";
constexpr const char* FORMAT = "rgba8";  // Unity's native readback -- four channels, NOT rgb8
constexpr std::uint64_t FRAMES = 120;    // then exit
constexpr double HZ = 100.0;

int main() {
    try {
        // ===== setup =====
        vrsdk::check_version();
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();

        // open_camera SUBSCRIBES to a camera the robot already has, without
        // mutating the sim. The name, resolution and format must match the
        // publisher exactly -- on iceoryx2 those three strings are the stream
        // identity -- so a mismatch surfaces as a timeout (ex13 shows that path).
        vrsdk::CameraStream cam = robot.open_camera(CAMERA, RESOLUTION, FORMAT);
        std::printf("camera stream: %s\n", cam.service_name().c_str());

        std::uint64_t seen = 0;
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex03_hello_image.py`:

```python
SYS_ID = 1  # the multirotor in the test scene
CAMERA = "front_left"  # every vrobot ships front_left and front_right
RESOLUTION = "720p"
FORMAT = "rgba8"  # Unity's native readback -- four channels, NOT rgb8
FRAMES = 300  # then exit
HZ = 100


def main() -> None:
    # ===== setup =====
    vrsdk.init_logging("info")
    mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=SYS_ID)
    mr.connect()

    # open_camera SUBSCRIBES to a camera the robot already has, without
    # mutating the sim. The name, resolution and format must match the
    # publisher exactly -- on iceoryx2 those three strings are the stream
    # identity -- so a mismatch surfaces as a TIMEOUT (ex13 shows that path).
    cam = mr.open_camera(CAMERA, RESOLUTION, FORMAT)
    print(f"camera stream: {cam.service_name}")

    seen = 0
```

{{#endtab }}
{{#endtabs }}

The printed name is the iceoryx2 service, and it matches a row of `vrobots topic list`
character for character:

```text
camera stream: vrobots/1/i/cam/front_left/720p_rgba8
```

Two processes can open the same stream and neither disturbs the other, because neither owns
the camera. `mount_camera` is the other verb: it **creates** a camera of your choosing, and
it is what page [Lens and mount pose](../ch05-cameras/05-lens-and-pose.md) covers. You need
it only when the pair the robot ships cannot serve you.

## The loop, and what fresh() means

Images and states are two independent streams. The image half of the loop runs once per
frame while the state half runs every iteration, and the code says which by branching on
`fresh()`.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    // ===== loop =====
    while seen < FRAMES {
        let s = robot.states();

        // Images are a separate stream with their own timestamps -- never assume
        // they match the state's. Compare t_ns explicitly when fusing.
        if let Some(frame) = cam.fresh() {
            // Some only if new since the last read
            seen += 1;
            println!(
                "Image {} t={:.3} size=({}x{}) seq={} lag_vs_state={:.1} ms",
                frame.camera_name,
                frame.elapsed,
                frame.width,
                frame.height,
                frame.seq,
                (s.t_ns - frame.t_ns) as f64 / 1e6
            );
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex03_hello_image.cpp`:

```cpp
        // ===== loop =====
        while (seen < FRAMES) {
            const vrsdk::State s = robot.states();

            // A value only if new since the last read.
            if (auto frame = cam.fresh()) {
                ++seen;
                std::printf("Image %s t=%.3f size=(%ux%u) seq=%llu lag_vs_state=%.1f ms\n",
                            frame->camera_name.c_str(), frame->elapsed(), frame->width(),
                            frame->height(),
                            static_cast<unsigned long long>(frame->seq()),
                            static_cast<double>(s.t_ns - frame->t_ns()) / 1e6);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex03_hello_image.py`:

```python
    # ===== loop =====
    while seen < FRAMES:
        s = mr.states

        # Images are a separate stream with their own timestamps -- never assume
        # they match the state's. Compare t_ns explicitly when fusing.
        if cam.fresh:
            frame = cam.frame  # metadata for the image we are about to read
            img = cam.image  # numpy (h, w, c) uint8, top-down, RGB(A)
            seen += 1

            print(
                f"Image {frame.camera_name} t={frame.elapsed:.3f} "
                f"size=({frame.width}x{frame.height}) seq={frame.seq} "
                f"lag_vs_state={(s.t_ns - frame.t_ns) / 1e6:.1f} ms"
            )
```

{{#endtab }}
{{#endtabs }}

Rust and C++ ask and receive in one move, so the frame arrives inside an `Option` that the
`if` unwraps. Python splits it: `cam.fresh` is a boolean property and `cam.image` is the
numpy array, and reading the image is what consumes the freshness.

```text
State t=0.410 (no new frame)
Image front_left t=0.412 size=(1280x720) seq=17 lag_vs_state=12.4 ms
      sky-ness (B-R) top=+98 bottom=-25 (top-down: sky above ground), fov_y=61.9 deg
State t=0.420 (no new frame)
```

`fresh()` returns `Some` only when a frame has arrived since the last call, and it hands
each frame out exactly once. That makes it the right read for work that must not run twice
on one image. Its counterpart `latest()` returns the current frame regardless and does not
consume freshness.

The `lag_vs_state` figure is why the two streams are never paired by the SDK. Frames
arrive at the render rate and states at 25 Hz, so no frame belongs to any state. Both
`t_ns` values are on the same clock, so subtracting them is meaningful, and doing that
subtraction explicitly is what fusion looks like here.

> **Gotcha.** Camera frames ride iceoryx2 shared memory, so they are **same-host only**.
> zenoh will happily reach a simulator on another machine and deliver states, and not one
> frame will follow. There is no error: `open_camera` times out after
> `ConnectOptions::camera_timeout`, five seconds by default.

Rows in `frame.data` are row-major and **top-down**: row 0 is the top of the picture. The
wire is bottom-up, following Unity's render order, and the SDK flips while copying. The
sky-ness figures are the check on that, measuring blue minus red on the first and last
rows: outdoors the top row is sky and the bottom is ground. Channel order is the
renderer's own RGBA, never swapped, so OpenCV users convert to BGR at the call site.

## Nothing to clean up

There is no unmount at the end, and that is the point of opening rather than mounting. This
handle never created a camera, so it has nothing to remove; letting the stream go ends this
subscription and `front_left` keeps rendering and publishing for everyone else.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    // Nothing to unmount: this handle never created a camera. Dropping the stream
    // ends this subscription only -- front_left keeps rendering and publishing for
    // everyone else.
    let stats = cam.stats();
    println!(
        "{seen} frame(s), received={} decode_errors={} seq_gaps={}",
        stats.received, stats.decode_errors, stats.seq_gaps
    );
    Ok(())
}
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex03_hello_image.cpp`:

```cpp
        // Nothing to unmount: this handle never created a camera. Letting the
        // stream go ends this subscription only -- front_left keeps rendering and
        // publishing for everyone else.
        const vrsdk_camera_stats_t st = cam.stats();
        std::printf("%llu frame(s), received=%llu decode_errors=%llu seq_gaps=%llu\n",
                    static_cast<unsigned long long>(seen),
                    static_cast<unsigned long long>(st.received),
                    static_cast<unsigned long long>(st.decode_errors),
                    static_cast<unsigned long long>(st.seq_gaps));
        return 0;
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex03_hello_image.py`:

```python
    # Nothing to unmount: this handle never created a camera. Letting the stream
    # be collected ends this subscription only -- front_left keeps rendering and
    # publishing for everyone else.
    st = cam.stats
    print(
        f"{seen} frame(s), received={st.received} "
        f"decode_errors={st.decode_errors} seq_gaps={st.seq_gaps}"
    )
```

{{#endtab }}
{{#endtabs }}

```text
120 frame(s), received=120 decode_errors=0 seq_gaps=0
```

That count is `FRAMES`, so the Python run prints 300 rather than 120: it opens a window and
wants more of them. Because nothing was mutated, the loop bound is a convenience rather
than a cleanup deadline, and Ctrl-C is as safe an exit as running to the end. The one
example that does have a cleanup step is `ex17_camera_pose`, which mounts a camera of its
own.

**Next:** [Hello service](07-hello-service.md)

**See also:** [Mount, open and unmount](../ch05-cameras/01-mount-open-unmount.md), [Freshness](../ch05-cameras/04-freshness.md), [Inside a frame](../ch05-cameras/03-frames.md), [Showing frames in a window](../ch05-cameras/08-showing-frames.md)
