# Recording and testing without the simulator

You capture real wire bytes once, check them in, and let `cargo test` decode them with Unity closed.

```sh
cargo run -p vrobots-sdk --bin vrobots -- record --sys-id 1 -n 5 --prefix state_multirotor
```

That writes `state_multirotor_000.bin` through `state_multirotor_004.bin` into
`crates/vrobots-sdk/tests/fixtures`, and those files are what makes the SDK's test
suite mean something on a machine with no simulator on it.

## Why the bytes have to come from the simulator

A fixture the SDK built itself would test the SDK's own builder against the SDK's own
decoder, which proves the crate is self-consistent and nothing else. It cannot catch
the failure that actually happens.

Recorded frames are **C#-produced golden payloads**: the exact bytes the simulator's
publisher put on the wire, captured once from a live simulator and checked in, then
decoded by the SDK's own decoder in unit tests. A schema change on the simulator side
breaks a unit test instead of surfacing months later as garbage fields at runtime.
That is the whole reason the recorder exists.

Recording is deliberately not a decode. What is written is the payload byte for byte
as it arrived, in raw `.bin` rather than base64 in text, because a fixture is only
useful if it is exact and a binary file cannot be reformatted or line-ending
converted on checkout.

## The command

| Flag | Default | Notes |
|---|---|---|
| `--sys-id <U32>` | `0` | Which robot. Without `--camera` this records `vrobots/<sys_id>/z/state`. |
| `--camera <NAME>` | | Record raw iceoryx2 camera slices instead. |
| `--resolution <STR>` | `360p` | With `--camera`. |
| `--format <STR>` | `mono8` | With `--camera`. |
| `--mount` | off | Mount the camera first and unmount it afterwards. Requires `--camera`. **Mutates the simulator.** |
| `-n`, `--count <USIZE>` | `5` | Frames to capture. |
| `-t`, `--timeout <SECS>` | `10.0` | Give up after this long. |
| `-o`, `--out <PATH>` | `crates/vrobots-sdk/tests/fixtures` | Output directory, created if missing. |
| `--prefix <STR>` | `state` | File name prefix. |
| `--router <ENDPOINT>` | | zenoh only. |

```text
crates/vrobots-sdk/tests/fixtures/state_multirotor_000.bin (1200 bytes)
crates/vrobots-sdk/tests/fixtures/state_multirotor_001.bin (1200 bytes)
crates/vrobots-sdk/tests/fixtures/state_multirotor_002.bin (1200 bytes)
crates/vrobots-sdk/tests/fixtures/state_multirotor_003.bin (1200 bytes)
crates/vrobots-sdk/tests/fixtures/state_multirotor_004.bin (1200 bytes)
5 frame(s) from vrobots/1/z/state
```

<!-- VERIFY: the per-file byte counts in the block above are illustrative; the line format is the CLI's. -->

> **Gotcha.** `--mount` mutates the simulator. It adds the named camera to that robot
> and removes it again at the end, leaving other cameras alone, but resolution is
> robot-wide: `--resolution 360p` against a robot whose cameras run at 720p restarts
> their streams under new names, and unmounting does not put them back. Without
> `--mount` the camera must already exist, because the recorder only subscribes.

## The calls behind it

From `crates/vrobots-sdk/src/record.rs`:

```rust
pub struct RecordedFrame { pub key: String, pub bytes: Vec<u8> }

pub fn record_frames(key: &str, count: usize, timeout: Duration, options: &ConnectOptions) -> VrResult<Vec<RecordedFrame>>
pub fn record_camera_frames(sys_id: u32, camera: &str, resolution: &str, format: &str, count: usize, timeout: Duration) -> VrResult<Vec<RecordedFrame>>
pub fn write_fixtures(frames: &[RecordedFrame], dir: &Path, prefix: &str) -> VrResult<Vec<PathBuf>>
```

These three are Rust-only. `RecordedFrame`, `record_frames`, `record_camera_frames` and
`write_fixtures` appear in none of `crates/vrobots-sdk-capi/include/vrobots_sdk.h`,
`cpp/include/vrobots_sdk.hpp` or `_vrsdk.pyi`, so there is nothing to show for the other
two surfaces. That is deliberate: the recorder exists to produce fixtures for this crate's
own test suite, and `vrobots record` is the interface every surface uses.

They return values rather than printing; the CLI prints what they return.

| Call | What it does |
|---|---|
| `record_frames` | A raw zenoh subscribe on one key, capturing each payload with no decode. `VrError::Timeout` when nothing arrives at all, with "is the sim in Play mode?" in the message. |
| `record_camera_frames` | The iceoryx2 counterpart, capturing shared-memory slices as `[5760-byte prefix][pixels]`. The camera must already be mounted; this only subscribes. `VrError::Timeout` when no publisher appears or no frame arrives. |
| `write_fixtures` | Writes `<prefix>_NNN.bin` as raw binary, creating the directory if needed, and returns the paths. `VrError::Config` when a write fails. |

`RecordedFrame` carries the key it arrived on beside the bytes, so a capture across a
wildcard still says which topic each payload came from.

## What the tests do with them

`crates/vrobots-sdk/tests/replay_decode.rs` decodes the state fixtures and
`camera_replay.rs` decodes the camera one. Because the bytes came from the other
language, those tests assert things nobody would bother asserting about their own
output: `src_id == 0`, which is reserved for the simulator, the schema version, a unit
quaternion, an accelerometer reading about 1 g at rest.

Each state set is consecutive samples off one subscriber, so `header.seq` increments
by exactly one across the set. `replay_decode.rs` asserts that too, which makes the
set a sequence-continuity fixture and not only a decode fixture.

`crates/vrobots-sdk/src/hz.rs` uses the same fixture from the other direction: its
generic header peek has to agree with the full decoder on the same bytes, or
`topic hz` would invent gaps.

## Refreshing them

Only when the schema genuinely moves. A fixture that gets regenerated whenever a test
fails is not a fixture. The exact commands live in
`crates/vrobots-sdk/tests/fixtures/README.md`, along with the reason to keep the
camera recording at 360p mono8: it is the smallest stream the simulator can produce,
where the same frame at 720p rgba8 would be 3.6 MB of checked-in binary.

Several assertions in `replay_decode.rs` encode the state the robot was in when
captured, at rest with idle PWM. Recording a flying drone fails them for a good
reason. Capture at rest, or change the assertions on purpose.

**Next:** [Appendix A: Topic reference](../appendix-a-topics.md)

**See also:** [Inside a frame](../ch05-cameras/03-frames.md), [Versions and pins](03-version-and-pins.md), [The vrobots command](01-cli.md)
