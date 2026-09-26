# Recording and testing without the simulator

You capture the exact bytes the simulator publishes, and see how such recordings let the SDK's decoder be tested with Unity closed.

```sh
vrobots record --sys-id 1 -n 5 --prefix state_multirotor
```

That writes `state_multirotor_000.bin` through `state_multirotor_004.bin` into a `captures`
folder under the current directory, which the command creates if it is missing. Each file
is one state payload exactly as it arrived. Pass `-o <PATH>` to write them somewhere else.

## Why the bytes have to come from the simulator

A fixture the SDK built itself would test the SDK's own builder against the SDK's own
decoder, which proves the SDK is self-consistent and nothing else. It cannot catch
the failure that actually happens.

Recorded frames are **C#-produced golden payloads**: the exact bytes the simulator's
publisher put on the wire, captured once from a live simulator and kept, then decoded
by the SDK's own decoder in its unit tests. A schema change on the simulator side
breaks a unit test instead of surfacing months later as garbage fields at runtime.
That is the whole reason the recorder exists.

Recording is deliberately not a decode. What is written is the payload byte for byte
as it arrived, in raw `.bin` rather than base64 in text, because a fixture is only
useful if it is exact and a binary file cannot be reformatted or line-ending
converted on checkout.

The same exactness makes a recording the most precise attachment a bug report can carry.
Next to the output of `vrobots --version`, it shows what the simulator actually sent rather
than what your program made of it.

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
| `-o`, `--out <PATH>` | `captures` | Output directory, created if missing. A relative path, the default included, is taken from the current directory. |
| `--prefix <STR>` | `state` | File name prefix. |
| `--router <ENDPOINT>` | | zenoh only. |

```text
captures/state_multirotor_000.bin (1200 bytes)
captures/state_multirotor_001.bin (1200 bytes)
captures/state_multirotor_002.bin (1200 bytes)
captures/state_multirotor_003.bin (1200 bytes)
captures/state_multirotor_004.bin (1200 bytes)
5 frame(s) from vrobots/1/z/state
```

<!-- VERIFY: the per-file byte counts in the block above are illustrative; the line format is the CLI's. -->

> **Gotcha.** `--mount` mutates the simulator. It adds the named camera to that robot
> and removes it again at the end, leaving other cameras alone, but resolution is
> robot-wide: `--resolution 360p` against a robot whose cameras run at 720p restarts
> their streams under new names, and unmounting does not put them back. Without
> `--mount` the camera must already exist, because the recorder only subscribes.

## What happens when it runs

The recorder has no library API in any language: nothing of it appears in the `vrobots-sdk`
crate, in `include/vrobots_sdk.h`, in `include/vrobots_sdk.hpp` or in `vrsdk/_vrsdk.pyi`.
That is deliberate. It exists to produce fixtures for the SDK's own test suite, and
`vrobots record` is the interface every language uses. A run has three parts:

| Part | What it does |
|---|---|
| State capture, without `--camera` | A raw zenoh subscribe on `vrobots/<sys_id>/z/state`, capturing each payload with no decode. When nothing arrives at all it fails with a timeout, and the message asks whether the simulator is in Play mode. |
| Camera capture, with `--camera` | The iceoryx2 counterpart, capturing shared-memory slices as `[5760-byte prefix][pixels]`. The camera must already be mounted, or be mounted by `--mount`, because the capture itself only subscribes. It fails with a timeout when no publisher appears or no frame arrives. |
| Writing | Each capture becomes `<prefix>_NNN.bin`, raw binary, in the output directory, which is created if needed. A write that fails ends the command with an error. |

## What the tests do with them

The SDK's own test suite decodes recorded state sets and a recorded camera frame.
Because the bytes came from the other language, those tests assert things nobody would
bother asserting about their own output: `src_id == 0`, which is reserved for the
simulator, the schema version, a unit quaternion, an accelerometer reading about 1 g at
rest.

Each state set is consecutive samples off one subscriber, so `header.seq` increments
by exactly one across the set. The state tests assert that too, which makes the set a
sequence-continuity fixture and not only a decode fixture.

The rate meter behind `topic hz` uses the same fixture from the other direction: its
generic header peek has to agree with the full decoder on the same bytes, or `topic hz`
would invent gaps.

## Refreshing them

Only when the schema genuinely moves. A fixture that gets regenerated whenever a test
fails is not a fixture. Camera recordings stay at 360p mono8, because that is the smallest
stream the simulator can produce, where the same frame at 720p rgba8 would be 3.6 MB per
file.

Several of the state assertions encode the state the robot was in when captured, at rest
with idle PWM, so a recording of a flying drone fails them for a good reason. Capture at
rest for a bug report too, so the numbers mean the same thing to whoever reads them.

**Next:** [Appendix A: Topic reference](../appendix-a-topics.md)

**See also:** [Inside a frame](../ch05-cameras/03-frames.md), [Versions and pins](03-version-and-pins.md), [The vrobots command](01-cli.md)
