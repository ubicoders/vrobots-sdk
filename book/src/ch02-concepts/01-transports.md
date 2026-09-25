# Two transports, one simulator

Why states travel over zenoh and camera frames over iceoryx2, and what that costs you.

## Publish and subscribe, not call and return

Nothing in this system is a function call. The simulator publishes what it knows and
subscribes to what you send, and both ends run on their own clocks. A publisher does
not know who is listening, a subscriber does not know who is sending, and neither
blocks waiting for the other. Services are the one exception, and even they are
request/response over the same publish machinery rather than a procedure call: the
reply says the request arrived, not that anything happened.

That is the whole reason the rest of this chapter exists. Every surprise in
[Five rules that explain everything](06-five-rules.md) follows from the fact that you
are talking to a process that is not waiting for you.

Two transports carry that traffic, and they are not interchangeable.

| | zenoh | iceoryx2 |
|---|---|---|
| Carries | states, commands, services, setpoints | camera frames |
| Scope | across a network (`--router tcp/host:7447`) | same host only, shared memory |
| Discovery | none; a topic appears only if it published during your window | a registry, so entries can exist without a live publisher |
| Tag in `topic list` | `[z]` | `[i]` |

## Why the split is not arbitrary

A state snapshot is around a kilobyte and arrives 25 times a second. A 720p RGBA
frame is nearly four megabytes. Pushing frames through a network-capable transport
would mean serialising and copying them for a subscriber that is, in practice,
always the process next door. iceoryx2 hands over a pointer into shared memory
instead, so the frame is never copied across a socket at all.

The price is exactly the property that makes it fast. Shared memory does not cross a
machine boundary. Set `ConnectOptions::router_endpoint` at a simulator on another
host and you get states, commands and services, and no images: the camera stream
never appears and [`open_camera`](../ch05-cameras/01-mount-open-unmount.md) times out
after `camera_timeout`. That is not a misconfiguration you can fix with a flag.

> **Gotcha.** A remote connection that works perfectly for control and returns
> nothing but timeouts from every camera call is not broken. Check whether the
> simulator is on this host before debugging the camera code.

The second consequence of the split is discovery. Zenoh has no registry, so
`vrobots topic list` can only report what actually published during its observation
window: a robot that is paused is a robot that does not exist as far as that listing
is concerned. iceoryx2 does have a registry, so a camera service can be listed
without anything streaming through it, and a dead entry is marked stale rather than
vanishing. [Discovery from code](../ch08-tooling/02-discovery-from-code.md) calls
this the observed-versus-registered distinction, and it is what the `[z]` and `[i]`
tags are telling you:

```text
wire       Hz     bytes  topic
[z]      25.0      1234  vrobots/1/z/state
[i]         -         -  vrobots/0/i/cam/front/720p_rgba8
```

The `-` columns are not missing data. They are a registry entry, which by
construction has no measured rate.

## FlatBuffers on the wire

Both transports carry FlatBuffers payloads, generated from the schemas in the
`vrobots_msgs` submodule and shared byte for byte with the simulator's C# side. The
submodule ships the generated Rust, so building the SDK needs no `flatc`.

Two properties of that choice show up in the API. Decoding verifies the buffer
before reading any field, so a truncated or hostile payload produces
`VrError::Decode` rather than an out-of-bounds read. And a nested table that is
missing from the wire decodes to its `Default` (all zero, false, empty) rather than
failing, which is what lets an older simulator talk to a newer SDK: fields it does
not know about arrive as zero, not as an error.

## The pins are exact, and that is not pedantry

`ipc_versions.json` at the repository root is the source of truth for the three IPC
versions this SDK must match.

| Package | Pin | Kind of pin |
|---|---|---|
| flatbuffers | `25.12.19` | exact, `=X.Y.Z` |
| iceoryx2 | `0.9.3` | exact, `=X.Y.Z` |
| zenoh | `1.9.0` | exact, `=X.Y.Z` |

The exactness is load-bearing for one specific reason. **iceoryx2 compares
major.minor.patch on every shared-memory open.** A caret pin that resolves one patch
release away from the simulator's does not error and does not warn: it silently
delivers nothing. The symptom is a camera stream that never produces a frame, which
reads as "the simulator is not publishing" and sends you looking in entirely the
wrong place.

Three mechanisms keep the pins honest, so this is a failure you should never
actually see: `build.rs` fails the build on drift from `ipc_versions.json`,
`scripts/check_versions.ps1` fails CI, and the release workflow refuses to build.
`vrobots --version` prints the versions a given binary was built against, which is
the first thing to check when a simulator and an SDK disagree.
[Versions and pins](../ch08-tooling/03-version-and-pins.md) covers the whole
procedure.

**Next:** [The topic namespace](02-topics.md)

**See also:** [Cameras and images](../ch05-cameras/00-intro.md), [Versions and pins](../ch08-tooling/03-version-and-pins.md), [Discovery from code](../ch08-tooling/02-discovery-from-code.md)
