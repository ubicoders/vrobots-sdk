# Cameras and images

Camera frames arrive on a second transport, at their own rate, with their own timestamps, and this chapter is how you read them.

**Assume a vrobot ships with `front_left` and `front_right` mounted, at 720p rgba8.** That
is the starting assumption throughout this chapter and every camera example in the book:
reading images means `open_camera` on one of that pair, which changes nothing in the
simulator. It holds for the multirotor and the truck; robot types that carry no camera at
all are the exception, and `vrobots topic list` settles it in one command. Creating a camera
of your own is a separate, mutating operation, covered on
[Lens and mount pose](05-lens-and-pose.md) and used by exactly one example.

## Frames never leave the host

Camera frames ride **iceoryx2 shared memory**, so they are **same-host only**. This holds
even when zenoh is talking to a simulator on another machine: states, commands and services
cross the network happily, and the images do not follow. The simulator loans a buffer and
your process reads the same physical pages, so there is no socket for a frame to travel
over and no machine boundary it can cross.

The failure is quiet rather than loud. An iceoryx2 service that does not exist on your host
is not an error at the far end, it is an absence, so `mount_camera` and `open_camera` wait
for a publisher that is never going to appear and return `VrError::Timeout` after
`ConnectOptions::camera_timeout` (5 s). A remote simulator, a typo in the format string and
a camera nobody mounted all present as the same timeout.

> **Gotcha.** If states work and cameras time out, check where the simulator is running
> before you check anything else. Run the simulator and your program on one machine to use
> cameras at all.

## The path a frame takes

Six steps, from the render to the `Frame` in your hand:

1. Unity renders the camera.
2. The simulator requests a GPU readback and stamps the capture time, `t_ns`, at that
   moment.
3. It loans one shared-memory sample, a fixed 5760-byte prefix followed by the pixels, and
   publishes it on the camera's iceoryx2 service, `vrobots/{sys_id}/i/cam/{name}/{res}_{fmt}`.
4. The stream's reader thread, one per `CameraStream`, receives the sample and parses the
   prefix.
5. It copies the pixel tail out, flipping rows as it copies, because the wire is bottom-up
   and `Frame::data` is top-down. Then it releases the sample: the pixels you are handed
   never alias shared memory.
6. Your thread calls `fresh()` and gets an `Arc<Frame>`, an owned immutable snapshot that
   stays valid for as long as you hold it.

The reader thread sleeps about 2 ms when a receive finds nothing, rather than spinning. At
60 fps a frame period is about 16 ms, so the added latency is under 15% of one frame, and
the SDK does not burn a core on a user's behalf.

## How this differs from the state stream

The two streams share a clock and nothing else.

| | State stream | Camera stream |
|---|---|---|
| Transport | zenoh | iceoryx2 shared memory |
| Reach | across a network, with `--router` | the local host only |
| Rate | 25 Hz | the render rate, about 60 fps |
| Read verb | `states()` | `fresh()`, `latest()`, `wait_new_frame()` |
| Freshness | always returns the latest snapshot, changed or not | each frame is handed out once |
| On a stall | keeps returning the last snapshot forever | `fresh()` returns `None`, `wait_new_frame` times out |
| Existence | created by `connect` | already on the robot (`front_left`, `front_right`), or created by `mount_camera` |

`Frame::t_ns` and `State::t_ns` are the same clock, and `Frame::elapsed` and
`State::elapsed` count from the same epoch, the robot's first state sample. That is the
only relationship the two streams have. **The SDK never pairs a frame with a state.** Code
that fuses them subtracts `t_ns` explicitly, as page
[Two cameras at once](06-two-cameras.md) shows.

## The rest of this chapter

| Page | What it answers |
|---|---|
| [Mount, open and unmount](01-mount-open-unmount.md) | Which of the three verbs changes the simulator, and what each one can undo |
| [Formats and resolution](02-formats-and-resolution.md) | The accepted strings, the byte cost of each, and the one setting that is robot-wide |
| [Inside a frame](03-frames.md) | Every field, plus row order, stride and channel order |
| [Freshness](04-freshness.md) | `fresh` against `latest` against `wait_new_frame`, and the stream's counters |
| [Lens and mount pose](05-lens-and-pose.md) | Where the camera sits, what lens it has, and what the simulator substitutes |
| [Two cameras at once](06-two-cameras.md) | Independent streams, and pairing frames by timestamp |
| [Saving a frame](07-saving-frames.md) | One image to disk with no image library |
| [Showing frames in a window](08-showing-frames.md) | A live OpenCV window, and the one conversion the SDK leaves to you |

Everything here runs against a live simulator. If a camera call times out and the simulator
is on this machine, `vrobots topic list` prints the streams that actually exist right now;
the `[i]` lines are the camera services.

**Next:** [Mount, open and unmount](01-mount-open-unmount.md)

**See also:** [Two transports, one simulator](../ch02-concepts/01-transports.md), [The topic namespace](../ch02-concepts/02-topics.md), [The vrobots command](../ch08-tooling/01-cli.md)
