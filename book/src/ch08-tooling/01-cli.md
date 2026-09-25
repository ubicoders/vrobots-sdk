# The vrobots command

Every subcommand and flag, and how to read the output column by column.

```sh
cargo run -p vrobots-sdk --bin vrobots -- topic list
```

The SDK builds a binary called `vrobots`. It needs no dev tooling, no configuration
and no robot handle, and it is the first thing to run when a program is silent.

## Global flags

Both are global, so they are accepted before or after a subcommand.

| Flag | Default | Notes |
|---|---|---|
| `-V`, `--version` | | Prints the build's version block and exits 0. Nothing touches the wire. |
| `--log <FILTER>` | `warn` | `tracing` filter, e.g. `debug` or `vrobots_sdk=debug,zenoh=info`. `RUST_LOG` overrides it. |

## `topic list`

Answers "is the simulator publishing at all, and under which ids".

| Flag | Default | Notes |
|---|---|---|
| `-t`, `--timeout <SECS>` | `1.5` | zenoh observation window. The iceoryx2 registry lookup ignores it. |
| `-k`, `--keyword <STR>` | | Case-insensitive substring filter on the topic name. |
| `--router <ENDPOINT>` | | An explicit zenoh router, e.g. `tcp/192.168.1.10:7447`. |

A run against a live simulator prints one row per topic, then a footer:

```text
wire       Hz     bytes  topic
[i]        -         -  vrobots/1/i/cam/front_left/720p_rgba8
[i]        -         -  vrobots/1/i/cam/front_right/720p_rgba8  (stale: no process attached)
[z]      1.3      1760  vrobots/1/z/frames
[z]     25.3     45600  vrobots/1/z/state

4 topic(s); zenoh observed over 1.5s
[z] zenoh, measured by listening.  [i] iceoryx2, read from the registry:
    it exists, but Hz/bytes were not measured -- same host only.
```

Column by column:

| Column | Meaning |
|---|---|
| `wire` | `[z]` zenoh, `[i]` iceoryx2. It repeats the transport segment in the topic name, so you can also read it off the key. |
| `Hz` | Samples divided by the window. A measurement for `[z]`, and `-` for `[i]` because nothing was watched. |
| `bytes` | Total payload bytes seen during the window, not bytes per sample. `-` for `[i]`. |
| `topic` | The full key. This is the exact string `topic hz`, `measure_rate` and `CameraStream::service_name` all use. |

The rows are sorted by system id first and key second, which is why the two camera
streams for `sys_id 1` come before its zenoh topics.

Two things in that listing carry information beyond their numbers. `1.3` Hz on a
topic that publishes at 1 Hz is the window being short, not the publisher being
fast: three samples over 1.5 s of wall clock reads high because the window includes
zenoh's discovery latency at one end and a partial period at the other. And
`(stale: no process attached)` marks an iceoryx2 service record whose owning process
is gone. The stream is dead rather than idle, and the footer counts them separately.

> **Gotcha.** An empty list exits 0, because "nothing is publishing" is a legitimate
> answer to "what is publishing". Only `topic hz` treats silence as a failure.

With nothing running, the command says so and tells you what to try:

```text
(no topics)

Nothing published on `vrobots/**` in 1.5s, and no iceoryx2 camera
stream is registered. Usually one of:
  - the simulator is not in Play mode
  - it is on another machine (pass --router tcp/<host>:7447)
  - the window was too short for zenoh discovery (try -t 5)
```

## `topic hz`

Answers "how fast, how steadily, and am I losing samples". One key, no wildcards: a
single rate across several interleaved topics is not a rate, and the SDK refuses it
with `InvalidArgument` before opening a session.

```sh
cargo run -p vrobots-sdk --bin vrobots -- topic hz vrobots/1/z/state -w 5
```

| Argument | Default | Notes |
|---|---|---|
| `<TOPIC>` | required | The exact key `topic list` printed. The transport is read off the name: an `/i/` segment is iceoryx2, anything else is zenoh. |
| `-w`, `--window <SECS>` | `5.0` | How long to watch. Longer is more accurate. |
| `--router <ENDPOINT>` | | Ignored for iceoryx2 topics, which are always same-host. |

A healthy state topic looks like this:

```text
vrobots/1/z/state  [z]  watched 5.0 s

  rate          25.00 Hz      126 samples over a 5.000 s span
  interval   mean 40.00 ms    min 40.00   max 40.00   jitter 0.00 (sd)
  latency    mean 1.20 ms    min 0.80   max 2.10   publish stamp -> here
  seq        1 -> 126      0 gap(s), 0 missed
  payload        1200 B avg   30.2 kB/s over the window
```

Line by line:

| Line | What it says |
|---|---|
| `rate` | `(samples - 1) / span`, where span is first arrival to last arrival. Not samples divided by the window, which would count zenoh's discovery latency as dead air. |
| `interval` | The gap distribution between arrivals, in ms. `jitter` is the population standard deviation of those gaps. |
| `latency` | Arrival wall clock minus the publisher's `header.timestamp_ns`. Printed only when the payloads carry a header. |
| `seq` | First and last `header.seq`, how many separate forward jumps occurred, and how many samples those jumps imply were lost. |
| `payload` | Mean payload size, and throughput computed over the window rather than the span. |

**`max` on the interval line and the gap count are the two numbers that explain a
stuttering control loop.** A mean of 40.00 ms says nothing about the 200 ms stall
that broke it; the maximum interval is the worst stall the loop actually saw, and a
non-zero gap count says the samples covering that stall were dropped rather than
delayed. Read those two before the rate.

The block grows extra lines when there is something to say: a `decode` line when
payloads failed to parse, a paragraph explaining that latency across machines
measures clock offset and can go negative, a paragraph when gaps occurred, and a
paragraph when `seq` went backwards. See [Measuring rates](05-rates.md) for what
each of those means.

## `record`

Captures raw wire payloads to files, for tests that run with the simulator closed.
[Recording and testing without the simulator](07-recording-and-testing.md) covers
the workflow; the flags are here.

| Flag | Default | Notes |
|---|---|---|
| `--sys-id <U32>` | `0` | The robot to record from. |
| `--camera <NAME>` | | Record raw iceoryx2 camera slices instead of zenoh state payloads. |
| `--resolution <STR>` | `360p` | Used with `--camera`. |
| `--format <STR>` | `mono8` | Used with `--camera`. |
| `--mount` | off | Mount the camera first and unmount it afterwards. Requires `--camera`. **Mutates the simulator.** |
| `-n`, `--count <USIZE>` | `5` | Frames to capture. |
| `-t`, `--timeout <SECS>` | `10.0` | Give up after this long. |
| `-o`, `--out <PATH>` | `crates/vrobots-sdk/tests/fixtures` | Where `<prefix>_NNN.bin` is written. |
| `--prefix <STR>` | `state` | File name prefix. |
| `--router <ENDPOINT>` | | zenoh only. Camera slices are shared memory, so a router does not apply. |

## Exit codes

| Code | Meaning |
|---|---|
| 0 | Success, including `--help` and `--version`, and including `topic list` finding nothing. |
| 1 | The command ran and failed. `topic hz` with no samples lands here, as does running `vrobots` with no subcommand. |
| 2 | The arguments did not parse. |

`cli::run(args)` returns that code rather than calling `process::exit`, so an
embedder such as the Python wheel's console script keeps control of the process.

**Next:** [Discovery from code](02-discovery-from-code.md)

**See also:** [Measuring rates](05-rates.md), [The topic namespace](../ch02-concepts/02-topics.md), [Appendix A: Topic reference](../appendix-a-topics.md)
