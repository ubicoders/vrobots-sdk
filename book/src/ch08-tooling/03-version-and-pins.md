# Versions and pins

You print what this build speaks, compare it with what the simulator speaks, and learn why an exact pin is not pedantry.

```sh
vrobots --version
cargo run -p vrobots-examples --bin ex12_version_info
./target/cpp-build/ex12_version_info
python examples/python/ex12_version_info.py
```

This is the first thing to compare when fields decode as garbage, and the second
thing to compare when a topic looks absent. A version mismatch never arrives as an
error, so nothing will tell you about it unless you ask.

## What the binary says

`vrobots --version` prints the `Display` of `VersionInfo`, which is the same block
`version_info()` returns to a program:

```text
vrobots-sdk 0.1.11
  vrobots_msgs  v2.0.2-31-gac335c0 (schema_version 3)
  flatbuffers   25.12.19
  zenoh         1.9.0
  iceoryx2      0.9.3
  src_id        122
```

| Field | Type | Notes |
|---|---|---|
| `sdk_version` | `String` | The SDK release. |
| `msgs_commit` | `String` | The revision of the `vrobots_msgs` message schemas the FlatBuffers code was generated from, in `git describe` form, or `"unknown"` when the release build could not record it. |
| `schema_version` | `u32` | The `schema_version` this SDK stamps on outbound headers. |
| `flatbuffers` | `String` | The flatbuffers pin the release was built with. |
| `zenoh` | `String` | The zenoh pin. |
| `iceoryx2` | `String` | The iceoryx2 pin. |
| `src_id` | `u32` | The `src_id` this build stamps by default. |

These strings are stamped into the library when a release is built, not read from a file
at run time. A wheel or a C bundle copied to another machine reports what it was actually
built against rather than anything installed beside it.

## What the simulator says

The other half of the comparison rides on every state snapshot. From
`examples/rust/src/bin/ex12_version_info.rs`, where the program first checks and prints its
own side:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    // ===== what this build is =====
    vrobots_sdk::check_version()?; // crate and linked library: one release
    let v = version_info();
    println!("{v}"); // the same block `vrobots --version` prints
    println!(
        "  crate         {} (must equal the library above)",
        vrobots_sdk::VERSION
    );

    // ===== what the other end is =====
    let robot = VirtualRobot::connect(RobotType::Multirotor, Some(SYS_ID))?;
    let first = robot.states();
    println!(
        "\nsim says: schema_version={} (ours {}), frame={:?} axes={:?}, \
         its header src_id={} (ours {})",
        first.schema_version,
        v.schema_version,
        first.coord_frame_id,
        first.axis_convention.name(),
        first.src_id,
        v.src_id
    );
    if first.schema_version != v.schema_version {
        println!(
            "  MISMATCH -- fields may decode as garbage. Install the SDK release \
             that matches the simulator build."
        );
    }
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex12_version_info.cpp`:

```cpp
        vrsdk::VirtualRobot robot(vrsdk::RobotType::Multirotor, SYS_ID);
        robot.connect();
        const vrsdk::State first = robot.states();
        std::printf(
            "\nsim says: schema_version=%u (ours %u), frame=\"%s\", its header src_id=%u "
            "(ours %u)\n",
            first.raw.schema_version, v.schema_version, first.coord_frame_id.c_str(),
            first.raw.src_id, v.src_id);
        if (first.raw.schema_version != v.schema_version) {
            std::printf(
                "  MISMATCH -- fields may decode as garbage. Install the SDK release that "
                "matches the simulator build.\n");
        }
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex12_version_info.py`:

```python
mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor, 0 = truck
mr.connect()
first = mr.states
print(
    f"\nsim says: schema_version={first.schema_version} (ours {v['schema_version']}), "
    f"frame={first.coord_frame_id!r} axes={first.axis_convention_name!r}, "
    f"its header src_id={first.src_id} (ours {v['src_id']})"
)
if first.schema_version != v["schema_version"]:
    print("  MISMATCH -- fields may decode as garbage; reinstall the matching wheel")
```

{{#endtab }}
{{#endtabs }}

`version_info()` returns a struct in Rust and C++ (`v.schema_version`) but a dict in
Python (`v['schema_version']`), and C++ reaches the header fields through `first.raw`.
Rust and C++ make one check that Python has no reason to make: `check_version()` fails
unless the linked library is the release that the Rust crate or the C++ header was built
for, because the snapshot structs are shared between them by layout. Both programs make that
check first, then print their own version under the `version_info()` block, as a `crate`
line in Rust and a `header` line in C++, so a mismatched pair shows even without the check. The mismatch warning names the same remedy in two wordings: Rust and C++ tell you to
install the SDK release that matches the simulator build, and Python tells you to reinstall
the matching wheel.

The Rust program prints the block shown at the top of this page, then:

```text
  crate         <version> (must equal the library above)

sim says: schema_version=3 (ours 3), frame="frd" axes="frd", its header src_id=0 (ours 122)
```

`<version>` is the crate's own release, which `check_version()` has already found equal to
the library's. C++ prints the same `sim says` fields without `axes`, and Python quotes the
two strings with single quotes, as in `frame='frd' axes='frd'`.

<!-- VERIFY: the frame and axes strings in the block above are the multirotor's reported values and need a live-simulator capture to confirm. -->

`src_id` 0 is the simulator's, reserved for it; 122 is this build's default. They are
supposed to differ. `schema_version` is not: a difference there means the two sides
were generated from different schema commits, and the decode that follows produces
plausible-looking wrong numbers rather than an error.

> **Gotcha.** A schema mismatch does not raise. FlatBuffers decodes a missing nested
> table to its `Default`, which is all zeroes, so an incompatible field arrives as
> `0.0` and not as `VrError::Decode`. A block of suspiciously round zeroes in the
> snapshot is the symptom to recognise.

## Why the pins are exact

Every release depends on each of the three packages at one exact version, `=X.Y.Z`, rather
than a caret range such as `^X.Y.Z`.

| Package | Pin |
|---|---|
| flatbuffers | `25.12.19` |
| iceoryx2 | `0.9.3` |
| zenoh | `1.9.0` |

The exactness is load-bearing for one specific reason. **iceoryx2 compares
major.minor.patch on every shared-memory open.** A caret pin that resolves one patch
release away from the simulator's vendored C# drop does not error and does not warn.
It silently delivers nothing. What you see is a camera stream that never produces a
frame, and what that reads like is "the simulator is not publishing", which sends you
looking at Play mode, at the topic list and at your own camera code, none of which
are wrong.

## What enforces them

The pins are enforced before anything ships. Every compile of the SDK's core checks them,
and a release build refuses to start when the release version, the package manifests and
the pinned versions disagree. For you, that means a wheel and a C bundle of the same release
carry the same three versions, and `vrobots --version` or `version_info()` reads them from
the library itself. An IPC pin mismatch you meet is therefore between an SDK release and a
simulator build.

## The order to check things in

1. `vrobots --version` on the machine running your program.
2. The simulator's `schema_version`, from any state snapshot, against the
   `schema_version` in that block.
3. The simulator's vendored iceoryx2 version against the `iceoryx2` line, if camera
   frames are the thing that is missing.

Paste all three into a bug report. They are the difference between a reproducible
report and a description of a symptom.

**Next:** [Logging](04-logging.md)

**See also:** [Two transports, one simulator](../ch02-concepts/01-transports.md), [Timestamps and sequence numbers](../ch03-reading-state/06-timestamps.md), [When nothing happens](../ch01-getting-started/08-troubleshooting.md)
