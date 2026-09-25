# Versions and pins

You print what this build speaks, compare it with what the simulator speaks, and learn why an exact pin is not pedantry.

```sh
cargo run -p vrobots-sdk --bin vrobots -- --version
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
vrobots-sdk 0.1.4
  vrobots_msgs  v2.0.2-31-gac335c0 (schema_version 3)
  flatbuffers   25.12.19
  zenoh         1.9.0
  iceoryx2      0.9.3
  src_id        122
```

| Field | Type | Notes |
|---|---|---|
| `sdk_version` | `&'static str` | This crate's version. |
| `msgs_commit` | `&'static str` | `git describe --tags --always --dirty` of the `vrobots_msgs` submodule the generated FlatBuffers code was compiled from, or `"unknown"` when built without git, from a source tarball for instance. |
| `schema_version` | `u32` | The `schema_version` this SDK stamps on outbound headers. |
| `flatbuffers` | `&'static str` | The flatbuffers pin, from `ipc_versions.json` at build time. |
| `zenoh` | `&'static str` | The zenoh pin. |
| `iceoryx2` | `&'static str` | The iceoryx2 pin. |
| `src_id` | `u32` | The `src_id` this build stamps by default. |

These strings are stamped into the binary by `build.rs`, not read from a file at run
time. A binary you copied to another machine reports what it was actually built
against rather than what happens to be checked out beside it.

## What the simulator says

The other half of the comparison rides on every state snapshot. From
`examples/rust/src/bin/ex12_version_info.rs`:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
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
            "  MISMATCH -- fields may decode as garbage. Rebuild the SDK against \
             the sim's vrobots_msgs commit."
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
                "  MISMATCH -- fields may decode as garbage. Rebuild the SDK against the sim's "
                "vrobots_msgs commit.\n");
        }
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex12_version_info.py`:

```python
    mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=SYS_ID)
    mr.connect()
    first = mr.states
    print(
        f"\nsim says: schema_version={first.schema_version} (ours {v['schema_version']}), "
        f"frame={first.coord_frame_id!r} axes={first.axis_convention_name!r}, "
        f"its header src_id={first.src_id} (ours {v['src_id']})"
    )
    if first.schema_version != v["schema_version"]:
        print(
            "  MISMATCH -- fields may decode as garbage. Reinstall the wheel built "
            "against the sim's vrobots_msgs commit."
        )
```

{{#endtab }}
{{#endtabs }}

`version_info()` returns a struct in Rust and C++ (`v.schema_version`) but a dict in
Python (`v['schema_version']`). C++ reaches the header fields through `first.raw`, and it
has one check the other two do not: `check_version()` asserts that this header and the
linked library are the same release, because the snapshot structs are shared between them
by layout.

```text
sim says: schema_version=3 (ours 3), frame="frd" axes="frd", its header src_id=0 (ours 122)
```

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

`ipc_versions.json` at the repository root is the source of truth, and the workspace
`Cargo.toml` mirrors it as `=X.Y.Z` rather than `^X.Y.Z`.

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

Three independent mechanisms, so the drift is caught before it ships.

| Mechanism | When it fires | What it checks |
|---|---|---|
| `crates/vrobots-sdk/build.rs` | every compile | The workspace `Cargo.toml` pins each package as exactly `="X.Y.Z"` from `ipc_versions.json`, and panics with the offending line when it does not. |
| `scripts/check_versions.ps1` | CI | The same pin rule without needing a toolchain, plus the one SDK version mirrored across the workspace manifest, the member crates, the wheel metadata, the C++ header and the README. |
| The release workflow | a pushed tag | Runs `check_versions.ps1 -Tag <tag>` as a guard job and refuses to build anything when the tag, the manifests and `ipc_versions.json` disagree. |

`build.rs` also fails when the `vrobots_msgs` submodule is missing, with the
`git submodule update --init --recursive` fix in the message rather than 45
`include!` errors.

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
