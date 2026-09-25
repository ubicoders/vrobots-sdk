# Tooling and diagnostics

This chapter gives you a four-rung ladder to climb when a program misbehaves, and names the page that answers each rung.

Most of the time you arrive here because something produced no output, no motion,
or numbers that look wrong. The fastest route out is not to read your own code. It
is to establish, in order, what the wire is actually doing.

## The ladder

| Rung | Question | Command | Library call | Page |
|---|---|---|---|---|
| 1 | Is anything publishing at all, and under which ids? | `vrobots topic list` | `list_topics` | [The vrobots command](01-cli.md), [Discovery from code](02-discovery-from-code.md) |
| 2 | At what rate, and how steadily? | `vrobots topic hz <TOPIC>` | `measure_rate` | [The vrobots command](01-cli.md), [Measuring rates](05-rates.md) |
| 3 | Do the two ends speak the same schema? | `vrobots --version` | `version_info` | [Versions and pins](03-version-and-pins.md) |
| 4 | What is the SDK doing internally? | `RUST_LOG=vrobots_sdk=debug ...` | `init_logging` | [Logging](04-logging.md) |

## Climb it in order

Rung 1 first, always. Every rung below it assumes traffic exists, and the single
most common cause of a silent program is a simulator that is not in Play mode. An
empty listing is an answer, not a failure.

Rung 2 next, because a topic that is present is not the same as a topic that is
healthy. A control loop that stutters usually has a perfectly good average rate and
a maximum interval five times the mean, and only the distribution shows that.

Rung 3 comes before rung 4 whenever rung 1 said nothing. A version mismatch does not
report itself as an error: iceoryx2 compares the full version triple on every
shared-memory open and silently delivers nothing when it disagrees, which presents
as absence rather than as a fault. Checking a version takes one command and rules
out a class of bug that logs will never explain.

Rung 4 last. `tracing` events describe what the SDK did on your behalf, which is the
right level of detail once you know the wire is alive and compatible, and far too
much detail before that.

> **Note.** The `vrobots` binary and the library calls answer the same questions
> from the same code. The CLI is the version you can run without writing a program;
> `list_topics` and `measure_rate` are the version whose result is data you can
> branch on.

## Two pages that are not rungs

[More than one robot](06-multi-robot.md) covers holding two handles in one process,
which is where frame disagreements between robot types stop being theoretical.

[Recording and testing without the simulator](07-recording-and-testing.md) covers
capturing real wire bytes once and replaying them in unit tests, which is what makes
`cargo test` meaningful with Unity closed.

## Where else to look

This chapter is organised by tool. If you would rather work from a symptom, start at
[When nothing happens](../ch01-getting-started/08-troubleshooting.md). If you have an
error value in hand and want to know what it means, go to
[Appendix C: Error reference](../appendix-c-errors.md).

**Next:** [The vrobots command](01-cli.md)

**See also:** [When nothing happens](../ch01-getting-started/08-troubleshooting.md), [Two transports, one simulator](../ch02-concepts/01-transports.md)
