# Concepts

The model you need before writing a real control loop, and why five correct behaviours still surprise everyone.

Chapter 1 got a program talking to the simulator. This chapter explains the system
it is talking to, so that the four API chapters after it can be short: once you know
how a command travels, "commands latch" is one sentence rather than one chapter.

## Five behaviours that are correct and still surprise everyone

Almost every confused bug report against this SDK is one of five behaviours. None of
them is a defect. Each is invisible until it bites, and each has a different failure
signature, which is why [Five rules that explain everything](06-five-rules.md) is the
page to read twice.

| The behaviour | What people expect instead |
|---|---|
| Commands latch and get no reply | A command is an impulse, and a bad one comes back as an error |
| An ack is a receipt, not a result | `ok = true` means the change was applied |
| `states()` never fails | A stopped simulator makes the next read return an error |
| Robots outlive the process | Dropping the handle removes the robot from the scene |
| Every vector is in the robot's frame, not yours | One scene has one axis convention |

The common thread is that the simulator is a separate process with its own clock.
You are not calling into it; you are publishing to it and reading what it publishes
back. Nothing in this SDK hides that, because hiding it is what produces the bugs
above.

## What this chapter covers

Read it in order the first time. Every later page assumes the vocabulary this one
defines, and terms are defined once, here or in
[Appendix D: Glossary](../appendix-d-glossary.md).

| Page | What you get |
|---|---|
| [Two transports, one simulator](01-transports.md) | Why states travel over zenoh and camera frames over iceoryx2, and why the version pins are exact |
| [The topic namespace](02-topics.md) | Every key the simulator publishes or subscribes, and where a robot's id sits in it |
| [System ids, and the two kinds of robot](03-sys-id.md) | Scene-authored robots you attach to, created robots you spawn |
| [The shape of a program](04-program-shape.md) | `main()` does setup and owns the loop; the SDK never calls your code |
| [What connect actually does](05-connect.md) | The four steps behind a create, and every option you can change |
| [Five rules that explain everything](06-five-rules.md) | The five behaviours above, with the failure signature of each |
| [Frames, axes and units](07-frames-and-units.md) | Which vector is in which frame, and what `coord_frame_id` decides |
| [Rotation conversions](08-rotation-conversions.md) | Quaternions, angles and matrices, and re-expressing a state in another frame |

## What it does not cover

This chapter is the model, not the API. It names methods where a name makes a
concept concrete, and it does not tabulate their arguments: that is what
[Reading state](../ch03-reading-state/00-intro.md),
[Sending commands](../ch04-commands/00-intro.md),
[Cameras and images](../ch05-cameras/00-intro.md) and
[Services and configuration](../ch06-services/00-intro.md) are for. Per-robot
behaviour lives in [Supported virtual robots](../ch07-robots/00-intro.md).

**Next:** [Two transports, one simulator](01-transports.md)

**See also:** [Getting started](../ch01-getting-started/00-intro.md), [Appendix D: Glossary](../appendix-d-glossary.md)
