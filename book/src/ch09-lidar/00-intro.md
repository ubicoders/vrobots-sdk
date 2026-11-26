# Lidar

Lidar scans arrive on their own topics, one revolution at a time, and this chapter is how you open, read and place them.

**Lidar is a Pro plan feature.** On the Community plan the simulator publishes no lidar topic
at all, so every open in this chapter fails with a message that says so. The section
[Pro plan only](#pro-plan-only) below covers what that looks like from your program.

## The lidars that exist

Two kinds of unit exist, and a robot type either carries one or does not. You do not mount a
lidar from the SDK: each one is part of its robot, under a fixed name.

| | 3-D unit | 2-D unit |
|---|---|---|
| Carried by | truck, multirotor, forklift | pallet robot |
| Name on the robot | `lidar` | `lidar_front_left` and `lidar_rear_right` |
| Rings | 16, from -15 to +15 degrees of elevation, 2 degrees apart | 1 |
| Columns per ring | 900, 0.4 degrees apart | 360, 1 degree apart |
| Ranges per scan | 14,400 | 360 |
| Range | 0.4 to 100 m | 0.15 to 12 m |
| Scan rate | 20 Hz | 10 Hz |
| Transport | iceoryx2 shared memory | zenoh |
| Reach | the simulator's host only | that host, or across a network |
| Topic | `vrobots/{sys_id}/i/lidar/{name}` | `vrobots/{sys_id}/z/lidar/{name}` |
| Opened with | `open_lidar_3d(name)` | `open_lidar_2d(name)` |

The scan rate is a property of the unit. It is not a setting, and it is the same on every
robot that carries that kind of unit.

A scan from the 3-D unit is 14,400 ranges, 57.6 kB, twenty times a second. That is why it
rides shared memory, like camera frames, and why it has the same limit: **the simulator and
your program must run on one machine** to read a 3-D lidar. The 2-D units are small enough
for the network transport, so they reach a program on another machine through a zenoh router
exactly as states do.

## One scan type, one stream type

The two kinds of unit differ in transport and in layout, and in nothing else you program
against. Both decode into the same `LidarScan`, and both are read through the same
`LidarStream`, so a function written for one takes the other. The worked example for this
chapter, `ex42_lidar`, has one `show()` function and calls it with a 3-D stream and then
with a 2-D one.

A scan carries **ranges, not points**: one distance per beam, plus the layout that says
where each beam points. The SDK builds points on request, so a program that wants three
beams pays for three. Page [Inside a scan](02-scan-layout.md) covers the ranges and
[Points and frames](03-points-and-frames.md) covers the conversion.

## Pro plan only

On the Community plan a lidar topic does not exist. It is absent from `vrobots topic list`,
and the simulator casts no rays for it. Nothing refuses your open, because there is nothing
there to refuse it: the open waits for a topic that never appears and then fails with a
timeout.

The SDK cannot tell a lidar that does not exist from a plan without lidar, since both are
the same absence. It does not guess. The error message names both possibilities:

```text
lidar topic "vrobots/0/i/lidar/lidar" not found within 5s. Lidar needs the Pro tier: on Community the simulator publishes no lidar topic at all, so a plan without lidar and a lidar that does not exist look the same from here, and the SDK cannot tell which this is. Otherwise check the lidar name, that the sim is in Play mode, and that the sim runs on this host (3-D lidar rides iceoryx2, which is same-host only). `vrobots topic list` shows the lidars that do exist
```

Page [Opening a lidar stream](01-opening.md) shows how to catch it. The one place that
states the plan outright is the lidar configuration service, which answers a Community
request with a refusal; see [Configuring a lidar](04-configuring.md).

> **Gotcha.** On the Community plan only the focused robot is on the wire at all. A program
> that fails to attach to the forklift has not reached the lidar question yet, so check the
> attach before you read anything into a lidar error.

## How a lidar stream differs from a camera stream

The two streams are built alike, and a reader of the camera chapter already knows most of
this one. The differences are the ones worth holding on to.

| | Camera stream | Lidar stream |
|---|---|---|
| Created by | already on the robot, or `mount_camera` | always already on the robot |
| Identity | name, resolution and format | name |
| Transport | iceoryx2 | iceoryx2 for 3-D, zenoh for 2-D |
| Rate | the render rate | 20 Hz for 3-D, 10 Hz for 2-D |
| Payload | pixels | ranges and the beam layout |
| Timestamp means | when the capture was requested | when the revolution completed |
| Read verbs | `fresh`, `latest`, `wait_new_frame` | `fresh`, `latest`, `wait_new_scan` |
| Plan | every plan | Pro |

`LidarScan::t_ns`, `Frame::t_ns` and `State::t_ns` are the same clock, so they subtract
directly. As with cameras, **the SDK never pairs a scan with a state.** Code that needs the
robot's pose at the moment of a scan subtracts timestamps itself.

## The rest of this chapter

| Page | What it answers |
|---|---|
| [Opening a lidar stream](01-opening.md) | The two open calls, the three read verbs, stream health, and what each open timeout means |
| [Inside a scan](02-scan-layout.md) | Every field, the beam order, the no-return value, and when scans arrive |
| [Points and frames](03-points-and-frames.md) | Range to point, the default frame, and what a frame override changes |
| [Configuring a lidar](04-configuring.md) | Moving a lidar on its robot and tuning how a revolution is sampled |

Everything here runs against a live simulator on the Pro plan. The forklift and the pallet
robot of the Warehouse scene are the robots the example opens; the
[Forklift](../ch07-robots/08-forklift.md) and
[Pallet robot](../ch07-robots/09-palletrobot.md) pages describe them.

**Next:** [Opening a lidar stream](01-opening.md)

**See also:** [Cameras and images](../ch05-cameras/00-intro.md), [Two transports, one simulator](../ch02-concepts/01-transports.md), [The topic namespace](../ch02-concepts/02-topics.md)
