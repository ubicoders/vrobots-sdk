# The showcase attitude estimator

Getting a working attitude estimate out of the SDK instead of writing one, and putting it on the wire as your belief.

[Publishing estimates](09-publishing-estimates.md) leaves one half of the loop to you: read
`sensors`, run your filter, publish the result. `vrsdk.showcase` is a small set of finished
filters for the half in the middle, so a program that needs a believed attitude can have one
before it has an estimator. Its first member, `AttitudeEstimator`, is the velocity-aided
attitude the Global Hawk pages keep pointing at.

## It is Python only, and it ships compiled

`showcase` is a submodule of the `ubicoders-vrsdk` wheel and exists nowhere else. There is
no Rust, C++ or C ABI equivalent, and nothing in it is re-exported at the top level of
`vrsdk`: it reads as `showcase.AttitudeEstimator` at the call site on purpose.

| Question | Answer |
|---|---|
| Import | `from vrsdk import showcase`, or `import vrsdk.showcase` |
| Other languages | none; the core crate, the C ABI and the C++ header do not know it exists |
| Source | not shipped. The behaviour is the API; the implementation is compiled into the wheel |
| Privileges | none. It reads `robot.states`, the same public snapshot your loop reads |
| Required | no. Nothing else in the SDK depends on it |

That last pair of rows is the honest summary: everything a showcase class does, your own
code could do, over the surface the rest of this book documents. What you cannot do is read
how it does it.

## The class

From `crates/vrobots-sdk-py/python/vrsdk/showcase.pyi`:

```python
DEFAULT_ALPHA: float

class AttitudeEstimator:
    def __init__(self, robot: VirtualRobot, alpha: float = ...) -> None: ...
    def fused_euler(self) -> Euler: ...
    def vel_acc_pitch(self) -> float: ...
    def reset(self) -> None: ...
```

`Euler` is `(roll, pitch, yaw)` in radians, applied ZYX, in the body frame the robot
reports in, so it goes straight into `publish_estimate_euler` and into
[`rotations`](../ch02-concepts/08-rotation-conversions.md) without reordering.
`DEFAULT_ALPHA` is `0.98`.

## Building one and reading it

Construction takes a robot that is already connected and seeds the estimate from the
current snapshot; every read afterwards folds in whatever newer snapshot has arrived. There
is no pump to call and no clock to feed. Stale and time-warped snapshots are skipped inside,
reading twice in one sample is harmless, and if the simulator stalls the estimate holds
rather than running away.

This program flies nothing. It watches the Global Hawk fly itself, prints the fused pitch
beside the raw one, and publishes the belief so the `_Est` gauges follow it:

```python
"""A velocity-aided attitude estimate on the Global Hawk, published as a belief."""

import math

from vrsdk import RobotType, VirtualRobot, rotations, showcase

ORDER = rotations.EulerOrder.ZYX

# the estimator is built for a robot reporting frd, so connect in the aircraft's own frame
robot = VirtualRobot(RobotType.GLOBALHAWK, sys_id=0, coord_frame_id="frd", axis_convention=2)
robot.connect()

# built once, after connect(); every read folds in whatever snapshot has arrived
est = showcase.AttitudeEstimator(robot)

while True:
    s = robot.states
    roll, pitch, yaw = est.fused_euler()

    robot.publish_estimate_euler(
        (roll, pitch, yaw), ORDER, s.sensors.gyroscope.angular_velocity, True
    )

    print(
        f"t={s.elapsed:7.2f}s  "
        f"accel={math.degrees(est.vel_acc_pitch()):8.3f}  "
        f"fused={math.degrees(pitch):8.3f} deg"
    )
    robot.rate(50)
```

The Global Hawk is scene-authored and its id is allocated when the IMU scene loads, so
confirm the live one with `vrobots topic list` rather than trusting the `0` above. The two
columns are the point of the run: the left one is a measurement and jitters, the right one
is the estimate and does not.

```text
t=   0.00s  accel=   0.800  fused=   0.800 deg
t=   0.02s  accel=   0.628  fused=   0.800 deg
t=   0.07s  accel=   0.674  fused=   0.808 deg
t=   0.10s  accel=   0.620  fused=   0.811 deg
t=   0.12s  accel=   0.624  fused=   0.810 deg
...
t=  10.01s  accel=   0.594  fused=   0.895 deg
t=  10.03s  accel=   0.591  fused=   0.892 deg
```

Reading at 50 Hz against a 25 Hz state stream repeats each line, which is why the timestamps
double up. That costs nothing and needs no guard.

## What each of the three angles is worth

The three components of `fused_euler()` are not equally trustworthy, and the difference is
structural rather than a tuning problem.

| Angle | Units | Reference | Notes |
|---|---|---|---|
| `roll` | rad | gravity | absolute. The accelerometer observes gravity, and the motion part of the specific force is removed using the reported body velocity, so it stays honest under sustained thrust and in turns |
| `pitch` | rad | gravity | absolute, on the same footing as roll |
| `yaw` | rad | none | relative. Gyro-integrated from zero at construction, and it drifts at the gyro's bias rate. Nothing here observes a heading |

`vel_acc_pitch()` is the fourth number and a different kind of thing: the instantaneous
gravity-only pitch from the same sample the fused estimate last consumed, unfiltered.
It is noisy and honest under thrust, and it is exposed so the measurement and the estimate
can be printed side by side, as above.

> **Gotcha.** Absolute is not the same as exact. The estimate is only as good as the
> gyroscope and accelerometer beneath it, and the Global Hawk's gyroscope reports a small
> steady rate even in straight and level cruise. Over a minute the yaw therefore walks
> visibly away from where it started, and the roll settles to a standing offset instead of to
> zero error, while the pitch stays close. Print
> `rotations.quat_to_euler(s.kin.quat, ORDER)` beside `fused_euler()` for one run before you
> trust any of the three.

## Alpha is the only knob

`alpha` is the gyroscope's weight per sample, and it is the whole tuning surface.

| Value | Behaviour |
|---|---|
| `1.0` | pure gyro integration |
| `0.98` | the default, `showcase.DEFAULT_ALPHA`, right for a 50 Hz loop |
| `0.0` | raw accelerometer angles |

Anything outside `[0, 1]`, and any non-finite value, raises `VrError` with
`err.INVALID_ARGUMENT` and a message naming what you passed. The check runs before the
connection check, so a bad `alpha` reads as a bad `alpha` even on a handle that was never
connected; constructing on a connected-looking handle that has no session raises
`err.SESSION` with the usual "call connect() first". `reset()` re-seeds from the current
snapshot exactly as construction does, with yaw back to zero.

An estimator is thread-safe and shareable across threads, like the robot it reads.

**Next:** [Cameras and images](../ch05-cameras/00-intro.md)

**See also:** [Publishing estimates](09-publishing-estimates.md), [Fixed wing control](05-fixed-wing.md), [Global Hawk](../ch07-robots/06-globalhawk.md), [Sensors](../ch03-reading-state/03-sensors.md)
