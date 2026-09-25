"""ex18 - drive the truck while watching the multirotor from one process."""

import math

from vrsdk import RobotType, VirtualRobot

STEER_US = 1500.0  # straight ahead
THROTTLE_US = 1650.0  # light forward
HZ = 10

# sys_id 0 = truck, 1 = multirotor; each connect blocks for its own first snapshot
truck = VirtualRobot(RobotType.TRUCK, sys_id=0)
truck.connect()
drone = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)
drone.connect()
print(
    f"truck sys_id={truck.sys_id} ({truck.robot_type.key}), "
    f"drone sys_id={drone.sys_id} ({drone.robot_type.key})"
)

while True:
    truck.set_car(STEER_US, THROTTLE_US, 1100.0)
    t = truck.states
    d = drone.states

    tx, ty, tz = t.kin.lin_pos
    dx, dy, dz = d.kin.lin_pos
    # t_ns is the shared sim clock; elapsed counts from each robot's own first sample
    skew_ms = (t.t_ns - d.t_ns) / 1e6
    separation = math.dist(t.kin.lin_pos, d.kin.lin_pos)

    # the frames differ: truck "fru" (third component UP), drone "frd" (DOWN)
    print(
        f"truck[{t.sys_id}] pos=({tx:.2f},{ty:.2f},{tz:.2f}) "
        f"[{t.coord_frame_id!r}] echo={t.actuator.pwm}  |  "
        f"drone[{d.sys_id}] pos=({dx:.2f},{dy:.2f},{dz:.2f}) "
        f"[{d.coord_frame_id!r}] alt={-dz:.2f} m"
    )
    warn = "" if t.coord_frame_id == d.coord_frame_id else " (WRONG: mixed frames, convert first)"
    print(
        f"    naive separation={separation:.2f} m{warn}  "
        f"snapshot skew={skew_ms:+.1f} ms  "
        f"(elapsed: truck {t.elapsed:.2f}s vs drone {d.elapsed:.2f}s -- "
        "different epochs)"
    )

    truck.rate(HZ)  # pace on ONE handle only; rate() on both would sleep twice
