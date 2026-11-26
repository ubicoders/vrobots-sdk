"""ex42 - lidar: one revolution, read as ranges plus the layout that places them.

    python examples/python/ex42_lidar.py                           # warehouse: forklift 0, pallet robot 1
    python examples/python/ex42_lidar.py <forklift> <palletrobot>  # other ids

Two lidars, one API. The forklift carries a 3-D unit named `lidar` (VLP-16
class: 16 rings x 900 columns, 0.4-100 m, 20 Hz) and the pallet robot two 2-D
units, `lidar_front_left` and `lidar_rear_right` (RPLIDAR A2 class: one ring
of 360, 0.15-12 m, 10 Hz). They ride different wires --

| | 3-D | 2-D |
|---|---|---|
| open with | `open_lidar_3d(name)` | `open_lidar_2d(name)` |
| topic | `vrobots/{id}/i/lidar/{name}`, iceoryx2 | `vrobots/{id}/z/lidar/{name}`, zenoh |
| reaches | this host only | this host or the network |

-- and decode into the same `LidarScan`, read through the same
`LidarStream`, so `show()` below takes either. Opening one subscribes and
changes nothing in the simulator: like a camera (ex13), there is nothing to
clean up.

Ranges, not points
------------------

The wire carries one range per beam, plus the layout that places it. Beam
`ring * columns + column` points along

    azimuth   = azimuth_start   + column * azimuth_step    column 0 forward, counter-clockwise from above
    elevation = elevation_start + ring   * elevation_step  ring 0 the lowest
    point     = (r cos(el) cos(az), r cos(el) sin(az), r sin(el))    FLU: x forward, y left, z up

**`+inf` means no return** -- nothing within `max_range` -- never 0, so the
nearest-return search below skips infinities, not zeros. `beam_point()` is
that formula; `scan.points()` applies it to every beam at once and, under a
`srv/frames` override of device `lidar/<name>`, re-expresses the points in
that frame, while `beam_point()` stays in FLU. Either way a point is in the
**lidar's** frame, origin at its optical centre: the mount in `params`, given
in the robot's frame, is what takes it onto the body.

`scan.ranges` holds the ranges as a numpy array indexed `[ring, column]`;
this example reads one beam at a time with `scan.range(ring, column)`, which
needs no numpy.

The 2-D units publish their full 360 degrees, so on the pallet robot the
inward quarter of each ring looks straight through the robot's own body (its
colliders are skipped). Masking that quarter is the consumer's call.

Lidar is Pro
------------

On the Community tier the simulator publishes no lidar topic at all, so the
open finds nothing and fails with a `VrError` whose `code` is `err.TIMEOUT`:
after 5 s for the 3-D (a registry lookup, `camera_timeout`) and 15 s for the
2-D (zenoh has no registry, so the open waits for a first scan,
`probe_timeout`). The SDK cannot tell "no such lidar" from "not on this plan"
and says both. That is an answer, not a crash: this example prints it and
moves on. On Community only the focused robot is on the wire at all, so a
robot may not attach either -- that too is printed, and its half skipped.
"""

import math
import sys

from vrsdk import RobotType, VirtualRobot, VrError, err

USAGE = """\
The warehouse scene pins its forklift at sys_id 0 and its pallet robot at 1,
which are the defaults. For others, pass both ids:

    python examples/python/ex42_lidar.py 0 1

The scene has to be loaded and in Play mode. List what is publishing -- the
lidars included -- with:

    vrobots topic list"""

FORKLIFT_ID = 0  # the warehouse scene's forklift
PALLETROBOT_ID = 1  # and its pallet robot
LIDAR_3D = "lidar"  # the forklift's one 3-D unit
LIDAR_2D = "lidar_front_left"  # the pallet robot's front-left 2-D unit
SCAN_TIMEOUT = 1.0  # s; a 2-D revolution is 100 ms
RATE_SCANS = 10  # scans timed to measure the rate
BEARINGS_DEG = (0.0, 90.0, 180.0, 270.0)  # front, left, back, right


def attach(robot_type, sys_id):
    """Attach to a robot for its lidar, or say why not and skip that half.

    No type check here (ex37-ex41 make one): a lidar is opened by name, and a
    name the robot does not carry is the same clean "not there" as a missing
    topic.
    """
    robot = VirtualRobot(robot_type, sys_id=sys_id)
    try:
        robot.connect()
    except VrError as e:
        print(f"no {robot_type.key} at sys_id {sys_id}: [{e.code}] {e}")
        print(
            "(is the warehouse scene loaded and in Play mode? On the Community tier only "
            "the focused robot is on the wire at all.)"
        )
        return None
    return robot


def show(lidar):
    """One scan, taken apart: the layout, the returns, the nearest one, four
    bearings; then the rate the stream runs at.
    """
    # Scan-paced, like ex13's frames: block until the next revolution lands.
    try:
        lidar.wait_new_scan(SCAN_TIMEOUT)
    except VrError as e:
        print(f"no scan: [{e.code}] {e}")
        return
    scan = lidar.latest
    if scan is None:
        return
    p = scan.params
    print(
        f"{p.name} on {lidar.topic} [{lidar.transport}], scan seq {scan.seq} at "
        f"t={scan.elapsed:.2f} s"
    )

    # ===== the layout: where each range points =====
    beams = p.rings * p.columns
    print(
        f"  layout    {p.rings} ring(s) x {p.columns} columns = {beams} ranges, "
        f"{p.scan_rate:.0f} Hz, segments {p.segments}"
    )
    print(
        f"  column c  azimuth {math.degrees(p.azimuth_start):+.1f} + c x "
        f"{math.degrees(p.azimuth_step):.2f} deg (0 forward, counter-clockwise from above)"
    )
    print(
        f"  ring r    elevation {math.degrees(p.elevation_start):+.1f} + r x "
        f"{math.degrees(p.elevation_step):.1f} deg (ring 0 the lowest)"
    )
    print(f"  range     {p.min_range:.2f} to {p.max_range:.1f} m; no return reads +inf, never 0")
    print(
        f"  mount     ({p.mount_position[0]:+.2f}, {p.mount_position[1]:+.2f}, "
        f"{p.mount_position[2]:+.2f}) m, euler ({math.degrees(p.mount_euler_rad[0]):+.0f}, "
        f"{math.degrees(p.mount_euler_rad[1]):+.0f}, {math.degrees(p.mount_euler_rad[2]):+.0f}) "
        f"deg, in {p.mount_coord_frame_id!r} -- the robot's frame; the scan's own frame is "
        f"{scan.coord_frame_id!r}"
    )

    # ===== the returns, and the nearest one =====
    hits = 0
    nearest_index = 0
    nearest_range = math.inf
    for i in range(beams):
        r = scan.range(i // p.columns, i % p.columns)
        if math.isfinite(r):
            hits += 1
            if r < nearest_range:
                nearest_range = r
                nearest_index = i
    print(f"  returns   {hits} of {beams} beams")
    if hits > 0:
        ring = nearest_index // p.columns
        column = nearest_index % p.columns
        x, y, z = beam_point(scan, nearest_index)
        print(
            f"  nearest   {nearest_range:.2f} m on ring {ring}, column {column} (az "
            f"{math.degrees(p.azimuth_start + column * p.azimuth_step):+.1f}, el "
            f"{math.degrees(p.elevation_start + ring * p.elevation_step):+.1f} deg) -> "
            f"({x:+.2f}, {y:+.2f}, {z:+.2f}) m FLU"
        )

    # ===== four bearings on the lowest ring =====
    print(f"  ring 0, elevation {math.degrees(p.elevation_start):+.1f} deg, at four bearings:")
    for bearing in BEARINGS_DEG:
        # The column nearest that azimuth; ring 0 is the first `columns` ranges.
        column = round((math.radians(bearing) - p.azimuth_start) / p.azimuth_step) % p.columns
        r = scan.range(0, column)
        if math.isfinite(r):
            x, y, z = beam_point(scan, column)
            print(
                f"     az {bearing:+6.1f} deg  {r:6.2f} m  -> "
                f"({x:+6.2f}, {y:+6.2f}, {z:+6.2f}) m FLU"
            )
        else:
            print(f"     az {bearing:+6.1f} deg     inf    -> no return")

    # ===== the stream =====
    # seq counts revolutions and t_ns stamps each one's completion on the sim
    # clock, so the rate comes out of the scans, not out of this process's
    # scheduling.
    last = scan
    for _ in range(RATE_SCANS):
        try:
            lidar.wait_new_scan(SCAN_TIMEOUT)
        except VrError as e:
            print(f"  the stream stopped: [{e.code}] {e}")
            return
        newer = lidar.latest
        if newer is not None:
            last = newer
    revolutions = max(last.seq - scan.seq, 0)
    seconds = (last.t_ns - scan.t_ns) * 1e-9
    stats = lidar.stats
    print(
        f"  stream    {revolutions} more scans in {seconds:.2f} s of sim time = "
        f"{revolutions / seconds:.1f} Hz; received {stats.received}, seq gaps {stats.seq_gaps}, "
        f"decode errors {stats.decode_errors}"
    )


def beam_point(scan, index):
    """The point beam `index` hit, metres, in FLU: the wire's own formula.

    `index` is `ring * columns + column`. Only for a finite range: `+inf` is no
    return, and has no point.
    """
    p = scan.params
    ring = index // p.columns
    column = index % p.columns
    az = p.azimuth_start + column * p.azimuth_step
    el = p.elevation_start + ring * p.elevation_step
    r = scan.range(ring, column)
    return (
        r * math.cos(el) * math.cos(az),
        r * math.cos(el) * math.sin(az),
        r * math.sin(el),
    )


def sys_ids_from_args():
    """The forklift's and the pallet robot's sys_ids: the two command-line
    arguments, or `FORKLIFT_ID` and `PALLETROBOT_ID`.
    """
    forklift = sys_id_or(sys.argv[1] if len(sys.argv) > 1 else None, FORKLIFT_ID)
    palletrobot = sys_id_or(sys.argv[2] if len(sys.argv) > 2 else None, PALLETROBOT_ID)
    return forklift, palletrobot


def sys_id_or(arg, default):
    """One sys_id argument, or `default` when it was not given."""
    if arg is None:
        return default
    if arg.isdecimal() and int(arg) < 2**32:  # a u32, as the wire carries it
        return int(arg)
    print(
        f"usage: {sys.argv[0]} [forklift_sys_id] [palletrobot_sys_id]\n\n{USAGE}",
        file=sys.stderr,
    )
    sys.exit(2)


# ===== setup =====
forklift_id, palletrobot_id = sys_ids_from_args()

# ===== 3-D: the forklift's lidar, on iceoryx2 =====
print(f"\n== 3-D: '{LIDAR_3D}' on sys_id {forklift_id} ==")
forklift = attach(RobotType.FORKLIFT, forklift_id)
if forklift is not None:
    try:
        lidar = forklift.open_lidar_3d(LIDAR_3D)
    except VrError as e:
        # Absent: the answer, not a crash. On Community it is the tier.
        if e.code == err.TIMEOUT:
            print(f"not there: {e.detail}")
        else:
            print(f"could not open it: [{e.code}] {e}")
    else:
        show(lidar)

# ===== 2-D: the pallet robot's front-left lidar, on zenoh =====
print(f"\n== 2-D: '{LIDAR_2D}' on sys_id {palletrobot_id} ==")
palletrobot = attach(RobotType.PALLETROBOT, palletrobot_id)
if palletrobot is not None:
    print(
        "zenoh has no registry: the open waits up to "
        f"{palletrobot.options['probe_timeout']:g}s for a first scan"
    )
    try:
        lidar = palletrobot.open_lidar_2d(LIDAR_2D)
    except VrError as e:
        if e.code == err.TIMEOUT:
            print(f"not there: {e.detail}")
        else:
            print(f"could not open it: [{e.code}] {e}")
    else:
        show(lidar)

# Each stream ends with its handle; both lidars keep scanning for everyone else.
