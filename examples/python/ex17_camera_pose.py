"""ex17 - mount a camera with a custom pose and lens, read the pose back, unmount."""

import math

import numpy as np

import vrsdk
from vrsdk import RobotType, VirtualRobot

MOUNT_POSITION = (0.10, 0.20, 0.30)  # metres, in OUR header frame ("unity")
MOUNT_EULER_DEG = (0.0, 0.0, 180.0)  # upside down
FOCAL_PX = 400.0  # smaller focal = wider angle (default 600)
FRAMES = 40
HZ = 100


def sky_ness(img, row):
    # mean blue - red across one row: positive for sky, negative for ground
    if img.shape[2] < 3:
        return 0.0
    line = img[row].astype(np.int16)
    return float(np.mean(line[:, 2] - line[:, 0]))


mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor, 0 = truck
mr.connect()

# the ONE example that mounts its own camera; all others open front_left/front_right
print(
    f"requested: position={MOUNT_POSITION} m, euler={MOUNT_EULER_DEG} deg, "
    f"fx=fy={FOCAL_PX} px"
)
cam = mr.mount_camera(
    "tilt",
    "720p",
    "rgb8",
    mount_position=MOUNT_POSITION,
    mount_euler_deg=MOUNT_EULER_DEG,
    fx=FOCAL_PX,
    fy=FOCAL_PX,
    near_clip=0.2,
    far_clip=500.0,
)
print(f"camera stream: {cam.service_name}")

seen = 0
settled = None

while seen < FRAMES:
    frame = cam.read()
    if frame is not None:
        seen += 1
        m, i = frame.mount, frame.intrinsics

        # degrees on the way in, radians on the way out: the wire is SI
        euler_deg = tuple(round(math.degrees(a), 1) for a in m.euler_rad)
        if euler_deg != settled or seen % 20 == 0:
            # the read-back is in the ROBOT's frame, not the one we sent
            print(
                f"frame {seen} seq={frame.seq}: mount "
                f"pos=({m.position[0]:+.2f},{m.position[1]:+.2f},{m.position[2]:+.2f}) m  "
                f"euler=({euler_deg[0]:+.1f},{euler_deg[1]:+.1f},{euler_deg[2]:+.1f}) deg  "
                f"frame={m.coord_frame_id!r} axes={vrsdk.axes_name(m.axis_convention)!r}"
            )
            default_fov = math.degrees(2 * math.atan(frame.height / 2 / 600.0))
            print(
                f"         lens fx={i.fx:.0f} fy={i.fy:.0f} -> "
                f"fov_y={math.degrees(i.fov_y):.1f} deg "
                f"(600 px would be {default_fov:.1f}), "
                f"clip {i.near_clip:.2f}..{i.far_clip:.0f} m"
            )
            settled = euler_deg

        # rolled 180: the sky lands in the BOTTOM rows (row 0 is always the top)
        if seen == FRAMES:
            img = frame.image
            top, bottom = sky_ness(img, 0), sky_ness(img, -1)
            if bottom > top + 20:
                verdict = "sky is at the BOTTOM: the camera really is upside down"
            elif top > bottom + 20:
                verdict = "sky is at the top: the roll did not take effect"
            else:
                verdict = "no sky/ground split -- check where the camera points"
            print(f"\nsky-ness (B-R) top={top:+.0f} bottom={bottom:+.0f} -> {verdict}")

    mr.rate(HZ)

# mounting is the half of the API with a cleanup step; front_left/right untouched
mr.unmount_camera("tilt")
print("unmounted tilt")
