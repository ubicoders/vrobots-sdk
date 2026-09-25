"""ex25 - change the axes the robot reports in, and read the scene's frame."""

import vrsdk
from vrsdk import DeviceFrame, RobotType, VirtualRobot, device

HZ = 25

robot = VirtualRobot(RobotType.TRUCK)
robot.connect()
print(f"created sys_id={robot.sys_id}")


def settle():
    for _ in range(12):  # the change lands in phase 0 of the next physics step
        robot.rate(HZ)


def report(label):
    s = robot.states
    x, y, z = s.kin.lin_pos
    print(
        f"{label} robot={s.coord_frame_id!r:<7} ({s.axis_convention_name:<5}) "
        f"pos=({x:+7.3f},{y:+7.3f},{z:+7.3f})  "
        f"gyro frame={s.sensors.gyroscope.coord_frame_id!r}  "
        f"gnss frame={s.sensors.gnss.coord_frame_id!r}"
    )


# Scene scope, not robot scope: the answer is the same for every robot loaded.
scene = robot.scene_frame()
print(f"scene frame: {scene.coord_frame_id!r} ({scene.axis_convention_name!r})")

report("default   ")

# Frames are presentation, never physics: the motion is unchanged, the numbers permute.
robot.set_frames(
    "frd",
    [
        DeviceFrame(device.GYROSCOPE, "fru"),  # keep the gyro reading as it was
        # The frames service matches "gps"; the state block it moves is "gnss".
        DeviceFrame(device.GPS, vrsdk.INHERIT_FRAME),
    ],
)
settle()
report("overridden")

# INHERIT_FRAME clears an override so the level below wins again.
robot.set_frames(vrsdk.INHERIT_FRAME, [DeviceFrame(device.GYROSCOPE, vrsdk.INHERIT_FRAME)])
settle()
report("cleared   ")

robot.delete()
print(f"deleted sys_id={robot.sys_id}")
