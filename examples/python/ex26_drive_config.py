"""ex26 - retune the truck's drivetrain with configure_drive."""

import math

from vrsdk import PwmBand, RobotType, VirtualRobot

STEER_US = 1100.0  # full left
THROTTLE_US = 1700.0  # steady forward
BRAKE_US = 1100.0  # released
HZ = 25

robot = VirtualRobot(RobotType.TRUCK)
robot.connect()
print(f"created sys_id={robot.sys_id}")


def circle(label):
    # Steady turn radius = speed / yaw rate. The truck publishes "fru", so yaw
    # about UP is ang_vel[2].
    robot.reset()
    for _ in range(75):
        robot.set_car(STEER_US, THROTTLE_US, BRAKE_US)
        robot.rate(HZ)
    speed = yaw = 0.0
    for _ in range(40):
        robot.set_car(STEER_US, THROTTLE_US, BRAKE_US)
        s = robot.states
        speed += math.dist(s.kin.lin_vel, (0.0, 0.0, 0.0))
        yaw += s.kin.ang_vel[2]
        robot.rate(HZ)
    speed /= 40
    yaw /= 40
    radius = speed / abs(yaw) if abs(yaw) > 1e-6 else math.inf
    print(f"  {label:<38} r={radius:6.2f} m  speed={speed:5.2f} m/s  yaw={yaw:+6.3f} rad/s")


circle("as spawned")

robot.configure_drive(max_steer_deg=15.0)
circle("max_steer_deg = 15")

# Out-of-range values are substituted silently: 90 is hard-clamped to 60, acked ok.
robot.configure_drive(max_steer_deg=90.0)
circle("max_steer_deg = 90 -> clamped to 60")

robot.configure_drive(
    drive_mode=2,  # rear axle only (4 = all wheels)
    max_steer_deg=30.0,
    steer_rate_dps=120.0,
    max_motor_torque_nm=40.0,
    no_load_wheel_rpm=200.0,
    idle_brake_torque_nm=5.0,
    max_brake_torque_nm=150.0,
    pwm_band=PwmBand(1100, 1500, 1900, 30),  # the factory band; all four move together
)
circle("drive_mode = 2, 40 N.m, 30 deg")

robot.delete()
print(f"deleted sys_id={robot.sys_id}")
