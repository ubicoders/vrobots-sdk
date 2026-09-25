"""ex30 - roll a scene-authored half-drone with differential rotor PWM."""

import math
import sys

from vrsdk import RobotType, VirtualRobot

sys_id = int(sys.argv[1])  # scene-authored: find the live id with `vrobots topic list`

THROTTLE_US = 1600.0
PWM_MIN_US = 1100.0  # this airframe's floor, and its idle
PWM_MAX_US = 1900.0  # this airframe's ceiling, not the stock rotor's 2000
HZ = 25

robot = VirtualRobot(RobotType.HALFDRONE, sys_id=sys_id)
robot.connect()
print(f"attached to sys_id={robot.sys_id}, frame={robot.states.coord_frame_id!r}")

# differential added to rotor1 (FRD left arm) and subtracted from rotor2 (FRD right arm)
for diff in (0.0, 90.0, 0.0, -90.0, 0.0):
    pwm = [
        min(max(THROTTLE_US + diff, PWM_MIN_US), PWM_MAX_US),
        min(max(THROTTLE_US - diff, PWM_MIN_US), PWM_MAX_US),
    ]
    print(f"diff={diff:+6.0f} us -> pwm={pwm}")
    for i in range(50):  # ~2 s per setting
        robot.set_mr_pwm(pwm)
        if i % 25 == 0:
            s = robot.states
            # no Euler angles on the wire: FRD roll from kin.quat, ordered [x, y, z, w]
            x, y, z, w = s.kin.quat
            roll = math.degrees(
                math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
            )
            print(
                f"   t={s.elapsed:7.2f}s roll={roll:+7.2f} deg  "
                f"roll_rate={math.degrees(s.kin.ang_vel[0]):+6.2f} deg/s  "
                f"echo={s.actuator.pwm}"
            )
        robot.rate(HZ)

robot.set_mr_pwm(PWM_MIN_US, PWM_MIN_US)  # pulse widths latch; idle both rotors on the way out
