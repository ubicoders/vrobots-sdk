"""ex35 - publish an attitude estimate and make the autopilot fly a pitched-up lie."""

import math
import sys

from vrsdk import RobotType, VirtualRobot, cmd, rotations

sys_id = int(sys.argv[1])  # scene-authored (IMU scene): find the live id with `vrobots topic list`

ORDER = rotations.EulerOrder.ZYX
PITCH_BIAS_DEG = 5.0  # under the altitude assist's 10 deg clamp, so the aircraft sags, not departs
HZ = 25.0
SETTLE_SAMPLES = 100
MEASURE_SAMPLES = 150

# the estimate inherits this header frame, so stamp the aircraft's own (frd)
robot = VirtualRobot(RobotType.GLOBALHAWK, sys_id=sys_id, coord_frame_id="frd", axis_convention=2)
robot.connect()

robot.set_fw_ctrl_mode(cmd.FW_ONBOARD_RATE)  # only the onboard loop consults an attitude
robot.set_angvel([0.0, 0.0, 0.0])  # sent once: a command latches

# publish_estimate_euler builds the same wire message from angles, shown once
s = robot.states
robot.publish_estimate_euler(
    rotations.quat_to_euler(s.kin.quat, ORDER),
    ORDER,
    s.sensors.gyroscope.angular_velocity,
    True,
)

bias = rotations.euler_to_quat((0.0, math.radians(PITCH_BIAS_DEG), 0.0), ORDER)

results = []
for label, source, estimator in (
    ("1 truth source, truth-copy", cmd.FW_EST_TRUTH, "copy"),
    ("2 observer source, truth-copy", cmd.FW_EST_OBSERVER, "copy"),
    ("3 observer source, +5 deg pitch lie", cmd.FW_EST_OBSERVER, "bias"),
    ("4 observer source, nothing published", cmd.FW_EST_OBSERVER, "silent"),
):
    if estimator == "silent":
        # phase 4 needs a level start; reset() clears the source and the SET_ANGVEL latch
        robot.reset()
        robot.set_angvel([0.0, 0.0, 0.0])
    robot.set_fw_est_source(source)
    print(f"-- {label} --")

    pitch_sum = altitude = 0.0
    for i in range(SETTLE_SAMPLES + MEASURE_SAMPLES):
        s = robot.states
        if estimator == "copy":
            robot.publish_estimate(s.kin.quat, s.sensors.gyroscope.angular_velocity, True)
        elif estimator == "bias":
            # post-multiply: the bias is applied about the BODY pitch axis
            quat = rotations.quat_multiply(s.kin.quat, bias)
            robot.publish_estimate(quat, s.sensors.gyroscope.angular_velocity, True)
        # "silent": nothing on the wire; after 0.5 s the loop falls back to truth
        if i >= SETTLE_SAMPLES:
            pitch_sum += math.degrees(rotations.quat_to_euler(s.kin.quat, ORDER)[1])
            altitude = -s.kin.lin_pos[2]  # frd: the third component is down
        robot.rate(HZ)
    results.append((label, pitch_sum / MEASURE_SAMPLES, altitude))

print("\nwhat each phase flew on:")
for label, mean_pitch, altitude in results:
    print(f"  {label:<38} true pitch={mean_pitch:+6.2f} deg  alt={altitude:9.1f} m")
print("phase 3 pitches down by about the lie; phases 1, 2 and 4 match")

robot.set_fw_est_source(cmd.FW_EST_TRUTH)  # hand it back; scene-authored, never deleted
robot.set_angvel([0.0, 0.0, 0.0])
