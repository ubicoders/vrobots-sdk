"""ex33 - switch the Global Hawk's onboard loop between true attitude and z/estimate."""

import sys

from vrsdk import RobotType, VirtualRobot, cmd

sys_id = int(sys.argv[1])  # scene-authored (IMU scene): find the live id with `vrobots topic list`

YAW_RATE = 0.05  # rad/s, nose right
HZ = 25
SETTLE_SAMPLES = 100  # ~4 s for the integrator to null the error
MEASURE_SAMPLES = 150  # ~6 s averaged

# SET_ANGVEL carries a vec3 that IS re-expressed into the robot's frame, so stamp frd
robot = VirtualRobot(RobotType.GLOBALHAWK, sys_id=sys_id, coord_frame_id="frd", axis_convention=2)
robot.connect()

# the estimate source only matters to the onboard loop; direct-surface mode never reads it
robot.set_fw_ctrl_mode(cmd.FW_ONBOARD_RATE)

results = []
for label, phase in (
    ("truth (source 0)", "truth"),
    ("observer (source 1), no publisher", "observer"),
    ("after reset", "reset"),
):
    if phase == "truth":
        robot.set_fw_est_source(cmd.FW_EST_TRUTH)
    elif phase == "observer":
        # nothing publishes z/estimate here, so after 0.5 s the loop falls back to truth
        robot.set_fw_est_source(cmd.FW_EST_OBSERVER)
    else:
        robot.reset()  # reverts the source to truth and clears the SET_ANGVEL latch

    print(f"-- {label} --")
    for _ in range(SETTLE_SAMPLES):
        # the generic path: same bytes as set_angvel(), demonstrated on purpose
        robot.send_cmd(cmd.SET_ANGVEL, vec3=[0.0, 0.0, YAW_RATE])
        robot.rate(HZ)

    error_sum = 0.0
    for i in range(MEASURE_SAMPLES):
        robot.send_cmd(cmd.SET_ANGVEL, vec3=[0.0, 0.0, YAW_RATE])
        s = robot.states
        r = s.kin.ang_vel[2]  # FRD: the third body rate is yaw
        error_sum += YAW_RATE - r
        if i % 50 == 0:
            print(f"   t={s.elapsed:7.2f}s r={r:+7.4f} rad/s  err={YAW_RATE - r:+7.4f}")
        robot.rate(HZ)
    results.append((label, error_sum / MEASURE_SAMPLES))

print(f"\nsteady yaw-rate tracking, commanded {YAW_RATE} rad/s:")
for label, mean_error in results:
    print(f"  {label:<34} mean error={mean_error:+7.4f} rad/s")
print("the observer phase matching truth IS the fallback: no fresh estimate, so truth flew")

robot.send_cmd(cmd.SET_ANGVEL, vec3=[0.0, 0.0, 0.0])  # zero the latch; never deleted
