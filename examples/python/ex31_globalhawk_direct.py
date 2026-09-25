"""ex31 - fly the Global Hawk's six panels directly, bypassing its onboard rate PIDs."""

import sys

from vrsdk import RobotType, VirtualRobot, cmd

sys_id = int(sys.argv[1])  # scene-authored (IMU scene): find the live id with `vrobots topic list`

PANELS = 6
DEFLECT_RAD = 0.15  # ~8.6 deg, inside the airframe's 20 deg clamp
CRUISE_N = 3800.0
CLIMB_N = 8000.0
HZ = 25

robot = VirtualRobot(RobotType.GLOBALHAWK, sys_id=sys_id)
robot.connect()

# mode first, then thrust: entering direct mode inherits the autopilot's current thrust
robot.set_fw_ctrl_mode(cmd.FW_DIRECT_SURFACE)
robot.set_fw_thrust(CRUISE_N)

# panel order [LF, RF, LIF, RIF, RLF, RRF]; the onboard mixer cannot move the inner flaps
for label, surfaces in (
    ("neutral", [0.0] * PANELS),
    ("inner flaps only", [0.0, 0.0, DEFLECT_RAD, -DEFLECT_RAD, 0.0, 0.0]),
    ("roll right", [DEFLECT_RAD, -DEFLECT_RAD, 0.0, 0.0, 0.0, 0.0]),
    ("nose up", [0.0, 0.0, 0.0, 0.0, -DEFLECT_RAD, -DEFLECT_RAD]),
    ("yaw right", [0.0, 0.0, 0.0, 0.0, DEFLECT_RAD, -DEFLECT_RAD]),
    ("neutral", [0.0] * PANELS),
):
    print(f"-- {label} --")
    for i in range(60):  # ~2.4 s per pose
        robot.set_fw_surfaces(surfaces)
        robot.set_fw_thrust(CRUISE_N)
        if i % 30 == 0:
            m = robot.states.actuator.measured  # [0:6] panels in rad, [6] engine in N
            print(f"   panels={[round(v, 3) for v in m[:PANELS]]} engine={m[PANELS]:.0f} N")
        robot.rate(HZ)

# the engine channel is newtons, not a pulse width
robot.set_fw_surfaces([0.0] * PANELS)
robot.set_fw_thrust(CLIMB_N)
for _ in range(60):
    robot.rate(HZ)
print(f"engine={robot.states.actuator.measured[PANELS]:.0f} N for a commanded {CLIMB_N} N")

# commands latch with no watchdog: nothing sent for ~2 s, the deflections hold
latched = [DEFLECT_RAD, -DEFLECT_RAD, DEFLECT_RAD, -DEFLECT_RAD, 0.0, 0.0]
robot.set_fw_surfaces(latched)
for _ in range(50):
    robot.rate(HZ)
print(f"latched: panels={[round(v, 3) for v in robot.states.actuator.measured[:PANELS]]}")

# reset() reverts the mode to onboard, so direct mode must be re-asserted after every reset
robot.reset()
robot.set_fw_ctrl_mode(cmd.FW_DIRECT_SURFACE)
robot.set_fw_thrust(CRUISE_N)  # thrust after every mode entry: bumpless is not zeroed

robot.set_fw_ctrl_mode(cmd.FW_ONBOARD_RATE)  # hand it back; scene-authored, never deleted
