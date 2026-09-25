"""ex32 - external rate PID for the Global Hawk, reading the in-game stick off z/cmd."""

import math
import sys

from vrsdk import RobotType, VirtualRobot, cmd

sys_id = int(sys.argv[1])  # scene-authored (IMU scene): find the live id with `vrobots topic list`

LIMIT_RAD = math.radians(20.0)  # the airframe's surface clamp
TRIM_MPS = 72.8
CRUISE_N = 3800.0
KP = (0.16, 0.24, 0.35)  # the simulator's own rate gains, FRD [roll, pitch, yaw]
KI = (0.05, 0.08, 0.10)
FF = (0.29, 0.09, 0.14)

robot = VirtualRobot(RobotType.GLOBALHAWK, sys_id=sys_id, coord_frame_id="frd", axis_convention=2)
robot.connect()
own_src_id = robot.options["src_id"]

# z/cmd is a bus: the IMU panel's SET_ANGVEL setpoints are readable by subscribing
setpoints = robot.subscribe_setpoint()
print(f"watching {setpoints.key}; fly with the sim's IMU panel")

robot.set_fw_ctrl_mode(cmd.FW_DIRECT_SURFACE)
robot.set_fw_thrust(CRUISE_N)  # thrust after every mode entry: direct mode inherits, not zeroes

integral = [0.0, 0.0, 0.0]
previous_elapsed = robot.states.elapsed

for i in range(1500):  # ~60 s at the 25 Hz state rate
    robot.wait_new_state(0.5)
    s = robot.states
    dt = min(max(s.elapsed - previous_elapsed, 0.0), 0.2)
    previous_elapsed = s.elapsed

    # a setpoint latches, so read the CURRENT one every cycle, and skip our own traffic
    setpoint = setpoints.latest
    if setpoint is None or setpoint.src_id == own_src_id:
        demand = (0.0, 0.0, 0.0)  # no setpoint yet: hold zero rates
    else:
        demand = setpoint.value
    measured = s.kin.ang_vel

    # surface effectiveness grows as v^2, so scale the loop with (V_trim / v)^2
    airspeed = max(math.dist(s.kin.lin_vel, (0.0, 0.0, 0.0)), 1.0)
    q_scale = min(max(TRIM_MPS**2 / airspeed**2, 0.05), 2.0)

    out = []
    for axis in range(3):
        error = q_scale * (demand[axis] - measured[axis])
        integral[axis] += error * dt
        limit = LIMIT_RAD / KI[axis]  # anti-windup: the integral alone never exceeds the clamp
        integral[axis] = min(max(integral[axis], -limit), limit)
        raw = FF[axis] * demand[axis] * q_scale + KP[axis] * error + KI[axis] * integral[axis]
        out.append(min(max(raw, -LIMIT_RAD), LIMIT_RAD))
    aileron, elevator, rudder = out

    # the simulator's own mixer: the inner flaps (2, 3) have zero gain
    surfaces = [aileron, -aileron, 0.0, 0.0, -elevator + rudder, -elevator - rudder]
    surfaces = [min(max(d, -LIMIT_RAD), LIMIT_RAD) for d in surfaces]
    robot.set_fw_surfaces(surfaces)
    robot.set_fw_thrust(CRUISE_N)

    if i % 25 == 0:
        print(
            f"t={s.elapsed:7.2f}s "
            f"demand=({demand[0]:+6.3f},{demand[1]:+6.3f},{demand[2]:+6.3f}) "
            f"measured=({measured[0]:+6.3f},{measured[1]:+6.3f},{measured[2]:+6.3f}) rad/s  "
            f"v={airspeed:5.1f} m/s q={q_scale:4.2f}"
        )

robot.set_fw_ctrl_mode(cmd.FW_ONBOARD_RATE)  # hand it back; scene-authored, never deleted
