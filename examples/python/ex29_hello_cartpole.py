"""ex29 - balance a scene-authored cart-pole with a rate-and-position force loop."""

import math
import sys

from vrsdk import RobotType, VirtualRobot

sys_id = int(sys.argv[1])  # scene-authored: find the live id with `vrobots topic list`

K_THETA = 25.0  # N per rad
K_THETA_DOT = 8.0  # N per rad/s
K_X = 0.5  # N per m
K_V = 0.8  # N per m/s
MAX_FORCE_N = 20.0
FALLEN_RAD = 0.35  # ~20 deg: past here, reset the episode

robot = VirtualRobot(RobotType.CARTPOLE, sys_id=sys_id)
robot.connect()

# cart mass belongs to configure_cartpole; set_physical_params would be overwritten
robot.configure_cartpole(
    cart_mass=1.0,
    travel_half_range=4.0,
    pole_rod_mass=0.1,
    bob_mass=0.2,
    pole_length=1.2,
    pole_angular_damping=0.01,
    max_force=MAX_FORCE_N,
    initial_pole_angle_deg=-3.0,  # DEGREES, and it re-seats the pole immediately
)

# reset() returns the cart to the rail centre, which is NOT the world origin
robot.reset()
for _ in range(15):
    robot.rate(25)
rail_centre = robot.states.kin.lin_pos[0]
print(f"rail centre at world x = {rail_centre:+.2f} m")

for i in range(750):  # ~30 s at the 25 Hz state rate
    robot.wait_new_state(0.5)
    s = robot.states
    x = s.kin.lin_pos[0] - rail_centre  # position is world; subtract the rail centre
    v = s.kin.lin_vel[0]  # velocity is body, already along the rail
    theta = s.actuator.measured[1]  # pole angle, radians; 0 = upright
    theta_dot = s.actuator.measured[2]

    if abs(theta) > FALLEN_RAD:
        print(f"fallen ({math.degrees(theta):+.1f} deg) -- resetting the episode")
        robot.set_cartpole_force(0.0)
        robot.reset()
        for _ in range(15):
            robot.rate(25)
        continue

    force = -K_THETA * theta - K_THETA_DOT * theta_dot + K_X * x + K_V * v
    force = min(max(force, -MAX_FORCE_N), MAX_FORCE_N)
    robot.set_cartpole_force(force)

    if i % 25 == 0:
        print(
            f"t={s.elapsed:7.2f}s  theta={math.degrees(theta):+7.2f} deg  "
            f"rail={x:+6.2f} m  x'={v:+6.2f} m/s  F={force:+6.2f} N"
        )

robot.set_cartpole_force(0.0)  # the force latches; scene-authored robot, never deleted
