"""ex28 - step-force a mass-spring-damper and compare against the closed-form response."""

import math

from vrsdk import RobotType, VirtualRobot

MASS_KG = 1.0
STEP_N = 20.0
HZ = 25

robot = VirtualRobot(RobotType.MSD)
robot.connect()
robot.set_physical_params(mass=MASS_KG)

for label, k, c, retune in (
    ("as spawned (k=20, c=1)", 20.0, 1.0, False),
    ("k=80, c=1", 80.0, 1.0, True),
    ("k=80, c=16", 80.0, 16.0, True),
):
    print(f"-- {label} --")
    if retune:
        robot.configure_msd(spring_k=k, damping_c=c)
    robot.set_msd_force(0.0)  # the force latches; clear it before resetting
    robot.reset()

    for i in range(125):  # ~5 s pushing
        robot.set_msd_force(STEP_N)
        if i % 25 == 0:
            s = robot.states
            m = s.actuator.measured  # [0] net force F - k*x - c*x', [1] displacement
            print(
                f"   t={s.elapsed:6.2f}s x={s.kin.lin_pos[0]:+7.3f} m  "
                f"x'={s.kin.lin_vel[0]:+7.3f} m/s  disp={m[1]:+7.3f} m  net F={m[0]:+8.2f} N"
            )
        robot.rate(HZ)

    print("   release (set_msd_force(0.0)) -- ring back to equilibrium")
    for i in range(125):  # ~5 s ringing down
        robot.set_msd_force(0.0)
        if i % 50 == 0:
            s = robot.states
            print(
                f"   t={s.elapsed:6.2f}s x={s.kin.lin_pos[0]:+7.3f} m  "
                f"x'={s.kin.lin_vel[0]:+7.3f} m/s"
            )
        robot.rate(HZ)

    print(
        f"   predicted: settles at F/k={STEP_N / k:.3f} m, "
        f"period 2*pi*sqrt(m/k)={math.tau * math.sqrt(MASS_KG / k):.2f} s, "
        f"zeta={c / (2.0 * math.sqrt(k * MASS_KG)):.2f}"
    )

robot.delete()
