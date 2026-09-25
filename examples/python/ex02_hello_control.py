"""ex02 - send raw per-rotor PWM; the echo in the state is the receipt."""

from vrsdk import RobotType, VirtualRobot

PWM_US = 1501.0  # 1100-2000 band; barely off idle, edit to 1700 to climb

mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor
mr.connect()

while True:
    s = mr.states
    x, y, z = s.kin.lin_pos
    print(
        f"State t={s.elapsed:.3f} pos=({x:.3f},{y:.2f},{z:.2f}) "
        f"echo={s.actuator.pwm}"
    )

    # Compute your control here and publish; commands latch until the next one.
    mr.set_mr_pwm([PWM_US] * 4)

    mr.rate(100)
