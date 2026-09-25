"""ex24 - configure sensor noise and watch it appear in the state stream."""

import math

import vrsdk
from vrsdk import RobotType, VirtualRobot

SAMPLES = 100  # ~4 s at the 25 Hz state rate

robot = VirtualRobot(RobotType.MULTIROTOR)
robot.connect()
print(f"created sys_id={robot.sys_id}")

# Let the fresh spawn land and stop moving: with the robot at rest, truth is
# constant and the spread of each sensor reading IS its noise.
for _ in range(50):
    robot.rate(25)


def stddev(values):
    mean = sum(values) / len(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))


def measure(label):
    gyro, accel, baro = [[], [], []], [[], [], []], []
    for _ in range(SAMPLES):
        robot.wait_new_state(0.5)  # one sample per iteration, or the spread is understated
        s = robot.states
        for axis in range(3):
            gyro[axis].append(s.sensors.gyroscope.angular_velocity[axis])
            accel[axis].append(s.sensors.accelerometer.linear_acceleration[axis])
        baro.append(s.sensors.barometer.pressure)
    s = robot.states
    print(
        f"{label:<11} gyro sigma={[round(stddev(a), 3) for a in gyro]} rad/s  "
        f"accel sigma={[round(stddev(a), 3) for a in accel]} m/s^2  "
        f"baro sigma={stddev(baro):.2f} Pa  eph={s.sensors.gnss.eph:.2f} m  "
        f"flow_valid={s.sensors.optical_flow.valid}"
    )


measure("as spawned")

# ImuNoise() defaults to an IDEAL channel; a field left at zero is applied as
# zero, not "keep the current value".
robot.configure_sensors(
    gyro_noise=vrsdk.ImuNoise(
        white_std=(0.02, 0.02, 0.02),
        bias_instability=(0.002,) * 3,
        bias_tau_s=60.0,
    ),
    accel_noise=vrsdk.ImuNoise(white_std=(0.4, 0.4, 0.4)),
    baro_pressure_noise_std=25.0,
    gps_quality=vrsdk.GpsQuality(eph=4.5, epv=9.0),  # reported quality: echoes back exactly
    gps_noise=vrsdk.GpsNoise(
        position_std=(2.0, 2.0, 3.0),  # NED metres, the error actually applied
        velocity_std=(0.2, 0.2, 0.3),
    ),
    optical_flow_mounted=True,
    optical_flow_noise_std=(0.05,) * 3,
)

measure("configured")

robot.delete()
print(f"deleted sys_id={robot.sys_id}")
