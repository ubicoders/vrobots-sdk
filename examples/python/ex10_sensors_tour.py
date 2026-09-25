"""ex10 - walk every block of one state snapshot, printed once a second."""

from vrsdk import RobotType, VirtualRobot


def v3(v):
    return "(" + ",".join(f"{c:+8.3f}" for c in v) + ")"


def stamp(valid, timestamp):
    return f"[{'valid' if valid else 'INVALID'} t={timestamp:.3f}]"


mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor, 0 = truck
mr.connect()

while True:
    s = mr.states

    print(
        f"\n=== {s.name} sys_id={s.sys_id} seq={s.seq} t={s.elapsed:.3f}s "
        f"schema={s.schema_version} frame={s.coord_frame_id!r} "
        f"({s.axis_convention_name}) ==="
    )

    # truth: simulator ground truth, absent on a real robot
    k = s.kin
    print("TRUTH  kinematics")
    print(f"  lin_pos   {v3(k.lin_pos)} m       (world)")
    quat = ",".join(f"{c:+.3f}" for c in k.quat)
    print(f"  quat      [{quat}] (world, xyzw)")
    print(f"  lin_vel   {v3(k.lin_vel)} m/s     (body)")
    print(f"  ang_vel   {v3(k.ang_vel)} rad/s   (body -- what a gyro measures)")
    print(f"  lin_acc   {v3(k.lin_acc)} m/s^2   (body)")
    print(f"  ang_acc   {v3(k.ang_acc)} rad/s^2 (body)")
    print(f"  wrench    F={v3(s.wrench.force)} N  T={v3(s.wrench.torque)} N.m")

    # measured: the noisy, robot-observable view; each sensor has its own clock
    n = s.sensors
    print("MEASURED  sensors")
    print(
        f"  accel     {v3(n.accelerometer.linear_acceleration)} m/s^2  "
        f"{stamp(n.accelerometer.valid, n.accelerometer.timestamp)}"
        "   [specific force: +1 g at rest]"
    )
    print(
        f"  gyro      {v3(n.gyroscope.angular_velocity)} rad/s  "
        f"{stamp(n.gyroscope.valid, n.gyroscope.timestamp)}"
    )
    print(
        f"  mag       {v3(n.magnetometer.magnetic_field)} gauss  "
        f"{stamp(n.magnetometer.valid, n.magnetometer.timestamp)}"
    )
    print(
        f"  baro      {n.barometer.pressure:.1f} Pa  alt={n.barometer.altitude:.2f} m "
        f"(qnh {n.barometer.qnh:.1f} hPa)  "
        f"{stamp(n.barometer.valid, n.barometer.timestamp)}"
    )
    g = n.gnss.geo_point
    print(
        f"  gnss      lat={g.latitude:.6f} lon={g.longitude:.6f} alt={g.altitude:.2f} m  "
        f"vel={v3(n.gnss.velocity)} m/s (NED)"
    )
    print(
        f"            fix={n.gnss.fix_type} eph={n.gnss.eph:.2f} epv={n.gnss.epv:.2f} m  "
        f"{stamp(n.gnss.valid, n.gnss.timestamp)}   [slowest device, ~5 Hz]"
    )
    print(
        f"  flow      {v3(n.optical_flow.velocity)} m/s  "
        f"{stamp(n.optical_flow.valid, n.optical_flow.timestamp)}"
        "   [optional; mount it via srv/sensors]"
    )

    # believed: the robot's own filter; valid=False means no estimator runs
    e = s.estimate
    print(
        f"BELIEVED  estimate  {stamp(e.valid, e.timestamp)}  "
        f"frame={e.coord_frame_id!r}"
    )
    print(f"  lin_pos   {v3(e.kin.lin_pos)} m       (estimate.kin - kin IS the error)")
    print(f"  lin_vel   {v3(e.kin.lin_vel)} m/s")

    env = s.env
    print("WORLD  environment")
    print(
        f"  gravity   {v3(env.gravity)} m/s^2   air {env.air_pressure:.1f} Pa "
        f"{env.air_density:.3f} kg/m^3 {env.temperature:.1f} C"
    )
    print(
        f"  agl       {env.agl:.2f} m    home lat={env.geo_point.latitude:.6f} "
        f"lon={env.geo_point.longitude:.6f}   "
        f"[agl is hard-coded 0 in sim v3.0.0 -- use -lin_pos[2]]"
    )
    print("ACTUATORS  command in, motion out")
    print(f"  pwm        {s.actuator.pwm} us      (echo of the last command)")
    print(f"  normalized {[round(v, 3) for v in s.actuator.normalized]}")
    print(
        f"  measured   {[round(v, 3) for v in s.actuator.measured]}"
        "   (rotor rad/s -- what the devices did)"
    )

    mr.rate(1)
