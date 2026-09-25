"""ex36 - vrsdk.rotations: frame definitions, Euler orders, and the four conversion rules."""

import math

from vrsdk import RobotType, VirtualRobot, rotations

HZ = 25.0
AXES_UNITY = 1
AXES_FRD = 2

robot = VirtualRobot(RobotType.TRUCK)
robot.connect()
for _ in range(75):  # drive a left arc ~3 s, so there is something to convert
    robot.set_car(1300.0, 1650.0, 1100.0)
    robot.rate(HZ)

# 1: the frame this robot reports in, as the nine numbers off z/frames
fdef = robot.frame_def()
state = robot.states
print(
    f"frame id={fdef.id!r} axis_convention={fdef.axis_convention_name!r} "
    f"euler_order={fdef.euler_order_name!r}"
)
for row in fdef.rows:
    print(f"  [{row[0]:+6.3f} {row[1]:+6.3f} {row[2]:+6.3f}]")
basis = rotations.AxisBasis.from_frame_def(fdef)
print(
    f"det(R)={basis.det:+.0f} right_handed={basis.is_right_handed}  "
    f"north={basis.north} east={basis.east} down={basis.down}"
)

# 2: the wire carries a quaternion [x, y, z, w]; angles are derived, in the frame's order
order = basis.euler_order
roll, pitch, yaw = (math.degrees(a) for a in rotations.quat_to_euler(state.kin.quat, order))
print(f"quat={state.kin.quat} -> {order.name} roll={roll:+.2f} pitch={pitch:+.2f} yaw={yaw:+.2f} deg")

# 3: re-express the live state in frd; the physical category picks the rule
t = rotations.FrameTransform.between(basis, rotations.AxisBasis.frd())
print(f"det(M)={t.det:+.0f} flips_handedness={t.flips_handedness}")
position_frd = t.apply_vec3(state.kin.lin_pos)  # polar: M * v
gyro = state.sensors.gyroscope.angular_velocity
gyro_frd = t.apply_axial_vec3(gyro)  # axial: det(M) * M * v -- differs across a handedness flip
quat_frd = t.apply_quat(state.kin.quat)  # orientation: M * C * M^T
print(f"position polar  {state.kin.lin_pos} -> {position_frd}")
print(f"gyro     axial  {gyro} -> {gyro_frd}  (polar rule would give {t.apply_vec3(gyro)})")
print(f"quat     orient {state.kin.quat} -> {quat_frd}")

back = t.inverse()  # a change of basis is exact, so the round trip is too
assert max(abs(a - b) for a, b in zip(state.kin.quat, back.apply_quat(quat_frd))) < 1e-12

# 4: the tag-keyed shortcuts, on fixed unity inputs
up = (0.0, 10.0, 0.0)  # ten metres up
rate = (0.0, 0.0, 1.0)  # 1 rad/s about unity's forward axis
moi = (0.1, 0.2, 0.3)
print(f"convert_vec3         {up} -> {rotations.convert_vec3(up, AXES_UNITY, AXES_FRD)}")
print(f"convert_axial_vec3   {rate} -> {rotations.convert_axial_vec3(rate, AXES_UNITY, AXES_FRD)}")
print(f"convert_inertia_vec3 {moi} -> {rotations.convert_inertia_vec3(moi, AXES_UNITY, AXES_FRD)}")
yaw90 = rotations.euler_to_quat((0.0, math.pi / 2, 0.0), rotations.EulerOrder.ZXY)
print(f"convert_quat         {yaw90} -> {rotations.convert_quat(yaw90, AXES_UNITY, AXES_FRD)}")

# 5: gimbal lock in zyx: yaw pins to 0 and roll carries the determined combination
locked = [math.radians(a) for a in (0.0, 90.0, 40.0)]
quat = rotations.euler_to_quat(locked, rotations.EulerOrder.ZYX)
extracted = rotations.quat_to_euler(quat, rotations.EulerOrder.ZYX)
print(f"in (0, 90, 40) deg -> out {[round(math.degrees(a), 2) for a in extracted]} deg")
m1 = rotations.quat_to_rotmat(quat)
m2 = rotations.euler_to_rotmat(extracted, rotations.EulerOrder.ZYX)
assert max(abs(a - b) for r1, r2 in zip(m1, m2) for a, b in zip(r1, r2)) < 1e-12  # same rotation

robot.delete()
