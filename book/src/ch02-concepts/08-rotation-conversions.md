# Rotation conversions

Turning an attitude between quaternions, angles and matrices, and re-expressing a whole state in another frame.

```sh
cargo run -p vrobots-examples --bin ex36_rotations
./target/cpp-build/ex36_rotations
python examples/python/ex36_rotations.py
```

`vrobots_sdk::rotations` talks to nothing: pure math, shared with the C++ namespace
`vrsdk::rotations` and the Python submodule `vrsdk.rotations`, so "convert this quaternion
to roll, pitch and yaw" has one answer. ex36 creates its own truck, drives an arc so there
is something non-zero to convert, and deletes it, so it takes no `sys_id`. [Frames, axes
and units](07-frames-and-units.md) owns the storage conventions; this page is the
arithmetic on top of them.

## Storage order is fixed; application order is an argument

**Storage** never changes. An Euler triple is `[about x, about y, about z]`, reading as
`[roll, pitch, yaw]` in a body frame whatever sequence the rotations are applied in; a
quaternion is `[x, y, z, w]`, scalar last; a matrix is row-major 3x3, so `r[1][2]` is row 1,
column 2.

**Application** is the `EulerOrder` argument. `Zyx` applies yaw first, then pitch, then
roll, and reads `euler[2]`, `euler[1]`, `euler[0]` in that sequence. It never reorders the
array, so `euler_to_quat([roll, pitch, yaw], Zyx)` and the same triple under `Xyz` build
different rotations from identical numbers. There is no `[yaw, pitch, roll]` layout here.

| Constant | Wire value | Applied | Built-in frames reporting in it |
|---|---|---|---|
| `Xyz` | 1 | about x, then the new y, then the newest z | none |
| `Xzy` | 2 | about x, then the new z, then the newest y | none |
| `Yxz` | 3 | about y, then the new x, then the newest z | none |
| `Yzx` | 4 | about y, then the new z, then the newest x | none |
| `Zxy` | 5 | about z, then the new x, then the newest y | `"unity"` |
| `Zyx` | 6 | about z, then the new y, then the newest x | `"frd"`, `"fru"`, `"cv"` |

The values are `swarmbotix.coordinates.EulerOrder`'s, so `to_wire()` is a cast. Zero is
`EULER_ORDER_UNSPECIFIED`, not an order and with no variant here. Python spells the
variants `EulerOrder.ZYX`; Rust and C++ spell them `EulerOrder::Zyx`.

> **Gotcha.** `vrobots_sdk::EulerOrder` and `rotations::EulerOrder` are different types with
> the same name: the first is the **wire tag**, a transparent `i32` keeping whatever a
> publisher sent, the second the **math parameter**, closed because a conversion handed a
> value that names no order has nothing correct to do. Cross with
> `EulerOrder::from_wire(tag.0)` and handle the `None`.

## The six conversions

A quaternion here is Hamilton, unit and body-to-world: `rotate_vec3` takes body components
and returns world components. Euler angles are intrinsic Tait-Bryan, each rotation about an
axis of the frame the previous one produced, matching the simulator.

| From | To | Function |
|---|---|---|
| Euler | quaternion | `euler_to_quat(euler, order)` |
| Euler | matrix | `euler_to_rotmat(euler, order)` |
| quaternion | Euler | `quat_to_euler(quat, order)` |
| quaternion | matrix | `quat_to_rotmat(quat)` |
| matrix | Euler | `rotmat_to_euler(r, order)` |
| matrix | quaternion | `rotmat_to_quat(r)` |

Around them sit `quat_multiply`, `quat_conjugate`, `quat_normalize`, `rotate_vec3`,
`rotmat_multiply`, `rotmat_transpose`, `rotmat_det`, `rotmat_abs` and `rotmat_apply`.
`quat_multiply(a, b)` is "b first, then a", which is why ex35 post-multiplies to bias about
a body axis. All are pure and total: no panics, no `NaN` out of finite input, no
allocation, and a zero-length quaternion normalizes to `IDENTITY_QUAT` rather than
erroring.

## A frame is nine numbers, and the wire carries them

A convention is one signed-permutation matrix `R` whose rows are the Unity axes that map
onto that frame's axes, so `R * v` re-expresses a Unity vector in frame components. That is
what `z/frames` publishes, and it is all a peer needs: the inverse is the transpose, the
handedness `det(R)`, and north, east and down are `R` on the Unity world anchor.

| Frame id | Axes | Handed | Euler order | `Axes` constant |
|---|---|---|---|---|
| `"unity"` | +x right, +y up, +z forward | left | `Zxy` | `Axes::UNITY` |
| `"frd"` | +x forward, +y right, +z down | right | `Zyx` | `Axes::FRD` |
| `"fru"` | +x forward, +y right, +z up | left | `Zyx` | none |
| `"cv"` | +x right, +y down, +z forward | right | `Zyx` | `Axes::CV` |

`AxisBasis` is that matrix plus the frame's reporting order; `FrameTransform` is the
`R_to * R_from^T` product between two of them. Both have constructors for the four
built-ins, and `AxisBasis::from_frame_def` builds one from a definition read off the wire,
the only route that reaches a frame with no `Axes` constant. `"fru"` is that case today,
and a scene registering a convention at runtime is the same problem further out.

ex36 reads the definition with `frame_def()` and turns it into a basis. From
`examples/rust/src/bin/ex36_rotations.rs`:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    let basis = AxisBasis::from_frame_def(&def).ok_or_else(|| {
        VrError::InvalidArgument(format!(
            "frame {:?} names no euler order and its axis convention has no built-in default, \
             so there is no order to report angles in",
            def.id
        ))
    })?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex36_rotations.cpp`:

```cpp
// Throws when the definition names no order and its axis convention has
// no built-in default: there is then no order to report angles in.
const vrsdk::rotations::AxisBasis basis = vrsdk::rotations::AxisBasis::from_frame_def(def);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex36_rotations.py`:

```python
    basis = rotations.AxisBasis.from_frame_def(fdef)
    if basis is None:
        raise SystemExit(
            f"frame {fdef.id!r} names no euler order and its axis convention has no "
            "built-in default, so there is no order to report angles in"
        )
```

{{#endtab }}
{{#endtabs }}

Rust returns `Option`, Python `None` and C++ throws, all saying the same thing: a basis
with a guessed order extracts angles that look plausible and mean nothing. For the truck's
`"fru"` the call succeeds, and the line under it reads

```text
  det(R)=+1, right-handed=false
```

> **Gotcha.** `det(R) = -1` means **right-handed**, not left: the determinant is of the map
> out of left-handed Unity, so a right-handed frame is the one that flips the sign.
> `is_right_handed()` exists so nothing has to remember that.

## Which rule applies depends on what the vector is

`M * v` is right for a polar vector and wrong for an axial one. An axial vector is defined
by a cross product, so it picks up an extra sign when the basis flips handedness. The
simulator splits its conversions the same way, so this is a rule the SDK agrees with rather
than one it chose.

| Category | Rule | Method | State fields |
|---|---|---|---|
| polar | `M * v` | `apply_vec3` | `lin_pos`, `lin_vel`, `lin_acc`, `wrench.force`, `env.gravity`, accelerometer, magnetometer, GNSS and optical-flow velocity |
| axial | `det(M) * M * v` | `apply_axial_vec3` | `ang_vel`, `ang_acc`, `wrench.torque`, gyroscope |
| diagonal inertia | `abs(M) * v` | `apply_inertia_vec3` | `PhysicalParams::moi`, a permutation with no sign, because a moment of inertia is positive |
| orientation | `M * C * M^T` | `apply_quat`, `apply_rotmat` | `quat` |

Between two frames of the same handedness `det(M) = +1` and the first two rules coincide,
which is why picking the wrong one survives testing until somebody crosses a flip. ex36
converts the live gyro both ways. From `examples/rust/src/bin/ex36_rotations.rs`:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    // axial: the gyro, and the mistake beside it
    let gyro = state.sensors.gyroscope.angular_velocity;
    let gyro_frd = t.apply_axial_vec3(gyro);
    let gyro_wrong = t.apply_vec3(gyro);
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex36_rotations.cpp`:

```cpp
// axial: the gyro, and the mistake beside it
const vrsdk::Vec3 gyro = gyro_of(state);
const vrsdk::Vec3 gyro_frd = t.apply_axial_vec3(gyro);
const vrsdk::Vec3 gyro_wrong = t.apply_vec3(gyro);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex36_rotations.py`:

```python
    # axial: the gyro, and the mistake beside it
    gyro = state.sensors.gyroscope.angular_velocity
    gyro_frd = t.apply_axial_vec3(gyro)
    gyro_wrong = t.apply_vec3(gyro)
```

{{#endtab }}
{{#endtabs }}

The truck reports in `"fru"` and the target is `"frd"`, so `M` is `diag(1, 1, -1)` and
`det(M) = -1`: every component of the axial answer is the negation of the polar one, and a
body rate converted with the polar rule sends the robot spinning the other way. A roll rate
of `[0, 0, 1]` in Unity axes converts to `[-1, 0, 0]` under `apply_axial_vec3` and to
`[+1, 0, 0]` under `apply_vec3`, which is the whole trap in two lines.

For a pair the `Axes` tags can name, `convert_vec3`, `convert_axial_vec3`,
`convert_inertia_vec3`, `convert_quat`, `convert_rotmat` and `convert_euler` do the same in
one call, each returning `VrResult`. Handed the truck's own `axis_convention`, which is
`UNSPECIFIED`, they fail with `InvalidArgument` naming the three constants that exist. That
is why `frame_def()` is there: the tag is a convenience and the nine numbers are the fact.

## Gimbal lock has an answer, and it is the simulator's

With the middle angle at a quarter turn the two outer rotations act about the same axis and
only their difference survives, so no unique triple is left. `rotmat_to_euler` and
`quat_to_euler` return one anyway: the angle about z is pinned to zero, because heading is
the meaningless quantity when the nose is vertical, and the whole determined combination
goes into the other outer angle. Where z is itself the middle axis and cannot be pinned,
the last angle of the sequence is zeroed instead. That matches
`CoordFrame.MatrixToEulerDeg`, so a locked attitude decodes here to the triple the
simulator shows.

ex36's last section builds an attitude from `LOCKED_DEG`, which is `(0, 90, 40)` degrees
with the pitch exactly at the pole, and asks for the angles back through the same
`euler_to_quat` and `quat_to_euler` pair the table above lists:

```text
  in    roll=  +0.00 deg  pitch= +90.00 deg  yaw= +40.00 deg
  out   roll= -40.00 deg  pitch= +90.00 deg  yaw=  +0.00 deg
```

Different numbers, same rotation: the example rebuilds the matrix from the extracted triple
and checks it element by element. A naive `atan2` pair returns neither answer, because both
its terms are rounding noise at the pole, which is how `(0, 90, 40)` comes back from a
hand-inlined formula as `(26.6, 90, 90)`.

**Next:** [Reading state](../ch03-reading-state/00-intro.md)

**See also:** [Frames, axes and units](07-frames-and-units.md), [Coordinate frames](../ch06-services/04-frames-config.md), [Publishing estimates](../ch04-commands/09-publishing-estimates.md)
