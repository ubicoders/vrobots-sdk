# Appendix B: Command reference

Every command id, its arguments, the robots that act on it, and its status.

Command ids share one numbering space across every robot type, so an id is only ever
implemented by some of them, and sending one to a robot that does not implement it is
ignored exactly like sending an id that does not exist. Nothing on this wire is
acknowledged: the state stream's actuator echo is the only receipt. In the Status column,
**live** means a robot type acts on the id today, **not yet** means it is on the wire and no
robot acts on it, and **absent** means the robot type it belongs to is not in the simulator.
The constants live in `vrobots_sdk::cmd`, and `cmd::name(id)` maps a value back to its
constant name for logging, returning `""` for an unknown id.

## Command ids

| Constant | Value | `CmdArgs` field | Units | SDK method | Acted on by | Status |
|---|---|---|---|---|---|---|
| `SET_ACC` | 1 | not documented | m/s² | `send_cmd` | nothing | not yet |
| `SET_VEL` | 2 | not documented | m/s | `send_cmd` | nothing | not yet |
| `SET_POS` | 3 | not documented | m | `send_cmd` | nothing | not yet |
| `SET_ANGACC` | 50 | not documented | rad/s² | `send_cmd` | nothing | not yet |
| `SET_ANGVEL` | 51 | `vec3` | rad/s, body rates in your header frame; converted as an **axial** vector, so it carries the handedness sign | `set_angvel`; read back with `subscribe_setpoint` | GlobalHawk onboard rate loop | **live** |
| `SET_EULER` | 52 | not documented | rad | `send_cmd` | nothing | not yet |
| `SET_EULER_DOT` | 53 | not documented | rad/s | `send_cmd` | nothing | not yet |
| `SET_QUAT` | 54 | not documented | unit quaternion, `[x, y, z, w]` | `send_cmd` | nothing | not yet |
| `SET_MASS` | 100 | not documented | kg | `send_cmd`; use `set_physical_params` instead | nothing | not yet |
| `SET_MOI_3X1` | 101 | not documented | kg·m² | `send_cmd`; use `set_physical_params` | nothing | not yet |
| `SET_MOI_3X3` | 102 | not documented | kg·m² | `send_cmd`; use `set_physical_params` | nothing | not yet |
| `SET_BODY_FORCE` | 200 | `vec3` | N, header frame | `set_body_force` | nothing | not yet |
| `SET_BODY_TORQUE` | 201 | `vec3` | N·m, header frame | `set_body_torque` | nothing | not yet |
| `SET_BODY_FT` | 202 | `vec3` force, `vec3_arr[0]` torque | N and N·m | `set_body_ft` | nothing | not yet |
| `ADD_BODY_FORCE` | 203 | not documented | N | `send_cmd` | nothing | not yet |
| `ADD_BODY_TORQUE` | 204 | not documented | N·m | `send_cmd` | nothing | not yet |
| `ADD_BODY_FT` | 205 | not documented | N and N·m | `send_cmd` | nothing | not yet |
| `SET_MR_PWM` | 300 | `int_arr`, one per rotor | µs, 1100 to 2000, checked client-side | `set_mr_pwm`, `set_mr_pwm_n` | Multirotor (any rotor count), HalfDrone (exactly 2, `[left, right]`) | **live** |
| `SET_MR_THROTTLE` | 301 | `float_arr` | normalised | `set_mr_throttle` | nothing | not yet |
| `SET_OMROVER` | 302 | not documented | not documented | `send_cmd` | omnidirectional rover | absent |
| `SET_HELI` | 303 | not documented | not documented | `send_cmd` | helicopter | absent |
| `SET_CAR` | 304 | `int_arr`, `[steer, throttle]` or `[steer, throttle, brake]` | µs, 1100 to 2000 checked client-side; the truck's factory band is 1100/1500/1900 | `set_car` | Truck | **live** |
| `SET_MSD` | 305 | `float_val` | N, clamped simulator-side to `max_force` (100 N by default) | `set_msd_force` | Msd | **live** |
| `SET_INVPEN` | 306 | `float_val` | N along the rail's +x, clamped to `CartPoleConfig::max_force` (20 N by default) | `set_cartpole_force` | CartPole | **live** |
| `SET_FW_SURFACES` | 307 | `float_arr`, one per panel | rad, clamped simulator-side to the airframe limit (20 degrees by default) | `set_fw_surfaces` | GlobalHawk | **live** |
| `SET_FW_THRUST` | 308 | `float_val` | N, clamped to `[0, max_thrust]` (20 kN on the RQ-4B) | `set_fw_thrust` | GlobalHawk | **live** |
| `SET_FW_THRUST_BIAS` | 309 | `float_val` | N, signed trim; ignored in `FW_DIRECT_SURFACE` | `set_fw_thrust_bias` | GlobalHawk | **live** |
| `SET_FW_CTRL_MODE` | 310 | `int_val` | `FW_ONBOARD_RATE` or `FW_DIRECT_SURFACE` | `set_fw_ctrl_mode` | GlobalHawk | **live** |
| `SET_FW_EST_SOURCE` | 311 | `int_val` | `FW_EST_TRUTH` or `FW_EST_OBSERVER` | `set_fw_est_source` | GlobalHawk | **live** |

Where the `CmdArgs` field says "not documented", the SDK ships no typed method for that id
and the source does not name the field it reads, so the payload shape has to come from the
simulator before you build on it. The units given for those ids follow from the SDK's
everything-is-SI rule and the constant's name rather than from a statement in the source.
<!-- VERIFY: units for the not-yet-acted-on ids (1, 2, 3, 50, 52, 53, 54, 100, 101, 102, 203, 204, 205) are inferred from the SI rule plus the constant name; nothing in the repo states them. -->

The `CmdArgs` field for `SET_ANGVEL` is `vec3`, verifiable from both ends: `set_angvel`
builds `CmdArgs::vector(rates)`, and on the way back `subscribe_command` yields a
`Setpoint` only for commands carrying a `vec3`, with `subscribe_setpoint` as shorthand for
`subscribe_command(cmd::SET_ANGVEL)`.

> **Not yet.** Seventeen ids above are carried by the schema and acted on by nothing. They
> publish without error and the state stream does not change, which is indistinguishable
> from a wrong `sys_id` or a wrong array length. [Commands nothing acts
> on](ch04-commands/07-ignored-commands.md) shows what that looks like from the outside.

## Fixed wing mode constants

These are `i32` values carried in `CmdArgs::int_val`, not command ids. They select the
behaviour of `SET_FW_CTRL_MODE` and `SET_FW_EST_SOURCE`.

| Constant | Value | Belongs to | Meaning |
|---|---|---|---|
| `FW_ONBOARD_RATE` | 0 | `SET_FW_CTRL_MODE` | the default; the aircraft flies itself on an onboard rate loop tracking `SET_ANGVEL`, with airspeed hold |
| `FW_DIRECT_SURFACE` | 1 | `SET_FW_CTRL_MODE` | the rate loop is bypassed and the panels take `SET_FW_SURFACES` verbatim |
| `FW_EST_TRUTH` | 0 | `SET_FW_EST_SOURCE` | the default; the onboard loop uses the simulator's true attitude |
| `FW_EST_OBSERVER` | 1 | `SET_FW_EST_SOURCE` | the onboard loop uses the attitude published on the robot's `z/estimate` topic, by `publish_estimate` or any other peer |

`reset()` returns both settings to their defaults, deliberately: direct surface control with
zeroed latches would relaunch the aircraft unflyable. A direct-surface client re-asserts both
after every reset.

## The `CmdArgs` shape

Every command carries the same argument struct, and each id reads the one or two fields it
cares about.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
#[non_exhaustive]
pub struct CmdArgs {
    pub int_val: i32,
    pub float_val: f64,
    pub int_arr: Vec<i32>,
    pub float_arr: Vec<f64>,
    pub vec3: Option<[f64; 3]>,
    pub vec4: Option<[f64; 4]>,   // [x, y, z, w]
    pub vec3_arr: Vec<[f64; 3]>,
    pub vec4_arr: Vec<[f64; 4]>,
}
```

{{#endtab }}
{{#tab name="C++" }}

`crates/vrobots-sdk-capi/include/vrobots_sdk.h`:

```c
typedef struct vrsdk_cmd_args_t {
    int32_t int_val;
    double float_val;
    const int32_t *int_arr;
    size_t int_arr_len;
    const double *float_arr;
    size_t float_arr_len;
    const double *vec3;
    const double *vec4;
    const double *vec3_arr;
    size_t vec3_arr_len;
    const double *vec4_arr;
    size_t vec4_arr_len;
} vrsdk_cmd_args_t;
```

{{#endtab }}
{{#tab name="Python" }}

`crates/vrobots-sdk-py/python/vrsdk/_vrsdk.pyi`:

```python
int_val: int = 0
float_val: float = 0.0
int_arr: Optional[Sequence[int]] = None
float_arr: Optional[Sequence[float]] = None
vec3: Optional[Sequence[float]] = None
vec4: Optional[Sequence[float]] = None
vec3_arr: Optional[Sequence[Sequence[float]]] = None
vec4_arr: Optional[Sequence[Sequence[float]]] = None
```

{{#endtab }}
{{#endtabs }}

The same eight fields carry the same meanings in all three. C++ uses the C struct directly, so
every array is a pointer with an explicit `_len`, and `vec3_arr` and `vec4_arr` are flat
`double` arrays whose length counts vectors rather than doubles. Python has no argument type at
all: the eight are keyword arguments of `send_cmd`.

The struct is `#[non_exhaustive]`, so it cannot be built with a struct literal. Build it from
`CmdArgs::default()` plus chained setters, or from a shorthand.

| Builder | Sets | Typical use |
|---|---|---|
| `CmdArgs::ints(&[i32])` | `int_arr` | `SET_MR_PWM`, `SET_CAR` |
| `CmdArgs::floats(&[f64])` | `float_arr` | `SET_FW_SURFACES` |
| `CmdArgs::vector([f64;3])` | `vec3` | `SET_BODY_FORCE` |
| `with_int_val`, `with_float_val` | `int_val`, `float_val` | `SET_FW_CTRL_MODE`, `SET_MSD` |
| `with_int_arr`, `with_float_arr` | `int_arr`, `float_arr` | |
| `with_vec3`, `with_vec4` | `vec3`, `vec4` | |
| `with_vec3_arr`, `with_vec4_arr` | `vec3_arr`, `vec4_arr` | `SET_BODY_FT`'s torque half |

Each `with_*` setter consumes `self` and is `#[must_use]`, so they chain. Empty vectors and
`None` are omitted from the wire entirely rather than sent at zero length. Floats narrow:
`f64` in the API, `f32` on the wire.

`VirtualRobot::send_cmd(&self, cmd_id: u32, args: &CmdArgs) -> VrResult<()>` is the escape
hatch for an id with no typed method. It returns `Deleted` if the robot has been deleted and
`Publish` if zenoh refuses the put, and nothing else: it cannot tell you whether any robot
acted on the id.

**Next:** [Appendix C: Error reference](appendix-c-errors.md)

**See also:** [The generic command](ch04-commands/06-generic-cmd.md), [Commands latch](ch04-commands/01-latching.md)
