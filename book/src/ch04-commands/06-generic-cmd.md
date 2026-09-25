# The generic command

`send_cmd` reaches the whole command id space, including the ids the SDK has no typed
method for.

```sh
cargo run -p vrobots-examples --bin ex08_generic_cmd
./target/cpp-build/ex08_generic_cmd
python examples/python/ex08_generic_cmd.py
```

## One message, and cmd_id decides what it means


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
pub fn send_cmd(&self, cmd_id: u32, args: &CmdArgs) -> VrResult<()>
```

{{#endtab }}
{{#tab name="C++" }}

`cpp/include/vrobots_sdk.hpp`:

```cpp
/// Publish any command by id -- the escape hatch for the whole
/// `VROBOTS_CMDS` space. Fill `args` with `vrsdk_cmd_args_default` first.
void send_cmd(std::uint32_t cmd_id, const vrsdk_cmd_args_t* args = nullptr)
```

{{#endtab }}
{{#tab name="Python" }}

`crates/vrobots-sdk-py/python/vrsdk/_vrsdk.pyi`:

```python
def send_cmd(
    self,
    cmd_id: int,
    *,
    int_val: int = 0,
    float_val: float = 0.0,
    int_arr: Optional[Sequence[int]] = None,
    float_arr: Optional[Sequence[float]] = None,
    vec3: Optional[Sequence[float]] = None,
    vec4: Optional[Sequence[float]] = None,
    vec3_arr: Optional[Sequence[Sequence[float]]] = None,
    vec4_arr: Optional[Sequence[Sequence[float]]] = None,
) -> None: ...
```

{{#endtab }}
{{#endtabs }}

Python has no `CmdArgs` type at all: the eight payload fields are keyword arguments on the
call, and anything you omit stays off the wire. C++ passes the plain C struct by pointer,
and every field in it is a **borrowed** pointer, so a null means "absent" and nothing is
retained after the call returns.

`Command` is a union by convention: one message shape, and the id decides which payload
fields mean anything. `SET_MR_PWM` reads `int_arr` and ignores the rest, `SET_BODY_FORCE`
reads `vec3`. Fill the fields the target id reads and leave the others alone. Empty vectors
and `None` vectors are omitted from the wire entirely rather than sent as zero length, so
the receiver's "field absent" default applies.

Errors are `VrError::Deleted` if the robot was deleted and `VrError::Publish` if zenoh
refuses the put. There is no validation here, which is the trade: every typed method
validates its arguments, and `send_cmd` publishes whatever you build.

## Sending an implemented id and an ignored one, side by side

`examples/rust/src/bin/ex08_generic_cmd.rs` sends two commands per iteration to the truck,
on purpose:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    loop {
        // (1) An implemented id, built by hand. CmdArgs::ints fills int_arr,
        //     which is the field SET_CAR reads; everything else stays off the
        //     wire.
        let drive = CmdArgs::ints(&[STEER_US, THROTTLE_US, BRAKE_US]);
        robot.send_cmd(cmd::SET_CAR, &drive)?;

        // (2) An id nothing acts on, whose payload rides vec3. Same call, same
        //     Ok(()), no effect. `cmd::name` turns an id back into its schema
        //     name, which is what makes a log line readable.
        let gust = CmdArgs::default().with_vec3(GUST_N);
        robot.send_cmd(cmd::ADD_BODY_FORCE, &gust)?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex08_generic_cmd.cpp`:

```cpp
// ===== loop =====
for (;;) {
    // (1) An implemented id, built by hand. int_arr is the field SET_CAR
    //     reads; everything else stays NULL and off the wire.
    vrsdk_cmd_args_t drive;
    vrsdk_cmd_args_default(&drive);
    drive.int_arr = channels;
    drive.int_arr_len = 3;
    robot.send_cmd(SET_CAR, &drive);

    // (2) An id nothing acts on, whose payload rides vec3. Same call,
    //     no exception, no effect. Every payload field is a borrowed
    //     pointer: NULL means "absent" and nothing is retained after
    //     the call returns, so a stack array is fine.
    vrsdk_cmd_args_t force;
    vrsdk_cmd_args_default(&force);
    force.vec3 = gust;  // exactly 3 doubles
    robot.send_cmd(ADD_BODY_FORCE, &force);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex08_generic_cmd.py`:

```python
# ===== loop =====
while True:
    # (1) An implemented id, built by hand. int_arr is the field SET_CAR
    #     reads; everything else stays off the wire.
    car.send_cmd(cmd.SET_CAR, int_arr=[STEER_US, THROTTLE_US, BRAKE_US])

    # (2) An id nothing acts on, whose payload rides vec3. Same call, no
    #     exception, no effect. `cmd.name` turns an id back into its schema
    #     name, which is what makes a log line readable.
    car.send_cmd(cmd.ADD_BODY_FORCE, vec3=GUST_N)
```

{{#endtab }}
{{#endtabs }}

Python is the shortest because the payload is keyword arguments. C++ is the longest because
each call needs a `vrsdk_cmd_args_default` first, and it also declares the two ids as its
own constants: the C surface ships no `cmd` namespace, so `SET_CAR = 304` and
`ADD_BODY_FORCE = 203` are written out from the schema and there is no `cmd::name` to turn
one back into a label.

Both calls return `Ok(())`. Only one of them changes anything, and the printed lines say
which:

```text
sent SET_CAR(304) + ADD_BODY_FORCE(203) -> echo=[1500, 1600, 1100]
      SET_CAR landed (the echo is the receipt); ADD_BODY_FORCE was ignored -- wrench=(<fx>,<fy>,<fz>) N unchanged
```

`cmd::name(cmd_id) -> &'static str` maps an id back to its constant name for logging, and
returns `""` for an id that is not in the schema.

## CmdArgs


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
    pub vec4: Option<[f64; 4]>,
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

The eight fields in the table below are the same eight in all three. The C struct pairs every
array with an explicit `_len`, and it flattens `vec3_arr` and `vec4_arr` into one `double`
array of three or four values per entry, so `vec3_arr_len` counts vectors, not doubles.
Python's are the keyword arguments of `send_cmd` rather than a type of their own.

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `int_val` | `i32` | per command | `0` | mode selectors live here (`SET_FW_CTRL_MODE`, `SET_FW_EST_SOURCE`) |
| `float_val` | `f64` | per command | `0.0` | scalar forces (`SET_MSD`, `SET_INVPEN`, `SET_FW_THRUST`); narrows to `f32` |
| `int_arr` | `Vec<i32>` | microseconds | empty | pulse widths (`SET_MR_PWM`, `SET_CAR`) |
| `float_arr` | `Vec<f64>` | per command | empty | per-panel radians (`SET_FW_SURFACES`); narrows to `f32` |
| `vec3` | `Option<[f64; 3]>` | per command | `None` | frame-tagged and converted by the robot |
| `vec4` | `Option<[f64; 4]>` | per command | `None` | ordered `[x, y, z, w]`, matching the wire |
| `vec3_arr` | `Vec<[f64; 3]>` | per command | empty | `SET_BODY_FT` puts the torque half here |
| `vec4_arr` | `Vec<[f64; 4]>` | per command | empty | |

The struct is `#[non_exhaustive]`, so it cannot be built with a struct literal: a future
schema field would otherwise be a breaking change across three languages. Build it with a
shorthand or with `CmdArgs::default()` plus chained setters.

| Shorthand | Sets | The shape used by |
|---|---|---|
| `CmdArgs::ints(&[i32])` | `int_arr` | `SET_MR_PWM`, `SET_CAR` |
| `CmdArgs::floats(&[f64])` | `float_arr` | `SET_FW_SURFACES` |
| `CmdArgs::vector([f64; 3])` | `vec3` | `SET_BODY_FORCE`, `SET_ANGVEL` |

The builders are `with_int_val`, `with_float_val`, `with_int_arr`, `with_float_arr`,
`with_vec3`, `with_vec4`, `with_vec3_arr` and `with_vec4_arr`. Each consumes `self`, returns
`Self` and is `#[must_use]`, so they chain and a dropped result is a compiler warning rather
than a silent no-op.

## Every command id

**Live** means a robot type acts on the id today. **Not yet** means it is defined on the
wire and no robot type acts on it. **Absent** means the robot type it belongs to does not
exist in the simulator.

| Constant | Value | Status | Acted on by |
|---|---|---|---|
| `SET_ACC` | 1 | not yet | |
| `SET_VEL` | 2 | not yet | |
| `SET_POS` | 3 | not yet | |
| `SET_ANGACC` | 50 | not yet | |
| `SET_ANGVEL` | 51 | live | GlobalHawk onboard rate loop; the in-game IMU panel publishes it at 50 Hz |
| `SET_EULER` | 52 | not yet | |
| `SET_EULER_DOT` | 53 | not yet | |
| `SET_QUAT` | 54 | not yet | |
| `SET_MASS` | 100 | not yet | use `srv/params` instead |
| `SET_MOI_3X1` | 101 | not yet | use `srv/params` |
| `SET_MOI_3X3` | 102 | not yet | use `srv/params` |
| `SET_BODY_FORCE` | 200 | not yet | |
| `SET_BODY_TORQUE` | 201 | not yet | |
| `SET_BODY_FT` | 202 | not yet | |
| `ADD_BODY_FORCE` | 203 | not yet | |
| `ADD_BODY_TORQUE` | 204 | not yet | |
| `ADD_BODY_FT` | 205 | not yet | |
| `SET_MR_PWM` | 300 | live | Multirotor, HalfDrone |
| `SET_MR_THROTTLE` | 301 | not yet | |
| `SET_OMROVER` | 302 | absent | robot type not in the simulator |
| `SET_HELI` | 303 | absent | robot type not in the simulator |
| `SET_CAR` | 304 | live | Truck |
| `SET_MSD` | 305 | live | Msd |
| `SET_INVPEN` | 306 | live | CartPole |
| `SET_FW_SURFACES` | 307 | live | GlobalHawk |
| `SET_FW_THRUST` | 308 | live | GlobalHawk |
| `SET_FW_THRUST_BIAS` | 309 | live | GlobalHawk |
| `SET_FW_CTRL_MODE` | 310 | live | GlobalHawk |
| `SET_FW_EST_SOURCE` | 311 | live | GlobalHawk |

The mode constants are argument values rather than ids: `FW_ONBOARD_RATE` (0) and
`FW_DIRECT_SURFACE` (1) for `SET_FW_CTRL_MODE`, `FW_EST_TRUTH` (0) and `FW_EST_OBSERVER`
(1) for `SET_FW_EST_SOURCE`.

## When to reach for it

Use `send_cmd` for ids the SDK has no method for, `SET_ANGVEL` being the one that comes up
in practice. Prefer the typed methods everywhere they exist: they fill the right field for
you and they validate, and a pulse width outside 1100 to 2000 microseconds is refused
before anything is sent, where `send_cmd` publishes it.

**Next:** [Commands nothing acts on](07-ignored-commands.md)

**See also:** [Sending commands](00-intro.md), [Fixed wing control](05-fixed-wing.md), [Appendix B: Command reference](../appendix-b-commands.md)
