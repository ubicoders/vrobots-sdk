# Mass spring damper and cart pole

Two plants whose entire physics is a handful of numbers you can set.

```sh
cargo run -p vrobots-examples --bin ex28_hello_msd
./target/cpp-build/ex28_hello_msd
python examples/python/ex28_hello_msd.py
cargo run -p vrobots-examples --bin ex29_hello_cartpole -- <sys_id>
./target/cpp-build/ex29_hello_cartpole <sys_id>
python examples/python/ex29_hello_cartpole.py <sys_id>
```

`ex29_hello_cartpole` **requires** a `sys_id`, because a cart pole is scene-authored and is not
in the spawn catalog. System ids are allocated at scene load and keep incrementing, so no
constant could stay true; find the live one with `vrobots topic list`.

## The mass spring damper

A mass spring damper is one mass on one axis:

```text
m*x'' + c*x' + k*x = F
```

and the SDK owns every letter of it. `m` is [`set_physical_params`](02-physical-params.md),
`k` and `c` are `configure_msd`, and `F` is `set_msd_force`. That makes it the one robot whose
answer you can predict before you run it.

| `MsdConfig` field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `spring_k` | `Option<f64>` | N/m | `None`, untouched | negatives are **committed as zero** by the simulator, so the SDK refuses them first |
| `damping_c` | `Option<f64>` | N·s/m | `None`, untouched | same |

From `examples/rust/src/bin/ex28_hello_msd.rs`, retuning the plant between runs:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    if retune {
        robot.configure_msd(&MsdConfig::default().with_spring_k(k).with_damping_c(c))?;
    }
    // Home, at rest, with the force latch cleared -- otherwise the previous run's
    // step is still pushing.
    robot.set_msd_force(0.0)?;
    robot.reset()?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex28_hello_msd.cpp`:

```cpp
if (retune) {
    auto config = vrsdk::msd_config();
    config.has_spring_k = true;
    config.spring_k = k;
    config.has_damping_c = true;
    config.damping_c = c;
    robot.configure_msd(config);
}
// Home, at rest, with the force latch cleared -- otherwise the previous run's
// step is still pushing.
robot.set_msd_force(0.0);
robot.reset();
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex28_hello_msd.py`:

```python
if retune:
    robot.configure_msd(spring_k=k, damping_c=c)
# Home, at rest, with the force latch cleared -- otherwise the previous run's
# step is still pushing.
robot.set_msd_force(0.0)
robot.reset()
```

{{#endtab }}
{{#endtabs }}

Two optional fields, three spellings: Rust chains `with_spring_k` and `with_damping_c` onto a
default, Python names them as keyword arguments, and C++ writes each value beside its own `has_*`
flag on a struct from `vrsdk::msd_config()`. In C++ the flag is what separates "set this value"
from "leave it alone", so a number assigned without its flag never reaches the simulator.

The confirmation is arithmetic. A 20 N step settles at `F / k`, rings with period
`2*pi*sqrt(m/k)`, and its damping ratio is `c / (2*sqrt(k*m))`. The example prints the measured
value beside the predicted one for three plants:

```text
20 N step, 1 kg, three plants:
                           x_final       F/k    period  2pi*sqrt    zeta
  as spawned (k=20, c=1)   <value>   <value>   <value>   <value> <value>
  k=80, c=1                <value>   <value>   <value>   <value> <value>
  k=80, c=16               <value>   <value>   <value>   <value> <value>
```

> **Gotcha.** A negative `k` or `c` is not an error in the simulator, it is committed as zero:
> a spring that quietly vanished. `configure_msd` refuses negatives before they are sent, and
> that is the only protection there is. For any value it does accept, the committed number may
> still differ from the one you asked for, and the state stream is the only place that says
> which.

The last thing `ex28` does is ask for a spring that pulls the wrong way, and read the refusal:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    match robot.configure_msd(&MsdConfig::default().with_spring_k(-5.0)) {
        Ok(()) => println!("\nUNEXPECTED: a negative spring constant was accepted"),
        Err(e) => println!("\nspring_k = -5.0 -> [{}] {}", e.code(), e.detail()),
    }
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex28_hello_msd.cpp`:

```cpp
try {
    auto config = vrsdk::msd_config();
    config.has_spring_k = true;
    config.spring_k = -5.0;
    robot.configure_msd(config);
    std::printf("\nUNEXPECTED: a negative spring constant was accepted\n");
} catch (const vrsdk::Error& e) {
    std::printf("\nspring_k = -5.0 -> [%d] %s\n", e.code(), e.what());
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex28_hello_msd.py`:

```python
try:
    robot.configure_msd(spring_k=-5.0)
    print("\nUNEXPECTED: a negative spring constant was accepted")
except vrsdk.VrError as e:
    print(f"\nspring_k = -5.0 -> [{e.code} {e.kind}] {e.detail}")
```

{{#endtab }}
{{#endtabs }}

The refusal is client-side on all three, so nothing reaches the wire. Rust returns it as a `Result`
you match on, while C++ throws `vrsdk::Error` and Python raises `vrsdk.VrError`, which is why the
two of them wrap the call in a `try` that the Rust version does not need.

```text
spring_k = -5.0 -> [<code>] <message>
(the simulator would commit that as 0 and ack `ok`: no spring, no error)
```

## The cart pole

`srv/cartpole` owns the whole plant, cart mass included.

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `cart_mass` | `Option<f64>` | kg | 1.0 | owned here, not by `srv/params` |
| `travel_half_range` | `Option<f64>` | m | 4.0 | rail travel each side of the spawn point; the cart dead-stops at the end of it |
| `pole_rod_mass` | `Option<f64>` | kg | 0.1 | uniform rod between hinge and bob |
| `bob_mass` | `Option<f64>` | kg | 0.2 | point mass at the tip |
| `pole_length` | `Option<f64>` | m | 1.2 | hinge to bob; the pole has no collider, so this one number is the whole pendulum geometry |
| `pole_angular_damping` | `Option<f64>` | dimensionless | not documented | `0` is a frictionless pivot |
| `max_force` | `Option<f64>` | N | 20 | the clamp on `set_cartpole_force` |
| `initial_pole_angle_deg` | `Option<f64>` | **degrees** | not documented | 0 upright, ±180 hanging, wrapped into [-180, 180] |

Out-of-range values are not refused by the simulator, they are silently replaced by the
defaults above and acked `ok`. `configure_cartpole` therefore refuses non-positive masses,
lengths and forces itself.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex29_hello_cartpole.rs`:

```rust
    robot.configure_cartpole(
        &CartPoleConfig::default()
            .with_cart_mass(CART_MASS_KG)
            .with_travel_half_range(4.0)
            .with_pole_rod_mass(ROD_MASS_KG)
            .with_bob_mass(BOB_MASS_KG)
            .with_pole_length(POLE_LENGTH_M)
            .with_pole_angular_damping(0.01)
            .with_max_force(MAX_FORCE_N)
            .with_initial_pole_angle_deg(SEED_DEG),
    )?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex29_hello_cartpole.cpp`:

```cpp
{
    auto config = vrsdk::cartpole_config();
    config.has_cart_mass = true;
    config.cart_mass = CART_MASS_KG;
    config.has_travel_half_range = true;
    config.travel_half_range = 4.0;
    config.has_pole_rod_mass = true;
    config.pole_rod_mass = ROD_MASS_KG;
    config.has_bob_mass = true;
    config.bob_mass = BOB_MASS_KG;
    config.has_pole_length = true;
    config.pole_length = POLE_LENGTH_M;
    config.has_pole_angular_damping = true;
    config.pole_angular_damping = 0.01;
    config.has_max_force = true;
    config.max_force = MAX_FORCE_N;
    config.has_initial_pole_angle_deg = true;
    config.initial_pole_angle_deg = SEED_DEG;
    robot.configure_cartpole(config);
}
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex29_hello_cartpole.py`:

```python
robot.configure_cartpole(
    cart_mass=CART_MASS_KG,
    travel_half_range=4.0,
    pole_rod_mass=ROD_MASS_KG,
    bob_mass=BOB_MASS_KG,
    pole_length=POLE_LENGTH_M,
    pole_angular_damping=0.01,
    max_force=MAX_FORCE_N,
    initial_pole_angle_deg=SEED_DEG,
)
```

{{#endtab }}
{{#endtabs }}

Eight fields is where the C++ shape costs the most: sixteen assignments against eight setters or
eight keyword arguments. A missed `has_*` flag among them leaves that one field at the plant's
current value, and the ack is `ok` either way, so the pole standing up is the only receipt.

The pole is standing at the seed angle before the next line prints, which is the trap this page
exists for:

```text
plant set; the pole is re-seated at -3 deg AT REST, immediately -- not at the next reset
```

## The degrees and radians trap

`initial_pole_angle_deg` is the only field in this API expressed in degrees, and the same angle
comes back on the state topic in radians as `actuator.measured[1]`. A number that looks
sensible in one unit is nonsense in the other, and nothing warns you.

It also behaves unlike every other configuration field. Changing it **re-seats the pole at rest
immediately**, not at the next `reset()`. It is the episode's initial condition, so sending it
mid-swing stops the swing dead. Re-sending the value it already has does nothing.

That makes it useful rather than dangerous once you know: `ex29` uses it to stand the pole up
before the balance loop starts, because that loop has no swing-up in it and cannot catch the
pole from the simulator's own home angle of -45 degrees. Simulated offline against the
linearised plant, its gains recover from about 15 degrees and no more.

> **Note.** `reset()` does both halves of an episode restart at once: the cart returns to the
> rail centre and the pole is re-hung at rest at whatever home angle was last configured.

## Mass belongs to this service, not to `srv/params`

A cart pole re-stamps its body mass from `cart_mass` on every parameter apply, so a mass sent
to `set_physical_params` is overwritten a step later. Use `configure_cartpole`.

## Find the rail centre before you balance

The cart slides along world x, but the rail is centred wherever the scene parked the rig, and
`travel_half_range` is measured from that rather than from the world origin. Measured live,
one scene's cart pole sits at x = -14.9. The simulator knows the difference internally and does
not publish it, so a controller that regulates `lin_pos[0]` toward zero is ordering the cart
metres away, past a dead stop it cannot cross.

Capture the origin yourself, the same way `ex21` learns a multirotor's home: reset, settle,
read.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    robot.reset()?;
    settle(&robot);
    let rail_centre = robot.states().kin.lin_pos[0];
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex29_hello_cartpole.cpp`:

```cpp
robot.reset();
settle(robot);
const double rail_centre = robot.states().kin().lin_pos[0];
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex29_hello_cartpole.py`:

```python
robot.reset()
settle(robot)
rail_centre = robot.states.kin.lin_pos[0]
```

{{#endtab }}
{{#endtabs }}

Only the path to the field differs. C++ reaches the kinematics block through the `kin()` accessor
over the raw C state, and Python exposes `states` as a property rather than a call.

Every position term after that is relative to a number you measured rather than one you
assumed:

```text
rail centre measured at x = <metres> m (world). Every position term below is relative to THAT, not to 0.
```

**Next:** [Skins](08-skins.md)

**See also:** [Single degree of freedom plants](../ch04-commands/04-single-dof.md), [Mass spring damper](../ch07-robots/03-msd.md), [Cart pole](../ch07-robots/04-cartpole.md)
