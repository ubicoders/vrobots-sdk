# Mass and inertia

Change what the robot weighs, and confirm it by watching the robot move.

```sh
cargo run -p vrobots-examples --bin ex22_physical_params -- 1
./target/cpp-build/ex22_physical_params 1
python examples/python/ex22_physical_params.py 1
cargo run -p vrobots-examples --bin ex22_physical_params
./target/cpp-build/ex22_physical_params
python examples/python/ex22_physical_params.py
```

`ex22_physical_params` takes an **optional** `sys_id`. With it, the example attaches to the
scene's own multirotor; without it, the example creates one and both climb runs read 0.00 m/s,
because a client-created multirotor does not integrate physics in simulator v3.0.0
([known issue](../ch07-robots/07-known-issues.md)). Prefer the argument.

## The request

`srv/params` carries two numbers and is the only channel for either.

| Field | Type | Units | Default | Notes |
|---|---|---|---|---|
| `mass` | `Option<f64>` | kg | `None`, meaning untouched | frame-invariant; must be positive and finite |
| `moi` | `Option<[f64;3]>` | kg·m² | `None`, meaning untouched | principal moments, read in **your** header frame and permuted into the robot's; all three axes must be positive and finite |

Build one with `PhysicalParams::default()` and the `with_mass` / `with_moi` setters;
`is_empty()` reports whether the request would change nothing. Moments of inertia are positive
quantities, so the frame conversion reorders the triple and never flips a sign.

The service works mid-flight, which is the reason it exists: changing the mass under a running
loop is the standard way to test a controller against a payload it was not tuned for.

## The confirmation is behavioural

Neither figure appears in the state message. Mass properties are quasi-static configuration,
not state, so there is nothing to read back and nothing to diff. The same pulse width has to
produce a different acceleration, or the change did not land.

`ex22` therefore flies a fixed 1800 us collective twice, at two different masses, and compares
the climb rate.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

`examples/rust/src/bin/ex22_physical_params.rs`:

```rust
    // ===== run 1: light =====
    robot.set_physical_params(&PhysicalParams::default().with_mass(LIGHT_KG).with_moi(MOI))?;
    let light = climb_run(&robot, LIGHT_KG)?;

    // ===== run 2: heavy, same command =====
    robot.set_physical_params(&PhysicalParams::default().with_mass(HEAVY_KG))?;
    let heavy = climb_run(&robot, HEAVY_KG)?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex22_physical_params.cpp`:

```cpp
// ===== run 1: light =====
{
    auto params = vrsdk::physical_params();
    params.has_mass = true;
    params.mass = LIGHT_KG;
    params.has_moi = true;
    params.moi[0] = MOI[0];
    params.moi[1] = MOI[1];
    params.moi[2] = MOI[2];
    robot.set_physical_params(params);
}
const double light = climb_run(robot, LIGHT_KG);

// ===== run 2: heavy, same command =====
{
    auto params = vrsdk::physical_params();
    params.has_mass = true;
    params.mass = HEAVY_KG;
    robot.set_physical_params(params);
}
const double heavy = climb_run(robot, HEAVY_KG);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex22_physical_params.py`:

```python
# ===== run 1: light =====
robot.set_physical_params(mass=LIGHT_KG, moi=MOI)
light = climb_run(robot, LIGHT_KG)

# ===== run 2: heavy, same command =====
robot.set_physical_params(mass=HEAVY_KG)
heavy = climb_run(robot, HEAVY_KG)
```

{{#endtab }}
{{#endtabs }}

The three spell "leave this field alone" differently, and it is the pattern for every service
in this chapter. Rust chains `with_*` setters on a default; Python takes keyword arguments and
omits what it does not set; C++ builds the plain C struct from `vrsdk::physical_params()` and
sets a `has_*` flag beside each value. Never write `{}` in C++ and never forget the `has_*`
flag: the field is then silently not applied.

Same command, same rotors, same air: the heavier aircraft climbs slower, and the gap between
the two numbers is the entire receipt.

```text
1800 us on every rotor, 2.0 s of climb, twice:
  1 kg -> <rate> m/s
  2 kg -> <rate> m/s
The difference IS the receipt -- there is no mass field in the state message to read back.
```

Each run starts with `reset()`, so the two are comparable. That does not undo the mass:
configuration survives a state reset.

## Positive only, and refused before it is sent

The simulator does not validate either field. It keeps the prefab's value and acks `ok`, so a
bad request is indistinguishable from a good one from outside.

| You send | The simulator does | What you would see |
|---|---|---|
| `mass <= 0` | keeps the body's current mass | nothing |
| a moment of inertia not strictly positive on **all three** axes | keeps Unity's collider-derived tensor | nothing |

`set_physical_params` refuses both cases itself, as `VrError::InvalidArgument` naming the
field, before anything reaches the wire.


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    show_refusal(
        "mass = 0.0",
        robot.set_physical_params(&PhysicalParams::default().with_mass(0.0)),
    );
    show_refusal(
        "moi = [0.02, 0.0, 0.04] (one axis left at zero)",
        robot.set_physical_params(&PhysicalParams::default().with_moi([0.02, 0.0, 0.04])),
    );
    show_refusal(
        "nothing set at all",
        robot.set_physical_params(&PhysicalParams::default()),
    );
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex22_physical_params.cpp`:

```cpp
show_refusal("mass = 0.0", [&] {
    auto params = vrsdk::physical_params();
    params.has_mass = true;
    params.mass = 0.0;
    robot.set_physical_params(params);
});
show_refusal("moi = [0.02, 0.0, 0.04] (one axis left at zero)", [&] {
    auto params = vrsdk::physical_params();
    params.has_moi = true;
    params.moi[0] = 0.02;
    params.moi[1] = 0.0;
    params.moi[2] = 0.04;
    robot.set_physical_params(params);
});
show_refusal("nothing set at all",
             [&] { robot.set_physical_params(vrsdk::physical_params()); });
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex22_physical_params.py`:

```python
show_refusal("mass = 0.0", lambda: robot.set_physical_params(mass=0.0))
show_refusal(
    "moi = [0.02, 0.0, 0.04] (one axis left at zero)",
    lambda: robot.set_physical_params(moi=(0.02, 0.0, 0.04)),
)
show_refusal("nothing set at all", lambda: robot.set_physical_params())
```

{{#endtab }}
{{#endtabs }}

The helper takes a callable in C++ and Python because the refusal arrives as a thrown
exception, where Rust's takes the returned `Result` directly. The third case reads
differently for the same reason the first two do: an empty request is a bare
`vrsdk::physical_params()` in C++ and a no-argument call in Python.

Each prints the error code and the SDK's explanation instead of a receipt:

```text
-- what the SDK refuses before anything reaches the wire --
  mass = 0.0                                       [<code>] <message>
  moi = [0.02, 0.0, 0.04] (one axis left at zero)  [<code>] <message>
  nothing set at all                               [<code>] <message>
All three are acked `ok` by the simulator and silently ignored, which is why they are caught here instead.
```

> **Gotcha.** The half-filled inertia triple is the one that catches people: two axes set, the
> third left at zero. It reads like a partial update and is not. The simulator needs all three
> strictly positive or it keeps the tensor it already had, so a partial triple changes nothing
> at all.

## The cart pole exception

A [CartPole](../ch07-robots/04-cartpole.md) re-stamps its cart's mass from
`CartPoleConfig::cart_mass` on **every** parameter apply, so a mass sent to `srv/params` is
overwritten a step later on that one robot type. Set it with
[`configure_cartpole`](07-msd-cartpole-config.md) instead. Inertia is unaffected.

## The defaults are not documented

The prefab mass and inertia a multirotor or a truck spawns with are not recorded anywhere in
this repository, and the simulator does not publish them. The only documented behaviour is
that a non-positive value leaves them alone.

That matters when you attach rather than create. There is no getter, so the SDK cannot read
the old value first and put it back, and `reset()` will not either. Write down what you
started with, or reload the scene when you are done.

**Next:** [Sensor noise](03-sensor-config.md)

**See also:** [Robot lifecycle](01-lifecycle.md), [Kinematics](../ch03-reading-state/02-kinematics.md), [Mass spring damper and cart pole](07-msd-cartpole-config.md)
