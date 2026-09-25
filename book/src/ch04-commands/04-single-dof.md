# Single degree of freedom plants

The mass spring damper and the cart pole each take one number, a force in newtons, and
nothing else.

```sh
cargo run -p vrobots-examples --bin ex28_hello_msd
./target/cpp-build/ex28_hello_msd
python examples/python/ex28_hello_msd.py
cargo run -p vrobots-examples --bin ex29_hello_cartpole -- <sys_id>
./target/cpp-build/ex29_hello_cartpole <sys_id>
python examples/python/ex29_hello_cartpole.py <sys_id>
```

## The two commands

| Method | Signature | Command | Units | Simulator-side clamp |
|---|---|---|---|---|
| `set_msd_force` | `(&self, newtons: f64) -> VrResult<()>` | `SET_MSD` (305) | N along the plant's `+x` | magnitude clamped to the plant's `max_force`, 100 N by default |
| `set_cartpole_force` | `(&self, newtons: f64) -> VrResult<()>` | `SET_INVPEN` (306) | N along the rail's `+x` | magnitude clamped to `CartPoleConfig::max_force`, 20 N by default |

Both send their value in `float_val`, and both validate only that it is finite. Both clamps
are silent: nothing is refused, nothing is reported, and the state stream is the only place
that says what force was actually applied.

## The mass spring damper

An Msd is in the sandbox catalog, so `connect(RobotType::Msd, None)` spawns one. It is the
only plant in the simulator whose response you can work out on paper first, because the SDK
owns every letter of `m*x'' + c*x' + k*x = F`: `m` through `set_physical_params`, `k` and
`c` through `configure_msd`, and `F` through `set_msd_force`.

From `examples/rust/src/bin/ex28_hello_msd.rs`, the start of one step-response run:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    println!("-- {label} --");
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
std::printf("-- %s --\n", label);
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
print(f"-- {label} --")
if retune:
    robot.configure_msd(spring_k=k, damping_c=c)
# Home, at rest, with the force latch cleared -- otherwise the previous run's
# step is still pushing.
robot.set_msd_force(0.0)
robot.reset()
```

{{#endtab }}
{{#endtabs }}

The three spell the optional configuration fields differently, and the difference is worth
noticing because it is the pattern for every service in
[chapter 6](../ch06-services/00-intro.md). Rust chains `with_*` setters on a default; Python
takes keyword arguments and omits what it does not set; C++ builds the plain C struct from
`vrsdk::msd_config()` and sets a `has_*` flag beside each value. Forgetting the `has_*` flag
in C++ means the field is silently not applied.

The progress lines that follow have this shape, one every 25 samples:

```text
   t=<seconds>s x=<metres> m  x'=<m/s> m/s  disp=<metres> m  net F=<newtons> N
```

Note the order in that snippet. Clearing the force latch before the reset matters, because
a reset with a step force still latched puts the mass back home and immediately starts
pushing it again.

### Predicting it before you run it

The example prints its own predictions, from the arithmetic in its header:

```text
settles at   F / k          metres
period       2*pi*sqrt(m/k) seconds
damping      c / (2*sqrt(k*m))   -- < 1 rings, ~1 slides home, > 1 crawls
```

The actuator block carries the plant's own arithmetic rather than an echo of your command:

| Channel | Meaning | Units |
|---|---|---|
| `actuator.measured[0]` | the **total** force on the mass, `F - k*x - c*x'` | N |
| `actuator.measured[1]` | displacement from equilibrium | m |

> **Gotcha.** `measured[0]` is not the force you sent. It is what the spring and the damper
> left of it, which is why it crosses zero at every peak of the oscillation. To check that
> a clamp did not eat your command, compare against the displacement rather than against
> this channel.

## The cart pole

A cart pole is scene-authored rather than creatable, so it takes a `sys_id` argument: find
the live one with `cargo run -p vrobots-sdk --bin vrobots -- topic list`. Ids are allocated
at scene load and keep incrementing, so no constant in an example could stay true.

One actuator, a force on the cart, and two things to control with it. That is what
underactuated means: the pole is unactuated by design, and there is no command anywhere in
the SDK that touches it. From `examples/rust/src/bin/ex29_hello_cartpole.rs`, the whole
control output is three lines:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
        let force = (-K_THETA * theta - K_THETA_DOT * theta_dot + K_X * x + K_V * v)
            .clamp(-MAX_FORCE_N, MAX_FORCE_N);
        robot.set_cartpole_force(force)?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex29_hello_cartpole.cpp`:

```cpp
const double raw = -K_THETA * theta - K_THETA_DOT * theta_dot + K_X * x + K_V * v;
const double force = std::fmin(std::fmax(raw, -MAX_FORCE_N), MAX_FORCE_N);
robot.set_cartpole_force(force);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex29_hello_cartpole.py`:

```python
force = min(
    max(-K_THETA * theta - K_THETA_DOT * theta_dot + K_X * x + K_V * v, -MAX_FORCE_N),
    MAX_FORCE_N,
)
robot.set_cartpole_force(force)
```

{{#endtab }}
{{#endtabs }}

Only the clamp differs, because only Rust has `f64::clamp` on the primitive. The control law
and the newtons on the wire are identical.

Clamping in the controller as well as trusting the simulator's clamp is worth the line: the
simulator's clamp is silent, so a controller that saturates without knowing it will report
a force it never applied. Each printed line reports both:

```text
t=<seconds>s  theta=<degrees> deg  theta'=<rad/s> rad/s  rail=<metres> m (world x=<metres>)  x'=<m/s> m/s  F=<commanded N> N  applied=<clamped N> N
```

The pole rides the actuator channels, which are exactly three:

| Channel | Meaning | Units |
|---|---|---|
| `actuator.measured[0]` | the force actually applied, after the clamp | N |
| `actuator.measured[1]` | pole angle theta | radians |
| `actuator.measured[2]` | pole rate theta prime | rad/s |

`theta` is 0 upright and plus or minus pi hanging, wrapped into `[-pi, pi]`, so a fallen
pole may print either sign.

### Latching is unforgiving here

On a plant this unstable, a controller that stops publishing is not neutral. The last force
stays applied, and the pole is on the floor within about a second. Two habits follow: treat
a late state sample as a reason to hold the last force rather than to send something new,
and release the latch explicitly on the way out:


{{#tabs global="lang" }}
{{#tab name="Rust" }}

```rust
    robot.set_cartpole_force(0.0)?;
```

{{#endtab }}
{{#tab name="C++" }}

`examples/cpp/ex29_hello_cartpole.cpp`:

```cpp
// ===== hand it back =====
// A command latches: without this the cart keeps pushing forever.
robot.set_cartpole_force(0.0);
```

{{#endtab }}
{{#tab name="Python" }}

`examples/python/ex29_hello_cartpole.py`:

```python
# ===== hand it back =====
# A command latches: without this the cart keeps pushing forever.
robot.set_cartpole_force(0.0)
```

{{#endtab }}
{{#endtabs }}

A Ctrl-C skips that line and leaves the cart pushing.

**Next:** [Fixed wing control](05-fixed-wing.md)

**See also:** [Commands latch](01-latching.md), [Mass spring damper and cart pole](../ch06-services/07-msd-cartpole-config.md), [Mass spring damper](../ch07-robots/03-msd.md), [Cart pole](../ch07-robots/04-cartpole.md)
