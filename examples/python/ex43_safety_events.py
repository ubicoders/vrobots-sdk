"""ex43 - safety_events: every robot's incidents, one at a time, and the silence between them.

    python examples/python/ex43_safety_events.py

A robot that comes close to a person, touches one or hits one publishes a
`SafetyEvent` on its own `vrobots/{id}/z/event/safety`, and an arm's
protective stop publishes one when a link meets an object or another robot.
This example listens to all of them at once, on `vrobots/*/z/event/safety`,
with no robot handle at all: `subscribe_all_safety_events()` opens a zenoh
session of its own and closes it when the stream is stopped.

Silence is the normal state
---------------------------

Every other stream in the SDK is periodic -- a state at 25 Hz, a scan at 20
-- and a quiet one is a broken one. This one is the opposite: **one sample
per incident, nothing in between.** A minute without an event is a minute in
which nobody got hurt, so `next_timeout()` coming back with nothing is an
answer, not an error, and there is no rate to watch.

The flip side is that silence cannot tell "nothing happened" from "nobody is
publishing". So this example first asks which robots are on the wire at all
(ex11's `list_topics`), and refuses to listen to a simulator that is not
there.

Events queue
------------

A state is superseded by the next one, so the state stream keeps only the
latest. An incident is superseded by nothing -- a contact that turns into a
strike is two records -- so the stream **queues** every event and hands each
one out once, oldest first: `next_timeout()` waits for one, `try_next()` never
waits, and `drain()` takes all of them, the read for a control loop. A robot
handle has a stream of its own too, `subscribe_safety_events()`.

What an event says
------------------

| field | meaning |
|---|---|
| `sys_id`, `robot_name`, `seq` | whose incident it is -- never who was hit -- and that robot's event counter |
| `tier` | `near_miss` (close, moving in, never touched), `contact`, `strike` (fast, or a large impulse) |
| `other_kind`, `other_name` | `person`, `robot` or `object`, and its name in the scene |
| `closing_speed` | the robot's speed toward the other party, m/s, + closing |
| `impulse` | N*s; 0 for a near miss, and for an arm link, which is posed, not pushed |
| `robot_at_fault` | the robot was the one moving in |
| `position` | where, a **world** position in the robot's own frame (`coord_frame_id`) |

Only incidents after the subscriber is live arrive; zenoh replays nothing,
and a fresh session needs a moment of discovery before the first one can.
On the Community tier only the focused robot is on the wire, and another
robot's incident is dropped inside the simulator, not queued for later.

Making one happen
-----------------

In the warehouse scene people walk between the racks. Drive the forklift
toward one with the keyboard (arrows, focused in the simulator) while this
runs, or jog the rail arm into a box stack -- its protective stop reports
the link that met it.
"""

import sys
import time

import vrsdk
from vrsdk import SafetyTier, VrError, list_topics, subscribe_all_safety_events

WINDOW = 1.5  # ex11's listening window, s
LISTEN = 30.0  # s
QUIET_NOTE = 5.0  # s: say so after this much silence


def robots_on_the_wire():
    """The sys_ids publishing a state topic: the robots on the wire right now."""
    topics = list_topics(WINDOW)
    robots = []
    for t in topics:
        if t.sys_id is None:
            continue
        if t.key == vrsdk.topics(t.sys_id)["state"] and t.sys_id not in robots:
            robots.append(t.sys_id)
    return robots


def print_event(t, e):
    """One event, as it arrived."""
    print(
        f"t={t:5.1f}s  robot {e.sys_id} '{e.robot_name}' event #{e.seq}: {e.tier_name} with "
        f"{e.other_kind_name} '{e.other_name}'"
    )
    print(
        f"          closing {e.closing_speed:+.2f} m/s, impulse {e.impulse:.1f} N*s, robot at "
        f"fault: {e.robot_at_fault}, at ({e.position[0]:+.2f}, {e.position[1]:+.2f}, "
        f"{e.position[2]:+.2f}) m in {e.coord_frame_id!r}"
    )


# ===== setup: is anybody there? =====
print(f"listening for {WINDOW:g}s for the robots on the wire ...")
try:
    robots = robots_on_the_wire()
except VrError as e:
    print(f"could not look: [{e.code}] {e}", file=sys.stderr)
    sys.exit(1)
if not robots:
    print(
        "no robot is publishing, so no event could ever arrive. Is the simulator running, "
        "with a scene in Play mode? `vrobots topic list` shows what is on the wire.",
        file=sys.stderr,
    )
    sys.exit(1)
print(f"robots on the wire: {robots} -- an event can come from any of them")

# ===== subscribe, swarm-wide =====
try:
    events = subscribe_all_safety_events()
except VrError as e:
    print(f"could not subscribe: [{e.code}] {e}", file=sys.stderr)
    sys.exit(1)
print(f"subscribed to {events.key} for {LISTEN:g}s\n")

# ===== listen =====
start = time.monotonic()
near_misses = 0
contacts = 0
strikes = 0
unspecified = 0
while time.monotonic() - start < LISTEN:
    # Wait for the next event, but no longer than QUIET_NOTE, so that a
    # quiet stretch is reported as what it is.
    wait = min(QUIET_NOTE, max(0.0, LISTEN - (time.monotonic() - start)))
    event = events.next_timeout(wait)
    if event is None:
        print(f"t={time.monotonic() - start:5.1f}s  quiet: no incident, which is the normal state")
        continue
    print_event(time.monotonic() - start, event)
    if event.tier == SafetyTier.NEAR_MISS:
        near_misses += 1
    elif event.tier == SafetyTier.CONTACT:
        contacts += 1
    elif event.tier == SafetyTier.STRIKE:
        strikes += 1
    else:
        # Unspecified, or a tier newer than this SDK: unknown severity, not harmless.
        unspecified += 1

# ===== what the stream counted =====
# On an aperiodic topic a lost event looks exactly like no event, so these
# counters are the only way to know the record is complete.
stats = events.stats
print(
    f"\n{stats.received} event(s) in {LISTEN:g}s: {near_misses} near miss, {contacts} "
    f"contact, {strikes} strike, {unspecified} unspecified"
)
print(
    f"dropped {stats.dropped} (queue full, nobody reading), missed {stats.missed_events} in "
    f"{stats.seq_gaps} seq gap(s) (lost on the way), decode errors {stats.decode_errors}, "
    f"still queued {events.pending}"
)
# Stopping the stream undeclares the subscriber and closes its own session.
events.stop()
