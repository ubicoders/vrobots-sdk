"""ex11 - list the topics on the wire right now, no robot handle needed."""

from collections import defaultdict

import vrsdk

WINDOW = 1.5  # seconds to listen for zenoh traffic

print(f"listening for {WINDOW}s ...")
topics = vrsdk.list_topics(WINDOW)
if not topics:
    raise SystemExit("no vrobots topics: sim not in Play mode, on another host, or window too short")

print(f"\n{'wire':<4} {'Hz':>7} {'bytes':>9}  topic")
for t in topics:
    # zenoh topics are measured (observed); iceoryx2 come from a registry, unmeasured
    if t.observed:
        hz, nbytes = f"{t.hz:.1f}", str(t.bytes)
    elif t.live:
        hz, nbytes = "-", "-"
    else:
        hz, nbytes = "stale", "-"
    print(f"[{t.transport}] {hz:>7} {nbytes:>9}  {t.key}")

by_robot = defaultdict(list)
for t in topics:
    by_robot[t.sys_id].append(t.key)

print("\nby robot:")
for sys_id in sorted(by_robot, key=lambda k: (k is None, k)):
    keys = by_robot[sys_id]
    if sys_id is None:  # manager/scene topics carry no sys_id
        print(f"  swarm-wide (manager/scene): {len(keys)} topic(s)")
    else:
        print(f"  sys_id {sys_id}: {len(keys)} topic(s)")
    for key in keys:
        print(f"      {key}")

# the names the SDK builds for one robot, without asking the network
first = next((t.sys_id for t in topics if t.sys_id is not None), None)
if first is not None:
    print(f"\ntopic names for sys_id {first}:")
    for role, key in vrsdk.topics(first).items():
        print(f"  {role:<15} {key}")
