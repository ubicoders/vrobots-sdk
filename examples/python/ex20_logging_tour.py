"""ex20 - route the SDK's log records through Python logging."""

import logging
from collections import Counter

import vrsdk
from vrsdk import RobotType, VirtualRobot

# zenoh's crates log under several sibling roots; hold them at WARNING before turning the SDK up.
for name in ("zenoh", "zenoh_transport", "zenoh_config", "zenoh_codec", "iceoryx2", "tracing"):
    logging.getLogger(name).setLevel(logging.WARNING)

# logging.basicConfig(level=...) plus raising the vrobots_sdk logger's level.
vrsdk.init_logging("debug")


# The SDK's records are ordinary log records: any handler of your own sees them.
class CountingHandler(logging.Handler):
    def __init__(self):
        super().__init__()
        self.by_level = Counter()
        self.by_logger = Counter()

    def emit(self, record):
        self.by_level[record.levelname] += 1
        self.by_logger[record.name] += 1


counter = CountingHandler()
logging.getLogger().addHandler(counter)

# Configure logging BEFORE connecting: connect() is the noisy moment.
mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor, 0 = truck
mr.connect()

for _ in range(10):
    mr.wait_new_state(0.5)
s = mr.states
print(f"seq={s.seq} t={s.elapsed:.3f}")

# Malformed payloads are logged and counted, never raised from `states`.
st = mr.stats
err = mr.last_error
print(
    f"received={st.received} decode_errors={st.decode_errors} last_error="
    + ("none" if err is None else f"[{err.code}] {err.detail}")
)

print(f"records by level: {dict(counter.by_level)}")
print(f"records by logger: {dict(counter.by_logger)}")
