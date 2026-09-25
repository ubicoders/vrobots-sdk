"""ex12 - print version identity, subscriber stats, and last_error diagnostics."""

import time

import vrsdk
from vrsdk import RobotType, VirtualRobot

SAMPLES = 50  # ~2 s at the 25 Hz state rate
HZ = 25

v = vrsdk.version_info()
print(f"vrsdk {v['sdk_version']}")
print(f"  vrobots_msgs  {v['msgs_commit']} (schema_version {v['schema_version']})")
print(f"  flatbuffers   {v['flatbuffers']}")
print(f"  zenoh         {v['zenoh']}")
print(f"  iceoryx2      {v['iceoryx2']}")
print(f"  src_id        {v['src_id']}")

mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor, 0 = truck
mr.connect()
first = mr.states
print(
    f"\nsim says: schema_version={first.schema_version} (ours {v['schema_version']}), "
    f"frame={first.coord_frame_id!r} axes={first.axis_convention_name!r}, "
    f"its header src_id={first.src_id} (ours {v['src_id']})"
)
if first.schema_version != v["schema_version"]:
    print("  MISMATCH -- fields may decode as garbage; reinstall the matching wheel")

print(f"\nwatching for {SAMPLES} loop iterations at {HZ} Hz ...")
started = time.perf_counter()
for _ in range(SAMPLES):
    mr.rate(HZ)
elapsed = max(time.perf_counter() - started, 1e-3)

st = mr.stats
last = mr.states
print(
    f"\nstats after {elapsed:.1f} s: received={st.received} "
    f"decode_errors={st.decode_errors} seq_gaps={st.seq_gaps} "
    f"missed_samples={st.missed_samples} last_seq={st.last_seq}"
)
print(
    f"  effective rate {st.received / elapsed:.1f} Hz over the window; "
    f"sim clock advanced {last.elapsed - first.elapsed:.2f} s"
)

# decode errors are counted, not raised; last_error is where they went
err = mr.last_error
if err is None:
    print("  last_error: none -- every payload decoded")
else:
    print(f"  last_error: [{err.code} {err.kind}] {err.detail}")
