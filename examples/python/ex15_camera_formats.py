"""ex15 - measure what the pixel format and resolution cost in bytes and MB/s."""

from vrsdk import RobotType, VirtualRobot

FORMAT = "rgba8"  # rgba8: 4 channels, not rgb8; per camera (resolution is robot-wide)
FRAMES = 60
HZ = 100
BYTES_PER_PIXEL = {"mono8": 1, "rgb8": 3, "rgba8": 4}

mr = VirtualRobot(RobotType.MULTIROTOR, sys_id=1)  # sys_id 1 = multirotor, 0 = truck
mr.connect()

# the iceoryx2 service name IS <camera>/<resolution>_<format>
cam = mr.open_camera("front_left", "720p", FORMAT)
print(f"camera stream: {cam.service_name}")
spec = cam.spec
print(f"spec: name={spec.name} resolution={spec.resolution} format={spec.format}")

seen = 0
total_bytes = 0
first_ns = last_ns = 0

while seen < FRAMES:
    frame = cam.read()
    if frame is not None:
        seen += 1
        total_bytes += len(frame.data)
        last_ns = frame.t_ns
        if seen == 1:
            first_ns = frame.t_ns
            img = frame.image  # (h, w, c) uint8, top-down
            px = frame.width * frame.height
            print(
                f"\nframe {frame.width}x{frame.height} = {px} px, "
                f"{frame.format} at {frame.channels} B/px, step={frame.step} B/row, "
                f"{len(frame.data)} B/frame  numpy {img.shape}"
            )
            for name, bpp in BYTES_PER_PIXEL.items():
                mark = "  <- this stream" if name == frame.format else ""
                print(f"  {name:>5}: {px * bpp:>9} B/frame{mark}")
        elif seen % 20 == 0:
            print(f"frame {seen}: seq={frame.seq} t={frame.elapsed:.3f}")

    mr.rate(HZ)

# rate off the capture stamps, not the wall clock: the stream's own fps
span_s = (last_ns - first_ns) / 1e9
if span_s > 0:
    fps = (seen - 1) / span_s
    print(
        f"\n{seen} frames over {span_s:.2f}s = {fps:.1f} fps, "
        f"{total_bytes / span_s / 1e6:.1f} MB/s at {FORMAT}"
    )
    # mono8 is 1 B/px instead of 4, and 360p is a quarter of the pixels
    ratio = BYTES_PER_PIXEL["mono8"] / BYTES_PER_PIXEL[FORMAT] / 4
    print(
        f"the same frames as 360p mono8 would be "
        f"{total_bytes * ratio / span_s / 1e6:.1f} MB/s (see ex17 to mount one)"
    )

st = cam.stats
print(f"received={st.received} decode_errors={st.decode_errors} seq_gaps={st.seq_gaps}")
