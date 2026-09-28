"""Frame extraction from an MJPEG byte stream.

The reader has to cope with frames split across reads, part headers between them, the renderer
re-sending an identical frame to keep browsers alive, and a stream that desynchronises. All of
these are exercised without opening a socket.
"""

import pytest

from coproc.runtime.mjpeg import MjpegReader

SOI = b"\xff\xd8"
EOI = b"\xff\xd9"


def jpeg(body: bytes) -> bytes:
    return SOI + body + EOI


def part(payload: bytes) -> bytes:
    """One multipart chunk, framed exactly as the renderer's MjpegServer frames it."""
    return (
        b"--frameboundary\r\n"
        b"Content-Type: image/jpeg\r\n"
        + f"Content-Length: {len(payload)}\r\n\r\n".encode("ascii")
        + payload
        + b"\r\n"
    )


@pytest.fixture
def reader() -> MjpegReader:
    r = MjpegReader("http://localhost:0/stream.mjpg")
    r._running = True  # consume without the network thread
    return r


def collect(reader: MjpegReader, buffer: bytearray) -> list[bytes]:
    seen: list[bytes] = []
    reader._publish = lambda data, *_: seen.append(data)  # type: ignore[method-assign]
    reader._drain(buffer)
    return seen


def test_single_frame(reader: MjpegReader):
    payload = jpeg(b"first")
    assert collect(reader, bytearray(part(payload))) == [payload]


def test_several_frames_in_one_read(reader: MjpegReader):
    a, b, c = jpeg(b"a"), jpeg(b"bb"), jpeg(b"ccc")
    buffer = bytearray(part(a) + part(b) + part(c))
    assert collect(reader, buffer) == [a, b, c]


def test_frame_split_across_reads(reader: MjpegReader):
    """The common case: a 50 KB frame never arrives in one 64 KB read cleanly aligned."""
    payload = jpeg(b"x" * 500)
    framed = part(payload)
    buffer = bytearray()
    seen: list[bytes] = []
    reader._publish = lambda data, *_: seen.append(data)  # type: ignore[method-assign]

    for start in range(0, len(framed), 37):  # deliberately awkward chunk size
        buffer.extend(framed[start : start + 37])
        reader._drain(buffer)

    assert seen == [payload]


def test_partial_tail_is_kept_for_the_next_read(reader: MjpegReader):
    payload = jpeg(b"complete")
    buffer = bytearray(part(payload) + b"--frameboundary\r\n\r\n" + SOI + b"incomplete")
    assert collect(reader, buffer) == [payload]
    assert SOI in bytes(buffer), "the started frame must survive for the next read"


def test_leading_garbage_is_discarded(reader: MjpegReader):
    payload = jpeg(b"good")
    buffer = bytearray(b"HTTP/1.1 200 OK\r\nContent-Type: whatever\r\n\r\n" + part(payload))
    assert collect(reader, buffer) == [payload]


def test_buffer_does_not_grow_without_a_frame(reader: MjpegReader):
    """A stream carrying no JPEG at all must not be buffered forever.

    A short tail is kept, because a part's headers can arrive in a different read from its image,
    but it is bounded no matter how much image-free data comes in.
    """
    buffer = bytearray(b"n" * 100_000)
    collect(reader, buffer)
    assert len(buffer) <= 512


def stamped_part(payload: bytes, timestamp: float) -> bytes:
    """A part carrying the capture-time header, exactly as the renderer's MjpegServer writes it."""
    return (
        b"--frameboundary\r\n"
        b"Content-Type: image/jpeg\r\n"
        + f"X-Timestamp: {timestamp:.6f}\r\n".encode("ascii")
        + f"Content-Length: {len(payload)}\r\n\r\n".encode("ascii")
        + payload
        + b"\r\n"
    )


def collect_stamped(reader: MjpegReader, buffer: bytearray) -> list[tuple[bytes, object]]:
    seen: list[tuple[bytes, object]] = []
    reader._publish = lambda data, ts=None: seen.append((data, ts))  # type: ignore[method-assign]
    reader._drain(buffer)
    return seen


def test_capture_timestamp_travels_with_its_frame(reader: MjpegReader):
    """The robot places detections using its pose at this instant, so it must stay attached."""
    a, b = jpeg(b"a"), jpeg(b"b")
    buffer = bytearray(stamped_part(a, 12.34) + stamped_part(b, 12.44))
    assert collect_stamped(reader, buffer) == [(a, 12.34), (b, 12.44)]


def test_capture_timestamp_survives_headers_and_image_in_different_reads(reader: MjpegReader):
    """Chunked so the header line itself is cut in two, and the image lands in a later read."""
    payload = jpeg(b"x" * 300)
    framed = stamped_part(payload, 101.5)
    buffer = bytearray()
    seen: list[tuple[bytes, object]] = []
    reader._publish = lambda data, ts=None: seen.append((data, ts))  # type: ignore[method-assign]

    for start in range(0, len(framed), 11):
        buffer.extend(framed[start : start + 11])
        reader._drain(buffer)

    assert seen == [(payload, 101.5)]


def test_unstamped_source_reports_no_timestamp(reader: MjpegReader):
    """A real camera that sends no header must not inherit the previous frame's timestamp."""
    a, b = jpeg(b"a"), jpeg(b"b")
    buffer = bytearray(stamped_part(a, 5.0) + part(b))
    assert collect_stamped(reader, buffer) == [(a, 5.0), (b, None)]


def test_frame_carries_the_timestamp_to_the_consumer(reader: MjpegReader):
    reader._drain(bytearray(stamped_part(jpeg(b"z"), 7.25)))
    got = reader.next_frame(after_sequence=-1, timeout=0.1)
    assert got is not None
    assert got[0].source_timestamp == 7.25


def test_identical_frames_are_suppressed(reader: MjpegReader):
    """The renderer resends its current frame every two seconds to keep browsers connected.

    Re-detecting on bytes already processed would waste the little CPU headroom there is and
    report an FPS the pipeline is not really achieving.
    """
    payload = jpeg(b"same")
    reader._drain(bytearray(part(payload)))
    assert reader._sequence == 1

    reader._drain(bytearray(part(payload)))
    assert reader._sequence == 1, "a repeated frame must not count as new"

    reader._drain(bytearray(part(jpeg(b"different"))))
    assert reader._sequence == 2


def test_newest_frame_wins(reader: MjpegReader):
    """Three frames arriving while the detector is busy leave the newest, not a backlog."""
    reader._drain(bytearray(part(jpeg(b"a")) + part(jpeg(b"b")) + part(jpeg(b"c"))))
    got = reader.next_frame(after_sequence=-1, timeout=0.1)
    assert got is not None
    frame, sequence = got
    assert frame.jpeg == jpeg(b"c")
    assert sequence == 3, "sequence must still count what was skipped"


def test_next_frame_times_out_when_nothing_arrives(reader: MjpegReader):
    assert reader.next_frame(after_sequence=0, timeout=0.05) is None


def test_next_frame_waits_before_the_first_frame(reader: MjpegReader):
    """The runner asks from -1. With no frame yet that must block, not return at once.

    Returning at once made the runner spin, publishing "waiting" tens of thousands of times a
    second until the source came up, and burning a CPU core the renderer needed.
    """
    import time

    started = time.monotonic()
    assert reader.next_frame(after_sequence=-1, timeout=0.2) is None
    assert time.monotonic() - started >= 0.15


def test_next_frame_returns_immediately_when_already_newer(reader: MjpegReader):
    reader._drain(bytearray(part(jpeg(b"a"))))
    got = reader.next_frame(after_sequence=0, timeout=0.05)
    assert got is not None and got[1] == 1
