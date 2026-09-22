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
    reader._publish = lambda data: seen.append(data)  # type: ignore[method-assign]
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
    reader._publish = lambda data: seen.append(data)  # type: ignore[method-assign]

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
    """A stream carrying no JPEG at all must not be buffered forever."""
    buffer = bytearray(b"n" * 100_000)
    collect(reader, buffer)
    assert len(buffer) <= 1


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


def test_next_frame_returns_immediately_when_already_newer(reader: MjpegReader):
    reader._drain(bytearray(part(jpeg(b"a"))))
    got = reader.next_frame(after_sequence=0, timeout=0.05)
    assert got is not None and got[1] == 1
