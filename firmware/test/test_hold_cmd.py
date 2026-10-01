"""PC test for tools/hold_cmd.py with a mock serial port (no hardware, no pyserial needed).
Run: python firmware/test/test_hold_cmd.py"""
import io
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
import hold_cmd  # noqa: E402


class FakeSerial:
    def __init__(self, fail_on_stop=False):
        self.lines = []
        self.fail_on_stop = fail_on_stop

    def write(self, b):
        if self.fail_on_stop and b == b"s\n":
            raise OSError("port gone")
        self.lines.append(b)

    def flush(self):
        pass


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, s):
        self.t += s


def test_duration():
    ser, c = FakeSerial(), FakeClock()
    n = hold_cmd.run(ser, 800, -300, duration=1.0, sleep=c.sleep, clock=c.now)
    assert n == 5, n  # t = 0, .2, .4, .6, .8
    assert ser.lines[:5] == [b"m 800 -300\n"] * 5
    assert ser.lines[-2:] == [b"s\n", b"s\n"]


def test_clamp():
    ser, c = FakeSerial(), FakeClock()
    hold_cmd.run(ser, 5000, -5000, duration=0.1, sleep=c.sleep, clock=c.now)
    assert ser.lines[0] == b"m 1000 -1000\n"


def test_ctrl_c_sends_stop():
    ser, c = FakeSerial(), FakeClock()
    calls = {"n": 0}

    def sleep(s):
        calls["n"] += 1
        if calls["n"] == 3:
            raise KeyboardInterrupt
        c.sleep(s)

    n = hold_cmd.run(ser, 100, 100, sleep=sleep, clock=c.now)
    assert n == 3 and ser.lines[-1] == b"s\n"


def test_error_still_stops_and_port_loss_is_survived():
    ser, c = FakeSerial(), FakeClock()

    def boom(s):
        raise RuntimeError("x")

    try:
        hold_cmd.run(ser, 1, 1, sleep=boom, clock=c.now)
        assert False, "exception should propagate"
    except RuntimeError:
        pass
    assert ser.lines[-1] == b"s\n"
    ser2, out = FakeSerial(fail_on_stop=True), io.StringIO()
    c2 = FakeClock()
    hold_cmd.run(ser2, 1, 1, duration=0.1, sleep=c2.sleep, clock=c2.now, out=out)
    assert "WARN" in out.getvalue()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
    print("hold_cmd: 4 tests OK")
