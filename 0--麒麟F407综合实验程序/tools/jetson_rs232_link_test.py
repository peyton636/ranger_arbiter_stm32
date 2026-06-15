#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Basic Jetson RS232 link test: V3 uplink listen + optional downlink heartbeat.

Checks physical layer before time sync:
  python3 tools/jetson_rs232_link_test.py --port /dev/ttyUSB7 --listen-only --time 5
  python3 tools/jetson_rs232_link_test.py --port /dev/ttyUSB7 --time 5

If --listen-only shows V3 type 0x02 > 0, MCU USART2 TX works.
If --time 5 shows Windows [JETSON CMD], Jetson USART2 RX to MCU works.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("Install: pip install pyserial", file=sys.stderr)
    sys.exit(1)

V3_MAGIC = 0xAA
V3_LEN = 24
SVC_MAGIC = 0xA5
SVC_LEN = 11


def open_port(port: str, baud: int) -> serial.Serial:
    if not port or not str(port).strip():
        print("ERROR: --port is empty", file=sys.stderr)
        print("  python3 tools/jetson_rs232_link_test.py --port /dev/ttyUSB7 --listen-only", file=sys.stderr)
        sys.exit(2)
    try:
        return serial.Serial(port.strip(), baud, timeout=0.05)
    except serial.SerialException as e:
        print(f"ERROR: cannot open {port!r}: {e}", file=sys.stderr)
        sys.exit(2)


def xor_v3(frame: bytes) -> int:
    x = 0
    for i in range(V3_LEN - 1):
        x ^= frame[i]
    return x


def build_v3_down(seq: int, v_mm_s: int = 100) -> bytes:
    """Minimal V3 downlink type 0x01 (24 bytes)."""
    frame = bytearray(V3_LEN)
    frame[0] = V3_MAGIC
    frame[1] = 0x01  # down
    frame[2] = seq & 0xFF
    frame[3] = 0x01  # mode normal
    struct.pack_into(">h", frame, 6, v_mm_s)
    frame[23] = xor_v3(bytes(frame))
    return bytes(frame)


class StreamStats:
    def __init__(self) -> None:
        self.total = 0
        self.aa = 0
        self.a5 = 0
        self.ascii_n = 0
        self.v3_down = 0
        self.v3_up_st = 0
        self.v3_up_ex = 0
        self.v3_bad_xor = 0
        self.svc_frames = 0
        self._buf = bytearray()

    def feed(self, data: bytes) -> None:
        self.total += len(data)
        self._buf.extend(data)
        self._scan()

    def _scan(self) -> None:
        i = 0
        while i < len(self._buf):
            b = self._buf[i]
            if b == V3_MAGIC and i + V3_LEN <= len(self._buf):
                frame = bytes(self._buf[i : i + V3_LEN])
                if xor_v3(frame) == frame[23]:
                    t = frame[1]
                    if t == 0x01:
                        self.v3_down += 1
                    elif t == 0x02:
                        self.v3_up_st += 1
                    elif t == 0x03:
                        self.v3_up_ex += 1
                else:
                    self.v3_bad_xor += 1
                i += V3_LEN
                continue
            if b == SVC_MAGIC and i + SVC_LEN <= len(self._buf):
                self.svc_frames += 1
                i += SVC_LEN
                continue
            if b == V3_MAGIC:
                self.aa += 1
            elif b == SVC_MAGIC:
                self.a5 += 1
            elif b >= 0x20:
                self.ascii_n += 1
            i += 1
        if i > 0:
            del self._buf[:i]

    def report(self, seconds: float, label: str) -> None:
        v3_up = self.v3_up_st + self.v3_up_ex
        print(f"\n=== {label} ({seconds}s) ===")
        print(f"  total_rx     = {self.total} bytes")
        print(f"  V3 uplink    = {v3_up}  (0x02 status={self.v3_up_st}, 0x03 detail={self.v3_up_ex})")
        print(f"  V3 downlink  = {self.v3_down}")
        print(f"  service 0xA5 = {self.svc_frames}")
        print(f"  0xAA raw     = {self.aa}  ASCII chars = {self.ascii_n}")
        if self.v3_bad_xor:
            print(f"  V3 xor fail  = {self.v3_bad_xor}")
        if self.total == 0:
            print("  VERDICT: FAIL - no data (check PA2 TX -> Jetson RX, GND, port, MCU running)")
        elif v3_up == 0:
            print("  VERDICT: FAIL - no V3 uplink from MCU (~25 Hz expected)")
        else:
            rate = v3_up / seconds
            print(f"  VERDICT: OK uplink ~{rate:.1f} frames/s")


def listen(ser: serial.Serial, seconds: float, label: str) -> StreamStats:
    st = StreamStats()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        n = ser.in_waiting
        if n:
            st.feed(ser.read(n))
        else:
            time.sleep(0.005)
    st.report(seconds, label)
    return st


def send_heartbeat(ser: serial.Serial, seconds: float, hz: float = 50.0) -> int:
    """Send V3 down at ~hz for seconds. Returns frames sent."""
    interval = 1.0 / hz
    end = time.monotonic() + seconds
    seq = 0
    sent = 0
    next_t = time.monotonic()
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_t:
            ser.write(build_v3_down(seq))
            ser.flush()
            seq = (seq + 1) & 0xFF
            sent += 1
            next_t += interval
        else:
            time.sleep(0.001)
    return sent


def main() -> None:
    ap = argparse.ArgumentParser(description="Jetson RS232 V3 link test (USART2)")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--time", type=float, default=5.0, help="test duration seconds")
    ap.add_argument("--listen-only", action="store_true", help="only receive, no TX")
    ap.add_argument("--hz", type=float, default=50.0, help="downlink rate when sending")
    args = ap.parse_args()

    print(f"Open {args.port} @ {args.baud}")
    ser = open_port(args.port, args.baud)
    time.sleep(0.2)
    ser.reset_input_buffer()

    if args.listen_only:
        st = listen(ser, args.time, "listen-only")
        ser.close()
        sys.exit(0 if (st.total > 0 and st.v3_up_st + st.v3_up_ex > 0) else 1)

    # bidirectional: listen while sending heartbeat
    st = StreamStats()
    end = time.monotonic() + args.time
    seq = 0
    interval = 1.0 / args.hz
    next_tx = time.monotonic()
    sent = 0
    print(f"Sending V3 down @ {args.hz} Hz for {args.time}s (check Windows for [JETSON CMD])...")
    while time.monotonic() < end:
        n = ser.in_waiting
        if n:
            st.feed(ser.read(n))
        now = time.monotonic()
        if now >= next_tx:
            ser.write(build_v3_down(seq))
            ser.flush()
            seq = (seq + 1) & 0xFF
            sent += 1
            next_tx += interval
        else:
            time.sleep(0.001)
    st.report(args.time, "bidirectional")
    print(f"  sent_down    = {sent} V3 frames")
    print("  Check Windows USART1: should see motion / arbiter react if downlink OK")
    ser.close()
    sys.exit(0 if sent > 0 else 1)


if __name__ == "__main__":
    main()
