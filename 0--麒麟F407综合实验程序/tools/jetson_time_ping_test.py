#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Jetson RS232 time sync test: START / PING / QUERY / STOP on USART2 (PA2/PA3).

Usage:
  python3 tools/jetson_time_ping_test.py --port /dev/ttyUSB7 --probe
  python3 tools/jetson_time_ping_test.py --port /dev/ttyUSB7 --query-only
  python3 tools/jetson_time_ping_test.py --port /dev/ttyUSB7 --ping-only --count 10
  python3 tools/jetson_time_ping_test.py --port /dev/ttyUSB7 --count 10 --query --timeout 1.5
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

SVC_MAGIC = 0xA5
V3_MAGIC = 0xAA
ID_TIME_REQ = 0x107
ID_TIME_RSP = 0x108
CMD_QUERY = 0x01
CMD_START = 0x02
CMD_PING = 0x03
CMD_STOP = 0x04


def u32be(b: bytes, i: int) -> int:
    return struct.unpack(">I", b[i : i + 4])[0]


def build_service(can_id: int, payload: bytes) -> bytes:
    if len(payload) < 8:
        payload = payload + bytes(8 - len(payload))
    return bytes([SVC_MAGIC, (can_id >> 8) & 0xFF, can_id & 0xFF]) + payload[:8]


def mono_ms() -> float:
    return time.monotonic() * 1000.0


def open_port(port: str, baud: int) -> serial.Serial:
    if not port or not str(port).strip():
        print("ERROR: --port is empty. Example:", file=sys.stderr)
        print("  python3 tools/jetson_time_ping_test.py --port /dev/ttyUSB7 --probe", file=sys.stderr)
        sys.exit(2)
    try:
        ser = serial.Serial(port.strip(), baud, timeout=0.05)
    except serial.SerialException as e:
        print(f"ERROR: cannot open {port!r}: {e}", file=sys.stderr)
        sys.exit(2)
    return ser


class ServiceScanner:
    def __init__(self) -> None:
        self._buf = bytearray()
        self.total_rx = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.total_rx += len(data)
        self._buf.extend(data)
        out: list[tuple[int, bytes]] = []
        i = 0
        while i < len(self._buf):
            if self._buf[i] != SVC_MAGIC:
                i += 1
                continue
            if i + 11 > len(self._buf):
                break
            can_id = (self._buf[i + 1] << 8) | self._buf[i + 2]
            payload = bytes(self._buf[i + 3 : i + 11])
            out.append((can_id, payload))
            i += 11
        if i > 0:
            del self._buf[:i]
        return out


class TimeSyncTester:
    def __init__(self, ser: serial.Serial, default_timeout_ms: float) -> None:
        self.ser = ser
        self.scanner = ServiceScanner()
        self.default_timeout_ms = default_timeout_ms
        self.session_id = 1
        self.offset_ms = 0.0
        self.delay_ms = 0.5
        self.min_rtt = 1e9
        self.rx_before_wait = 0

    def flush_rx(self) -> None:
        self.ser.reset_input_buffer()
        self.scanner = ServiceScanner()

    def _drain(self, timeout_s: float = 0.05) -> None:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            n = self.ser.in_waiting
            if n:
                self.scanner.feed(self.ser.read(n))
            else:
                time.sleep(0.002)

    def _wait_rsp(
        self,
        timeout_ms: float | None = None,
        expect_cmd: int | None = None,
        expect_echo: int | None = None,
    ) -> tuple[bytes, float] | None:
        """Return (payload, t_recv_mono_ms) at the instant a matching 0x108 is parsed."""
        if timeout_ms is None:
            timeout_ms = self.default_timeout_ms
        self.rx_before_wait = self.scanner.total_rx
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            n = self.ser.in_waiting
            if n:
                chunk = self.ser.read(n)
                t_now = mono_ms()
                for can_id, payload in self.scanner.feed(chunk):
                    if can_id != ID_TIME_RSP:
                        continue
                    if expect_cmd is not None:
                        if expect_cmd in (CMD_START, CMD_PING):
                            if payload[0] != expect_cmd:
                                continue
                            if expect_echo is not None and payload[1] != (expect_echo & 0xFF):
                                continue
                    return payload, t_now
            else:
                time.sleep(0.001)
        return None

    def _timeout_msg(self, label: str) -> None:
        got = self.scanner.total_rx - self.rx_before_wait
        print(f"{label}: timeout (rx {got} bytes while waiting for 0x108)")
        if got == 0:
            print("  -> MCU may be stuck (check V3 uplink with --probe)")

    def send_start(self, timeout_ms: float | None = None) -> bool:
        self.flush_rx()
        t0 = mono_ms()
        payload = struct.pack(">BBIHH", CMD_START, self.session_id, int(t0) & 0xFFFFFFFF, 0, 0)
        self.ser.write(build_service(ID_TIME_REQ, payload))
        self.ser.flush()
        rsp = self._wait_rsp(timeout_ms, expect_cmd=CMD_START, expect_echo=self.session_id)
        if not rsp:
            self._timeout_msg("START")
            return False
        data, t_rsp = rsp
        mcu_t0 = u32be(data, 2)
        rtt = t_rsp - t0
        self.offset_ms = mcu_t0 - t0
        print(
            f"START ok: RTT={rtt:.1f} ms mcu_tick={mcu_t0} "
            f"offset~={self.offset_ms:.1f} ms flags=0x{data[6]:02X}"
        )
        return True

    def send_ping(self, seq: int, timeout_ms: float | None = None) -> float | None:
        self.flush_rx()
        t1 = mono_ms()
        payload = struct.pack(">BBIHH", CMD_PING, seq & 0xFF, int(t1) & 0xFFFFFFFF, 0, 0)
        self.ser.write(build_service(ID_TIME_REQ, payload))
        self.ser.flush()
        rsp = self._wait_rsp(timeout_ms, expect_cmd=CMD_PING, expect_echo=seq)
        if not rsp:
            self._timeout_msg(f"PING #{seq}")
            return None
        data, t4 = rsp
        mcu_rx = u32be(data, 2)
        proc_ms = data[7] * 0.1
        rtt = t4 - t1
        one_way = max(0.0, (rtt - proc_ms) / 2.0)
        self.min_rtt = min(self.min_rtt, rtt)
        self.delay_ms = min(self.delay_ms, one_way)
        off = mcu_rx - t1 - self.delay_ms
        self.offset_ms = 0.8 * self.offset_ms + 0.2 * off
        print(
            f"PING #{seq}: RTT={rtt:.2f} ms proc={proc_ms:.1f} ms "
            f"delay~={one_way:.2f} ms mcu_rx={mcu_rx} offset={self.offset_ms:.2f} ms"
        )
        return rtt

    def send_query(self, timeout_ms: float | None = None) -> bool:
        self.flush_rx()
        payload = bytes([CMD_QUERY]) + bytes(7)
        self.ser.write(build_service(ID_TIME_REQ, payload))
        self.ser.flush()
        rsp = self._wait_rsp(timeout_ms)
        if not rsp:
            self._timeout_msg("QUERY")
            return False
        data, _t_rsp = rsp
        tick = u32be(data, 0)
        utc = u32be(data, 4)
        print(f"QUERY ok: mcu_tick={tick} utc_unix={utc} ({'no GPS' if utc == 0 else 'UTC ok'})")
        return True

    def send_stop(self) -> None:
        payload = bytes([CMD_STOP, self.session_id]) + bytes(6)
        self.ser.write(build_service(ID_TIME_REQ, payload))
        self.ser.flush()


def probe_port(ser: serial.Serial, seconds: float = 5.0) -> int:
    aa = a5 = ascii_n = total = 0
    v3_type02 = v3_type03 = 0
    buf = bytearray()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        n = ser.in_waiting
        if not n:
            time.sleep(0.01)
            continue
        chunk = ser.read(n)
        total += len(chunk)
        buf.extend(chunk)
        for b in chunk:
            if b == V3_MAGIC:
                aa += 1
            elif b == SVC_MAGIC:
                a5 += 1
            elif b >= 0x20:
                ascii_n += 1
    for i in range(len(buf) - 1):
        if buf[i] == V3_MAGIC:
            if buf[i + 1] == 0x02:
                v3_type02 += 1
            elif buf[i + 1] == 0x03:
                v3_type03 += 1
    print(
        f"probe {seconds}s: total_rx={total} B  "
        f"0xAA={aa}  V3_up(0x02)={v3_type02}  V3_up(0x03)={v3_type03}  "
        f"0xA5={a5}  ASCII={ascii_n}"
    )
    if total == 0:
        print("FAIL: zero bytes on USART2")
    elif v3_type02 == 0 and v3_type03 == 0:
        print("FAIL: no V3 uplink")
    else:
        print(f"OK: V3 uplink ~{(v3_type02 + v3_type03) / seconds:.0f} frames/s")
    return total


def main() -> None:
    ap = argparse.ArgumentParser(description="Jetson-MCU RS232 time sync test")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--count", type=int, default=20)
    ap.add_argument("--timeout", type=float, default=0.5, help="max wait for 0x108 (seconds)")
    ap.add_argument("--query", action="store_true")
    ap.add_argument("--probe", action="store_true")
    ap.add_argument("--probe-time", type=float, default=5.0)
    ap.add_argument("--query-only", action="store_true")
    ap.add_argument("--ping-only", action="store_true", help="PING only, no START (isolate START bug)")
    ap.add_argument("--no-start", action="store_true", help="same as --ping-only")
    args = ap.parse_args()

    timeout_ms = args.timeout * 1000.0
    ping_only = args.ping_only or args.no_start

    print(f"Open {args.port} @ {args.baud}  timeout={args.timeout}s")
    ser = open_port(args.port, args.baud)
    time.sleep(0.2)
    ser.reset_input_buffer()

    if args.probe:
        rc = probe_port(ser, args.probe_time)
        ser.close()
        sys.exit(0 if rc > 0 else 1)

    ts = TimeSyncTester(ser, timeout_ms)

    if args.query_only:
        ok = ts.send_query()
        ser.close()
        sys.exit(0 if ok else 1)

    if ping_only:
        print("Mode: ping-only (Windows should show [JETSON TIME] cmd=0x03 per PING)")
        ok_count = 0
        for i in range(1, args.count + 1):
            if ts.send_ping(i) is not None:
                ok_count += 1
            time.sleep(0.05)
        print(f"\nPING ok: {ok_count}/{args.count}")
        ser.close()
        sys.exit(0 if ok_count > 0 else 1)

    if not ts.send_start():
        ser.close()
        sys.exit(1)

    ok_count = 0
    for i in range(1, args.count + 1):
        if ts.send_ping(i) is not None:
            ok_count += 1
        time.sleep(0.05)

    if args.query:
        ts.send_query()
    ts.send_stop()

    print(f"\nPING ok: {ok_count}/{args.count}")
    if ts.min_rtt < 1e8:
        print(f"Stats: min_rtt={ts.min_rtt:.2f} ms delay_est={ts.delay_ms:.2f} ms offset={ts.offset_ms:.2f} ms")
    ser.close()
    sys.exit(0 if ok_count > 0 else 1)


if __name__ == "__main__":
    main()
