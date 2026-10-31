#!/usr/bin/env python3
"""PC-side RS485 bus tester.

Listens on the USB-RS485 dongle, verifies ESP32CAM beacons,
and can ping / echo-test the ESP.

Setup:
  ESP32CAM (GPIO12 TX / GPIO13 RX via RS485 transceiver): A<->A, B<->B, GND<->GND
  This machine:
    /dev/cu.usbserial-1440 = ESP32CAM USB debug console (115200, view with `pio device monitor`)
    /dev/cu.usbserial-1430 = USB-RS485 dongle (9600, this script)

Usage:
  python3 rs485_pc_test.py                                  # listen only
  python3 rs485_pc_test.py --ping                           # 1 ping, expect PONG/ACK
  python3 rs485_pc_test.py --stress 20                       # 20 pings, report loss
  python3 rs485_pc_test.py --port /dev/cu.usbserial-1430 --baud 9600 --timeout 30
  python3 rs485_pc_test.py --send "HELLO FROM PC"            # send one line then listen
  python3 rs485_pc_test.py --interactive                     # type lines to send
Requires: pip install pyserial
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: pip3 install pyserial")


def open_port(port, baud):
    try:
        ser = serial.Serial(port, baudrate=baud, bytesize=8,
                            parity='N', stopbits=1, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"Cannot open {port} @ {baud}: {e}\n"
                 f"Hint: ls /dev/cu.*  (expect 1430=RS485 dongle, 1440=ESP console)")
    ser.reset_input_buffer()
    return ser


def parse_hello(line):
    """Check 'ESP:HELLO <n> FROM ESP32CAM CK=<xx>'. Returns (n, ok, msg)."""
    if "HELLO" not in line:
        return None, None, None
    try:
        n = int(line.split("HELLO")[1].split()[0])
    except (ValueError, IndexError):
        return None, False, "unparseable counter"
    if "CK=" in line:
        try:
            payload = line.split("ESP:")[1].split(" CK=")[0]
            want = line.split("CK=")[1].strip().split()[0]
            cks = 0
            for ch in payload:
                cks ^= ord(ch)
            ok = f"{cks:x}" == want.lower().lstrip("0") or f"{cks:02x}" == want.lower().zfill(2)[-2:]
            # simpler: compare as ints
            ok = int(want, 16) == cks
            return n, ok, "ck ok" if ok else f"CK MISMATCH got={want} want={cks:02x}"
        except (ValueError, IndexError):
            return n, False, "ck parse fail"
    return n, True, "no ck"


def main():
    ap = argparse.ArgumentParser(description="RS485 loop tester (PC side)")
    ap.add_argument("--port", default="/dev/cu.usbserial-1430")
    ap.add_argument("--baud", type=int, default=9600,
                    help="must match RS485_BAUD in ESP firmware (default 9600)")
    ap.add_argument("--timeout", type=float, default=15,
                    help="listen duration in seconds (0 = forever)")
    ap.add_argument("--send", default=None, help="send this line once, then listen")
    ap.add_argument("--ping", action="store_true", help="send PING once, wait for PONG")
    ap.add_argument("--stress", type=int, default=0, metavar="N",
                    help="send N PINGs 0.5s apart, report reply rate")
    ap.add_argument("--interactive", action="store_true",
                    help="read stdin lines and send them to the bus")
    ap.add_argument("--hex", action="store_true", help="also dump raw hex")
    args = ap.parse_args()

    ser = open_port(args.port, args.baud)
    print(f"Listening on {args.port} @ {args.baud} 8N1 "
          f"(ESP beacons every 2s). Ctrl-C to stop.")
    print("Expect: ESP:HELLO <n> FROM ESP32CAM CK=..")
    print("-" * 60)

    if args.ping or args.stress:
        n = args.stress if args.stress > 0 else 1
        got = 0
        last_hello = None
        for i in range(n):
            ser.write(b"PING\n")
            print(f"[TX] PING {i+1}/{n}")
            deadline = time.time() + 3.0
            buf = b""
            replied = False
            while time.time() < deadline:
                chunk = ser.read(64)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        s = line.decode(errors="replace").strip()
                        if not s:
                            continue
                        print(f"[RX] {s}")
                        if "PONG" in s or s.startswith("ACK:"):
                            replied = True
                        if "HELLO" in s:
                            num, ok, msg = parse_hello(s)
                            last_hello = (num, ok)
                    if replied:
                        break
                time.sleep(0.01)
            print(f"  -> {'REPLY OK' if replied else 'NO REPLY (check wiring/baud/A/B swap)'}")
            got += replied
            time.sleep(0.5)
        print("-" * 60)
        print(f"Ping result: {got}/{n} replied.")
        if last_hello:
            print(f"Last beacon: HELLO {last_hello[0]} checksum-{'ok' if last_hello[1] else 'BAD'}")
        if got < n:
            print("Troubleshoot: 1) A<->A B<->B (try swapping), 2) common GND, "
                  "3) baud=9600 both sides, 4) dongle DE/auto-dir, 5) ESP actually flashed.")
            sys.exit(1)
        return

    if args.send:
        ser.write((args.send + "\n").encode())
        print(f"[TX] {args.send}")

    if args.interactive:
        import select
        print("Interactive: type a line + Enter to send. Ctrl-C/D to quit.")
    buf = b""
    hellos = 0
    bad_ck = 0
    last_n = None
    missed = 0
    t0 = time.time()
    try:
        while True:
            if args.timeout and (time.time() - t0) > args.timeout:
                break
            if args.interactive:
                r, _, _ = select.select([sys.stdin], [], [], 0)
                if r:
                    line = sys.stdin.readline()
                    if not line:
                        break
                    ser.write(line.encode())
                    print(f"[TX] {line.strip()}")
            chunk = ser.read(256)
            if chunk:
                if args.hex:
                    print(f"  hex: {chunk.hex(' ')}")
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode(errors="replace").strip()
                    if not s:
                        continue
                    ts = time.time() - t0
                    n, ok, msg = parse_hello(s)
                    if n is not None:
                        hellos += 1
                        flag = ""
                        if ok is False:
                            bad_ck += 1
                            flag = f"  <-- {msg}!"
                        if last_n is not None and n != last_n + 1:
                            gap = n - last_n - 1
                            missed += max(gap, 0)
                            flag += f"  <-- GAP: missed {gap} packet(s)!"
                        last_n = n
                        print(f"[{ts:6.1f}s] [RX] {s}{flag}")
                    else:
                        print(f"[{ts:6.1f}s] [RX] {s}")
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    print("-" * 60)
    print(f"Done: {hellos} beacons, {bad_ck} bad checksums, ~{missed} missed, "
          f"last counter={last_n}.")
    if hellos == 0:
        print("Got NOTHING. Check: dongle on 1430? baud 9600? A/B swapped? "
              "GND connected? ESP flashed & running (watch 1440 @115200)?")
        sys.exit(1)
    elif bad_ck or missed:
        print("Bus works but lossy: add 120R termination, shorten cable, "
              "check loose A/B/GND.")
    else:
        print("BUS OK: ESP -> PC path verified.")


if __name__ == "__main__":
    main()
