#!/usr/bin/env python3
"""
Serial Bus Servo ID Scanner & Programmer
Scans a TTL servo bus (Feetech SCS/SMS/STS or Dynamixel Protocol 1.0)
for connected servos and can assign new IDs one at a time.

Requires: pip install pyserial
"""

import sys
import time
import serial
import serial.tools.list_ports

# ── Protocol ────────────────────────────────────────────────────────────────

# Common ID register addresses — the script probes both automatically.
ID_REGISTERS = [0x05, 0x03]   # 0x05 = Feetech SCS/SMS/STS, 0x03 = Dynamixel AX/MX

def _checksum(payload: list[int]) -> int:
    return (~sum(payload)) & 0xFF

def build_ping(servo_id: int) -> bytes:
    payload = [servo_id, 0x02, 0x01]
    return bytes([0xFF, 0xFF] + payload + [_checksum(payload)])

def build_write(servo_id: int, address: int, data: list[int]) -> bytes:
    length  = 3 + len(data)
    payload = [servo_id, length, 0x03, address] + data
    return bytes([0xFF, 0xFF] + payload + [_checksum(payload)])

def read_response(ser: serial.Serial, expected_id: int,
                  window: float = 0.15, debug: bool = False) -> bool:
    """Read bytes for `window` seconds and search for a valid status packet."""
    buf      = bytearray()
    deadline = time.monotonic() + window
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        buf.extend(chunk)
        if len(buf) >= 24:
            break

    if debug and buf:
        print(f"\n    raw rx [{expected_id:3d}]: {buf.hex(' ')}")

    for i in range(len(buf) - 5):
        if buf[i] != 0xFF or buf[i + 1] != 0xFF:
            continue
        sid      = buf[i + 2]
        length   = buf[i + 3]
        error    = buf[i + 4]
        checksum = buf[i + 5] if i + 5 < len(buf) else -1
        if sid == expected_id and length == 0x02:
            if _checksum([sid, length, error]) == checksum:
                return True
    return False

def ping(ser: serial.Serial, servo_id: int, debug: bool = False) -> bool:
    ser.reset_input_buffer()
    ser.write(build_ping(servo_id))
    return read_response(ser, servo_id, debug=debug)

def write_id(ser: serial.Serial, current_id: int, new_id: int) -> tuple[bool, int]:
    """
    Write new_id persistently to EEPROM.
    Sequence: unlock EEPROM → write ID to EEPROM register → relock.
    Returns (success, eeprom_register_used).
    """
    # Step 1: Unlock EEPROM (LOCK register 0x37 = 0 for Feetech STS series)
    unlock = build_write(current_id, 0x37, [0x00])
    print(f"\n    Unlock EEPROM:")
    print(f"    TX: {unlock.hex(' ').upper()}")
    ser.reset_input_buffer()
    ser.write(unlock)
    time.sleep(0.1)
    raw = bytearray(ser.read(ser.in_waiting))
    print(f"    RX: {raw.hex(' ').upper() if raw else '(empty)'}")

    # Step 2: Write new ID — try EEPROM register 0x03 first, then RAM 0x05
    for reg in [0x03, 0x05]:
        packet = build_write(current_id, reg, [new_id])
        print(f"\n    Write ID to register 0x{reg:02X}:")
        print(f"    TX: {packet.hex(' ').upper()}")
        ser.reset_input_buffer()
        ser.write(packet)
        time.sleep(0.3)   # EEPROM writes are slower than RAM
        raw = bytearray(ser.read(ser.in_waiting))
        print(f"    RX: {raw.hex(' ').upper() if raw else '(empty)'}")

        # Servo switches IDs immediately after write
        switched = ping(ser, new_id)
        print(f"    Ping on new ID {new_id}: {'OK — ID changed' if switched else 'no response'}")
        if switched:
            # Relock EEPROM
            relock = build_write(new_id, 0x37, [0x01])
            ser.reset_input_buffer()
            ser.write(relock)
            time.sleep(0.1)
            ser.read(ser.in_waiting)
            return True, reg

    return False, -1

# ── UI helpers ───────────────────────────────────────────────────────────────

DIVIDER = "─" * 48

def header(title: str):
    print(f"\n{DIVIDER}")
    print(f"  {title}")
    print(DIVIDER)

def prompt(msg: str, default: str = "") -> str:
    suffix = f" [{default}]" if default else ""
    try:
        val = input(f"  {msg}{suffix}: ").strip()
    except (KeyboardInterrupt, EOFError):
        print("\n  Cancelled.")
        sys.exit(0)
    return val if val else default

def choose(options: list[tuple[str, str]], title: str) -> int:
    header(title)
    for i, (label, desc) in enumerate(options):
        desc_str = f"  —  {desc}" if desc else ""
        print(f"  [{i}]  {label}{desc_str}")
    while True:
        raw = prompt("Enter number")
        if raw.isdigit() and 0 <= int(raw) < len(options):
            return int(raw)
        print("  Invalid choice, try again.")

# ── Port / baud selection ─────────────────────────────────────────────────────

def select_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("\n  No serial ports found — is the adapter plugged in?")
        sys.exit(1)
    options = [(p.device, p.description) for p in ports]
    options.append(("Enter manually", ""))
    idx = choose(options, "Select Serial Port")
    if idx == len(ports):
        return prompt("Port path (e.g. /dev/tty.usbserial-0001)")
    return ports[idx].device

def select_baud() -> int:
    common  = [1_000_000, 57600, 115200, 2_000_000]
    options = [(f"{b:,}", "") for b in common] + [("Enter manually", "")]
    idx     = choose(options, "Select Baud Rate  (Feetech default: 1,000,000)")
    if idx == len(common):
        return int(prompt("Baud rate"))
    return common[idx]

# ── Mode 1: Scan ─────────────────────────────────────────────────────────────

def select_range() -> tuple[int, int]:
    options = [
        ("Quick scan",   "IDs 1 – 30   (~5 s, covers most setups)"),
        ("Full scan",    "IDs 1 – 253  (~40 s)"),
        ("Custom range", "You enter start and end"),
    ]
    idx = choose(options, "Scan Range")
    if idx == 0: return 1, 30
    if idx == 1: return 1, 253
    start = int(prompt("Start ID", "1"))
    end   = int(prompt("End ID",   "30"))
    return start, end

def run_scan(port: str, baud: int):
    start, end = select_range()
    debug = prompt("Debug mode — show raw bytes? (y/n)", "n").lower() == "y"

    total = end - start + 1
    found = []

    header(f"Scanning  {port}  @  {baud:,}  —  IDs {start}–{end}")

    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid in range(start, end + 1):
                pct = (sid - start + 1) / total * 100
                if not debug:
                    print(f"\r  [{pct:5.1f}%]  Checking ID {sid:3d}  —  found so far: {found}",
                          end="", flush=True)
                if ping(ser, sid, debug=debug):
                    found.append(sid)
                    print(f"\r  [ HIT ]  ID {sid:3d} responded{' ' * 36}")
    except serial.SerialException as e:
        print(f"\n\n  Serial error: {e}")
        return

    print(f"\r  Done.{' ' * 50}")

    header("Scan Results")
    if not found:
        print("  No servos found.\n")
        print("  Troubleshooting:")
        print("   • Try baud 1,000,000 — Feetech/SCS factory default")
        print("   • Try baud 57,600    — Dynamixel factory default")
        print("   • Confirm servo power supply is on")
        print("   • If all servos share the same factory ID, connect")
        print("     only ONE at a time and use 'Assign IDs' mode")
    else:
        print(f"  Found {len(found)} servo(s):\n")
        for sid in found:
            print(f"    ID {sid:3d}  (0x{sid:02X})")
        if len(found) == 1:
            print("\n  Only 1 responded — if you have more servos, they likely")
            print("  all share the same factory ID and are colliding on the bus.")
            print("  Use 'Assign IDs' mode to number them one at a time.")
    print()

# ── Mode 2: Assign IDs one at a time ─────────────────────────────────────────

def run_assign(port: str, baud: int):
    header("Assign Servo IDs  —  One Servo at a Time")
    print("  Connect a single servo, press Enter to scan for it,")
    print("  assign a new ID, then swap in the next servo.\n")

    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            next_id = int(prompt("Start assigning from ID", "1"))

            while True:
                prompt(f"Connect ONE servo and press Enter to scan for it")

                # Quick scan IDs 1-30 to find whatever is connected
                print("  Scanning...", end="", flush=True)
                found_id = None
                for sid in range(0, 31):
                    if ping(ser, sid):
                        found_id = sid
                        break
                print(f"\r            ")

                if found_id is None:
                    print("  No servo found. Check power and connection.")
                    retry = prompt("Try again? (y/n)", "y")
                    if retry.lower() != "y":
                        break
                    continue

                print(f"  Found servo at ID {found_id}.")

                if found_id == next_id:
                    print(f"  Already set to ID {found_id} — no change needed.")
                else:
                    confirm = prompt(f"Assign it ID {next_id}? (y/n)", "y")
                    if confirm.lower() == "y":
                        print(f"  Writing ID {next_id}...", end="", flush=True)
                        ok, reg = write_id(ser, found_id, next_id)
                        if ok:
                            print(f"  [ OK ]  Servo is now ID {next_id} (register 0x{reg:02X}).")
                        else:
                            print(f"\r  [FAIL]  No response after write attempts.")
                            print( "          Check connection and try again.")

                next_id += 1
                another = prompt(f"\n  Swap in the next servo? (y/n)", "y")
                if another.lower() != "y":
                    break

    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return

    header("Done")
    print(f"  IDs assigned up to {next_id - 1}.")
    print("  Daisy-chain all servos and run a Scan to confirm.\n")

# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print(f"\n{'═' * 48}")
    print( "   Serial Bus Servo Scanner & ID Programmer")
    print(f"{'═' * 48}")

    port = select_port()
    baud = select_baud()

    mode = choose(
        [
            ("Scan",        "Find all servo IDs currently on the bus"),
            ("Assign IDs",  "Number servos 1-by-1 (use when all share the same factory ID)"),
        ],
        "Mode"
    )

    if mode == 0:
        run_scan(port, baud)
    else:
        run_assign(port, baud)

    again = prompt("Return to main menu? (y/n)", "n")
    if again.lower() == "y":
        main()

if __name__ == "__main__":
    main()
