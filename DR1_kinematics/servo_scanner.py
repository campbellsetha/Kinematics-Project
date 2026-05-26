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

# Servo motion test constants (Feetech STS3215: 4096 ticks / 360°)
TICKS_PER_DEGREE  = 4096 / 360
TEST_DEGREES      = 20
TEST_TICKS        = round(TEST_DEGREES * TICKS_PER_DEGREE)  # ≈ 228
POSITION_MIN      = 0
POSITION_MAX      = 4095

# Limit-setting constants (STS3215 EEPROM register addresses)
EEPROM_MIN_ANGLE  = 0x09  # 2 bytes (0x09 low, 0x0A high)
EEPROM_MAX_ANGLE  = 0x0B  # 2 bytes (0x0B low, 0x0C high)
TORQUE_ENABLE_REG = 0x28  # SRAM, 1 byte — 0x00 = off, 0x01 = on
LIMIT_BUFFER      = 5     # ticks of inward buffer from each physical endpoint

# Single-turn mode reset
MODE_REG          = 0x12  # EEPROM, 1 byte — bit 4 = multi-turn enable
MULTI_TURN_BIT    = 4     # clear this bit to force single-turn (0-4095) mode

def _checksum(payload: list[int]) -> int:
    return (~sum(payload)) & 0xFF

def build_ping(servo_id: int) -> bytes:
    payload = [servo_id, 0x02, 0x01]
    return bytes([0xFF, 0xFF] + payload + [_checksum(payload)])

def build_write(servo_id: int, address: int, data: list[int]) -> bytes:
    length  = 3 + len(data)
    payload = [servo_id, length, 0x03, address] + data
    return bytes([0xFF, 0xFF] + payload + [_checksum(payload)])

def build_read(servo_id: int, address: int, length: int) -> bytes:
    payload = [servo_id, 0x04, 0x02, address, length]
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

def read_present_position(ser: serial.Serial, servo_id: int) -> int | None:
    """Return the servo's present position in ticks, or None on failure."""
    ser.reset_input_buffer()
    ser.write(build_read(servo_id, 0x38, 2))
    buf      = bytearray()
    deadline = time.monotonic() + 0.15
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        buf.extend(chunk)
        if len(buf) >= 8:
            break
    # Response: FF FF [ID] 04 00 [LSB] [MSB] [CHECKSUM]
    for i in range(len(buf) - 6):
        if buf[i] == 0xFF and buf[i+1] == 0xFF and buf[i+2] == servo_id:
            if buf[i+3] == 0x04 and buf[i+4] == 0x00:
                return (buf[i+6] << 8) | buf[i+5]
    return None

def read_byte_register(ser: serial.Serial, servo_id: int, address: int) -> int | None:
    """Read a single byte from any register. Returns the value or None on failure."""
    ser.reset_input_buffer()
    ser.write(build_read(servo_id, address, 1))
    buf      = bytearray()
    deadline = time.monotonic() + 0.15
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        buf.extend(chunk)
        if len(buf) >= 7:
            break
    # Response: FF FF [ID] 03 00 [DATA] [CHECKSUM]
    for i in range(len(buf) - 5):
        if buf[i] == 0xFF and buf[i+1] == 0xFF and buf[i+2] == servo_id:
            if buf[i+3] == 0x03 and buf[i+4] == 0x00:
                return buf[i+5]
    return None

def write_eeprom_byte(ser: serial.Serial, servo_id: int, address: int, value: int):
    """Write a single byte to an EEPROM register with unlock/relock."""
    unlock = build_write(servo_id, 0x37, [0x00])
    ser.reset_input_buffer()
    ser.write(unlock)
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    ser.reset_input_buffer()
    ser.write(build_write(servo_id, address, [value & 0xFF]))
    time.sleep(0.3)
    ser.read(ser.in_waiting)

    relock = build_write(servo_id, 0x37, [0x01])
    ser.reset_input_buffer()
    ser.write(relock)
    time.sleep(0.1)
    ser.read(ser.in_waiting)

def write_goal_position(ser: serial.Serial, servo_id: int, position: int):
    """Write a goal position (ticks) to the servo, clamped to valid range."""
    position = max(POSITION_MIN, min(POSITION_MAX, position))
    low  = position & 0xFF
    high = (position >> 8) & 0xFF
    ser.reset_input_buffer()
    ser.write(build_write(servo_id, 0x2A, [low, high]))  # LSB first per memory map
    time.sleep(0.05)
    ser.read(ser.in_waiting)

def read_word_register(ser: serial.Serial, servo_id: int, address: int) -> int | None:
    """Read two consecutive bytes as a 16-bit little-endian word. Returns None on failure."""
    ser.reset_input_buffer()
    ser.write(build_read(servo_id, address, 2))
    buf      = bytearray()
    deadline = time.monotonic() + 0.15
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        buf.extend(chunk)
        if len(buf) >= 8:
            break
    # Response: FF FF [ID] 04 00 [L] [H] [CHECKSUM]
    for i in range(len(buf) - 6):
        if buf[i] == 0xFF and buf[i+1] == 0xFF and buf[i+2] == servo_id:
            if buf[i+3] == 0x04 and buf[i+4] == 0x00:
                return (buf[i+6] << 8) | buf[i+5]
    return None

def write_eeprom_word(ser: serial.Serial, servo_id: int, address: int, word_value: int):
    """Write a 16-bit word (L then H) to an EEPROM register with unlock/relock."""
    low  = word_value & 0xFF
    high = (word_value >> 8) & 0xFF

    unlock = build_write(servo_id, 0x37, [0x00])
    ser.reset_input_buffer()
    ser.write(unlock)
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    ser.reset_input_buffer()
    ser.write(build_write(servo_id, address, [low, high]))
    time.sleep(0.3)
    ser.read(ser.in_waiting)

    relock = build_write(servo_id, 0x37, [0x01])
    ser.reset_input_buffer()
    ser.write(relock)
    time.sleep(0.1)
    ser.read(ser.in_waiting)

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

# ── Diagnostic register map ──────────────────────────────────────────────────

BAUD_MAP = {
    0: "1,000,000",
    1: "500,000",
    2: "250,000",
    3: "128,000",
    4: "115,200",
    5: "76,800",
    6: "57,600",
    7: "38,400",
}

RETURN_LEVEL_MAP = {
    0: "READ + PING only  (WRITE commands execute silently)",
    1: "All commands",
}

# ── Zero-point calibration ────────────────────────────────────────────────────

OFFSET_REG = 0x1F   # EEPROM, 2 bytes L/H — Position Correction (Offset)
                    # Encoding: BIT11 = sign (0=positive, 1=negative),
                    #           bits 0–10 = magnitude (0–2047 steps)
                    # reported_position = raw_encoder + offset
                    # requires power cycle to take effect
                    # ⚠  Writing 128 (0x80) to SRAM register 0x28 (Torque Enable)
                    #    immediately sets the current position as 2048 in SRAM —
                    #    a quick in-session calibration, but does NOT update 0x1F.
                    #    Write 0x1F explicitly for persistent calibration.

# ── Full EEPROM register map ──────────────────────────────────────────────────
# Verified against official STS3215 Memory Map (SMS&STS Magnetic Encoder series,
# firmware v3.x, document date 2022-03-28).
# Word entries occupy addr (L) and addr+1 (H), little-endian.

EEPROM_MAP = [
    # (addr, label,                    size, notes)
    (0x00, "Firmware Major",           1, None),                             # R/O
    (0x01, "Firmware Minor",           1, None),                             # R/O
    (0x03, "HW Major Version",         1, None),                             # R/O
    (0x04, "HW Minor Version",         1, None),                             # R/O
    (0x05, "ID",                       1, None),
    (0x06, "Baud Rate",                1, BAUD_MAP),
    (0x07, "Return Delay",             1, "2 µs/count  (no function on STS)"),
    (0x08, "Response Status Level",    1, RETURN_LEVEL_MAP),
    (0x09, "Min Angle Limit",          2, "steps 0–4094  (set 0 for multi-turn)"),
    (0x0B, "Max Angle Limit",          2, "steps 1–4095  (set 0 for multi-turn)"),
    (0x0D, "Max Temperature Limit",    1, "°C  (default 70)"),
    (0x0E, "Max Input Voltage",        1, "0.1 V/count  (e.g. 80 = 8.0 V)"),
    (0x0F, "Min Input Voltage",        1, "0.1 V/count  (e.g. 40 = 4.0 V)"),
    (0x10, "Max Torque",               2, "0–1000  (1000 = 100% stall)"),
    (0x12, "Phase",                    1, "bit7=direction, bit4=multi-turn — see datasheet"),
    (0x13, "Unload Condition",         1, "bit mask — protection enable flags"),
    (0x14, "LED Alarm Condition",      1, "bit mask — alarm enable flags"),
    (0x15, "Position P Gain",          1, None),
    (0x16, "Position D Gain",          1, None),
    (0x17, "Position I Gain",          1, None),
    (0x18, "Min Startup Force",        1, "0.001 × stall torque"),
    (0x19, "Integral Limit",           1, "max integral = value × 4  (0 = disabled)"),
    (0x1A, "CW Dead Band",             1, "steps  (default 1)"),
    (0x1B, "CCW Dead Band",            1, "steps  (default 1)"),
    (0x1C, "Protection Current",       2, "6.5 mA/count  (max 500 = 3250 mA)"),
    (0x1E, "Angle Resolution",         1, "1–3, amplification factor for min angle"),
    (0x1F, "Position Correction",      2, "BIT11=sign, bits0-10=magnitude (0–2047 steps)"),
    (0x21, "Operating Mode",           1, {0: "position", 1: "velocity/wheel",
                                           2: "PWM open-loop", 3: "step"}),
    (0x22, "Protection Torque",        1, "% of max torque  (default 20)"),
    (0x23, "Protection Time",          1, "×10 ms  (default 200 = 2 s)"),
    (0x24, "Overload Torque",          1, "% of max torque  (default 80)"),
    (0x25, "Speed Loop P Gain",        1, None),
    (0x26, "Overcurrent Protect Time", 1, "×10 ms  (max 254 = 2540 ms)"),
    (0x27, "Speed Loop I Gain",        1, None),
]

# Correct STS3215 register map (verified against SMS&STS memory table):
#   0x00  Firmware major version    (read-only)
#   0x01  Firmware minor version    (read-only)
#   0x05  ID                        (EEPROM, read/write)
#   0x06  Baud rate                 (EEPROM, 0=1M 1=500k 2=250k 3=128k 4=115200…)
#   0x07  Return delay              (no function on STS series)
#   0x08  Response status level     (EEPROM, 0=READ+PING only, 1=all commands)
#   0x37  Lock flag                 (SRAM, 0=unlock EEPROM, 1=lock)
DIAG_REGISTERS = [
    (0x00, "Firmware version (major)"),
    (0x01, "Firmware version (minor)"),
    (0x05, "ID (EEPROM)"),
    (0x06, "Baud rate"),
    (0x08, "Response status level"),
]

def run_diagnostics(port: str, baud: int):
    """
    Read key configuration registers from every servo found on the bus.

    Highlights:
      Register 0x06  Baud rate      — must match the host baud rate (0 = 1 Mbps)
      Register 0x08  Response level — 0 = READ+PING only, 1 = all commands
    """
    header("Servo Diagnostics  —  Key Configuration Registers")
    print("  Scanning for servos (IDs 1–10)...", end="", flush=True)
    found = []
    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid in range(1, 11):
                if ping(ser, sid):
                    found.append(sid)
    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return
    print(f"\r  Found: {found}{' ' * 40}")

    if not found:
        print("  No servos found. Cannot read registers.")
        return

    warnings = []

    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid in found:
                header(f"Servo ID {sid}")
                for reg, label in DIAG_REGISTERS:
                    val = read_byte_register(ser, sid, reg)
                    if val is None:
                        print(f"    0x{reg:02X}  {label:<36}  no response")
                        continue

                    if reg == 0x06:
                        annotation = BAUD_MAP.get(val, "unknown")
                        line = f"    0x{reg:02X}  {label:<36}  {val}  ({annotation} bps)"
                        if val != 0:
                            line += "  ← not 1 Mbps"
                            warnings.append(
                                f"  ID {sid}  reg 0x06  baud={val} ({annotation} bps) — host baud may differ")
                    elif reg == 0x08:
                        annotation = RETURN_LEVEL_MAP.get(val, "unknown")
                        line = f"    0x{reg:02X}  {label:<36}  {val}  ({annotation})"
                    else:
                        line = f"    0x{reg:02X}  {label:<36}  {val}  (0x{val:02X})"

                    print(line)

    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return

    header("Diagnostic Summary")
    if warnings:
        print("  ⚠  Issues found:\n")
        for w in warnings:
            print(f"  {w}")
    else:
        print("  All registers look normal.")
        print("  Baud rate is 1 Mbps, response level set correctly.")
    print()

# ── Mode 8: Fix baud rate and response level ─────────────────────────────────

def run_fix_config(port: str, baud: int):
    """
    Correct EEPROM registers on all found servos using the verified STS3215 register map:

      0x06  Baud rate              → 0  (1,000,000 bps)
      0x08  Response status level  → 0  (READ + PING responses only)

    A previous run of this script accidentally wrote to register 0x06 with the
    wrong value (2 = 250 kbps), so the servos may now be at an unknown baud rate.
    This version scans all possible baud rates automatically to locate them first.

    A power cycle is required after this operation for EEPROM changes to take effect.
    """
    header("Fix Servo Configuration  —  Baud Rate + Response Level")
    print("  Correct register addresses (verified against STS3215 memory map):")
    print("    reg 0x06  baud rate            → 0  (1,000,000 bps)")
    print("    reg 0x08  response status lvl  → 0  (READ + PING responses only)\n")
    print("  ⚠  After this runs, power-cycle the servos before testing.")
    print("  ⚠  Connect ONE arm at a time if both arms share this bus.\n")

    confirm = prompt("Proceed? (y/n)", "n")
    if confirm.lower() != "y":
        print("  Aborted.")
        return

    # ── Auto-detect current baud rate ────────────────────────────────────────
    # A previous script run may have changed reg 0x06 to an unknown value,
    # so scan all possible STS3215 baud rates to find the servos.
    scan_rates = [1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400]
    found      = []
    active_baud = None

    print("  Scanning all baud rates to locate servos...")
    for rate in scan_rates:
        try:
            with serial.Serial(port, rate, timeout=0.15) as ser:
                hits = []
                for sid in range(1, 11):
                    if ping(ser, sid):
                        hits.append(sid)
                if hits:
                    found       = hits
                    active_baud = rate
                    print(f"  Found {hits} at {rate:,} bps")
                    break
                else:
                    print(f"  {rate:>10,} bps — no response")
        except serial.SerialException as e:
            print(f"  {rate:>10,} bps — port error: {e}")
            break

    if not found:
        print("\n  No servos found at any baud rate.")
        print("  Check power and USB connection, then try again.")
        return

    print(f"\n  Working at {active_baud:,} bps with servos {found}\n")

    results = {}

    # ── Phase 1: Write registers at active_baud ──────────────────────────────
    try:
        with serial.Serial(port, active_baud, timeout=0.15) as ser:
            for sid in found:
                header(f"Servo ID {sid}")
                servo_ok = True

                # Step 1: Response status level → 0 (READ + PING only)
                print(f"  Writing reg 0x08 (response status level) = 0 ...", end="", flush=True)
                write_eeprom_byte(ser, sid, 0x08, 0x00)
                readback = read_byte_register(ser, sid, 0x08)
                if readback == 0:
                    print("  OK")
                else:
                    rb = f"0x{readback:02X}" if readback is not None else "no response"
                    print(f"  WARNING — readback: {rb}")
                    servo_ok = False

                # Step 2: Baud rate → 0 (1 Mbps)
                print(f"  Writing reg 0x06 (baud rate)             = 0 ...", end="", flush=True)
                write_eeprom_byte(ser, sid, 0x06, 0x00)
                print("  written")

                results[sid] = "written" if servo_ok else "response level readback failed"

    except serial.SerialException as e:
        print(f"\n  Serial error during write phase: {e}")
        return

    # ── Phase 2: Verify at 1 Mbps (separate connection, no overlap) ──────────
    print()
    print("  Write phase complete. Verifying at 1 Mbps...")
    time.sleep(0.2)
    try:
        with serial.Serial(port, 1_000_000, timeout=0.15) as ser_fast:
            for sid in found:
                if ping(ser_fast, sid):
                    print(f"  ID {sid:3d}  ✓  responds at 1 Mbps")
                    if results[sid] == "written":
                        results[sid] = "OK"
                else:
                    print(f"  ID {sid:3d}  —  no ping at 1 Mbps (will confirm after power cycle)")
    except serial.SerialException as e:
        print(f"  Could not open port at 1 Mbps for verification: {e}")
        print("  Confirm with Diagnostics after power cycle.")

    header("Fix Complete")
    all_ok = all(v == "OK" for v in results.values())
    for sid, status in results.items():
        mark = "✓" if status == "OK" else "⚠"
        print(f"  {mark}  ID {sid:3d}  —  {status}")

    print()
    print("  Next steps:")
    print("    1.  Power-cycle the servos (disconnect and reconnect 12 V).")
    print("    2.  Re-run Diagnostics at 1,000,000 bps to confirm:")
    print("          reg 0x06 = 0  (1 Mbps)")
    print("          reg 0x08 = 0  (READ + PING responses)")
    print("    3.  ESP firmware SERVO_BAUD_RATE is already 1,000,000 — no change needed.")
    print()

# ── Mode 8: Full EEPROM dump ─────────────────────────────────────────────────

def run_eeprom_dump(port: str, baud: int):
    """
    Read and display every known EEPROM register for all servos on the bus.

    Run on the leader arm, copy the output, then run on the follower arm and
    compare side-by-side. Differences in 0x09/0x0B (angle limits) or
    0x1E (position offset) are common causes of follower misbehaviour.
    """
    header("Full EEPROM Dump  —  All Known Registers")
    print("  Register addresses are based on the STS3215 SMS&STS memory table")
    print("  (firmware v3.x). Verify against your datasheet if values look wrong.\n")
    print("  Run on each arm separately and compare output to find differences.")
    print("  ⚠  Key registers to compare:")
    print("       0x09/0x0B  Angle limits      — mismatches cap follower travel")
    print("       0x1F       Pos correction    — expected to differ; fix with Set Zero Point")
    print("       0x20       Mode              — must be 0 (position) on both arms\n")

    print("  Scanning for servos (IDs 1–10)...", end="", flush=True)
    found = []
    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid in range(1, 11):
                if ping(ser, sid):
                    found.append(sid)
    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return
    print(f"\r  Found: {found}{' ' * 40}")

    if not found:
        print("  No servos found.")
        return

    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid in found:
                header(f"Servo ID {sid}  —  EEPROM Register Dump")
                for (addr, label, size, notes) in EEPROM_MAP:
                    if size == 1:
                        val = read_byte_register(ser, sid, addr)
                        if val is None:
                            print(f"    0x{addr:02X}  {label:<26}  — no response")
                            continue
                        if isinstance(notes, dict):
                            annot = notes.get(val, f"unknown")
                            line  = f"    0x{addr:02X}  {label:<26}  {val:4d}  (0x{val:02X})  {annot}"
                        elif isinstance(notes, str):
                            line  = f"    0x{addr:02X}  {label:<26}  {val:4d}  (0x{val:02X})  {notes}"
                        else:
                            line  = f"    0x{addr:02X}  {label:<26}  {val:4d}  (0x{val:02X})"
                        print(line)

                    else:  # 2-byte word
                        val = read_word_register(ser, sid, addr)
                        if val is None:
                            print(f"    0x{addr:02X}  {label:<26}  — no response")
                            continue
                        if addr == OFFSET_REG:
                            # Sign-magnitude: BIT11=sign, bits 0-10=magnitude
                            sign  = -1 if (val & 0x800) else 1
                            mag   = val & 0x7FF
                            signed = sign * mag
                            deg    = signed / TICKS_PER_DEGREE
                            line   = (f"    0x{addr:02X}  {label:<26}  {val:4d}  (0x{val:04X})"
                                      f"  → {signed:+d} ticks  ({deg:+.2f}°) signed")
                        elif isinstance(notes, str):
                            line = f"    0x{addr:02X}  {label:<26}  {val:4d}  (0x{val:04X})  {notes}"
                        else:
                            line = f"    0x{addr:02X}  {label:<26}  {val:4d}  (0x{val:04X})"
                        # Flag non-default angle limits
                        if addr == 0x09 and val != 0:
                            line += "  ← ⚠ non-default min limit"
                        if addr == 0x0B and val != 4095:
                            line += "  ← ⚠ non-default max limit"
                        print(line)

    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return
    print()

# ── Mode 9: Set zero point per servo ─────────────────────────────────────────

def run_set_zero_point(port: str, baud: int):
    """
    Sequentially calibrate the zero/reference angle for each servo by writing
    a position offset to EEPROM register 0x1F/0x20.

    Procedure for each servo:
      1. User positions the arm at the desired reference angle.
      2. Script reads the current reported position.
      3. Script computes the new offset so that position reads as the target
         (default 2048 = mechanical centre of the servo's range).
      4. Offset is written to EEPROM.

    Formula:  new_correction = current_reported − target + old_correction
    The STS3215 applies correction via subtraction: reported = raw − correction.
    This correctly accounts for any correction already written in a previous run.
    On factory-fresh servos the script prompts whether to treat the existing
    register value as 0 (recommended) rather than accumulating random factory data.

    ⚠  Power cycle required after all servos are set.
    ⚠  Run EEPROM Dump to confirm 0x1F shows the new correction values after power cycle.
    ⚠  Connect one arm at a time.
    """
    header("Set Zero Point  —  Sequential Per-Servo Calibration")
    print("  For each servo the arm is positioned at a reference angle, then the")
    print("  position correction register (0x1F/0x20) is written so that angle reads")
    print("  as the chosen target value.\n")
    print("  ── Why 2048, not 0? ─────────────────────────────────────────────────")
    print("  The correction register can only shift a reading by ±2047 steps (~180°).")
    print("  The raw encoder at any joint can land anywhere from 0 to 4095 depending")
    print("  on where the magnetic sensor sits physically — it is not reset by power")
    print("  cycling or by writing the offset register.")
    print()
    print("  Target 0:    only reachable if raw encoder is already ≤ 2047 at home.")
    print("               If raw encoder > 2047, the clamp silently caps the offset")
    print("               and the servo will NOT read 0 after power cycle.")
    print()
    print("  Target 2048: always reachable from any raw encoder value (0–4095),")
    print("               because 2047 steps of correction covers the full range.")
    print("               Use 2048 as hardware home. In firmware, subtract 2048")
    print("               if you need home = 0 in software.")
    print("  ─────────────────────────────────────────────────────────────────────\n")
    print("  ⚠  Register 0x1F is the position correction register (verified against STS3215 map).")
    print("  ⚠  Power cycle required after all servos are set.")
    print("  ⚠  Disconnect the ESP32 from the servo bus before calibrating.")
    print("     If the ESP32 powers up on the same bus it will command servos to move")
    print("     before you can verify with the position monitor.\n")

    confirm = prompt("Proceed? (y/n)", "n")
    if confirm.lower() != "y":
        print("  Aborted.")
        return

    print("\n  Scanning for servos (IDs 1–10)...", end="", flush=True)
    found = []
    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid in range(1, 11):
                if ping(ser, sid):
                    found.append(sid)
    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return
    print(f"\r  Found: {found}{' ' * 40}")

    if not found:
        print("  No servos found.")
        return

    raw_t = prompt("Default target value for all servos (0–4095)", "2048")
    default_target = int(raw_t) if raw_t.isdigit() and 0 <= int(raw_t) <= 4095 else 2048
    print(f"  Using default target: {default_target} ticks\n")

    results = {}

    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid in found:
                header(f"Servo ID {sid}")

                # 1. Read the existing offset before touching anything
                old_raw = read_word_register(ser, sid, OFFSET_REG)
                if old_raw is None:
                    print(f"  Could not read offset register — skipping.")
                    results[sid] = "skipped (offset read failed)"
                    continue

                # Sign-magnitude decode: BIT11=sign, bits 0-10=magnitude
                old_sign   = -1 if (old_raw & 0x800) else 1
                old_offset = old_sign * (old_raw & 0x7FF)
                print(f"  Existing correction (0x1F): {old_offset:+d} ticks  (0x{old_raw:04X})")

                # Guard: if the existing value is non-zero the user must decide whether
                # it is a valid previous calibration (keep it → formula accumulates correctly)
                # or garbage factory data (clear it → write 0x0000 to SRAM first so the
                # capture reflects the true raw encoder, then formula uses old_offset = 0).
                #
                # ⚠  Simply zeroing old_offset in the formula WITHOUT clearing the servo's
                # SRAM correction is wrong: the capture would still include the old correction
                # and the formula would be off by exactly that amount.
                if old_raw != 0x0000:
                    keep = prompt(
                        f"  Existing value is non-zero (0x{old_raw:04X}).\n"
                        f"  y = keep (re-calibrating a previously set servo)\n"
                        f"  n = clear (factory-fresh or garbage data — writes 0 to SRAM first)\n"
                        f"  Keep existing value? (y/n)",
                        "n"
                    )
                    if keep.lower() != "y":
                        # Clear the servo's correction in SRAM so the upcoming
                        # read_present_position returns the raw encoder value.
                        print("  Clearing correction in SRAM ...", end="", flush=True)
                        write_eeprom_word(ser, sid, OFFSET_REG, 0x0000)
                        time.sleep(0.05)   # let SRAM settle
                        old_offset = 0
                        print("  done.")

                # 2. User positions the servo
                print(f"\n  Move servo ID {sid} to the desired reference angle,")
                print(f"  then press Enter to capture the position.")
                prompt("  Ready")

                current = read_present_position(ser, sid)
                if current is None:
                    print("  Could not read position — skipping.")
                    results[sid] = "skipped (position read failed)"
                    continue

                deg = current / TICKS_PER_DEGREE
                print(f"\n  Captured position:  {current} ticks  ({deg:.1f}°)")

                # 3. Allow per-servo target override
                raw_target = prompt(f"  Target value at this position", str(default_target))
                if raw_target.isdigit() and 0 <= int(raw_target) <= 4095:
                    target = int(raw_target)
                else:
                    target = default_target
                    print(f"  Invalid input — using {target}.")

                # 4. Calculate new offset:
                #    STS3215 applies correction via SUBTRACTION:
                #      reported = raw_encoder − correction
                #    Therefore:
                #      raw_encoder = reported + old_correction = current + old_offset
                #    We want: target = raw_encoder − new_correction
                #      new_correction = raw_encoder − target
                #                     = (current + old_offset) − target
                #                     = current − target + old_offset
                new_offset = current - target + old_offset
                new_offset = max(-2047, min(2047, new_offset))

                # Sign-magnitude encode: BIT11=sign, bits 0-10=magnitude
                magnitude        = abs(new_offset) & 0x7FF
                new_offset_enc   = (0x800 | magnitude) if new_offset < 0 else magnitude
                deg_shift = new_offset / TICKS_PER_DEGREE

                print(f"\n  New offset: {new_offset:+d} ticks  ({deg_shift:+.2f}°)")
                print(f"  Stored as:  0x{new_offset_enc:04X}")
                print(f"  After power cycle, this position will read as {target} ticks  "
                      f"({target / TICKS_PER_DEGREE:.1f}°).")

                confirm_w = prompt("  Write to EEPROM? (y/n)", "y")
                if confirm_w.lower() != "y":
                    print("  Skipped.")
                    results[sid] = "skipped by user"
                    continue

                print(f"  Writing ...", end="", flush=True)
                write_eeprom_word(ser, sid, OFFSET_REG, new_offset_enc)

                readback_raw = read_word_register(ser, sid, OFFSET_REG)
                if readback_raw == new_offset_enc:
                    print(f"  OK  (readback: 0x{readback_raw:04X})")
                    results[sid] = f"OK  new_offset={new_offset:+d}  target={target}"
                else:
                    rb = f"0x{readback_raw:04X}" if readback_raw is not None else "no response"
                    print(f"  ⚠ readback mismatch: got {rb}, expected 0x{new_offset_enc:04X}")
                    results[sid] = f"readback mismatch ({rb})"

    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return

    header("Set Zero Point  —  Summary")
    for sid, status in results.items():
        mark = "✓" if status.startswith("OK") else "⚠"
        print(f"  {mark}  ID {sid:3d}  —  {status}")
    print()
    print("  Next steps:")
    print("    1. Power-cycle the servos.")
    print("    2. Run EEPROM Dump → verify 0x1F shows the new correction values.")
    print("    3. Run Position Monitor → confirm reference angles read as expected.")
    print()

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

# ── Mode 3: Motion test ───────────────────────────────────────────────────────

def run_servo_test(port: str, baud: int):
    header(f"Servo Motion Test  —  ±{TEST_DEGREES}° per servo")
    print("  Each servo will move +20°, then −20°, then return to its start position.")
    print("  Make sure the arm has clearance in both directions before proceeding.\n")

    print("  Scanning for servos (IDs 1–10)...", end="", flush=True)
    found = []
    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid in range(1, 11):
                if ping(ser, sid):
                    found.append(sid)
    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return
    print(f"\r  Found: {found}{' ' * 30}")

    if not found:
        print("  No servos found. Run a Scan first to confirm connectivity.")
        return

    idx = choose(
        [
            ("All found servos", f"IDs: {found}"),
            ("Enter IDs manually", "Comma- or space-separated list"),
        ],
        "Which servos to test?"
    )
    if idx == 1:
        raw      = prompt("IDs to test (e.g. 1 2 3)")
        test_ids = [int(x) for x in raw.replace(",", " ").split() if x.strip().isdigit()]
    else:
        test_ids = found

    if not test_ids:
        print("  No valid IDs entered.")
        return

    # Diagnostic read — verify positions look sane before moving anything
    header("Position Check  (no movement)")
    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid in test_ids:
                pos = read_present_position(ser, sid)
                if pos is None:
                    print(f"  ID {sid:3d}  —  no response")
                else:
                    degrees = pos / TICKS_PER_DEGREE
                    print(f"  ID {sid:3d}  —  {pos} ticks  ({degrees:.1f}°)")
    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return

    confirm = prompt(f"\n  Positions look correct? Ready to move servos {test_ids}? (y/n)", "n")
    if confirm.lower() != "y":
        return

    results = {}
    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid in test_ids:
                header(f"Servo ID {sid}")

                # Read the position correction offset so we can work in raw encoder space.
                # Present Position (0x38) = raw_encoder + correction.
                # Goal Position   (0x2A) = raw_encoder target.
                # Using corrected position directly as a goal target causes the servo
                # to travel by (correction) extra ticks — potentially hundreds of degrees.
                offset_raw = read_word_register(ser, sid, OFFSET_REG)
                if offset_raw is None:
                    offset_raw = 0
                    print(f"  ⚠ Could not read offset register — assuming 0.")
                off_sign   = -1 if (offset_raw & 0x800) else 1
                correction = off_sign * (offset_raw & 0x7FF)
                print(f"  Offset : {correction:+d} ticks  (0x{offset_raw:04X})")

                corrected = read_present_position(ser, sid)
                if corrected is None:
                    print(f"  Could not read position — skipping.")
                    results[sid] = "no response"
                    continue

                # Convert corrected reading back to raw encoder ticks.
                # reported = raw − correction  →  raw = reported + correction
                raw_start = corrected + correction
                raw_start = max(POSITION_MIN, min(POSITION_MAX, raw_start))
                print(f"  Start  : {corrected} ticks reported  ({raw_start} raw)")

                pos_plus  = max(POSITION_MIN, min(POSITION_MAX, raw_start + TEST_TICKS))
                pos_minus = max(POSITION_MIN, min(POSITION_MAX, raw_start - TEST_TICKS))

                print(f"  +{TEST_DEGREES}°   : raw {pos_plus}  ...", end="", flush=True)
                write_goal_position(ser, sid, pos_plus)
                time.sleep(1.0)
                print("  done")

                print(f"  −{TEST_DEGREES}°   : raw {pos_minus}  ...", end="", flush=True)
                write_goal_position(ser, sid, pos_minus)
                time.sleep(1.0)
                print("  done")

                print(f"  Return : raw {raw_start}  ...", end="", flush=True)
                write_goal_position(ser, sid, raw_start)
                time.sleep(1.0)
                print("  done")

                results[sid] = "ok"

    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return

    header("Test Complete")
    for sid, status in results.items():
        print(f"  ID {sid:3d}  —  {status}")
    print()

# ── Mode 4: Set angle limits ──────────────────────────────────────────────────

def set_torque(ser: serial.Serial, servo_id: int, enable: bool):
    val = 0x01 if enable else 0x00
    ser.reset_input_buffer()
    ser.write(build_write(servo_id, TORQUE_ENABLE_REG, [val]))
    time.sleep(0.05)
    ser.read(ser.in_waiting)

def write_eeprom_limit(ser: serial.Serial, servo_id: int, address: int, value: int):
    """Write a 2-byte limit value to EEPROM with unlock/relock."""
    high = (value >> 8) & 0xFF
    low  = value & 0xFF

    unlock = build_write(servo_id, 0x37, [0x00])
    ser.reset_input_buffer()
    ser.write(unlock)
    time.sleep(0.1)
    ser.read(ser.in_waiting)

    ser.reset_input_buffer()
    ser.write(build_write(servo_id, address, [high, low]))
    time.sleep(0.3)
    ser.read(ser.in_waiting)

    relock = build_write(servo_id, 0x37, [0x01])
    ser.reset_input_buffer()
    ser.write(relock)
    time.sleep(0.1)
    ser.read(ser.in_waiting)

def capture_endpoint(ser: serial.Serial, servo_id: int, label: str) -> int | None:
    """
    Prompt the user to position the servo to an endpoint and capture it.
    Inline commands let the user lock/unlock torque on any servo mid-session.
    """
    print(f"\n  Position servo {servo_id} to the {label} endpoint.")
    print(f"  [Enter] read & capture   |   L <id>  lock torque   |   U <id>  unlock torque\n")
    while True:
        try:
            raw = input("  > ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\n  Cancelled.")
            return None

        parts = raw.lower().split()

        if raw == "":
            pos = read_present_position(ser, servo_id)
            if pos is None:
                print("  Could not read position — check connection and try again.")
                continue
            deg = pos / TICKS_PER_DEGREE
            try:
                ans = input(f"  {pos} ticks ({deg:.1f}°) — accept? [y/n]: ").strip().lower()
            except (KeyboardInterrupt, EOFError):
                return None
            if ans == "y":
                return pos

        elif len(parts) == 2 and parts[0] in ("l", "lock") and parts[1].isdigit():
            tid = int(parts[1])
            set_torque(ser, tid, True)
            print(f"  Torque LOCKED   — servo {tid}")

        elif len(parts) == 2 and parts[0] in ("u", "unlock") and parts[1].isdigit():
            tid = int(parts[1])
            set_torque(ser, tid, False)
            print(f"  Torque UNLOCKED — servo {tid}")

        else:
            print("  Commands: [Enter] capture | L <id> lock | U <id> unlock")

def run_set_limits(port: str, baud: int):
    header("Set Angle Limits")
    print("  Torque will be disabled on one servo at a time so you can")
    print("  back-drive it to each endpoint. Keep other servos powered.\n")

    print("  Scanning for servos (IDs 1–10)...", end="", flush=True)
    found = []
    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid in range(1, 11):
                if ping(ser, sid):
                    found.append(sid)
    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return
    print(f"\r  Found: {found}{' ' * 30}")

    if not found:
        print("  No servos found.")
        return

    idx = choose(
        [
            ("All found servos", f"IDs: {found}"),
            ("Enter IDs manually", ""),
        ],
        "Which servos to configure?"
    )
    if idx == 1:
        raw      = prompt("IDs to configure (e.g. 1 2 3)")
        servo_ids = [int(x) for x in raw.replace(",", " ").split() if x.strip().isdigit()]
    else:
        servo_ids = found

    if not servo_ids:
        print("  No valid IDs entered.")
        return

    profile = {}  # servo_id → {min, max, arc}

    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid in servo_ids:
                header(f"Servo ID {sid}")

                set_torque(ser, sid, False)
                print("  Torque disabled — you can now back-drive this servo.\n")

                physical_a = capture_endpoint(ser, sid, "MINIMUM")
                if physical_a is None:
                    set_torque(ser, sid, True)
                    continue

                physical_b = capture_endpoint(ser, sid, "MAXIMUM")
                if physical_b is None:
                    set_torque(ser, sid, True)
                    continue

                set_torque(ser, sid, True)
                print("\n  Torque re-enabled.")

                # Arc runs from physical_a → physical_b.
                # Buffer inward from both endpoints to prevent hard-stop collisions.
                if physical_a <= physical_b:
                    eeprom_min = physical_a + LIMIT_BUFFER
                    eeprom_max = physical_b - LIMIT_BUFFER
                else:
                    # Servo travels high → low; sort so EEPROM min < max
                    eeprom_min = physical_b + LIMIT_BUFFER
                    eeprom_max = physical_a - LIMIT_BUFFER

                arc = eeprom_max - eeprom_min

                print(f"\n  Physical endpoints : {physical_a} → {physical_b} ticks")
                print(f"  With {LIMIT_BUFFER}-tick buffer  : {eeprom_min} – {eeprom_max} ticks")
                print(f"  Allowed arc        : {arc} ticks  ({arc / TICKS_PER_DEGREE:.1f}°)")

                confirm = prompt("\n  Write these limits to EEPROM? (y/n)", "y")
                if confirm.lower() != "y":
                    print("  Skipped.")
                    continue

                print("  Writing min limit...", end="", flush=True)
                write_eeprom_limit(ser, sid, EEPROM_MIN_ANGLE, eeprom_min)
                print("  done")

                print("  Writing max limit...", end="", flush=True)
                write_eeprom_limit(ser, sid, EEPROM_MAX_ANGLE, eeprom_max)
                print("  done")

                profile[sid] = {
                    "min": eeprom_min,
                    "max": eeprom_max,
                    "arc_ticks": arc,
                    "arc_deg": round(arc / TICKS_PER_DEGREE, 1),
                }

    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return

    if not profile:
        return

    header("Movement Profile  —  copy this into any script for the same arm")
    print("SERVO_LIMITS = {")
    for sid, p in profile.items():
        print(f'    {sid}: {{"min": {p["min"]}, "max": {p["max"]}, '
              f'"arc_ticks": {p["arc_ticks"]}, "arc_deg": {p["arc_deg"]}}},')
    print("}")
    print()

# ── Mode 5: Live position monitor ────────────────────────────────────────────

def run_position_monitor(port: str, baud: int):
    header("Live Position Monitor")
    print("  Scanning for servos (IDs 1–10)...", end="", flush=True)
    found = []
    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid in range(1, 11):
                if ping(ser, sid):
                    found.append(sid)
    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return
    print(f"\r  Found: {found}{' ' * 30}")

    if not found:
        print("  No servos found.")
        return

    n = len(found)
    print("\n  Press Ctrl+C to stop.\n")

    # Print placeholder rows once so the cursor-up trick has lines to overwrite
    for sid in found:
        print(f"  ID {sid:3d}  —  ------ ticks  (------°)")

    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            while True:
                print(f"\033[{n}A", end="", flush=True)   # move cursor up n rows
                for sid in found:
                    pos = read_present_position(ser, sid)
                    if pos is None:
                        print(f"  ID {sid:3d}  —  no response              ")
                    else:
                        deg = pos / TICKS_PER_DEGREE
                        print(f"  ID {sid:3d}  —  {pos:5d} ticks  ({deg:6.1f}°)  ")
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n  Monitor stopped.")
    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")

# ── Mode 6: Reset multi-turn → single-turn ───────────────────────────────────

def run_reset_single_turn(port: str, baud: int):
    header("Reset to Single-Turn Mode")
    print("  Reads register 0x12 on each servo and clears bit 4 (multi-turn enable).")
    print("  Position range will be restored to 0–4095 after a power cycle.\n")

    print("  Scanning for servos (IDs 1–10)...", end="", flush=True)
    found = []
    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid in range(1, 11):
                if ping(ser, sid):
                    found.append(sid)
    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return
    print(f"\r  Found: {found}{' ' * 30}")

    if not found:
        print("  No servos found.")
        return

    idx = choose(
        [
            ("All found servos", f"IDs: {found}"),
            ("Enter IDs manually", ""),
        ],
        "Which servos to reset?"
    )
    if idx == 1:
        raw      = prompt("IDs to reset (e.g. 1 2 3)")
        servo_ids = [int(x) for x in raw.replace(",", " ").split() if x.strip().isdigit()]
    else:
        servo_ids = found

    if not servo_ids:
        print("  No valid IDs entered.")
        return

    header("Mode Register Check")
    needs_reset = []
    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid in servo_ids:
                val = read_byte_register(ser, sid, MODE_REG)
                if val is None:
                    print(f"  ID {sid:3d}  —  could not read register 0x12")
                    continue
                multi_turn = bool((val >> MULTI_TURN_BIT) & 1)
                status     = "MULTI-TURN ACTIVE" if multi_turn else "already single-turn"
                print(f"  ID {sid:3d}  —  0x12 = 0x{val:02X}  ({status})")
                if multi_turn:
                    needs_reset.append((sid, val))
    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return

    if not needs_reset:
        print("\n  All servos are already in single-turn mode. Nothing to do.")
        return

    ids_to_fix = [sid for sid, _ in needs_reset]
    confirm = prompt(f"\n  Clear multi-turn bit on servos {ids_to_fix}? (y/n)", "y")
    if confirm.lower() != "y":
        return

    try:
        with serial.Serial(port, baud, timeout=0.15) as ser:
            for sid, old_val in needs_reset:
                new_val = old_val & ~(1 << MULTI_TURN_BIT) & 0xFF
                print(f"  ID {sid:3d}  —  writing 0x{new_val:02X} to 0x12 ...", end="", flush=True)
                write_eeprom_byte(ser, sid, MODE_REG, new_val)

                # Verify the write
                readback = read_byte_register(ser, sid, MODE_REG)
                if readback is not None and not ((readback >> MULTI_TURN_BIT) & 1):
                    print(f"  OK  (readback: 0x{readback:02X})")
                else:
                    rb_str = f"0x{readback:02X}" if readback is not None else "no response"
                    print(f"  WARNING — readback: {rb_str}, may need retry")
    except serial.SerialException as e:
        print(f"\n  Serial error: {e}")
        return

    header("Done")
    print("  Power-cycle the servos to apply the mode change.")
    print("  After restart, position range will be 0–4095 per revolution.\n")

# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print(f"\n{'═' * 48}")
    print( "   Serial Bus Servo Scanner & ID Programmer")
    print(f"{'═' * 48}")

    port = select_port()
    baud = select_baud()

    mode = choose(
        [
            ("Scan",               "Find all servo IDs currently on the bus"),
            ("Assign IDs",         "Number servos 1-by-1 (use when all share the same factory ID)"),
            ("Motion Test",        f"Move each servo ±{TEST_DEGREES}° and return to start"),
            ("Set Limits",         "Back-drive each servo to its endpoints and write EEPROM angle limits"),
            ("Position Monitor",   "Live tick + degree readout for all servos simultaneously"),
            ("Single-Turn Reset",  "Clear multi-turn mode (fix position readings above 4095)"),
            ("Diagnostics",        "Read key config registers: baud rate, return delay, response level"),
            ("Fix Config",         "Set baud rate → 1 Mbps and response level → all commands"),
            ("EEPROM Dump",        "Read all known EEPROM registers — compare arms to find mismatches"),
            ("Set Zero Point",     "Calibrate reference angle per servo (writes position offset to EEPROM)"),
        ],
        "Mode"
    )

    if mode == 0:
        run_scan(port, baud)
    elif mode == 1:
        run_assign(port, baud)
    elif mode == 2:
        run_servo_test(port, baud)
    elif mode == 3:
        run_set_limits(port, baud)
    elif mode == 4:
        run_position_monitor(port, baud)
    elif mode == 5:
        run_reset_single_turn(port, baud)
    elif mode == 6:
        run_diagnostics(port, baud)
    elif mode == 7:
        run_fix_config(port, baud)
    elif mode == 8:
        run_eeprom_dump(port, baud)
    else:
        run_set_zero_point(port, baud)

    again = prompt("Return to main menu? (y/n)", "n")
    if again.lower() == "y":
        main()

if __name__ == "__main__":
    main()
