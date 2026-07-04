#!/usr/bin/env python3
"""Write baud rate to SHZK Modbus register 0x000E to set 115200."""
import serial
import struct
import time

def modbus_crc(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)

def write_reg(slave, reg, value):
    payload = struct.pack('>BBHH', slave, 0x06, reg, value)
    return payload + modbus_crc(payload)

def read_reg(slave, reg, count):
    payload = struct.pack('>BBHH', slave, 0x03, reg, count)
    return payload + modbus_crc(payload)

ser = serial.Serial('/dev/ttyUSB0', 9600, bytesize=8, parity='N', stopbits=1, timeout=0.5)
time.sleep(0.1)

# First read register 0x000E to see current value
print("=== Current register 0x000E value (at 9600) ===")
frame = read_reg(1, 0x000E, 1)
ser.write(frame)
time.sleep(0.1)
resp = ser.read(256)
if resp and len(resp) >= 7:
    val = struct.unpack('>H', resp[3:5])[0]
    print(f"  Current value: {val} (0x{val:04X})")
    print(f"  Upper byte (baud code): 0x{val >> 8:02X} ({val >> 8})")
    print(f"  Lower byte (slave addr): 0x{val & 0xFF:02X} ({val & 0xFF})")
else:
    print(f"  Response: {resp.hex() if resp else 'none'}")

# Write 0x000A to register 0x000E (baud=115200, slave addr stays 1)
# Actually need to preserve slave address in lower byte
# Current value upper byte = baud, lower byte = address
# For 115200: write value = 0x0A (baud code)
# But register 0x000E stores baud in upper byte and address in lower byte
# So we write (0x0A << 8) | current_addr
new_val = (0x0A << 8) | 1  # 0x0A01
print(f"\n=== Writing 0x{new_val:04X} ({new_val}) to register 0x000E ===")
print(f"  Setting baud code to 0x0A (115200), preserving slave address 1")
frame = write_reg(1, 0x000E, new_val)
print(f"  Frame: {frame.hex()}")
ser.write(frame)
time.sleep(0.2)
resp = ser.read(256)
print(f"  Response: {resp.hex() if resp else 'none'}")
if resp and len(resp) >= 8:
    resp_val = struct.unpack('>H', resp[4:6])[0]
    print(f"  Confirmed written value: 0x{resp_val:04X}")

# Now verify at 115200 baud
print("\n=== Verifying at 115200 baud ===")
ser.close()
time.sleep(0.5)
ser = serial.Serial('/dev/ttyUSB0', 115200, bytesize=8, parity='N', stopbits=1, timeout=0.5)
time.sleep(0.1)

frame = read_reg(1, 0x0000, 1)
ser.write(frame)
time.sleep(0.1)
resp = ser.read(256)
if resp and len(resp) >= 5 and resp[0] == 1 and resp[1] == 0x03:
    val = struct.unpack('>H', resp[3:5])[0]
    print(f"  SUCCESS! Register 0x0000 = {val} at 115200 baud")
else:
    print(f"  Response: {resp.hex() if resp else 'none'}")
    # Maybe the register 0x000E format is different
    # Try: just write 0x000A directly (some devices store baud rate value directly)
    print("  Trying alternative: direct write 0x000A to reg 0x000E at old baud...")
    ser.close()
    ser = serial.Serial('/dev/ttyUSB0', 9600, bytesize=8, parity='N', stopbits=1, timeout=0.5)
    time.sleep(0.1)
    # The register 0x000E might just store the baud rate divisor or code directly
    # without combining with address. Try writing just 0x000A.
    frame = write_reg(1, 0x000E, 0x000A)
    ser.write(frame)
    time.sleep(0.2)
    resp = ser.read(256)
    print(f"  Write response: {resp.hex() if resp else 'none'}")

    # Try 115200
    ser.close()
    time.sleep(0.5)
    ser = serial.Serial('/dev/ttyUSB0', 115200, bytesize=8, parity='N', stopbits=1, timeout=0.5)
    time.sleep(0.1)
    frame = read_reg(1, 0x0000, 1)
    ser.write(frame)
    time.sleep(0.1)
    resp = ser.read(256)
    if resp and len(resp) >= 5 and resp[0] == 1 and resp[1] == 0x03:
        val = struct.unpack('>H', resp[3:5])[0]
        print(f"  SUCCESS! Register 0x0000 = {val} at 115200 baud")
    else:
        print(f"  Still no response: {resp.hex() if resp else 'none'}")
        # Restore to 9600
        print("  Restoring to 9600...")
        ser.close()
        ser = serial.Serial('/dev/ttyUSB0', 9600, bytesize=8, parity='N', stopbits=1, timeout=0.5)
        time.sleep(0.1)

ser.close()
print("\nDone.")
