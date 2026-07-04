#!/usr/bin/env python3
"""Probe SHZK thermocouple Modbus RTU temperature transmitter."""
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

def build_read_frame(slave_addr: int, func_code: int, start_reg: int, num_regs: int) -> bytes:
    payload = struct.pack('>BBHH', slave_addr, func_code, start_reg, num_regs)
    return payload + modbus_crc(payload)

def try_read(ser, slave_addr, func_code, start_reg, num_regs, timeout=0.5):
    frame = build_read_frame(slave_addr, func_code, start_reg, num_regs)
    ser.timeout = timeout
    ser.reset_input_buffer()
    ser.write(frame)
    time.sleep(0.05)
    response = ser.read(256)
    return response

def parse_temp(raw_val):
    """Try to interpret a 16-bit register as temperature (signed, 0.1C resolution)."""
    signed = struct.unpack('>h', struct.pack('>H', raw_val))[0]
    return signed / 10.0

# Common baud rates for Modbus devices
BAUD_RATES = [9600, 4800, 19200, 38400, 115200]
# Common slave addresses
SLAVE_ADDRS = [1, 2, 3, 4, 5]
# Common function codes for reading
FUNC_CODES = [(0x03, "Holding Registers"), (0x04, "Input Registers")]

PORT = '/dev/ttyUSB0'

for baud in BAUD_RATES:
    print(f"\n=== Testing baud rate: {baud} ===")
    try:
        ser = serial.Serial(PORT, baud, bytesize=8, parity='N', stopbits=1, timeout=0.5)
        time.sleep(0.1)
    except Exception as e:
        print(f"  Cannot open port: {e}")
        continue

    found = False
    for slave in SLAVE_ADDRS:
        for func_code, func_name in FUNC_CODES:
            # Try reading 1 register from address 0
            resp = try_read(ser, slave, func_code, 0, 1)
            if resp and len(resp) >= 5:
                # Check if it's a valid Modbus response (not an exception)
                if resp[0] == slave and resp[1] == func_code:
                    print(f"  FOUND! slave={slave}, func=0x{func_code:02X} ({func_name})")
                    data_bytes = resp[2]
                    num_regs = data_bytes // 2
                    print(f"  Response data bytes: {data_bytes}")

                    # Read more registers to get temperature values
                    resp2 = try_read(ser, slave, func_code, 0, 8)
                    if resp2 and len(resp2) >= 21:  # 1+1+1+16+2
                        data = resp2[3:3+16]
                        print(f"  Raw register values (addr 0-7):")
                        for i in range(0, 16, 2):
                            val = struct.unpack('>H', data[i:i+2])[0]
                            temp = parse_temp(val)
                            print(f"    Reg[{i//2}]: raw={val} (0x{val:04X}), as_temp={temp:.1f}°C")
                    found = True
                    break
                elif resp[0] == slave and (resp[1] & 0x80):
                    # Exception response
                    pass
        if found:
            break

    if not found:
        # Also try with parity Even (some Modbus devices use 8E1)
        try:
            ser.close()
            ser = serial.Serial(PORT, baud, bytesize=8, parity='E', stopbits=1, timeout=0.5)
            time.sleep(0.1)
            for slave in SLAVE_ADDRS:
                for func_code, func_name in FUNC_CODES:
                    resp = try_read(ser, slave, func_code, 0, 1)
                    if resp and len(resp) >= 5 and resp[0] == slave and resp[1] == func_code:
                        print(f"  FOUND (8E1)! slave={slave}, func=0x{func_code:02X} ({func_name})")
                        resp2 = try_read(ser, slave, func_code, 0, 8)
                        if resp2 and len(resp2) >= 21:
                            data = resp2[3:3+16]
                            print(f"  Raw register values (addr 0-7):")
                            for i in range(0, 16, 2):
                                val = struct.unpack('>H', data[i:i+2])[0]
                                temp = parse_temp(val)
                                print(f"    Reg[{i//2}]: raw={val} (0x{val:04X}), as_temp={temp:.1f}°C")
                        found = True
                        break
                if found:
                    break
        except:
            pass

    try:
        ser.close()
    except:
        pass

print("\nDone.")
