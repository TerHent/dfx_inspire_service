#!/usr/bin/env python3
import os
import glob
import serial
import time

def calculate_checksum(data):
    # Sum from index 2 to end of data
    return sum(data[2:]) & 0xFF

def test_hand(port, hand_id):
    try:
        # Open serial port
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        return False, f"Failed to open port: {e}"

    # Construct custom protocol read command for hand_id
    # cmd: 0xEB, 0x90, id, len, cmd, addr_low, addr_high, reg_num, checksum
    packet = bytearray([0xEB, 0x90, hand_id, 0x04, 0x11, 0x0A, 0x06, 0x0C, 0x00])
    packet[-1] = calculate_checksum(packet[:-1])
    
    # Clear buffers
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    # Send
    ser.write(packet)
    time.sleep(0.01)
    
    # Read response (should be 20 bytes)
    response = ser.read(20)
    ser.close()
    
    if len(response) == 20:
        # Verify header (Inspire response header can be 0x90 0xEB or 0xEB 0x90)
        is_valid_header = (response[0] == 0xEB and response[1] == 0x90) or (response[0] == 0x90 and response[1] == 0xEB)
        if is_valid_header and response[2] == hand_id:
            expected_checksum = calculate_checksum(response[:-1])
            if response[-1] == expected_checksum:
                # Get angles
                angles = []
                for i in range(0, 12, 2):
                    val = response[7+i] | (response[8+i] << 8)
                    angles.append(val)
                return True, angles
            else:
                return False, f"Checksum mismatch: expected 0x{expected_checksum:02X}, got 0x{response[-1]:02X}"
        else:
            return False, f"Header mismatch or unexpected ID: {response[:3].hex()}"
    elif len(response) > 0:
        return False, f"Partial response of {len(response)} bytes: {response.hex()}"
    return False, "No response"

def main():
    ports = glob.glob("/dev/ttyUSB*")
    if not ports:
        print("No /dev/ttyUSB* serial ports found!")
        return

    print("Scanning serial ports for Inspire Hands...")
    for port in sorted(ports):
        print(f"\n--- Scanning {port} ---")
        for hand_id in [1, 2]:
            print(f"Querying Hand ID {hand_id}...", end=" ", flush=True)
            success, info = test_hand(port, hand_id)
            if success:
                print(f"SUCCESS! Detected Inspire Hand with ID {hand_id}.")
                print(f"Angles: {info}")
            else:
                print(f"Failed ({info})")

if __name__ == "__main__":
    main()
