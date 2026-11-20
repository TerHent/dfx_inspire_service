#!/usr/bin/env python3
import sys
import serial
import time

def calculate_checksum(data):
    # Sum from index 2 to end of data
    return sum(data[2:]) & 0xFF

def scan_port(port):
    print(f"Scanning all 254 Slave IDs on {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=0.01) # 10ms timeout for speed
    except Exception as e:
        print(f"Failed to open {port}: {e}")
        return

    found = False
    for hand_id in range(1, 255):
        # Read Angles command (register 1546 = 0x060A, data len 12 = 0x0C)
        packet = bytearray([0xEB, 0x90, hand_id, 0x04, 0x11, 0x0A, 0x06, 0x0C, 0x00])
        packet[-1] = calculate_checksum(packet[:-1])
        
        ser.reset_input_buffer()
        ser.write(packet)
        time.sleep(0.005) # 5ms wait
        
        response = ser.read(20)
        if len(response) > 0:
            print(f" -> Got response on ID {hand_id}: len={len(response)}, hex={response.hex()}")
            found = True
            
    if not found:
        print("Scan complete. No response from any Slave ID.")
    ser.close()

if __name__ == "__main__":
    port = "/dev/ttyUSB1"
    if len(sys.argv) > 1:
        port = sys.argv[1]
    scan_port(port)
