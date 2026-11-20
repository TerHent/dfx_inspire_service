#!/usr/bin/env encoding
# -*- coding: utf-8 -*-
#!/usr/bin/env python3
import os
import sys
import glob
import serial
import time

def calculate_checksum(data):
    # Sum from index 2 to end of data
    return sum(data[2:]) & 0xFF

def decode_error(err_byte):
    if err_byte == 0:
        return "OK"
    flags = []
    if err_byte & 0x01: flags.append("Locked Rotor")
    if err_byte & 0x02: flags.append("Over Temp")
    if err_byte & 0x04: flags.append("Over Current")
    if err_byte & 0x08: flags.append("Abnormal Op")
    if err_byte & 0x10: flags.append("Comm Err")
    return "|".join(flags)

def check_usb_power_state(usb_path):
    # usb_path is e.g. "1-2.3"
    path = f"/sys/bus/usb/devices/{usb_path}"
    control_path = os.path.join(path, "power/control")
    status_path = os.path.join(path, "power/runtime_status")
    
    control = "Unknown"
    status = "Unknown"
    
    if os.path.exists(control_path):
        try:
            with open(control_path, 'r') as f: control = f.read().strip()
        except:
            pass
    if os.path.exists(status_path):
        try:
            with open(status_path, 'r') as f: status = f.read().strip()
        except:
            pass
        
    return control, status

def get_serial_line_states(port):
    try:
        ser = serial.Serial(port, 115200, timeout=0.01)
        cts = ser.cts
        dsr = ser.dsr
        ri = ser.ri
        cd = ser.cd
        
        # Test write and check if output buffer drains
        ser.write(b'\x00')
        time.sleep(0.002)
        out_waiting = ser.out_waiting
        
        ser.close()
        return {
            "cts": cts,
            "dsr": dsr,
            "ri": ri,
            "cd": cd,
            "out_waiting": out_waiting,
            "error": None
        }
    except Exception as e:
        return {
            "error": str(e)
        }

def get_usb_sysfs_info(port_name):
    # port_name is e.g. "ttyUSB0"
    sys_path = f"/sys/class/tty/{port_name}/device"
    if not os.path.exists(sys_path):
        return None
    
    real_path = os.path.realpath(sys_path)
    # Find the USB path (e.g. 1-2.4)
    parts = real_path.split('/')
    usb_path = ""
    for p in parts:
        if '-' in p and ':' not in p and not p.startswith('usb'):
            usb_path = p
            
    # Go up parent directories to find USB device descriptors
    dev_dir = sys_path
    vendor = "Unknown"
    product = "Unknown"
    manufacturer = "Unknown"
    product_name = "Unknown"
    for _ in range(5):
        dev_dir = os.path.dirname(dev_dir)
        vendor_file = os.path.join(dev_dir, "idVendor")
        product_file = os.path.join(dev_dir, "idProduct")
        if os.path.exists(vendor_file) and os.path.exists(product_file):
            try:
                with open(vendor_file, 'r') as f: vendor = f.read().strip()
                with open(product_file, 'r') as f: product = f.read().strip()
                
                man_file = os.path.join(dev_dir, "manufacturer")
                prod_name_file = os.path.join(dev_dir, "product")
                if os.path.exists(man_file):
                    with open(man_file, 'r') as f: manufacturer = f.read().strip()
                if os.path.exists(prod_name_file):
                    with open(prod_name_file, 'r') as f: product_name = f.read().strip()
            except:
                pass
            break
            
    return {
        "usb_path": usb_path,
        "vendor_id": vendor,
        "product_id": product,
        "manufacturer": manufacturer,
        "product_name": product_name,
        "sysfs_path": real_path
    }

class InspireDebugger:
    def __init__(self, port, hand_id=1, baudrate=115200):
        self.port = port
        self.hand_id = hand_id
        self.baudrate = baudrate
        self.ser = None
        self.finger_names = ["Little", "Ring", "Middle", "Index", "Thumb Bend", "Thumb Rot"]
        
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.04)
            return True
        except Exception as e:
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_query(self, reg_addr, reg_num, response_len):
        if not self.ser or not self.ser.is_open:
            return None
        
        # Build read command packet
        # cmd: 0xEB, 0x90, id, len, cmd (0x11 = read), addr_low, addr_high, reg_num, checksum
        packet = bytearray([
            0xEB, 0x90, 
            self.hand_id, 
            0x04, 0x11, 
            reg_addr & 0xFF, (reg_addr >> 8) & 0xFF, 
            reg_num, 
            0x00
        ])
        packet[-1] = calculate_checksum(packet[:-1])
        
        try:
            self.ser.reset_input_buffer()
            self.ser.write(packet)
            time.sleep(0.005) # small wait
            
            response = self.ser.read(response_len)
            if len(response) != response_len:
                return None
                
            is_valid_header = (response[0] == 0xEB and response[1] == 0x90) or (response[0] == 0x90 and response[1] == 0xEB)
            if not is_valid_header or response[2] != self.hand_id:
                return None
                
            expected_checksum = calculate_checksum(response[:-1])
            if response[-1] != expected_checksum:
                return None
                
            return response[7:-1] # Extract data payload
        except Exception as e:
            return None

    def read_all(self):
        # 1. Read Angles (register 1546 = 0x060A, 12 bytes, response len = 20)
        angle_data = self.send_query(1546, 12, 20)
        if angle_data is None:
            return None
        
        # 2. Read Forces (register 1582 = 0x062E, 12 bytes, response len = 20)
        force_data = self.send_query(1582, 12, 20)
        
        # 3. Read Temps (register 1618 = 0x0652, 6 bytes, response len = 14)
        temp_data = self.send_query(1618, 6, 14)
        
        # 4. Read Errors (register 1606 = 0x0646, 6 bytes, response len = 14)
        error_data = self.send_query(1606, 6, 14)
            
        angles = []
        for i in range(6):
            angles.append(angle_data[2*i] | (angle_data[2*i+1] << 8))
            
        forces = []
        if force_data is not None:
            for i in range(6):
                forces.append(force_data[2*i] | (force_data[2*i+1] << 8))
        else:
            forces = [None] * 6
            
        temps = list(temp_data) if temp_data is not None else [None] * 6
        errors = list(error_data) if error_data is not None else [None] * 6
        
        return {
            "angles": angles,
            "forces": forces,
            "temps": temps,
            "errors": errors
        }

def run_diagnostics():
    print("=====================================================================")
    print("                INSPIRE ROBOTS DEXTEROUS HAND DIAGNOSTICS             ")
    print("=====================================================================")
    print(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")

    # Step 1: Check Permissions
    print("Step 1: Checking permissions...")
    is_root = os.geteuid() == 0
    if not is_root:
        print(" -> [WARNING] Script not run as root. Serial connections may fail due to permissions.")
    else:
        print(" -> [OK] Running with root (sudo) privileges.")
    print()

    # Step 2: Scan for Serial Ports
    print("Step 2: Scanning for serial ports...")
    ports = sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        print(" -> [FAILED] No active /dev/ttyUSB* serial ports found!")
        print("    Ensure the USB-to-serial cables are physically connected to the Jetson.")
        return
    else:
        print(f" -> Found active ports: {', '.join(ports)}")
    print()

    # Step 3: Deep inspection of each port
    print("Step 3: Beginning detailed port checks...\n")
    for port in ports:
        port_name = os.path.basename(port)
        role = "Right Hand" if port_name == "ttyUSB0" else "Left Hand"
        
        print(f"---------------------------------------------------------------------")
        print(f"Port: {port} | Configured Role: {role}")
        print(f"---------------------------------------------------------------------")
        
        # Get USB hardware info
        info = get_usb_sysfs_info(port_name)
        if info:
            print(f"Hardware Info:")
            print(f"  USB Port Hub Path : {info['usb_path']}")
            print(f"  Device Name       : {info['product_name']} (by {info['manufacturer']})")
            print(f"  Vendor:Product ID : {info['vendor_id']}:{info['product_id']}")
            
            # Check USB power management status
            control, status = check_usb_power_state(info['usb_path'])
            print(f"  USB Power Control : {control} (Runtime Status: {status})")
        else:
            print(f"Hardware Info: Could not retrieve USB descriptors from sysfs.")
            
        # Check Serial line states
        print(f"Serial Line States:")
        states = get_serial_line_states(port)
        if "error" in states and states["error"] is not None:
            print(f"  [ERROR] Could not query line states: {states['error']}")
        else:
            print(f"  CTS (Clear to Send)  : {states['cts']}")
            print(f"  DSR (Data Set Ready) : {states['dsr']}")
            print(f"  DCD (Carrier Detect) : {states['cd']}")
            print(f"  RI  (Ring Indicator) : {states['ri']}")
            print(f"  TX Buffer Status     : {'Drained (OK)' if states['out_waiting'] == 0 else 'STALLED (Data blocked)'}")
            
        print("\nScanning Port Baud Rates & Slave IDs:")
        
        found_hand = False
        # Try different baud rates
        for baud in [115200, 57600, 9600]:
            print(f"  Testing at {baud} baud:")
            
            for hand_id in [1, 2]:
                print(f"    Probing Slave ID {hand_id}...", end=" ", flush=True)
                dbg = InspireDebugger(port, hand_id, baud)
                if dbg.connect():
                    # Test query: Angles (Reg 1546)
                    res = dbg.send_query(1546, 12, 20)
                    if res is not None:
                        print(f"SUCCESS! Hand responded.")
                        # Run full telemetry read
                        telemetry = dbg.read_all()
                        if telemetry is not None:
                            print(f"\n  [OK] Telemetry Readout ({role} | ID: {hand_id} | {baud} Baud):")
                            print(f"    {'Finger':<15} | {'Angle (0-1000)':<15} | {'Force (g)':<10} | {'Temp (°C)':<10} | {'Error State':<20}")
                            print("    " + "-" * 75)
                            for i in range(6):
                                name = dbg.finger_names[i]
                                ang = telemetry["angles"][i]
                                frc = telemetry["forces"][i] if telemetry["forces"][i] is not None else "N/A"
                                tmp = telemetry["temps"][i] if telemetry["temps"][i] is not None else "N/A"
                                err = decode_error(telemetry["errors"][i]) if telemetry["errors"][i] is not None else "N/A"
                                print(f"    {name:<15} | {ang:<15} | {frc:<10} | {tmp:<10} | {err:<20}")
                        else:
                            print("\n  [WARNING] Connection established, but failed to retrieve full registers.")
                        found_hand = True
                        dbg.close()
                        break
                    else:
                        print("Failed (No response).")
                    dbg.close()
                else:
                    print("Failed (Could not open port).")
            
            if found_hand:
                break
                
        if not found_hand:
            print(f"\n  [FAILED] {role} ({port}) is completely unresponsive on ID 1 & 2.")
            print(f"           The microcontroller is likely crashed (fused/browned out) or lacks power.")
        print()

if __name__ == "__main__":
    run_diagnostics()
