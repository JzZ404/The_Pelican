#!/usr/bin/env python3
"""
Scan all Dynamixel IDs (1-253) across common baud rates.
Run this ON the TurtleBot3 (Raspberry Pi) where the motors are connected.

Usage:
    python3 scan_motors.py
    python3 scan_motors.py --port /dev/ttyACM0
"""

import argparse
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

BAUD_RATES = [57600, 1000000, 115200, 2000000]
ID_RANGE   = range(1, 20)   # check IDs 1-19 (covers any typical setup fast)

def scan(device, baud, protocol):
    port = PortHandler(device)
    if not port.openPort():
        return None
    port.setBaudRate(baud)
    pkt = PacketHandler(protocol)

    found = []
    for motor_id in ID_RANGE:
        model, result, _ = pkt.ping(port, motor_id)
        if result == COMM_SUCCESS:
            found.append((motor_id, model))

    port.closePort()
    return found


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=None, help='Serial port (auto-detect if omitted)')
    args = parser.parse_args()

    # Auto-detect port
    import glob
    if args.port:
        ports = [args.port]
    else:
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        if not ports:
            print("[ERROR] No serial ports found. Is the U2D2 / OpenCR plugged in?")
            return

    print(f"Scanning ports: {ports}")
    print(f"ID range: {ID_RANGE.start}–{ID_RANGE.stop - 1}\n")

    any_found = False
    for device in ports:
        for baud in BAUD_RATES:
            for protocol in [2.0, 1.0]:
                found = scan(device, baud, protocol)
                if found is None:
                    print(f"  [SKIP] Could not open {device}")
                    break
                if found:
                    any_found = True
                    print(f"  [FOUND] {device}  baud={baud}  protocol={protocol}")
                    for motor_id, model in found:
                        print(f"    Motor ID {motor_id}  model={model}")

    if not any_found:
        print("No motors found. Check wiring, power, and port.")


if __name__ == '__main__':
    main()
