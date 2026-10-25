#!/usr/bin/env python3
"""
@file qwiic_bus_scan.py
@brief I2C bus scan for SparkFun Qwiic pHAT 2.0 verification.
@author Srikanth Nanda
@date 2026-05-08
"""

import argparse


def get_smbus_module():
    """@brief Import an SMBus module.
    @return smbus-compatible module.
    """
    try:
        import smbus  # type: ignore
        return smbus
    except ImportError:
        try:
            import smbus2 as smbus  # type: ignore
            return smbus
        except ImportError:
            raise SystemExit("Missing smbus module. Install python3-smbus or smbus2.")


def probe_address(bus, address):
    """@brief Probe a single I2C address.
    @param bus SMBus instance.
    @param address I2C address to probe.
    @return True if device responds, otherwise False.
    """
    try:
        bus.write_quick(address)
        return True
    except OSError:
        try:
            bus.read_byte(address)
            return True
        except OSError:
            return False


def scan_i2c_bus(bus_id):
    """@brief Scan the specified I2C bus for devices.
    @param bus_id I2C bus number.
    @return List of detected device addresses.
    """
    smbus = get_smbus_module()
    try:
        bus = smbus.SMBus(bus_id)
    except FileNotFoundError:
        raise SystemExit("I2C bus /dev/i2c-{0} not found.".format(bus_id))
    except PermissionError:
        raise SystemExit("Permission denied on /dev/i2c-{0}. Use sudo or add user to i2c group.".format(bus_id))
    except OSError as exc:
        raise SystemExit("Failed to open I2C bus {0}: {1}".format(bus_id, str(exc)))

    found = []
    for address in range(0x03, 0x77):
        if probe_address(bus, address):
            found.append(address)
    bus.close()
    return found


def format_addresses(addresses):
    """@brief Format addresses as hex strings.
    @param addresses List of integer I2C addresses.
    @return List of hex strings.
    """
    return ["0x{0:02X}".format(addr) for addr in addresses]


def main():
    """@brief CLI entrypoint for Qwiic bus scan."""
    parser = argparse.ArgumentParser(description="Qwiic I2C bus scan")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number (default: 1)")
    args = parser.parse_args()

    addresses = scan_i2c_bus(args.bus)
    if not addresses:
        print("No I2C devices found on bus {0}.".format(args.bus))
        return

    print("Found {0} device(s) on bus {1}: {2}".format(
        len(addresses),
        args.bus,
        ", ".join(format_addresses(addresses)),
    ))


if __name__ == "__main__":
    main()
