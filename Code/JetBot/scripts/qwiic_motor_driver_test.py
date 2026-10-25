#!/usr/bin/env python3
"""
@file qwiic_motor_driver_test.py
@brief Motor driver test for SparkFun Qwiic Motor Driver.
@author Srikanth Nanda
@date 2026-05-08
"""

import argparse
import time


def import_driver_module():
    """@brief Import the Qwiic motor driver module.
    @return Imported module.
    """
    try:
        import qwiic_motor_driver as qwiic_motor_driver
        return qwiic_motor_driver
    except ImportError:
        try:
            import qwiic_scmd as qwiic_scmd
            return qwiic_scmd
        except ImportError:
            raise SystemExit(
                "Missing Qwiic motor driver library. Install sparkfun-qwiic-scmd or sparkfun-qwiic-motor-driver."
            )


def create_driver(module, address):
    """@brief Create a motor driver instance.
    @param module Imported motor driver module.
    @param address Optional I2C address.
    @return Driver instance.
    """
    if hasattr(module, "QwiicMotorDriver"):
        try:
            return module.QwiicMotorDriver(address=address)
        except TypeError:
            return module.QwiicMotorDriver()
    if hasattr(module, "QwiicScmd"):
        try:
            return module.QwiicScmd(address=address)
        except TypeError:
            return module.QwiicScmd()
    raise SystemExit("Unsupported motor driver module. Update the test script to match your driver.")


def is_connected(driver):
    """@brief Check if the driver is connected.
    @param driver Motor driver instance.
    @return True if connected, otherwise False.
    """
    if hasattr(driver, "is_connected") and callable(driver.is_connected):
        return bool(driver.is_connected())
    if hasattr(driver, "isConnected") and callable(driver.isConnected):
        return bool(driver.isConnected())
    if hasattr(driver, "connected"):
        return bool(driver.connected)
    return True


def begin_driver(driver):
    """@brief Initialize the motor driver if supported.
    @param driver Motor driver instance.
    """
    if hasattr(driver, "begin") and callable(driver.begin):
        driver.begin()


def resolve_constant(module, names, default):
    """@brief Resolve a constant from module names.
    @param module Imported module.
    @param names Candidate attribute names.
    @param default Default value.
    @return Resolved constant.
    """
    for name in names:
        if hasattr(module, name):
            return getattr(module, name)
    return default


def set_drive(driver, motor, direction, speed):
    """@brief Set motor direction and speed.
    @param driver Motor driver instance.
    @param motor Motor index or constant.
    @param direction Direction constant.
    @param speed Speed value.
    """
    if hasattr(driver, "set_drive") and callable(driver.set_drive):
        driver.set_drive(motor, direction, speed)
        return
    if hasattr(driver, "setDrive") and callable(driver.setDrive):
        driver.setDrive(motor, direction, speed)
        return
    raise SystemExit("Driver does not expose set_drive/setDrive methods.")


def stop_all(driver, motor_ids, direction_forward):
    """@brief Stop all motors.
    @param driver Motor driver instance.
    @param motor_ids List of motor identifiers.
    @param direction_forward Forward direction constant.
    """
    for motor in motor_ids:
        set_drive(driver, motor, direction_forward, 0)


def main():
    """@brief CLI entrypoint for motor driver test."""
    parser = argparse.ArgumentParser(description="Qwiic motor driver test")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=0x5D, help="I2C address (default: 0x5D)")
    parser.add_argument("--speed", type=int, default=120, help="Speed 0-255")
    parser.add_argument("--duration", type=float, default=1.0, help="Seconds per motor")
    parser.add_argument("--reverse", action="store_true", help="Also test reverse direction")
    args = parser.parse_args()

    module = import_driver_module()
    driver = create_driver(module, args.address)

    if not is_connected(driver):
        raise SystemExit("Qwiic Motor Driver not detected. Check wiring and I2C address.")

    begin_driver(driver)

    motor_a = resolve_constant(module, ["MOTOR_A", "MOTOR_1", "MOTOR0", "MOTOR_LEFT"], 0)
    motor_b = resolve_constant(module, ["MOTOR_B", "MOTOR_2", "MOTOR1", "MOTOR_RIGHT"], 1)
    dir_forward = resolve_constant(
        module,
        ["MOTOR_DIR_FORWARD", "DIR_FORWARD", "FORWARD"],
        0,
    )
    dir_reverse = resolve_constant(
        module,
        ["MOTOR_DIR_BACKWARD", "DIR_BACKWARD", "BACKWARD", "REVERSE", "DIR_REVERSE"],
        1,
    )

    motor_ids = [motor_a, motor_b]

    try:
        for motor in motor_ids:
            set_drive(driver, motor, dir_forward, args.speed)
            time.sleep(args.duration)
            set_drive(driver, motor, dir_forward, 0)
            time.sleep(0.2)

        if args.reverse:
            for motor in motor_ids:
                set_drive(driver, motor, dir_reverse, args.speed)
                time.sleep(args.duration)
                set_drive(driver, motor, dir_forward, 0)
                time.sleep(0.2)
    finally:
        stop_all(driver, motor_ids, dir_forward)

    print("Motor driver test complete.")


if __name__ == "__main__":
    main()
