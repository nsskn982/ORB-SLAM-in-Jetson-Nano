#!/usr/bin/env python3
"""
@file qwiic_oled_test.py
@brief OLED test for SparkFun Qwiic OLED displays.
@author Srikanth Nanda
@date 2026-05-08
"""

import argparse


def load_display_module():
    """@brief Load the Qwiic OLED driver module.
    @return Tuple of (module, display class).
    """
    try:
        import qwiic_micro_oled as qwiic_oled
        return qwiic_oled, qwiic_oled.QwiicMicroOled
    except ImportError:
        try:
            import qwiic_oled_display as qwiic_oled
            return qwiic_oled, qwiic_oled.QwiicOledDisplay
        except ImportError:
            raise SystemExit("Missing OLED driver. Install sparkfun-qwiic-micro-oled or sparkfun-qwiic-oled-display.")


def create_display(display_class, address):
    """@brief Create the OLED display instance.
    @param display_class Display driver class.
    @param address Optional I2C address.
    @return Display instance.
    """
    try:
        return display_class(address=address)
    except TypeError:
        return display_class()


def call_method(obj, names, *args):
    """@brief Call the first available method from a list.
    @param obj Target object.
    @param names Candidate method names.
    @param args Arguments for the method.
    @return True if a method was called, otherwise False.
    """
    for name in names:
        method = getattr(obj, name, None)
        if callable(method):
            method(*args)
            return True
    return False


def is_connected(display):
    """@brief Check if the OLED display is connected.
    @param display Display instance.
    @return True if connected, otherwise False.
    """
    if hasattr(display, "is_connected") and callable(display.is_connected):
        return bool(display.is_connected())
    if hasattr(display, "isConnected") and callable(display.isConnected):
        return bool(display.isConnected())
    if hasattr(display, "connected"):
        return bool(display.connected)
    return True


def begin_display(display):
    """@brief Initialize the OLED display if supported.
    @param display Display instance.
    """
    call_method(display, ["begin"])


def clear_display(display):
    """@brief Clear the display buffer.
    @param display Display instance.
    """
    if not call_method(display, ["clear", "clearDisplay"]):
        raise SystemExit("Display driver does not support clear/clearDisplay.")


def set_font(display, font_type):
    """@brief Set the font type if supported.
    @param display Display instance.
    @param font_type Font type index.
    """
    call_method(display, ["setFontType", "set_font_type"], font_type)


def set_cursor(display, x, y):
    """@brief Set the cursor position if supported.
    @param display Display instance.
    @param x X position.
    @param y Y position.
    """
    call_method(display, ["setCursor", "set_cursor", "setCursorXY"], x, y)


def write_text(display, text):
    """@brief Write text to the display buffer.
    @param display Display instance.
    @param text Text to write.
    """
    if not call_method(display, ["print", "write"], text):
        raise SystemExit("Display driver does not support print/write.")


def show_display(display):
    """@brief Flush the display buffer if supported.
    @param display Display instance.
    """
    call_method(display, ["display", "show"])


def main():
    """@brief CLI entrypoint for OLED test."""
    parser = argparse.ArgumentParser(description="Qwiic OLED test")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=0x3D, help="I2C address (default: 0x3D)")
    parser.add_argument("--line1", type=str, default="JetBot", help="First line text")
    parser.add_argument("--line2", type=str, default="OLED OK", help="Second line text")
    args = parser.parse_args()

    module, display_class = load_display_module()
    display = create_display(display_class, args.address)

    if not is_connected(display):
        raise SystemExit("OLED not detected. Check wiring and I2C address.")

    begin_display(display)
    clear_display(display)
    set_font(display, 1)
    set_cursor(display, 0, 0)
    write_text(display, args.line1)
    set_cursor(display, 0, 16)
    write_text(display, args.line2)
    show_display(display)

    print("OLED test complete using {0}.".format(module.__name__))


if __name__ == "__main__":
    main()
