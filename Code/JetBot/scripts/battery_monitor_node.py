#!/usr/bin/env python3
"""
@file battery_monitor_node.py
@brief Battery monitoring node for the SparkFun JetBot v2.1.

@details
Reads the 2S LiPo pack voltage through an ADS1115 ADC on the Qwiic I2C bus
(via a voltage divider), converts it to a state-of-charge estimate, and
publishes voltage, SOC, and low/critical flags. Logs warnings when the pack
reaches the warning or critical thresholds so the mission can be ended safely.

@author Srikanth Nanda
@date 2026-06-02
"""

import rospy
from std_msgs.msg import Float32, Bool
import board
import busio
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_ads1x15.ads1115 import ADS1115


class BatteryMonitorNode(object):
    """@brief Monitors JetBot battery voltage and publishes status."""

    def __init__(self):
        """@brief Initialize parameters, ADC, publishers, and timer."""
        # Battery parameters (2S LiPo = 6.0-8.4V)
        self.adc_channel = rospy.get_param("~adc_channel", 0)  # A0
        self.voltage_scale = rospy.get_param("~voltage_scale", 2.0)  # voltage divider
        self.max_voltage = rospy.get_param("~max_voltage", 8.4)
        self.min_voltage = rospy.get_param("~min_voltage", 6.0)
        self.warning_voltage = rospy.get_param("~warning_voltage", 6.5)
        self.critical_voltage = rospy.get_param("~critical_voltage", 6.0)
        self.publish_freq = rospy.get_param("~publish_freq", 1)  # Hz
        
        # Initialize ADC
        self.adc = None
        self.channel = None
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.adc = ADS1115(i2c)
            if self.adc_channel == 0:
                self.channel = AnalogIn(self.adc, ADS1115.P0)
            elif self.adc_channel == 1:
                self.channel = AnalogIn(self.adc, ADS1115.P1)
            elif self.adc_channel == 2:
                self.channel = AnalogIn(self.adc, ADS1115.P2)
            else:
                self.channel = AnalogIn(self.adc, ADS1115.P3)
            rospy.loginfo(f"Battery ADC initialized on channel {self.adc_channel}")
        except Exception as e:
            rospy.logerr(f"Failed to initialize ADC: {e}")
            rospy.logwarn("Battery monitoring disabled")
            self.adc = None
        
        # Publishers
        self.voltage_pub = rospy.Publisher("/battery/voltage", Float32, queue_size=10)
        self.is_low_pub = rospy.Publisher("/battery/is_low", Bool, queue_size=10)
        self.is_critical_pub = rospy.Publisher("/battery/is_critical", Bool, queue_size=10)
        self.soc_pub = rospy.Publisher("/battery/soc", Float32, queue_size=10)  # state of charge %
        
        # Timer
        rospy.Timer(rospy.Duration(1.0 / self.publish_freq), self._publish_battery_status)
        
        rospy.loginfo(
            f"Battery monitor started. Range: {self.min_voltage}V - {self.max_voltage}V, "
            f"Warning: {self.warning_voltage}V, Critical: {self.critical_voltage}V"
        )

    def _read_voltage(self):
        """@brief Read the scaled battery voltage from the ADC.
        @return Battery voltage in volts, or None on failure.
        """
        if self.adc is None or self.channel is None:
            return None
        
        try:
            # Read ADC value and scale
            adc_voltage = self.channel.voltage  # volts (0-4.096V range)
            battery_voltage = adc_voltage * self.voltage_scale
            return battery_voltage
        except Exception as e:
            rospy.logwarn(f"Error reading ADC: {e}")
            return None

    def _voltage_to_soc(self, voltage):
        """@brief Convert voltage to a linear state-of-charge estimate.
        @param voltage Battery voltage in volts.
        @return State of charge in percent [0, 100].
        """
        if voltage is None:
            return 0.0
        
        # Linear approximation for LiPo
        soc = (voltage - self.min_voltage) / (self.max_voltage - self.min_voltage) * 100.0
        soc = max(0.0, min(100.0, soc))  # clamp to 0-100
        return soc

    def _publish_battery_status(self, event):
        """@brief Read the battery and publish voltage, SOC, and flags.
        @param event ROS Timer event (unused).
        """
        voltage = self._read_voltage()
        
        if voltage is None:
            return
        
        # Publish voltage
        msg = Float32(voltage)
        self.voltage_pub.publish(msg)
        
        # Check status
        is_low = voltage < self.warning_voltage
        is_critical = voltage < self.critical_voltage
        
        self.is_low_pub.publish(Bool(is_low))
        self.is_critical_pub.publish(Bool(is_critical))
        
        # Calculate and publish state of charge
        soc = self._voltage_to_soc(voltage)
        self.soc_pub.publish(Float32(soc))
        
        # Log warnings
        if is_critical:
            rospy.logwarn(f"CRITICAL BATTERY: {voltage:.2f}V ({soc:.1f}%)")
        elif is_low:
            rospy.logwarn(f"Low battery: {voltage:.2f}V ({soc:.1f}%)")
        else:
            rospy.logdebug(f"Battery: {voltage:.2f}V ({soc:.1f}%)")


def main():
    """@brief ROS node entrypoint."""
    rospy.init_node("battery_monitor_node", anonymous=False)
    BatteryMonitorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
