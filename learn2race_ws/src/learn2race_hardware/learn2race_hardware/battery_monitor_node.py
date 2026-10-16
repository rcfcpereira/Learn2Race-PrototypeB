#!/usr/bin/env python3
"""Pack monitoring for Prototype B (FR11, NFR1).

Prototype A reads its pack from the INA219 fitted on the driver board. Prototype
B has no equivalent: the DDSM210 reports motor current, which is not pack
current and says nothing about the state of the battery. This node fills that
gap with either of two backends, chosen by parameter.

``ads1115``
    A resistor divider from the pack into a spare ADS1115 channel, and
    optionally a low-side shunt read differentially across AIN2-AIN3. The ADC is
    already fitted for the start-light sensor, so voltage costs two resistors
    and current costs one shunt.

    The shunt must sit in the ground return, not in the positive lead. ADS1115
    inputs must remain within GND - 0.3 V to VDD + 0.3 V, so a high-side shunt
    on a 4S pack would put 16.8 V of common mode onto a 3.3 V part. Low-side
    sensing keeps the common mode near zero at the cost of the load ground
    floating above pack negative by the shunt drop, which for a 5 mOhm shunt at
    20 A is 100 mV and harmless.

    Two wiring details decide whether the current reading is meaningful. Sense
    wires must land on the shunt body as a Kelvin connection, not on the power
    lugs, or the measurement includes the solder joints. And each sense line
    needs an RC filter, roughly 1 kOhm with 100 nF: the motor switches at
    20 kHz while the ADC samples at 8 to 860 SPS, so unfiltered switching noise
    aliases directly into the result.

``power_module``
    An inline APM/Pixhawk-pattern power module with XT60 connectors, read
    through two single-ended ADS1115 channels. This is the least invasive
    option: the module plugs between the battery and the robot and presents
    pack voltage and current as analogue signals already scaled for a 5 V ADC,
    so nothing is soldered into the power path and no shunt sizing is required.

    The scaling constants differ between manufacturers and between clones of
    the same design, so ``volt_multiplier`` and ``amps_per_volt`` must be
    calibrated rather than assumed. The defaults are the 3DR values that
    ArduPilot uses, which are correct for the original module and approximately
    correct for most copies.

``ina226``
    An INA226 across the battery lead on the same I2C bus. Gives voltage and
    current, and therefore real watt-hours per round rather than a discharge
    curve alone.

Divider sizing for a 4S pack: full charge is 16.8 V and the ADS1115 full-scale
range at the default gain is 4.096 V, so a ratio of at least 4.1:1 is required.
A 100k/33k divider gives 4.03:1, placing 16.8 V at 4.17 V, which clips. Use
100k/27k for 4.70:1, placing 16.8 V at 3.57 V with headroom. Measure the fitted
resistors and set divider_ratio from the measurement, not from the marking.
"""

# References (full entries: docs/references.md at the repository root):
#   [R69] INA219 / INA226 datasheets, INA219_WE: INA226
#   [R70] ADS1115 / ADS7830 / PCF8591 datasheets: ADS1115 differential mode
#   [R3]  Learn2Race dissertation, requirements FR/NFR: FR11, NFR1
# More details: README.md of this package

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

try:
    from smbus2 import SMBus
except ImportError:                                   # pragma: no cover
    SMBus = None


class BatteryMonitor(Node):

    def __init__(self):
        super().__init__('battery_monitor')

        self.declare_parameter('backend', 'ads1115')   # ads1115 | power_module | ina226
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('cells', 4)
        self.declare_parameter('cell_floor_volts', 3.50)
        self.declare_parameter('cell_full_volts', 4.20)
        self.declare_parameter('publish_hz', 1.0)

        # ADS1115 backend
        self.declare_parameter('ads_address', 0x48)
        self.declare_parameter('ads_channel', 1)
        self.declare_parameter('ads_full_scale_volts', 4.096)
        self.declare_parameter('divider_ratio', 4.70)
        # Shunt sensing. Set shunt_ohms to 0.0 to disable and report voltage only.
        self.declare_parameter('shunt_enabled', False)
        self.declare_parameter('ads_shunt_pair', [2, 3])
        self.declare_parameter('ads_shunt_full_scale_volts', 0.256)
        self.declare_parameter('ads_shunt_data_rate', 8)
        self.declare_parameter('ads_shunt_ohms', 0.005)
        self.declare_parameter('ads_shunt_sign', -1)  # low-side: drop is negative
        self.declare_parameter('current_offset_a', 0.0)

        # power_module backend: inline APM/Pixhawk module with XT60 connectors
        self.declare_parameter('pm_voltage_channel', 1)
        self.declare_parameter('pm_current_channel', 2)
        self.declare_parameter('pm_full_scale_volts', 4.096)
        self.declare_parameter('pm_volt_multiplier', 10.1)   # pack volts per sense volt
        self.declare_parameter('pm_amps_per_volt', 17.0)     # amps per sense volt
        self.declare_parameter('pm_current_zero_volts', 0.0) # sense volts at zero current

        # INA226 backend
        self.declare_parameter('ina_address', 0x40)
        self.declare_parameter('shunt_ohms', 0.002)
        self.declare_parameter('max_expected_amps', 20.0)

        self.backend = self.get_parameter('backend').value
        self.cells = int(self.get_parameter('cells').value)
        self.floor = self.cells * float(self.get_parameter('cell_floor_volts').value)
        self.full = self.cells * float(self.get_parameter('cell_full_volts').value)

        if SMBus is None:
            raise RuntimeError('smbus2 is required for the battery monitor')
        self.bus = SMBus(int(self.get_parameter('i2c_bus').value))

        self.ina = None
        if self.backend == 'power_module':
            from .ads1115 import Ads1115
            self.ads = Ads1115(self.bus,
                               address=int(self.get_parameter('ads_address').value))
            self.get_logger().info(
                'Pack monitored by an inline power module: voltage and current '
                'from two analogue channels. Calibrate pm_volt_multiplier and '
                'pm_amps_per_volt against a reference meter before trusting '
                'the readings.')
        elif self.backend == 'ina226':
            from .ina226 import Ina226
            self.ina = Ina226(
                self.bus,
                address=int(self.get_parameter('ina_address').value),
                shunt_ohms=float(self.get_parameter('shunt_ohms').value),
                max_expected_amps=float(self.get_parameter('max_expected_amps').value))
            self.get_logger().info('Pack monitored by INA226: voltage and current.')
        else:
            from .ads1115 import Ads1115
            self.ads = Ads1115(self.bus,
                               address=int(self.get_parameter('ads_address').value))
            if bool(self.get_parameter('shunt_enabled').value):
                self.get_logger().info(
                    'Pack monitored by ADS1115: voltage from a divider, current '
                    'from a low-side shunt read differentially.')
            else:
                self.get_logger().info(
                    'Pack monitored by ADS1115 divider: voltage only. Enable '
                    'shunt_enabled once the shunt is fitted to report current.')

        self.warned = False
        self.pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.create_timer(1.0 / float(self.get_parameter('publish_hz').value),
                          self._tick)

    def _read_pack(self):
        """Return (volts, amps). Amps is None when the backend cannot measure it."""
        if self.ina is not None:
            return self.ina.pack_voltage(), self.ina.current()

        if self.backend == 'power_module':
            full_scale = float(self.get_parameter('pm_full_scale_volts').value)
            sense_v = self.ads.read_volts(
                int(self.get_parameter('pm_voltage_channel').value),
                full_scale=full_scale)
            sense_i = self.ads.read_volts(
                int(self.get_parameter('pm_current_channel').value),
                full_scale=full_scale)

            volts = sense_v * float(self.get_parameter('pm_volt_multiplier').value)
            amps = (sense_i - float(self.get_parameter('pm_current_zero_volts').value)) \
                * float(self.get_parameter('pm_amps_per_volt').value)
            return volts, amps

        channel = int(self.get_parameter('ads_channel').value)
        full_scale = float(self.get_parameter('ads_full_scale_volts').value)
        ratio = float(self.get_parameter('divider_ratio').value)
        volts = self.ads.read_volts(channel, full_scale=full_scale) * ratio

        if not bool(self.get_parameter('shunt_enabled').value):
            return volts, None

        pair = tuple(int(v) for v in self.get_parameter('ads_shunt_pair').value)
        shunt_v = self.ads.read_differential(
            pair=pair,
            full_scale=float(self.get_parameter('ads_shunt_full_scale_volts').value),
            data_rate=int(self.get_parameter('ads_shunt_data_rate').value))

        shunt_ohms = float(self.get_parameter('ads_shunt_ohms').value)
        if shunt_ohms <= 0.0:
            return volts, None

        sign = float(self.get_parameter('ads_shunt_sign').value)
        amps = (sign * shunt_v / shunt_ohms) - \
            float(self.get_parameter('current_offset_a').value)
        return volts, amps

    def _tick(self):
        try:
            volts, amps = self._read_pack()
        except OSError as exc:
            self.get_logger().warn(f'Pack read failed: {exc}')
            return

        fraction = max(0.0, min(1.0, (volts - self.floor) / (self.full - self.floor)))

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(volts)
        msg.percentage = float(fraction)
        msg.present = True
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO

        if amps is None:
            msg.current = float('nan')       # REP-147: unmeasured, not zero
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        else:
            msg.current = -float(amps)       # negative while discharging
            msg.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_DISCHARGING if amps > 0.05
                else BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING)

        if volts <= self.floor:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
            if not self.warned:
                self.warned = True
                self.get_logger().error(
                    f'Pack at {volts:.2f} V, at or below the {self.floor:.2f} V '
                    f'floor for {self.cells}S. Stop and recharge.')
        else:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
