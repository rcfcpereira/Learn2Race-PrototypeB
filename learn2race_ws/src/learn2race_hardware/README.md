# learn2race_hardware

Raspberry Pi drivers and ROS 2 nodes for the sensors and lights that are not on the
ESP32. Drivers (plain Python, I2C/SPI/GPIO) are separate from the ROS adapter nodes, so
they can be tested on their own.

## Nodes

| Executable | Module | Hardware | References |
|---|---|---|---|
| `ltr303_node` | `ltr303_node.py` | LTR-303 ambient light sensor (start light) | R67 |
| `rgb_stick_node` | `grove_rgb_stick_node.py`, `grove_ws2813_rgb_stick.py` | Grove RGB Stick, 10 × WS2813: status lights (`mode:=status`); on 2026-10-01 the stick moved to Prototype A as its track light | R74, R75, R65 |
| `link_led_node` | `micro_ros_link_led_node.py` | Shows the ESP32 micro-ROS link on the stick | R14, R75 |
| `i2c_display_node` | `i2c_display_node.py`, `ssd1306_i2c.py`, `oled_font.py` | 0.91" SSD1306 OLED status display | R78 |
| `line_array_node` | `line_array_node.py`, `ads7830.py` | QYF-750 reflective line array through an ADS7830 | R80, R70 |
| `vl6180_tof_node` | `pca9548_vl6180_tof_node.py`, `vl6180x.py`, `pca9548.py` | Three VL6180X ToF sensors behind a PCA9548 multiplexer | R72, R77 |
| `tof050c_node` | `tof050c_node.py` | TOF050C ToF module | R72 |
| `ultrasonic_node` | `ultrasonic_node.py` | Grove Ultrasonic Ranger (single-pin trigger/echo) | R74 |
| `sharp_ir_node` | `sharp_ir_node.py` | Sharp GP2Y0A21 IR distance through a PCF8591 | R73, R70 |
| `battery_monitor` | `battery_monitor_node.py`, `ina226.py`, `ads1115.py` | Pack voltage/current (Prototype B) | R69, R70 |
| `lsm6ds3_imu_node` | `grove_lsm6ds3_imu_node.py`, `lsm6ds3.py` | Grove 6-axis IMU | R71, R74, R12 |

## RGB stick (WS2813 over SPI)

`grove_ws2813_rgb_stick.py` sends the WS2813 bit stream through `/dev/spidev0.0` at
2.5 MHz (each LED bit = 3 SPI bits). On the Raspberry Pi 5 the frame goes in ≤ 63-byte
transfers (longer ones time out through the RP1's DMA).

## References and more details

- [R60]–[R80] datasheets and module documentation (see the table); [R12] REP 103 axes
  and units; [R14] micro-ROS.
- Full entries: [`docs/references.md`](../../../docs/references.md),
  sections 2.1, 3.1 and 7.
