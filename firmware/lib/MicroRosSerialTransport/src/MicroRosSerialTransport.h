// References (full entries: docs/references.md at the repository root):
//   [R14] micro-ROS / micro_ros_arduino: transport
// More details: MicroRosSerialTransport.cpp

#ifndef L2R_MICRO_ROS_SERIAL_TRANSPORT_H
#define L2R_MICRO_ROS_SERIAL_TRANSPORT_H

namespace l2r {

// Configure the Arduino micro-ROS serial transport at the project baud rate.
void beginMicroRosSerialTransport();

}  // namespace l2r

#endif  // L2R_MICRO_ROS_SERIAL_TRANSPORT_H
