#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import dynamixel_sdk as dxl
import numpy as np

class DynamixelReaderNode(Node):
    def __init__(self):
        super().__init__('dynamixel_reader_node')

        # Declare parameters with default values
        self.declare_parameter('hand_name', 'dom')
        self.declare_parameter('baudrate', 3000000)
        self.declare_parameter('device_name', '/dev/serial/by-id/usb-FTDI_USB__--if00-port0')
        self.declare_parameter('pub_pos', True)
        self.declare_parameter('pub_vel', False)
        self.declare_parameter('pub_current', False)
        self.declare_parameter('kP', 800)
        self.declare_parameter('kI', 0)
        self.declare_parameter('kD', 200)
        self.declare_parameter('curr_lim', 2000)
        self.declare_parameter('start_pos', [0]*16)  # Initial positions for the motors

        # Get parameters from the parameter server
        self.hand_name = self.get_parameter('hand_name').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.device_name = self.get_parameter('device_name').get_parameter_value().string_value
        self.pub_pos = self.get_parameter('pub_pos').get_parameter_value().bool_value
        self.pub_vel = self.get_parameter('pub_vel').get_parameter_value().bool_value
        self.pub_current = self.get_parameter('pub_current').get_parameter_value().bool_value
        self.kP = self.get_parameter('kP').get_parameter_value().integer_value
        self.kI = self.get_parameter('kI').get_parameter_value().integer_value
        self.kD = self.get_parameter('kD').get_parameter_value().integer_value
        self.curr_lim = self.get_parameter('curr_lim').get_parameter_value().integer_value
        self.start_pos = self.get_parameter('start_pos').get_parameter_value().double_array_value

        self.curr_pos = self.start_pos

        print(self.device_name)

        # Ensure the curr_pos array has the correct length
        if len(self.curr_pos) != 16:
            self.curr_pos = [0] * 16

        # Control table addresses for MX series
        self.ADDR_PRESENT_POSITION = 132
        self.LEN_PRESENT_POSITION = 4
        self.ADDR_PRESENT_VELOCITY = 128
        self.LEN_PRESENT_VELOCITY = 4
        self.ADDR_PRESENT_CURRENT = 126
        self.LEN_PRESENT_CURRENT = 2
        self.ADDR_GOAL_POSITION = 116
        self.LEN_GOAL_POSITION = 4
        self.ADDR_P_GAIN = 84
        self.ADDR_I_GAIN = 82
        self.ADDR_D_GAIN = 80
        self.ADDR_CURRENT_LIMIT = 102
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_OPERATING_MODE = 11

        # Protocol version
        self.PROTOCOL_VERSION = 2.0

        # Initialize PortHandler and PacketHandler instances
        self.port_handler = dxl.PortHandler(self.device_name)
        self.packet_handler = dxl.PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open the port")
            self.destroy_node()
            return

        # Set port baudrate
        if not self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().error("Failed to change the baudrate")
            self.destroy_node()
            return

        self.motors = list(range(16))  # Dynamixel IDs 0-15

        # Initialize GroupSyncRead instances based on parameters
        if self.pub_pos:
            self.group_sync_read_position = dxl.GroupSyncRead(self.port_handler, self.packet_handler, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        if self.pub_vel:
            self.group_sync_read_velocity = dxl.GroupSyncRead(self.port_handler, self.packet_handler, self.ADDR_PRESENT_VELOCITY, self.LEN_PRESENT_VELOCITY)
        if self.pub_current:
            self.group_sync_read_current = dxl.GroupSyncRead(self.port_handler, self.packet_handler, self.ADDR_PRESENT_CURRENT, self.LEN_PRESENT_CURRENT)

        # Initialize GroupSyncWrite instance for position commands
        self.group_sync_write_position = dxl.GroupSyncWrite(self.port_handler, self.packet_handler, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION)

        # Add parameter storage for Dynamixel IDs 0-15
        for dxl_id in range(0, 16):
            if self.pub_pos and not self.group_sync_read_position.addParam(dxl_id):
                self.get_logger().error(f"Failed to add param for Dynamixel ID: {dxl_id} (position)")
                self.destroy_node()
                return
            if self.pub_vel and not self.group_sync_read_velocity.addParam(dxl_id):
                self.get_logger().error(f"Failed to add param for Dynamixel ID: {dxl_id} (velocity)")
                self.destroy_node()
                return
            if self.pub_current and not self.group_sync_read_current.addParam(dxl_id):
                self.get_logger().error(f"Failed to add param for Dynamixel ID: {dxl_id} (current)")
                self.destroy_node()
                return

        # Create publisher
        self.publisher = self.create_publisher(JointState, "/" + self.hand_name + '/dynamixel_joint_states', 10)

        # Create subscriber
        self.subscription = self.create_subscription(
            JointState,
            "/" + self.hand_name + '/command_joint_states',
            self.command_callback,
            10
        )

        # Initialize gains and operating mode
        self.initialize_gains()

        # Create timer to publish data
        timer_period = 1.0 / 60.0
        self.timer = self.create_timer(timer_period, self.read_and_publish_data)

    def __del__(self):
        # Torque off the motors
        try:
            self.sync_write(self.motors, np.zeros(len(self.motors)), self.ADDR_TORQUE_ENABLE, 1)
        except Exception as e:
            self.get_logger().error(f"Error torquing off the motors: {str(e)}")

        # Close port
        if self.port_handler.is_open:
            self.port_handler.closePort()

    def initialize_gains(self):
        try:
            # Enable torque for all motors
            self.sync_write(self.motors, np.ones(len(self.motors)), self.ADDR_TORQUE_ENABLE, 1)

            # Enable position-current control mode and default parameters
            self.sync_write(self.motors, np.ones(len(self.motors)) * 3, self.ADDR_OPERATING_MODE, 1)
            self.sync_write(self.motors, np.ones(len(self.motors)) * self.kP, self.ADDR_P_GAIN, 2)  # P gain stiffness
            self.sync_write([0, 4, 8], np.ones(3) * (self.kP * 0.75), self.ADDR_P_GAIN, 2)  # P gain stiffness for side to side should be a bit less
            self.sync_write(self.motors, np.ones(len(self.motors)) * self.kI, self.ADDR_I_GAIN, 2)  # I gain
            self.sync_write(self.motors, np.ones(len(self.motors)) * self.kD, self.ADDR_D_GAIN, 2)  # D gain damping
            self.sync_write([0, 4, 8], np.ones(3) * (self.kD * 0.75), self.ADDR_D_GAIN, 2)  # D gain damping for side to side should be a bit less
            self.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, self.ADDR_CURRENT_LIMIT, 2)  # Max current limit
            self.sync_write(self.motors, self.degrees_to_ticks_list(self.curr_pos), self.ADDR_GOAL_POSITION, 4)  # Set initial positions

        except Exception as e:
            self.get_logger().error(f"Error initializing gains: {str(e)}")

    def sync_write(self, ids, values, address, length):
        group_sync_write = dxl.GroupSyncWrite(self.port_handler, self.packet_handler, address, length)
        for i, dxl_id in enumerate(ids):
            param = [
                dxl.DXL_LOBYTE(dxl.DXL_LOWORD(int(values[i]))),
                dxl.DXL_HIBYTE(dxl.DXL_LOWORD(int(values[i]))),
                dxl.DXL_LOBYTE(dxl.DXL_HIWORD(int(values[i]))),
                dxl.DXL_HIBYTE(dxl.DXL_HIWORD(int(values[i])))
            ]
            if not group_sync_write.addParam(dxl_id, param[:length]):
                self.get_logger().error(f"Failed to add param for Dynamixel ID: {dxl_id} at address {address}")
        dxl_comm_result = group_sync_write.txPacket()
        if dxl_comm_result != dxl.COMM_SUCCESS:
            self.get_logger().warn(f"GroupSyncWrite txPacket failed at address {address}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
        group_sync_write.clearParam()

    def read_and_publish_data(self):
        try:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()

            if self.pub_pos:
                dxl_comm_result_position = self.group_sync_read_position.txRxPacket()
                if dxl_comm_result_position != dxl.COMM_SUCCESS:
                    self.get_logger().warn(f"GroupSyncRead txRxPacket failed for position: {self.packet_handler.getTxRxResult(dxl_comm_result_position)}")

            if self.pub_vel:
                dxl_comm_result_velocity = self.group_sync_read_velocity.txRxPacket()
                if dxl_comm_result_velocity != dxl.COMM_SUCCESS:
                    self.get_logger().warn(f"GroupSyncRead txRxPacket failed for velocity: {self.packet_handler.getTxRxResult(dxl_comm_result_velocity)}")

            if self.pub_current:
                dxl_comm_result_current = self.group_sync_read_current.txRxPacket()
                if dxl_comm_result_current != dxl.COMM_SUCCESS:
                    self.get_logger().warn(f"GroupSyncRead txRxPacket failed for current: {self.packet_handler.getTxRxResult(dxl_comm_result_current)}")

            for dxl_id in self.motors:
                if self.pub_pos:
                    pos_ticks = self.group_sync_read_position.getData(dxl_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
                    pos_deg = self.ticks_to_degrees(pos_ticks)
                    joint_state_msg.position.append(pos_deg)

                if self.pub_vel:
                    vel_ticks = self.group_sync_read_velocity.getData(dxl_id, self.ADDR_PRESENT_VELOCITY, self.LEN_PRESENT_VELOCITY)
                    vel_deg = self.ticks_to_degrees(vel_ticks)
                    joint_state_msg.velocity.append(vel_deg)

                if self.pub_current:
                    curr_ticks = self.group_sync_read_current.getData(dxl_id, self.ADDR_PRESENT_CURRENT, self.LEN_PRESENT_CURRENT)
                    joint_state_msg.effort.append(curr_ticks)

            self.publisher.publish(joint_state_msg)

        except Exception as e:
            self.get_logger().error(f"Error reading and publishing data: {str(e)}")

    def command_callback(self, msg):
        try:
            for i, pos_deg in enumerate(msg.position):
                pos_ticks = self.degrees_to_ticks(pos_deg)
                param_goal_position = [
                    dxl.DXL_LOBYTE(dxl.DXL_LOWORD(int(pos_ticks))),
                    dxl.DXL_HIBYTE(dxl.DXL_LOWORD(int(pos_ticks))),
                    dxl.DXL_LOBYTE(dxl.DXL_HIWORD(int(pos_ticks))),
                    dxl.DXL_HIBYTE(dxl.DXL_HIWORD(int(pos_ticks)))
                ]
                dxl_id = self.motors[i]
                if not self.group_sync_write_position.addParam(dxl_id, param_goal_position):
                    self.get_logger().error(f"Failed to add param for Dynamixel ID: {dxl_id} (goal position)")

            dxl_comm_result = self.group_sync_write_position.txPacket()
            if dxl_comm_result != dxl.COMM_SUCCESS:
                self.get_logger().warn(f"GroupSyncWrite txPacket failed for goal position: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            self.group_sync_write_position.clearParam()

        except Exception as e:
            self.get_logger().error(f"Error in command callback: {str(e)}")

    def ticks_to_degrees(self, ticks):
        return (ticks - 2048) * 0.088

    def degrees_to_ticks(self, degrees):
        return int(degrees / 0.088) + 2048

    def degrees_to_ticks_list(self, degrees_list):
        return [self.degrees_to_ticks(deg) for deg in degrees_list]

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
