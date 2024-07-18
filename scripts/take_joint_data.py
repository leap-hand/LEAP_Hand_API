import sys
import dynamixel_sdk as dxl

# Set your Dynamixel motor IDs and port name
DXL_IDS = list(range(16))  # IDs 0 to 15
SERIAL_PORT = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NQ6SW-if00-port0'  # Adjust this based on your system
BAUDRATE = 3000000  # Baud rate of Dynamixel

# Initialize PortHandler instance
port_handler = dxl.PortHandler(SERIAL_PORT)

# Initialize PacketHandler instance with protocol version 2
packet_handler = dxl.PacketHandler(2)

# Control table addresses for Dynamixel XM430-W350-R
ADDR_OPERATING_MODE = 11      # Address for operating mode (Torque Control Mode: 4, Position Control Mode: 3)
ADDR_GOAL_CURRENT = 102       # Address for goal current in current control mode
ADDR_GOAL_POSITION = 116      # Address for goal position in position control mode
ADDR_CURRENT_LIMIT = 38       # Address for current limit

# Constants for conversion and configuration
def convert_ticks_to_degrees(ticks):
    # Convert using the specified scale: (ticks - 2048) * 0.088
    return (ticks - 2048) * 0.088

def configure_motor_mode(dxl_id):
    # Set operating mode to current control position mode (Mode 5)
    dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_OPERATING_MODE, 5)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print(f"Error setting operating mode for motor {dxl_id}: {dxl_comm_result}, {dxl_error}")
    else:
        print(f"Operating mode set to current control position for motor {dxl_id}")

    # Set current limit to 1000 (example value, adjust as needed)
    dxl_comm_result, dxl_error = packet_handler.write2ByteTxRx(port_handler, dxl_id, ADDR_CURRENT_LIMIT, 1000)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print(f"Error setting current limit for motor {dxl_id}: {dxl_comm_result}, {dxl_error}")
    else:
        print(f"Current limit set to 1000 for motor {dxl_id}")

def read_motor_position(dxl_id):
    # Read present position
    dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, dxl_id, ADDR_GOAL_POSITION)
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print(f"Error reading motor {dxl_id} position: {dxl_comm_result}, {dxl_error}")
        return None
    else:
        # Convert position from ticks to degrees using the custom scale
        degrees = convert_ticks_to_degrees(dxl_present_position)
        return degrees

def torque_off():
    for dxl_id in DXL_IDS:
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, 64, 0)  # Address 64 is Torque Enable
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print(f"Error disabling torque for motor {dxl_id}: {dxl_comm_result}, {dxl_error}")
        else:
            print(f"Torque disabled for motor {dxl_id}")

def main():
    # Open port
    if port_handler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        sys.exit(1)

    # Set baudrate
    if port_handler.setBaudRate(BAUDRATE):
        print(f"Succeeded to set the baudrate: {BAUDRATE}")
    else:
        print(f"Failed to set the baudrate: {BAUDRATE}")
        sys.exit(1)

    try:
        for dxl_id in DXL_IDS:
            configure_motor_mode(dxl_id)

        while True:
            input("Press Enter to read motor positions...")
            positions_degrees = []
            for dxl_id in DXL_IDS:
                pos_deg = read_motor_position(dxl_id)
                if pos_deg is not None:
                    positions_degrees.append(pos_deg)
            print("Current positions in degrees:", positions_degrees)
    except KeyboardInterrupt:
        print("\nExiting...")
        torque_off()  # Disable torque before exiting
        port_handler.closePort()
        sys.exit()

if __name__ == "__main__":
    main()
