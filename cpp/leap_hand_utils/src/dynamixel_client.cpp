/* Communication using the DynamixelSDK */
//This is based off of the dynamixel SDK
#include <csignal>
#include <iostream>
#include <map>

#include <chrono>
#include <thread>

#include "dynamixel_client.h"


void dynamixel_cleanup_handler(std::set<DynamixelClient*> open_clients) {
    /* Cleanup function to ensure Dynamixels are disconnected properly */
    for (DynamixelClient* open_client : open_clients) {
        if (open_client->get_port_handler()->is_using_) {
            std::clog << "Forcing client to close.\n";
        }
        open_client->get_port_handler()->is_using_ = false;
        open_client->disconnect();
    }
}

int signed_to_unsigned(int value, int size) {
    /* Converts the given value to its unsigned representation */
    if (value < 0) {
        int bit_size = 8 * size;
        int max_value = (1 << bit_size) - 1;
        value += max_value;
    }
    return value;
}

int unsigned_to_signed(int value, int size) {
    /* Converts the given value from its unsigned representation */
    int bit_size = 8 * size;
    if ((value & (1 << (bit_size - 1))) != 0) {
        value = -((1 << bit_size) - value);
    }
    return value;
}


/**********************************************************************************
 * DynamixelClient Definitions                                                    *
 **********************************************************************************/

DynamixelClient::DynamixelClient(std::vector<int> motor_ids,
                                 std::string port,
                                 int baudrate,
                                 bool lazy_connect,
                                 float pos_scale,
                                 float vel_scale,
                                 float cur_scale)
    : motor_ids {motor_ids}
    , port_name {port.c_str()}
    , baudrate {baudrate}
    , lazy_connect {lazy_connect}

    , port_handler {dynamixel::PortHandler::getPortHandler(port_name)}
    , packet_handler {dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION)}

    , _pos_vel_cur_reader {}
    , _pos_vel_reader {}
    , _pos_reader {}
    , _vel_reader {}
    , _cur_reader {}
        
    , _sync_writers {}
    , OPEN_CLIENTS {}
{
    /* Initializes a new client.

    Args:
        motor_ids: All motor IDs being used by the client.
        port: The Dynamixel device to talk to. e.g.
            - Linux: /dev/ttyUSB0
            - Mac: /dev/tty.usbserial-*
            - Windows: COM1
        baudrate: The Dynamixel baudrate to communicate with.
        lazy_connect: If True, automatically connects when calling a method
            that requires a connection, if not already connected.
        pos_scale: The scaling factor for the positions.This is
            motor-dependent. If not provided, uses the default scale.
        vel_scale: The scaling factor for the velocities.This is
            motor-dependent. If not provided uses the default scale.
        cur_scale: The scaling factor for the currents.This is
            motor-dependent. If not provided uses the default scale.
    */

    _pos_vel_cur_reader = new DynamixelPosVelCurReader(
        this,
        motor_ids,
        pos_scale,
        vel_scale,
        cur_scale
    );
    _pos_vel_reader = new DynamixelPosVelReader(
        this,
        motor_ids,
        pos_scale,
        vel_scale
    );
    _pos_reader = new DynamixelPosReader(
        this,
        motor_ids,
        pos_scale,
        vel_scale,
        cur_scale
    );
    _vel_reader = new DynamixelVelReader(
        this,
        motor_ids,
        pos_scale,
        vel_scale,
        cur_scale
    );
    _cur_reader = new DynamixelCurReader(
        this,
        motor_ids,
        pos_scale,
        vel_scale,
        cur_scale
    );

    OPEN_CLIENTS.insert(this);
}

DynamixelClient::~DynamixelClient() {
    /* Automatically disconnect on destruction */
    dynamixel_cleanup_handler(OPEN_CLIENTS);
}

dynamixel::PortHandler* DynamixelClient::get_port_handler() {
    return port_handler;
}

dynamixel::PacketHandler* DynamixelClient::get_packet_handler() {
    return packet_handler;
}

bool DynamixelClient::is_connected() {
    return port_handler->is_open_;
}

void DynamixelClient::connect() {
    /* Connects to the Dynamixel motors.

    NOTE: This should be called after all DynamixelClients on the same
        process are created.
    */
    assert(!is_connected() && "Client is already connected.");

    if (port_handler->openPort()) {
        std::clog << "Succeeded to open port: " << port_name << '\n';
    } else {
        std::cerr << "Failed to open port at " << port_name
            << " (Check that the device is powered on and connected to your computer).\n";
        std::raise(SIGTERM);
    }

    if (port_handler->setBaudRate(baudrate)) {
        std::clog << "Succeeded to set baudrate to " << baudrate << '\n';
    } else {
        std::cerr << "Failed to set the baudrate to " << baudrate
            << " (Ensure that the device was configured for this baudrate).\n";
        std::raise(SIGTERM);
    }
}

void DynamixelClient::disconnect() {
    /* Disconnects from the Dynamixel device */
    if (!is_connected()) {
        return;
    }
    if (port_handler->is_using_) {
        std::clog << "Port handler in use; cannot disconnect.\n";
        return;
    }
    // Ensure motors are disabled at the end.
    set_torque_enabled(motor_ids, false, 0);
    port_handler->closePort();
    if (OPEN_CLIENTS.count(this) == 1) {
        OPEN_CLIENTS.erase(this);
    }
}

void DynamixelClient::set_torque_enabled(std::vector<int> motor_ids,
                                         bool enabled,
                                         int retries,
                                         float retry_interval) {
    /* Sets whether torque is enabled for the motors.

    Args:
        motor_ids: The motor IDs to configure.
        enabled: Whether to engage or disengage the motors.
        retries: The number of times to retry. If this is <0, will retry
            forever.
        retry_interval: The number of seconds to wait between retries.
    */
    std::vector<int> remaining_ids {motor_ids};
    while (!remaining_ids.empty()) {
        remaining_ids = write_byte(
            remaining_ids,
            int(enabled),
            ADDR_TORQUE_ENABLE
        );
        if (!remaining_ids.empty()) {
            std::string remaining_ids_str {};
            for (int id : remaining_ids) remaining_ids_str += std::to_string(id);
            std::clog << "Could not set torque "
                      << (enabled ? "enabled" : "disabled")
                      << " for IDs: "
                      << remaining_ids_str;
        }
        if (retries == 0) {
            break;
        }
        using namespace std::chrono_literals;
        int retry_interval_ms {retry_interval*1000};
        std::this_thread::sleep_for(std::chrono::duration<long double, std::milli>(retry_interval_ms));
        retries -= 1;
    }
}

std::vector<Eigen::MatrixXd> DynamixelClient::read_pos_vel_cur() {
    /* Returns the current positions and velocities */
    return _pos_vel_cur_reader->read();
}

std::vector<Eigen::MatrixXd> DynamixelClient::read_pos_vel() {
    /* Returns the current positions and velocities */
    return _pos_vel_reader->read();
}

Eigen::MatrixXd DynamixelClient::read_pos() {
    /* Returns the current positions and velocities */
    return _pos_reader->read();
}

Eigen::MatrixXd DynamixelClient::read_vel() {
    /* Returns the current positions and velocities */
    return _vel_reader->read();
}

Eigen::MatrixXd DynamixelClient::read_cur() {
    /* Returns the current positions and velocities */
    return _cur_reader->read();
}


void DynamixelClient::write_desired_pos(std::vector<int> motor_ids, Eigen::MatrixXd positions) {
    /* Writes the given desired positions.

    Args:
        motor_ids: The motor IDs to write to.
        positions: The joint angles in radians to write.
    */
    assert(motor_ids.size() == positions.size());

    // Convert to Dynamixel position space.
    positions /= _pos_reader -> _get_scale();
    sync_write(motor_ids, positions, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
}

std::vector<int> DynamixelClient::write_byte(std::vector<int> motor_ids, int value, int address) {
    /* Writes a value to the motors.

    Args:
        motor_ids: The motor IDs to write to.
        value: The value to write to the control table.
        address: The control table address to write to.

    Returns:
        A list of IDs that were unsuccessful.
    */
    check_connected();
    std::vector<int> errored_ids{};
    for (int motor_id : motor_ids) {
        int dxl_comm_result {COMM_TX_FAIL};
        int dxl_error {0};
        dxl_comm_result = packet_handler->write1ByteTxRx(port_handler, motor_id, address, value);
        bool success = handle_packet_result(dxl_comm_result, dxl_error, motor_id, "write_byte");
        if (!success) {
            errored_ids.push_back(motor_id);
        }
    }
    return errored_ids;
}

void DynamixelClient::sync_write(std::vector<int> motor_ids, Eigen::MatrixXd values,
                                 int address, int size) {
    /* Writes values to a group of motors.

    Args:
        motor_ids: The motor IDs to write to.
        values: The values to write.
        address: The control table address to write to.
        size: The size of the control table value being written to.
    */
    check_connected();
    std::vector<int> key = {address, size};
    if (_sync_writers.count(key) == 0) {
        _sync_writers.insert({key, dynamixel::GroupSyncWrite(port_handler, packet_handler, address, size)});
    }
    dynamixel::GroupSyncWrite sync_writer = _sync_writers.find(key) -> second;

    std::vector<int> errored_ids = {};
    std::vector<std::vector<int>> vec_zip {};
    for (int i {0}; i < motor_ids.size(); i++) {
        vec_zip.push_back({motor_ids[i], (int)values(i)});
    }

    for (std::vector<int> i : vec_zip) {
        uint8_t value[4];
        value[0] = DXL_LOBYTE(DXL_LOWORD(i[1]));
        value[1] = DXL_HIBYTE(DXL_LOWORD(i[1]));
        value[2] = DXL_LOBYTE(DXL_HIWORD(i[1]));
        value[3] = DXL_HIBYTE(DXL_HIWORD(i[1]));

        bool success = sync_writer.addParam(i[0], value);
        if (success != true) {
            errored_ids.push_back(i[0]);
        }
    }

    if (!errored_ids.empty()) {
        std::string error_msg {"Sync write failed for: "};
        for (int id : errored_ids) error_msg += std::to_string(id);
        error_msg += '\n';
        std::clog << error_msg;
    }

    int comm_result {sync_writer.txPacket()};
    handle_packet_result(comm_result, 0, 0, "sync_write");

    sync_writer.clearParam();
}

void DynamixelClient::check_connected() {
    /* Ensures the robot is connected */
    if (lazy_connect && !is_connected()) {
        connect();
    }
    if (!is_connected()) {
        std::cerr << "Must call connect() first.";
        std::raise(SIGTERM);
    }
}

bool DynamixelClient::handle_packet_result(int comm_result,
                                           int dxl_error,
                                           int dxl_id,
                                           std::string context) {
    /* Handles the result from a communication request */
    std::string error_message = "null";
    if (comm_result != COMM_SUCCESS) {
        error_message = packet_handler->getTxRxResult(comm_result);
    } else if (dxl_error) {
        error_message = packet_handler->getRxPacketError(dxl_error);
    }

    if (error_message != "null") {
        if (dxl_id) {
            error_message = "[Motor ID: " + std::to_string(dxl_id) + "] " + error_message;
        }
        if (context != "null") {
            error_message = "> " + context + ": " + error_message;
        }
        std::clog << error_message;
        return false;
    }
    return true;
}

int DynamixelClient::convert_to_unsigned(int value, int size) {
    /* Converts the given value to its unsigned representation */
    if (value < 0) {
        int max_value = (1 << (8 * size)) - 1;
        value += max_value;
    }
    return value;
}

/**********************************************************************************
 * DynamixelReader Definitions                                                    *
 **********************************************************************************/

DynamixelReader::DynamixelReader(DynamixelClient *client,
                                 std::vector<int> motor_ids,
                                 int address,
                                 int size)
    : client {client}
    , motor_ids {motor_ids}
    , address {address}
    , size {size}
    , _data {}
    , operation {dynamixel::GroupSyncRead(
        client->get_port_handler(),
        client->get_packet_handler(),
        address,
        size)}
{
    for (int motor_id : motor_ids) {
        bool success = operation.addParam(motor_id);
        if (!success) {
            std::cerr << "[Motor ID: " << motor_id << "] Could not add parameter to bulk read.\n";
            std::raise(SIGTERM);
        }
    }
}

DynamixelClient* DynamixelReader::get_client() {
    return client;
}

std::vector<int> DynamixelReader::get_motor_ids() {
    return motor_ids;
}

int DynamixelReader::get_address() {
    return address;
}

int DynamixelReader::get_size() {
    return size;
}

Eigen::MatrixXd DynamixelReader::read(int retries) {
    /* Reads data from the motors */
    client->check_connected();
    bool success{false};
    while (!success && retries >= 0) {
        bool comm_result = operation.txRxPacket();
        bool success = client->handle_packet_result(comm_result, 0, 0, "read");
        retries -= 1;
    }

    // If we failed, send a copy of the previous data.
    if (!success) {
        return _get_data();
    }

    std::vector<int> errored_ids{};
    int i{0};
    for (int motor_id : motor_ids) {
        // Check if the data is available.
        bool available = operation.isAvailable(motor_id, address, size);
        if (!available) {
            errored_ids.push_back(motor_id);
            continue;
        }
        _update_data(i++, motor_id);
    }

    if (!errored_ids.empty()) {
        std::string error_msg {"Bulk read data is unavailable for: "};
        for (int id : errored_ids) error_msg += std::to_string(id);
        error_msg += '\n';
        std::clog << error_msg;
    }

    return _get_data();
}

void DynamixelReader::_initialize_data() {
    /* Initializes the cached data */
    _data = Eigen::MatrixXd::Zero(1, motor_ids.size());
}

void DynamixelReader::_update_data(int index, int motor_id) {
    /* Updates the data index for the given motor ID */
    _data(index) = operation.getData(motor_id, address, size);
}

Eigen::MatrixXd DynamixelReader::_get_data() {
    /* Returns a copy of the data */
    return _data;
}


/**********************************************************************************
 * DynamixelPosVelCurReader Definitions                                           *
 **********************************************************************************/

DynamixelPosVelCurReader::DynamixelPosVelCurReader(DynamixelClient *client,
                                                   std::vector<int> motor_ids,
                                                   float pos_scale,
                                                   float vel_scale,
                                                   float cur_scale)
    : DynamixelReader {client,
                       motor_ids,
                       ADDR_PRESENT_POS_VEL_CUR,
                       LEN_PRESENT_POS_VEL_CUR}
    , pos_scale {pos_scale}
    , vel_scale {vel_scale}
    , cur_scale {cur_scale}
{
}

std::vector<float> DynamixelPosVelCurReader::_get_scale() {
    return {pos_scale, vel_scale, cur_scale};
}

std::vector<Eigen::MatrixXd> DynamixelPosVelCurReader::read(int retries) {
    /* Reads data from the motors */
    get_client()->check_connected();
    bool success{false};
    while (!success && retries >= 0) {
        bool comm_result {operation.txRxPacket()};
        success = get_client()->handle_packet_result(comm_result, 0, 0, "read");
        retries--;
    }

    // If we failed, send a copy of the previous data.
    if (!success) {
        return _get_data();
    }

    std::vector<int> errored_ids{};
    int i{0};
    for (int motor_id : get_motor_ids()) {
        // Check if the data is available.
        bool available = operation.isAvailable(motor_id, get_address(), get_size());
        if (!available) {
            errored_ids.push_back(motor_id);
            continue;
        }
        _update_data(i++, motor_id);
    }

    if (!errored_ids.empty()) {
        std::string error_msg {"Bulk read data is unavailable for: "};
        for (int id : errored_ids) error_msg += std::to_string(id);
        error_msg += '\n';
        std::clog << error_msg;
    }

    return _get_data();
}

void DynamixelPosVelCurReader::_initialize_data() {
    /* Initializes the cached data */
    _pos_data = Eigen::MatrixXd::Zero(1, get_motor_ids().size());
    _vel_data = Eigen::MatrixXd::Zero(1, get_motor_ids().size());
    _cur_data = Eigen::MatrixXd::Zero(1, get_motor_ids().size());
}

void DynamixelPosVelCurReader::_update_data(int index, int motor_id) {
    /* Updates the data index for the given motor ID */
    uint32_t cur {operation.getData(motor_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT)};
    uint32_t vel {operation.getData(motor_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)};
    uint32_t pos {operation.getData(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)};
    cur = unsigned_to_signed(cur, 2);
    vel = unsigned_to_signed(vel, 4);
    pos = unsigned_to_signed(pos, 4);
    _pos_data(index) = float(pos) * pos_scale;
    _vel_data(index) = float(vel) * vel_scale;
    _cur_data(index) = float(cur) * cur_scale;
}

std::vector<Eigen::MatrixXd> DynamixelPosVelCurReader::_get_data() {
    /* Returns a copy of the data */
    return {_pos_data, _vel_data, _cur_data};
}


/**********************************************************************************
 * DynamixelPosVelReader Definitions                                              *
 **********************************************************************************/

DynamixelPosVelReader::DynamixelPosVelReader(DynamixelClient *client,
                                             std::vector<int> motor_ids,
                                             float pos_scale,
                                             float vel_scale)
    : DynamixelReader {client,
                       motor_ids,
                       ADDR_PRESENT_POS_VEL,
                       LEN_PRESENT_POS_VEL}
    , pos_scale {pos_scale}
    , vel_scale {vel_scale}
{
    _initialize_data();
}

std::vector<float> DynamixelPosVelReader::_get_scale() {
    return {pos_scale, vel_scale};
}

std::vector<Eigen::MatrixXd> DynamixelPosVelReader::read(int retries) {
    /* Reads data from the motors */
    get_client()->check_connected();
    bool success{false};
    while (!success && retries >= 0) {
        bool comm_result {operation.txRxPacket()};
        success = get_client()->handle_packet_result(comm_result, 0, 0, "read");
        retries--;
    }

    // If we failed, send a copy of the previous data.
    if (!success) {
        return _get_data();
    }

    std::vector<int> errored_ids{};
    int i{0};
    for (int motor_id : get_motor_ids()) {
        // Check if the data is available.
        bool available = operation.isAvailable(motor_id, get_address(), get_size());
        if (!available) {
            errored_ids.push_back(motor_id);
            continue;
        }
        _update_data(i++, motor_id);
    }

    if (!errored_ids.empty()) {
        std::string error_msg {"Bulk read data is unavailable for: "};
        for (int id : errored_ids) error_msg += std::to_string(id);
        error_msg += '\n';
        std::clog << error_msg;
    }

    return _get_data();
}

void DynamixelPosVelReader::_initialize_data() {
    /* Initializes the cached data */
    _pos_data = Eigen::MatrixXd::Zero(1, get_motor_ids().size());
    _vel_data = Eigen::MatrixXd::Zero(1, get_motor_ids().size());
}

void DynamixelPosVelReader::_update_data(int index, int motor_id) {
    /* Updates the data index for the given motor ID */
    uint32_t vel {operation.getData(motor_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)};
    uint32_t pos {operation.getData(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)};
    vel = unsigned_to_signed(vel, 4);
    pos = unsigned_to_signed(pos, 4);
    _pos_data(index) = float(pos) * pos_scale;
    _vel_data(index) = float(vel) * vel_scale;
}

std::vector<Eigen::MatrixXd> DynamixelPosVelReader::_get_data() {
    /* Returns a copy of the data */
    return {_pos_data, _vel_data};
}


/**********************************************************************************
 * DynamixelPosReader Definitions                                                 *
 **********************************************************************************/

DynamixelPosReader::DynamixelPosReader(DynamixelClient *client,
                                       std::vector<int> motor_ids,
                                       float pos_scale,
                                       float vel_scale,
                                       float cur_scale)
    : DynamixelReader{client,
                      motor_ids,
                      ADDR_PRESENT_POSITION,
                      LEN_PRESENT_POSITION}
    , pos_scale {pos_scale}
{
    _initialize_data();
}

float DynamixelPosReader::_get_scale() {
    return pos_scale;
}

Eigen::MatrixXd DynamixelPosReader::read(int retries) {
    /* Reads data from the motors */
    get_client()->check_connected();
    bool success{false};
    while (!success && retries >= 0) {
        bool comm_result {operation.txRxPacket()};
        success = get_client()->handle_packet_result(comm_result, 0, 0, "read");
        retries--;
    }

    // If we failed, send a copy of the previous data.
    if (!success) {
        return _get_data();
    }

    std::vector<int> errored_ids{};
    int i{0};
    for (int motor_id : get_motor_ids()) {
        // Check if the data is available.
        bool available = operation.isAvailable(motor_id, get_address(), get_size());
        if (!available) {
            errored_ids.push_back(motor_id);
            continue;
        }
        _update_data(i++, motor_id);
    }

    if (!errored_ids.empty()) {
        std::string error_msg {"Bulk read data is unavailable for: "};
        for (int id : errored_ids) error_msg += std::to_string(id);
        error_msg += '\n';
        std::clog << error_msg;
    }

    return _get_data();
}

void DynamixelPosReader::_initialize_data() {
    /* Initializes the cached data */
    _pos_data = Eigen::MatrixXd::Zero(1, get_motor_ids().size());
}

void DynamixelPosReader::_update_data(int index, int motor_id) {
    /* Updates the data index for the given motor ID */
    uint32_t pos {operation.getData(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)};
    pos = unsigned_to_signed(pos, 4);
    _pos_data(index) = float(pos) * pos_scale;
}

Eigen::MatrixXd DynamixelPosReader::_get_data() {
    /* Returns a copy of the data */
    return _pos_data;
}


/**********************************************************************************
 * DynamixelVelReader Definitions                                                 *
 **********************************************************************************/

DynamixelVelReader::DynamixelVelReader(DynamixelClient *client,
                                       std::vector<int> motor_ids,
                                       float pos_scale,
                                       float vel_scale,
                                       float cur_scale)
    : DynamixelReader {client,
                       motor_ids,
                       ADDR_PRESENT_VELOCITY,
                       LEN_PRESENT_VELOCITY}
    , vel_scale {vel_scale}
{
    _initialize_data();
}

float DynamixelVelReader::_get_scale() {
    return vel_scale;
}

Eigen::MatrixXd DynamixelVelReader::read(int retries) {
    /* Reads data from the motors */
    get_client()->check_connected();
    bool success{false};
    while (!success && retries >= 0) {
        bool comm_result {operation.txRxPacket()};
        success = get_client()->handle_packet_result(comm_result, 0, 0, "read");
        retries--;
    }

    // If we failed, send a copy of the previous data.
    if (!success) {
        return _get_data();
    }

    std::vector<int> errored_ids{};
    int i{0};
    for (int motor_id : get_motor_ids()) {
        // Check if the data is available.
        bool available = operation.isAvailable(motor_id, get_address(), get_size());
        if (!available) {
            errored_ids.push_back(motor_id);
            continue;
        }
        _update_data(i++, motor_id);
    }

    if (!errored_ids.empty()) {
        std::string error_msg {"Bulk read data is unavailable for: "};
        for (int id : errored_ids) error_msg += std::to_string(id);
        error_msg += '\n';
        std::clog << error_msg;
    }

    return _get_data();
}

void DynamixelVelReader::_initialize_data() {
    /* Initializes the cached data */
    _vel_data = Eigen::MatrixXd::Zero(1, get_motor_ids().size());
}

void DynamixelVelReader::_update_data(int index, int motor_id) {
    /* Updates the data index for the given motor ID */
    uint32_t vel {operation.getData(motor_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)};
    vel = unsigned_to_signed(vel, 4);
    _vel_data(index) = float(vel) * vel_scale;
}

Eigen::MatrixXd DynamixelVelReader::_get_data() {
    /* Returns a copy of the data */
    return _vel_data;
}


/**********************************************************************************
 * DynamixelCurReader Definitions                                                 *
 **********************************************************************************/

DynamixelCurReader::DynamixelCurReader(DynamixelClient *client,
                                       std::vector<int> motor_ids,
                                       float pos_scale,
                                       float vel_scale,
                                       float cur_scale)
    : DynamixelReader {client,
                       motor_ids,
                       ADDR_PRESENT_CURRENT,
                       LEN_PRESENT_CURRENT}
    , cur_scale {cur_scale}
{
    _initialize_data();
}

float DynamixelCurReader::_get_scale() {
    return cur_scale;
}

Eigen::MatrixXd DynamixelCurReader::read(int retries) {
    /* Reads data from the motors */
    get_client()->check_connected();
    bool success{false};
    while (!success && retries >= 0) {
        bool comm_result {operation.txRxPacket()};
        success = get_client()->handle_packet_result(comm_result, 0, 0, "read");
        retries--;
    }

    // If we failed, send a copy of the previous data.
    if (!success) {
        return _get_data();
    }

    std::vector<int> errored_ids{};
    int i{0};
    for (int motor_id : get_motor_ids()) {
        // Check if the data is available.
        bool available = operation.isAvailable(motor_id, get_address(), get_size());
        if (!available) {
            errored_ids.push_back(motor_id);
            continue;
        }
        _update_data(i++, motor_id);
    }

    if (!errored_ids.empty()) {
        std::string error_msg {"Bulk read data is unavailable for: "};
        for (int id : errored_ids) error_msg += std::to_string(id);
        error_msg += '\n';
        std::clog << error_msg;
    }

    return _get_data();
}

void DynamixelCurReader::_initialize_data() {
    /* Initializes the cached data */
    _cur_data = Eigen::MatrixXd::Zero(1, get_motor_ids().size());
}

void DynamixelCurReader::_update_data(int index, int motor_id) {
    /* Updates the data index for the given motor ID */
    uint32_t cur {operation.getData(motor_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT)};
    cur = unsigned_to_signed(cur, 2);
    _cur_data(index) = float(cur) * cur_scale;
}

Eigen::MatrixXd DynamixelCurReader::_get_data() {
    /* Returns a copy of the data */
    return _cur_data;
}
