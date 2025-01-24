/* Communication using the DynamixelSDK */
// This is based off of the dynamixel SDK
#ifndef DXL_CLIENT_H
#define DXL_CLIENT_H

#include <set>
#include <vector>

#include "dynamixel_sdk.h"
#include <Eigen>


#define PROTOCOL_VERSION 2.0

// The following addresses assume XC motors.
#define ADDR_TORQUE_ENABLE       64
#define ADDR_GOAL_POSITION       116
#define ADDR_PRESENT_POSITION    132
#define ADDR_PRESENT_VELOCITY    128
#define ADDR_PRESENT_CURRENT     126
#define ADDR_PRESENT_POS_VEL_CUR 126
#define ADDR_PRESENT_POS_VEL     128

// Data Byte Length
#define LEN_PRESENT_POSITION    4
#define LEN_PRESENT_VELOCITY    4
#define LEN_PRESENT_CURRENT     2
#define LEN_PRESENT_POS_VEL_CUR 10
#define LEN_PRESENT_POS_VEL     8 
#define LEN_GOAL_POSITION       4

#define M_PI 3.14159265358979323846

#define DEFAULT_POS_SCALE (2.0 * M_PI / 4096)         // 0.088 degrees
#define DEFAULT_VEL_SCALE (0.229 * 2.0 * M_PI / 60.0) // 0.229 rpm
#define DEFAULT_CUR_SCALE 1.0


class DynamixelClient;
void dynamixel_cleanup_handler(std::set<DynamixelClient> open_clients);
int signed_to_unsigned(int value, int size);
int unsigned_to_signed(int value, int size);


class DynamixelPosVelCurReader;
class DynamixelPosVelReader;
class DynamixelPosReader;
class DynamixelVelReader;
class DynamixelCurReader;
class DynamixelClient
{
    /* Client for communicating with Dynamixel motors.

    NOTE: This only supports Protocol 2.
    */
private:
    std::vector<int> motor_ids;
    const char *port_name;
    int baudrate;
    bool lazy_connect;

    dynamixel::PortHandler *port_handler;
    dynamixel::PacketHandler *packet_handler;

    DynamixelPosVelCurReader *_pos_vel_cur_reader;
    DynamixelPosVelReader *_pos_vel_reader;
    DynamixelPosReader *_pos_reader;
    DynamixelVelReader *_vel_reader;
    DynamixelCurReader *_cur_reader;

    std::map<std::vector<int>, dynamixel::GroupSyncWrite> _sync_writers;
    std::set<DynamixelClient*> OPEN_CLIENTS;

public:
    DynamixelClient(std::vector<int> motor_ids,
                    std::string port = "/dev/ttyUSB0",
                    int baudrate = 1000000,
                    bool lazy_connect = false,
                    float pos_scale = DEFAULT_POS_SCALE,
                    float vel_scale = DEFAULT_VEL_SCALE,
                    float cur_scale = DEFAULT_CUR_SCALE);

    ~DynamixelClient();

    dynamixel::PortHandler* get_port_handler();
    dynamixel::PacketHandler* get_packet_handler();

    bool is_connected();
    void connect();
    void disconnect();
    void set_torque_enabled(std::vector<int> motor_ids,
                            bool enabled,
                            int retries = -1,
                            float retry_interval = 0.25);

    std::vector<Eigen::MatrixXd> read_pos_vel_cur();
    std::vector<Eigen::MatrixXd> read_pos_vel();
    Eigen::MatrixXd read_pos();
    Eigen::MatrixXd read_vel();
    Eigen::MatrixXd read_cur();

    void write_desired_pos(std::vector<int> motor_ids, Eigen::MatrixXd positions);
    std::vector<int> write_byte(std::vector<int> motor_ids, int value, int address);
    void sync_write(std::vector<int> motor_ids, Eigen::MatrixXd values, int address, int size);
    void check_connected();
    bool handle_packet_result(int comm_result,
                              int dxl_error = 0,
                              int dxl_id = 0,
                              std::string context = "null");
    int convert_to_unsigned(int value, int size);
};

class DynamixelReader
{
private:
    DynamixelClient *client;
    std::vector<int> motor_ids;
    int address;
    int size;
    Eigen::MatrixXd _data;

public:
    /* Reads data from Dynamixel motors.

    This wraps a GroupBulkRead from the DynamixelSDK.
    */
    DynamixelReader(
        DynamixelClient *client,
        std::vector<int> motor_ids,
        int address,
        int size
    );

    dynamixel::GroupSyncRead operation;

    DynamixelClient* get_client();
    std::vector<int> get_motor_ids();
    int get_address();
    int get_size();

    Eigen::MatrixXd read(int retries = 1);
    void _initialize_data();
    void _update_data(int index, int motor_id);
    Eigen::MatrixXd _get_data();
};

class DynamixelPosVelCurReader: public DynamixelReader
{
    /* Reads positions, currents and velocities */
private:
    float pos_scale;
    float vel_scale;
    float cur_scale;

    Eigen::MatrixXd _pos_data;
    Eigen::MatrixXd _vel_data;
    Eigen::MatrixXd _cur_data;

public:
    DynamixelPosVelCurReader(
        DynamixelClient *client,
        std::vector<int> motor_ids,
        float pos_scale = 1.0,
        float vel_scale = 1.0,
        float cur_scale = 1.0
    );

    std::vector<float> _get_scale();

    std::vector<Eigen::MatrixXd> read(int retries = 1);
    void _initialize_data();
    void _update_data(int index, int motor_id);
    std::vector<Eigen::MatrixXd> _get_data();
};

class DynamixelPosVelReader : public DynamixelReader
{
    /* Reads positions and velocities */
private:
    float pos_scale;
    float vel_scale;
    
    Eigen::MatrixXd _pos_data;
    Eigen::MatrixXd _vel_data;

public:
    DynamixelPosVelReader(
        DynamixelClient *client,
        std::vector<int> motor_ids,
        float pos_scale = 1.0,
        float vel_scale = 1.0
    );

    std::vector<float> _get_scale();

    std::vector<Eigen::MatrixXd> read(int retries = 1);
    void _initialize_data();
    void _update_data(int index, int motor_id);
    std::vector<Eigen::MatrixXd> _get_data();
};

class DynamixelPosReader : public DynamixelReader
{
    /* Reads positions */
private:
    float pos_scale;
    Eigen::MatrixXd _pos_data;

public:
    DynamixelPosReader(
        DynamixelClient *client,
        std::vector<int> motor_ids,
        float pos_scale = 1.0,
        float vel_scale = 1.0,
        float cur_scale = 1.0
    );

    float _get_scale();

    Eigen::MatrixXd read(int retries = 1);
    void _initialize_data();
    void _update_data(int index, int motor_id);
    Eigen::MatrixXd _get_data();
};

class DynamixelVelReader : public DynamixelReader
{
    /* Reads velocities */
private:
    float vel_scale;    
    Eigen::MatrixXd _vel_data;

public:
    DynamixelVelReader(
        DynamixelClient *client,
        std::vector<int> motor_ids,
        float pos_scale = 1.0,
        float vel_scale = 1.0,
        float cur_scale = 1.0
    );

    float _get_scale();

    Eigen::MatrixXd read(int retries = 1);
    void _initialize_data();
    void _update_data(int index, int motor_id);
    Eigen::MatrixXd _get_data();
};

class DynamixelCurReader : public DynamixelReader
{
    /* Reads currents */
private:
    float cur_scale;
    Eigen::MatrixXd _cur_data;

public:
    DynamixelCurReader(
        DynamixelClient *client,
        std::vector<int> motor_ids,
        float pos_scale = 1.0,
        float vel_scale = 1.0,
        float cur_scale = 1.0
    );

    float _get_scale();

    Eigen::MatrixXd read(int retries = 1);
    void _initialize_data();
    void _update_data(int index, int motor_id);
    Eigen::MatrixXd _get_data();
};

#endif
