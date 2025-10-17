#include <leap_hand_utils/leap_hand_utils.h>
#include <leap_hand_utils/dynamixel_client.h>

class LeapController
{
private:
    int kP;
    int kI;
    int kD;
    int curr_lim;

    Eigen::VectorXd prev_pos;
    Eigen::VectorXd curr_pos;
    Eigen::MatrixXd pos;

    std::vector<int> motors;
    DynamixelClient dxl_client;

public:
    LeapController(const std::string &usb_port = "/dev/ttyUSB0");

    /**
     * @brief Connect leap hand
     * 
     */
    void connect();

    /**
     * @brief Disconnect hand
     * 
     */
    void disconnect();

    /**
     * @brief Set kp, ki, kd gains
     * 
     * @param kp 
     * @param ki 
     * @param kd 
     */
    void setGains(int kp = 600, int ki = 0, int kd = 200);

    /**
     * @brief Get kp, ki, kd gains
     * 
     * @return std::tuple<int, int, int> 
     */
    inline std::tuple<int, int, int> getGains() {return {kP, kI, kD};}

    inline const std::vector<int> & getMotors() const {return motors;}

    /**
     * @brief Set the leap hand pose
     * 
     * @param pose radians per joint
     */
    void set_leap(Eigen::VectorXd pose);

    /**
     * @brief Set joint angle 
     * 
     * @param idx motor index 
     * @param rad  
     */
    void setJointPose(int idx, float rad);

    /**
     * @brief Get joint pose
     * 
     * @param idx 
     * @return double angle in rad
     */
    double getJointPose(int idx);

    /**
     * @brief Get joint pos, vel, cur
     * 
     * @param idx 
     * @return std::tuple<double, double, double> 
     */
    std::tuple<double, double, double> getJointData(int idx);

    //allegro compatibility
    void set_allegro(Eigen::VectorXd pose);

    //Sim compatibility, first read the sim value in range [-1,1] and then convert to leap
    void set_ones(Eigen::VectorXd pose) ;

    //Read pos and vel.  NOTE: if you want both pos and vel, this is a faster way to read from the motors than individually!
    std::vector<Eigen::MatrixXd> read_pos_vel() ;

    //Read pos and vel.  NOTE: if you want both pos and vel and cur, this is a faster way to read from the motors than individually!
    std::vector<Eigen::MatrixXd> read_pos_vel_cur();

    //read position
    Eigen::VectorXd read_pos();

    //read velocity
    Eigen::VectorXd read_vel();

    //read current
    Eigen::VectorXd read_cur();
};