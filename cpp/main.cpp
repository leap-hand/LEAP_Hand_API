#include <iostream>
#include <signal.h>
#include <vector>

#include <Eigen>
#include "dynamixel_client.h"
#include "leap_hand_utils.h"


/*****************************************************/
// This can control and query the LEAP Hand

// I recommend you only query when necessary and below 90 samples a second.  Each of position, velociy and current costs one sample, so you can sample all three at 30 hz or one at 90hz.

//Allegro hand conventions:
//0.0 is the all the way out beginning pose, and it goes positive as the fingers close more and more
//http://wiki.wonikrobotics.com/AllegroHandWiki/index.php/Joint_Zeros_and_Directions_Setup_Guide I belive the black and white figure (not blue motors) is the zero position, and the + is the correct way around.  LEAP Hand in my videos start at zero position and that looks like that figure.

//LEAP hand conventions:
//180 is flat out for the index, middle, ring, finger MCPs.
//Applying a positive angle closes the other joints more and more.
/*****************************************************/

volatile sig_atomic_t flag = 0;
void keyboard_interrupt(int sig) {
    flag = 1;
}

class LeapNode
{
private:
    int kP;
    int kI;
    int kD;
    int curr_lim;

    Eigen::MatrixXd prev_pos;
    Eigen::MatrixXd curr_pos;
    Eigen::MatrixXd pos;

    std::vector<int> motors;
    DynamixelClient dxl_client;

public:
    LeapNode()
        : kP {600}
        , kI {0}
        , kD {200}
        , curr_lim {350}
        , prev_pos {allegro_to_LEAPhand(Eigen::MatrixXd::Zero(1,16))}
        , curr_pos {allegro_to_LEAPhand(Eigen::MatrixXd::Zero(1,16))}
        , pos {allegro_to_LEAPhand(Eigen::MatrixXd::Zero(1,16))}
        , motors {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}
        , dxl_client {motors, "/dev/ttyUSB0", 4000000, false}
    {
        dxl_client.connect();
        
        //Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        dxl_client.sync_write(motors, Eigen::MatrixXd::Ones(1, motors.size()) * 3, 11, 1);  // Position control
        dxl_client.set_torque_enabled(motors, true);
        dxl_client.sync_write(motors, Eigen::MatrixXd::Ones(1, motors.size()) * kP, 84, 2); // Pgain stiffness     
        dxl_client.sync_write({0,4,8}, Eigen::MatrixXd::Ones(1,3) * (kP * 0.75), 84, 2);    // Pgain stiffness for side to side should be a bit less
        dxl_client.sync_write(motors, Eigen::MatrixXd::Ones(1, motors.size()) * kI, 82, 2); // Igain
        dxl_client.sync_write(motors, Eigen::MatrixXd::Ones(1, motors.size()) * kD, 80, 2); // Dgain damping
        dxl_client.sync_write({0,4,8}, Eigen::MatrixXd::Ones(1,3) * (kD * 0.75), 80, 2);    // Dgain damping for side to side should be a bit less

        //Max at current (in unit 1ma) so don't overheat and grip too hard //500 normal or //350 for lite
        dxl_client.sync_write(motors, Eigen::MatrixXd::Ones(1, motors.size()) * curr_lim, 102, 2);
        dxl_client.write_desired_pos(motors, curr_pos);
    }

    //Receive LEAP pose and directly control the robot
    void set_leap(Eigen::MatrixXd pose) {
        prev_pos = curr_pos;
        curr_pos = pose;
        dxl_client.write_desired_pos(motors, curr_pos);
    }

    //allegro compatibility
    void set_allegro(Eigen::MatrixXd pose) {
        pose = allegro_to_LEAPhand(pose, false, false); // default arguments cannot be skipped in C++
        prev_pos = curr_pos;
        curr_pos = pose;
        dxl_client.write_desired_pos(motors, curr_pos);
    }

    //Sim compatibility, first read the sim value in range [-1,1] and then convert to leap
    void set_ones(Eigen::MatrixXd pose) {
        pose = sim_ones_to_LEAPhand(pose);
        prev_pos = curr_pos;
        curr_pos = pose;
        dxl_client.write_desired_pos(motors, curr_pos);
    }

    //Read pos and vel.  NOTE: if you want both pos and vel, this is a faster way to read from the motors than individually!
    std::vector<Eigen::MatrixXd> read_pos_vel() {
        return dxl_client.read_pos_vel();
    }

    //Read pos and vel.  NOTE: if you want both pos and vel and cur, this is a faster way to read from the motors than individually!
    std::vector<Eigen::MatrixXd> read_pos_vel_cur() {
        return dxl_client.read_pos_vel_cur();
    }

    //read position
    Eigen::MatrixXd read_pos() {
        return dxl_client.read_pos();
    }

    //read velocity
    Eigen::MatrixXd read_vel() {
        return dxl_client.read_vel();
    }

    //read current
    Eigen::MatrixXd read_cur() {
        return dxl_client.read_cur();
    }

};

int main()
{
    // Register signals 
    signal(SIGINT, keyboard_interrupt);

    LeapNode leap_hand {};
    while (1) {
        if (flag) {
            printf("\nShutting down\n");
            break;
        }

        leap_hand.set_allegro(Eigen::MatrixXd::Zero(1, 16));
        std::cout << "Position: " << leap_hand.read_pos() << '\n';
    }
}