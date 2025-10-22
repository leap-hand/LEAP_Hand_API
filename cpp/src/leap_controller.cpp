#include <leap_hand_utils/leap_controller.h>

LeapController::LeapController(const std::string &usb_port) :
	kP{600}, kI{0}, kD{200}, curr_lim{350},
	prev_pos{ allegro_to_LEAPhand(Eigen::VectorXd::Zero(16)) },
	curr_pos{ allegro_to_LEAPhand(Eigen::VectorXd::Zero(16)) },
	pos{ allegro_to_LEAPhand(Eigen::VectorXd::Zero(16)) },
	motors{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
	dxl_client{motors, usb_port, 1000000, false}
{

}

void LeapController::connect()
{
  dxl_client.connect();
  
  // Enables position-current control mode and the default parameters, it commands a position and then caps the current
  // so the motors don't overload
  dxl_client.sync_write(motors, Eigen::MatrixXd::Ones(1, motors.size()) * 3, 11, 1); // Position control
  dxl_client.set_torque_enabled(motors, true);

  setGains();

  // Max at current (in unit 1ma) so don't overheat and grip too hard //500 normal or //350 for lite
  dxl_client.sync_write(motors, Eigen::MatrixXd::Ones(1, motors.size()) * curr_lim, 102, 2);
  dxl_client.write_desired_pos(motors, curr_pos);
}

void LeapController::disconnect()
{
  dxl_client.disconnect();
}

void LeapController::setGains(int kp, int ki, int kd)
{
  kP = kp;
  kI = ki;
  kD = kd;

  dxl_client.sync_write(motors, Eigen::MatrixXd::Ones(1, motors.size()) * kP, 84, 2); // Pgain stiffness
  dxl_client.sync_write({0, 4, 8}, Eigen::MatrixXd::Ones(1, 3) * (kP * 0.75), 84,
                        2); // Pgain stiffness for side to side should be a bit less
  dxl_client.sync_write(motors, Eigen::MatrixXd::Ones(1, motors.size()) * kI, 82, 2); // Igain
  dxl_client.sync_write(motors, Eigen::MatrixXd::Ones(1, motors.size()) * kD, 80, 2); // Dgain damping
  dxl_client.sync_write({0, 4, 8}, Eigen::MatrixXd::Ones(1, 3) * (kD * 0.75), 80,
                        2); // Dgain damping for side to side should be a bit less
}

// Receive LEAP pose and directly control the robot
void LeapController::set_leap(Eigen::VectorXd pose)
{
  prev_pos = curr_pos;
  curr_pos = pose;
  dxl_client.write_desired_pos(motors, curr_pos);
}

void LeapController::setJointPose(int idx, float rad)
{
  prev_pos = curr_pos;
  curr_pos[idx] = rad;
  dxl_client.write_desired_pos(motors, curr_pos);
}

double LeapController::getJointPose(int idx)
{
  return read_pos()[idx];
}

std::tuple<double, double, double> LeapController::getJointData(int idx)
{
  const auto & pvc = read_pos_vel_cur();
  return {pvc[0](0, idx) , pvc[1](0, idx), pvc[2](0, idx)};
}

// allegro compatibility
void LeapController::set_allegro(Eigen::VectorXd pose)
{
  pose = allegro_to_LEAPhand(pose, false, false); // default arguments cannot be skipped in C++
  prev_pos = curr_pos;
  curr_pos = pose;
  dxl_client.write_desired_pos(motors, curr_pos);
}

// Sim compatibility, first read the sim value in range [-1,1] and then convert to leap
void LeapController::set_ones(Eigen::VectorXd pose)
{
  pose = sim_ones_to_LEAPhand(pose);
  prev_pos = curr_pos;
  curr_pos = pose;
  dxl_client.write_desired_pos(motors, curr_pos);
}

// Read pos and vel.  NOTE: if you want both pos and vel, this is a faster way to read from the motors than individually!
std::vector<Eigen::MatrixXd> LeapController::read_pos_vel()
{
  return dxl_client.read_pos_vel();
}

// Read pos and vel.  NOTE: if you want both pos and vel and cur, this is a faster way to read from the motors than
// individually!
std::vector<Eigen::MatrixXd> LeapController::read_pos_vel_cur()
{
  return dxl_client.read_pos_vel_cur();
}

// read position
Eigen::VectorXd LeapController::read_pos()
{
  return dxl_client.read_pos();
}

// read velocity
Eigen::VectorXd LeapController::read_vel()
{
  return dxl_client.read_vel();
}

// read current
Eigen::VectorXd LeapController::read_cur()
{
  return dxl_client.read_cur();
}
