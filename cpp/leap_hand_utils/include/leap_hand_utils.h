#ifndef LEAP_HAND_UTILS_H
#define LEAP_HAND_UTILS_H

#include <string>
#include <vector>

#include <Eigen>


Eigen::MatrixXd angle_safety_clip(Eigen::MatrixXd joints);

std::vector<Eigen::MatrixXd> LEAPsim_limits(std::string type = "regular");

Eigen::MatrixXd scale(Eigen::MatrixXd x, Eigen::MatrixXd lower, Eigen::MatrixXd upper);

Eigen::MatrixXd unscale(Eigen::MatrixXd x, Eigen::MatrixXd lower, Eigen::MatrixXd upper);

/**********************************************************************************/

Eigen::MatrixXd sim_ones_to_LEAPhand(Eigen::MatrixXd joints, bool hack_thumb = false);

Eigen::MatrixXd LEAPhand_to_sim_ones(Eigen::MatrixXd joints, bool hack_thumb = false);

/**********************************************************************************/

Eigen::MatrixXd LEAPsim_to_LEAPhand(Eigen::MatrixXd joints);

Eigen::MatrixXd LEAPhand_to_LEAPsim(Eigen::MatrixXd joints);

/**********************************************************************************/

Eigen::MatrixXd allegro_to_LEAPhand(Eigen::MatrixXd joints, bool teleop = false, bool zeros = true);

Eigen::MatrixXd LEAPhand_to_allegro(Eigen::MatrixXd joints, bool teleop = false, bool zeros = true);

/**********************************************************************************/

#endif
