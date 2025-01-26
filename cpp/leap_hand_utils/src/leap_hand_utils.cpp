/*
Some utilities for LEAP Hand that help with converting joint angles between each convention.
*/
#include <string>
#include <vector>

#include "leap_hand_utils.h"
#include <Eigen>


/*
Embodiments:

LEAPhand: Real LEAP hand (180 for the motor is actual zero)
LEAPsim:  Leap hand in sim (has allegro-like zero positions)
one_range: [-1, 1] for all joints to facilitate RL
allegro:  Allegro hand in real or sim
*/

// Safety clips all joints so nothing unsafe can happen. Highly recommend using this before commanding
Eigen::MatrixXd angle_safety_clip(Eigen::MatrixXd joints) {
    std::vector<Eigen::MatrixXd> sim_values {LEAPsim_limits()};
    Eigen::MatrixXd real_min {LEAPsim_to_LEAPhand(sim_values[0])};
    Eigen::MatrixXd real_max {LEAPsim_to_LEAPhand(sim_values[1])};
    return joints.cwiseMin(real_max).cwiseMax(real_min);
}

// Sometimes it's useful to constrain the thumb more heavily(you have to implement here), but regular usually works good.
std::vector<Eigen::MatrixXd> LEAPsim_limits(std::string type) {
    Eigen::MatrixXd sim_min;
    Eigen::MatrixXd sim_max;

    Eigen::MatrixXd temp_min {{-1.047, -0.314, -0.506, -0.366, -1.047, -0.314, -0.506, -0.366, -1.047, -0.314, -0.506, -0.366, -0.349, -0.47, -1.20, -1.34}};
    Eigen::MatrixXd temp_max {{ 1.047,   2.23,  1.885,  2.042,  1.047,   2.23,  1.885,  2.042,  1.047,   2.23,  1.885,  2.042,  2.094,  2.443, 1.90,  1.88}};
    
    if (type == "regular") {
        sim_min = temp_min;
        sim_max = temp_max;
    }
    std::vector<Eigen::MatrixXd> m{sim_min, sim_max};
    return m;
}

// this goes from [-1, 1] to [lower, upper]
Eigen::MatrixXd scale(Eigen::MatrixXd x, Eigen::MatrixXd lower, Eigen::MatrixXd upper) {
    return (0.5 * (x.array() + 1.0).matrix() * (upper - lower) + lower);
}

// this goes from [lower, upper] to [-1, 1]
Eigen::MatrixXd unscale(Eigen::MatrixXd x, Eigen::MatrixXd lower, Eigen::MatrixXd upper) {
    return (2.0 * x - upper - lower).array() / (upper - lower).array();
}

/**********************************************************************************/

// Isaac has custom ranges from -1 to 1 so we convert that to LEAPHand real world
Eigen::MatrixXd sim_ones_to_LEAPhand(Eigen::MatrixXd joints, bool hack_thumb) {
    std::vector<Eigen::MatrixXd> sim_values = LEAPsim_limits(hack_thumb?"true":"false");
    joints = scale(joints, sim_values[0], sim_values[1]);
    joints = LEAPsim_to_LEAPhand(joints);
    return joints;
}

// LEAPHand real world to Isaac has custom ranges from -1 to 1
Eigen::MatrixXd LEAPhand_to_sim_ones(Eigen::MatrixXd joints, bool hack_thumb) {
    joints = LEAPhand_to_LEAPsim(joints);
    std::vector<Eigen::MatrixXd> sim_values = LEAPsim_limits(hack_thumb?"true":"false");
    joints = unscale(joints, sim_values[0], sim_values[1]);
    return joints;
}

/**********************************************************************************/

// Sim LEAP hand to real leap hand  Sim is allegro-like but all 16 joints are usable.
Eigen::MatrixXd LEAPsim_to_LEAPhand(Eigen::MatrixXd joints) {
    Eigen::MatrixXd ret_joints{joints.array() + 3.14159};
    return ret_joints;
}

// Real LEAP hand to sim leap hand  Sim is allegro-like but all 16 joints are usable.
Eigen::MatrixXd LEAPhand_to_LEAPsim(Eigen::MatrixXd joints) {
    Eigen::MatrixXd ret_joints{joints.array() - 3.14159};
    return ret_joints;
}

/**********************************************************************************/

// Converts allegrohand radians to LEAP (radians)
// Only converts the joints that match, all 4 of the thumb and the outer 3 for each of the other fingers
// All the clockwise/counterclockwise signs are the same between the two hands.  Just the offset (mostly 180 degrees off)
Eigen::MatrixXd allegro_to_LEAPhand(Eigen::MatrixXd joints, bool teleop, bool zeros) {
    Eigen::MatrixXd ret_joints {joints.array() + 3.14159};
    if (zeros) {
        ret_joints(0) = ret_joints(4) = ret_joints(8) = 3.14;
    }
    if (teleop) {
        ret_joints(12) = joints(12) + 0.2;
        ret_joints(14) = joints(14) - 0.2;
    }
    return ret_joints;
}

// Converts LEAP to allegrohand (radians)
Eigen::MatrixXd LEAPhand_to_allegro(Eigen::MatrixXd joints, bool teleop, bool zeros) {
    Eigen::MatrixXd ret_joints {joints.array() - 3.14159};
    if (zeros) {
        ret_joints(0) = ret_joints(4) = ret_joints(8) = 0;
    }
    if (teleop) {
        ret_joints(12) = joints(12) - 0.2;
        ret_joints(14) = joints(14) + 0.2;
    }
    return ret_joints;
}

/**********************************************************************************/
