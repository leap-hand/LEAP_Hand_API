
# Leap ROS2 Interface

- This repository contains code for running the [LEAP Hand](http://leaphand.com/).


## Table of Contents

- [Dependencies](#dependencies)
- [Setup Instructions](#setup-instructions)
- [Connecting Digits](#connecting-digits)
- [Running the Digit Driver](#running-the-digit-driver)
- [Debugging](#debugging)

## Dependencies
The table below lists the direct dependencies needed for this repository.

| **Dependency Name**                                                          | **Description**                                                                   |
|------------------------------------------------------------------------------|-----------------------------------------------------------------------------------|
| [imutils](https://github.com/PyImageSearch/imutils)                          | A series of convenience functions to make basic image processing functions.       |
| [digit-interface](https://github.com/facebookresearch/digit-interface)       | Python interface for the DIGIT tactile sensor.     |

## Setup Instructions

Follow these steps to set up the Digit ROS2 Interface:

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/BiomechatronicsLab/digit_ros2.git
   cd your-repo-directory
   ```

2. **Install Dependencies**:
   Ensure you have Python packages required for the project:
   ```bash
   pip install -r requirements.txt
   ```

3. **Install Tactile Sensor Messages**:
   Ensure that [Tacticle Sensor Messages](https://github.com/BiomechatronicsLab/tactile_sensor_msgs) is installed:
   ```bash
   git clone git@github.com:BiomechatronicsLab/tactile_sensor_msgs.git
   ```

4. **Compile the Package**:
   Build the package using `colcon`:
   ```bash
   colcon build --packages-select digit_ros2 tactile_sensor_msgs --symlink-install
   ```

## Connecting Digits

Once the Digits are plugged in, you can verify the connection by running:
```bash
ros2 run digit_ros2 print_connected_digits.py
```
This command will list all connected Digit sensors. If your sensor is not listed, ensure it is properly connected.

## Running the Digit Driver

To run the Digit driver, you first need to create a configuration file. It’s recommended to copy the existing `default_params.yaml` and modify it according to your requirements.

### Steps to Run the Driver:

1. **Create and Edit Configuration File**:
   Copy the default parameters:
   ```bash
   cp path/to/default_params.yaml path/to/your/config.yaml
   ```

   Modify `config.yaml` as needed to set your desired parameters.

2. **Launch the Driver**:
   Execute the following command, specifying the path to your configuration file:
   ```bash
   ros2 launch digit_ros2 digits_launch.py config_file:=/path/to/your/config.yaml
   ```

   If you do not specify a config file, the driver will default to the parameters in `default_params.yaml`.

### Zeroing the sensor

If you wish to zero out all of the force values use the command:
```bash 
ros2 service call /pinky/zero std_srvs/srv/Trigger {}\ 
```

## Debugging

If you encounter issues while running the scripts, ensure you have sourced your ROS2 workspace:
```bash
source install/setup.bash
```

### Additional Debugging Tips:

- Use the **RQT Image Viewer** to visualize the optical flow images. This can help identify potential issues with the optical flow algorithm.
- Check your cable connection and make sure it is secure.
- Tighten all of the screws on the DIGIT, especailly those holding the PCB in place.












## Welcome to the LEAP Hand SDK
- Please visit [our website](http://leaphand.com/) for more information about LEAP hand.
#### Software Setup
- Please see the [Python API](https://github.com/leap-hand/LEAP_Hand_API/tree/main/python), [ROS API](https://github.com/leap-hand/LEAP_Hand_API/tree/main/ros_module), [Useful Tools](https://github.com/leap-hand/LEAP_Hand_API/tree/main/useful_tools) folders for software specific details.

#### Hardware Setup
- Connect 5v power to the hand (the dynamixels should light up during boot up.)
- Connect the Micro USB cable to the hand (Do not use too many USB extensions)
- Open [Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/rplus1/dynamixel_wizard/) and find the correct port using the options button and put that in main.py or ros_example.py.  Note, you cannot have Dynamixel wizard open while using the hand's API, the port will be "busy" with the other process.
- On Ubuntu you can find the hand by ID using `/dev/serial/by-id` The ID will stay persistent on reboots.
- We offically support Python and ROS, but other languages are supported by [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/).

#### Functionality
- Leap Node allows you to command joint angles in different scalings.
- You can read position, velocity and current from the hand.  
- Do not query reads too often, going past 90hz for one set of angles or 30hz for all three will slow down the USB communication.
- The default controller follows the PID control, up to the current limit cap. 
- Other controllers including velocity control or current control are supported as per the [motor manual](https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/)
- For Lite, keep the current limit around 300ma.
- For Full, you can raise the current limit up to 550ma.
- If facing a jittery hand, adjust the PID values down.
- If the hand is too weak, adjust the PID values up.

#### Troubleshooting
- If your motor is 90/180/270 Degrees off, the horn is mounted incorrectly on the motor.  Remount it.
- If no motors show up, check that your serial port permissions are correct.
- If some motors are missing, make sure they are IDed corrrectly and are connected to the U2D2.
- If you get "overload error" and the motors are flashing red, then they have overloaded (self-collision etc). It should clear on a power cycle.  If it happens often, lower the current limits in the control code so that it does not happen as often.
- If you get "jittery" motors, try lowering the P and D values, either in the roslaunch file or the python file.
- If you feel the motors are too inaccurate, you can also try raising the P and D values.
- To improve latency on Ubuntu try these tips.   Configure [USB Latency Settings in Ubuntu](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) and the [Dynamixel Python SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/288)

#### Useful Tools:
- MANO to LEAP joint angle mapping.
- If you have useful tools you feel the community can benefit from, please make a pull request.
- I can also add tools to this upon request.  :)

#### Support:
- Please contact me at kshaw2@andrew.cmu.edu for any issues.
- This code is made available using an MIT License.
- The CAD files are provided with a CC BY-NC-SA Attribution-NonCommercial-ShareAlike license which allows you to use and build upon our work non-commercially.
- LEAP Hand is provided as-is and without warranty.
- If you use LEAP Hand in an academic setting, please cite our paper:
```
@article{shaw2023leaphand,
  title={LEAP Hand: Low-Cost, Efficient, and Anthropomorphic Hand for Robot Learning},
  author={Shaw, Kenneth and Agarwal, Ananye and Pathak, Deepak},
  journal={Robotics: Science and Systems (RSS)},
  year={2023}
}
```
