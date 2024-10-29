## Welcome to the LEAP Hand ROS2 SDK

### On Ubuntu
- Install [ROS 2](https://docs.ros.org/en/humble/Installation.html) normally. (Tested on Ubuntu 22 using conda environment)
#### Create a [ROS2 workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html):
- Add this package to an existing workspace or create one.
- Copy this `ros2_module` folder into the src folder of the new workspace.
- Build using:
- `colcon build --symlink-install`
- Source ros2 and the workspace and add to .bashrc if you like.
#### First time preparation
- `pip install dynamixel_sdk numpy`
- `chmod +x leaphand_node.py`
- `chmod +x ros2_example.py`
#### To Launch
- Connect 5v power to the hand (the dynamixels should light up during boot up.)
- Connect the Micro USB cable to the hand (Do not use too many USB extensions)
- Find the USB port using [Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
- `ros2 launch launch_leap.py`
#### How to use
- `ros2_example.py` is an example script that queries the LEAP service and also publishes out a pose for the hand to move to.  You can build off of this for your own project.
- For more info I recommend checking the ROS2 docs and the other docs for LEAP Hand.