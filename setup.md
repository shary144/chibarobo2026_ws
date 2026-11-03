最初
. /opt/ros/jazzy/setup.bash
colcon build --packages-select robomas_interfaces
colcon build --packages-select robomas_controller
source install/setup.bash
colcon build --packages-select manual_control
source install/setup.bash

次回以降
source install/setup.bash