# GoPiGo ROS Node
ROS node to control GoPiGo robots (http://www.dexterindustries.com/gopigo/).

## Installation
The node requires the GoPiGo driver to be installed either as [catkin package](https://github.com/ros-gopigo/catkin-libgopigo) or [manually](https://github.com/DexterInd/GoPiGo/tree/master/Software/C#dedicated-library).
The full instructions for building the GoPiGo library and ROS node as catkin packages are located at https://github.com/ros-gopigo/rosinstall-repo.

## Usage

To control the robot, you need to publish `geometry_msgs/Twist` messages on the `/cmd_vel` topic. Only the `linear.x` and `angular.z` parts of the Twist message are evaluated to compute left and right motor speed.

Twist messages are published for example by the _teleop_twist_joy_ or the _teleop_twist_keyboard_ nodes.
