# GoPiGo ROS Node
ROS node to control GoPiGo robots (http://www.dexterindustries.com/gopigo/).

The node subscribes to _/cmd_vel_ (_geometry_msgs/Twist_ messages) and evaluates linear.x and angular.z to compute speed values for the left and right motor.

To build, you also need to install the library ros-lib-gopigo that provides the functions to communicate with the GoPiGo firmware.

To run, publish Twist messages e.g. by the _teleop_twist_joy_ or the _teleop_twist_keyboard_ nodes and run the ropigo_node.
