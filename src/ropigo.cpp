#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

extern "C" {
#include <gopigo.h>
}

#include <cmath>

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_DEBUG("-------");

    const double t = msg->linear.x;
    const double r = msg->angular.z;

    const double mag = sqrt(pow(t,2)+pow(r,2));

    ROS_DEBUG("translation: %f, rotation: %f, magnitude: %f", t, r, mag);

    // ignore small values
    if(mag <= 0.1) {
        stop();
        return;
    }

    // left and right motor values
    double m1 = 0;
    double m2 = 0;

    m1 = t/mag * std::abs(t);
    m2 = t/mag * std::abs(t);

    m1 += -r/mag * std::abs(r);
    m2 += r/mag * std::abs(r);

    ROS_DEBUG("m1, m2: %f, %f", m1, m2);

    // restrict range [-1...1]
    m1 = m1/std::abs(m1) * std::min(std::abs(m1), 1.0);
    m2 = m2/std::abs(m2) * std::min(std::abs(m2), 1.0);

    ROS_DEBUG("m1, m2: %f(%i), %f(%i)", m1, m1>=0, m2, m2>=0);

    ROS_DEBUG("writing values (motor1, motor2): (%i)%i, (%i)%i", std::max(0, int(m1/std::abs(m1))), int(m1*255), std::max(0, int(m2/std::abs(m2))), int(m2*255));

    motor1(m1>=0, std::abs(int(m1*255)));
    motor2(m2>=0, std::abs(int(m2*255)));
}

int main(int argc, char **argv) {

    init();

    ROS_INFO("GoPiGo Firmware Version: %d\n",fw_ver());

    ros::init(argc, argv, "ropigo");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmdCallback);
    ros::spin();
    return 0;
}
