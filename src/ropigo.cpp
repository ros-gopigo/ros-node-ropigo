#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <smart_battery_msgs/SmartBatteryStatus.h>

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
    ROS_INFO("GoPiGo Board Version: %d\n",brd_rev());

    ros::init(argc, argv, "ropigo");
    ros::NodeHandle n;

    ros::Subscriber cmd = n.subscribe("cmd_vel", 1000, cmdCallback);

    ros::Publisher battery_pub = n.advertise<smart_battery_msgs::SmartBatteryStatus>("battery",1);

    // odometry
    ros::Publisher lwheel_pub = n.advertise<std_msgs::Int16>("lwheel",1);
    ros::Publisher rwheel_pub = n.advertise<std_msgs::Int16>("rwheel",1);

    ros::Rate loop(10);

    while(ros::ok()) {
        smart_battery_msgs::SmartBatteryStatus battery;
        std_msgs::Int16 lwheel, rwheel;

        battery.voltage = volt();

        lwheel.data = (int16_t)enc_read(0);
        rwheel.data = (int16_t)enc_read(1);

        // publish topics
        battery_pub.publish(battery);
        lwheel_pub.publish(lwheel);
        rwheel_pub.publish(rwheel);

        ros::spinOnce();
        loop.sleep();
    }

    ros::spin();
    return 0;
}
