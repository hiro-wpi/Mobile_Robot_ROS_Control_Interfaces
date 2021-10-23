#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"


class SubscribeAndPublish 
{
private:
    // ROS
    ros::NodeHandle n_; 
    ros::Subscriber twist_sub;
    ros::Publisher float_pub;

public:
    SubscribeAndPublish() 
    {
        // Subscriber
        twist_sub = n_.subscribe("pitch_yaw", 10, &SubscribeAndPublish::process_angle, this);
        // Publisher
        float_pub = n_.advertise<std_msgs::Float64>("angle", 1);
    }
    ~SubscribeAndPublish() {}

    void process_angle(const geometry_msgs::Twist twist)
    {
        // Initialize published message
        std_msgs::Float64 angle;

        // Map to angle
        float angle_map = 1.047;
        angle.data = twist.angular.z * angle_map;

        // ROS_DEBUG("Get angle %f", angle.data);
        float_pub.publish(angle);
    }
};


int main(int argc, char *argv[])
{
    // Launch ros node class
    ros::init(argc, argv, "sub_and_pub_template_node");
    SubscribeAndPublish SAP;

    // Set rate and run
    ros::Rate rate(60);
    while (ros::ok())
        ros::spinOnce();
        rate.sleep();
    return 0;
}
