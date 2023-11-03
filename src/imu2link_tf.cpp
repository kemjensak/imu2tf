#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>

tf2::Quaternion lowerImuQuaternion_tf,
                upperImuQuaternion_tf,
                upper2lowerRelativeQuaternion_tf;
                
geometry_msgs::Quaternion lowerImuQuaternion,
                          upperImuQuaternion,
                          upper2lowerRelativeQuaternion;

void waistImudataCallback(const geometry_msgs::Quaternion::ConstPtr &msg)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "earth";
    transformStamped.child_frame_id = "fbf";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0.5;
    transformStamped.transform.rotation = *msg;
    br.sendTransform(transformStamped);
}

void upperImudataCallback(const geometry_msgs::Quaternion::ConstPtr &msg)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "fbf";
    transformStamped.child_frame_id = "upper_arm";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = -0.20;
    transformStamped.transform.translation.z = 0.45;
    transformStamped.transform.rotation = *msg;
    br.sendTransform(transformStamped);
}

void lowerImudataCallback(const geometry_msgs::Quaternion::ConstPtr &msg)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    lowerImuQuaternion = *msg;
    tf2::convert(lowerImuQuaternion, lowerImuQuaternion_tf);

    upper2lowerRelativeQuaternion_tf = upperImuQuaternion_tf.inverse() * lowerImuQuaternion_tf;

    tf2::convert(upper2lowerRelativeQuaternion_tf, upper2lowerRelativeQuaternion);

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "upper_arm";
    transformStamped.child_frame_id = "lower_arm";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = -0.27;
    transformStamped.transform.rotation = upper2lowerRelativeQuaternion;
    br.sendTransform(transformStamped);

    transformStamped.header.frame_id = "lower_arm";
    transformStamped.child_frame_id = "wrist";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = -0.3;
    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;
    br.sendTransform(transformStamped);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu2link_tf");

    ros::NodeHandle nh;

    ros::Subscriber upperImuSub = nh.subscribe("/shoulder_fbf", 1, upperImudataCallback);
    ros::Subscriber lowerImuSub = nh.subscribe("/wrist_fbf", 1, lowerImudataCallback);
    ros::Subscriber waistImuSub = nh.subscribe("/dh_matrix_fbf", 1, waistImudataCallback);

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
};