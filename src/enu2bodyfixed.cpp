#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <ros/console.h>

tf2::Quaternion initialImuQuaternion_tf, imuQuaternion_tf, relativeQuaternion_tf, biggestRelativeQuaternion_tf, waist2bodyFixedQuaternion_tf;
geometry_msgs::Quaternion imuQuaternion, initialImuQuaternion, waist2bodyFixedQuaternion, relativeQuaternion;
tf2::Vector3 rotationAxis;
float relativeRotationAngle, biggestRotationAngle = 0;
bool initialized = false;

void imudataCallback(const sensor_msgs::Imu::ConstPtr& msg){

  if (initialized == false){
    initialImuQuaternion = msg->orientation;
    tf2::convert(initialImuQuaternion , initialImuQuaternion_tf);
    initialized = true;
    return;
  }

  imuQuaternion = msg->orientation;
  tf2::convert(imuQuaternion , imuQuaternion_tf);

  relativeQuaternion_tf = initialImuQuaternion_tf.inverse() * imuQuaternion_tf ;
  relativeRotationAngle = relativeQuaternion_tf.getAngleShortestPath();

  if (relativeRotationAngle > biggestRotationAngle){
    biggestRotationAngle = relativeRotationAngle;
    biggestRelativeQuaternion_tf = relativeQuaternion_tf;
    rotationAxis = biggestRelativeQuaternion_tf.getAxis();
  }
  

}


int main(int argc, char** argv){
  ros::init(argc, argv, "enu2bodyfixed");
  
  ros::NodeHandle nh;
  ros::Subscriber waistImuSub = nh.subscribe("/imu_1", 1, imudataCallback);

  ros::Duration(1).sleep();
  ROS_INFO("find ENU frame to BodyFixed frame quaternion!");

  for (int i=5; i>0; i--)
  {
    ROS_INFO("start in %d seconds.....",i);
    ros::Duration(1).sleep();
  }

  
  ros::spinOnce();
  ROS_INFO("saved initial IMU orientation!");

  ros::Time begin = ros::Time::now();
  while (ros::ok() && ((begin + ros::Duration(10)).toSec() > ros::Time::now().toSec()))
    {
        ROS_INFO("biggest rotation angle: %f \nrotation axis x: %f, y: %f, z: %f"
            ,biggestRotationAngle, rotationAxis.x(), rotationAxis.y(), rotationAxis.z());
        ros::spinOnce();

    }
  // for (int i=10; i>0; i--)
  // {
  //   ROS_INFO("finding biggest rotation ends in %d seconds.....",i);
  //   ros::Duration(1).sleep();
  //   ROS_INFO("%f",imuQuaternion_tf.x());
  // }
  
  waist2bodyFixedQuaternion_tf.setRotation(rotationAxis.cross(tf2::Vector3(0.0, 1.0, 0.0)), rotationAxis.angle(tf2::Vector3(0.0, 1.0, 0.0)));

  // ROS_INFO("biggest rotation angle: %f \nrotation axis x: %f, y: %f, z: %f"
  //           ,biggestRotationAngle, rotationAxis.x(), rotationAxis.y(), rotationAxis.z());

  // ROS_INFO("rotation angle: %f \nrotation axis x: %f, y: %f, z: %f"
  //           ,rotationAxis.angle(tf2::Vector3(0.0, 1.0, 0.0)), rotationAxis.cross(tf2::Vector3(0.0, 1.0, 0.0)).x(), rotationAxis.cross(tf2::Vector3(0.0, 1.0, 0.0)).y(), rotationAxis.cross(tf2::Vector3(0.0, 1.0, 0.0)).z());

  tf2::convert(waist2bodyFixedQuaternion_tf , waist2bodyFixedQuaternion);

  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  
  
 


  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "waist";
  transformStamped.child_frame_id = "body_fixed";
  transformStamped.transform.translation.x = 0;
  transformStamped.transform.translation.y = 0;
  transformStamped.transform.translation.z = 0;
  transformStamped.transform.rotation = waist2bodyFixedQuaternion;
  br.sendTransform(transformStamped);
  ros::spin();
  
  return 0;
};