#include <cmath>
#include <chrono>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#define SOURCE_FRAME "odom"
#define TARGET_FRAME "base_link"

using namespace std;
int nodeRate = 20;

float right_speed = 0; 
float left_speed = 0;


void receive_right_speed(const std_msgs::Float32 &received_speed){
    right_speed = received_speed.data;
}

void receive_left_speed(const std_msgs::Float32 &received_speed){
    left_speed = received_speed.data;
}

void broadcast_odom_tf(nav_msgs::Odometry odom){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = SOURCE_FRAME;
    transformStamped.child_frame_id = TARGET_FRAME;

    transformStamped.transform.translation.x = odom.pose.pose.position.x;
    transformStamped.transform.translation.y = odom.pose.pose.position.y;
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation.x = odom.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odom.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odom.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = odom.pose.pose.orientation.w;

    br.sendTransform(transformStamped);
}

nav_msgs::Odometry get_odom_message(float robot_x, float robot_y, float robot_theta, float velocity, float angular_vel){
    nav_msgs::Odometry odom;
    // Convert orientation to quaternion
    double roll = 0;
    double pitch = 0;
    double yaw = robot_theta;
    tf2::Quaternion robot_quaternion;
    robot_quaternion.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion quat_msg;
    quat_msg = tf2::toMsg(robot_quaternion);
    // Robot position
    odom.pose.pose.position.x = robot_x;
    odom.pose.pose.position.y = robot_y;
    odom.pose.pose.orientation = quat_msg;
    // Get current velocity
    odom.twist.twist.linear.x = velocity;
    odom.twist.twist.angular.z = angular_vel;

    odom.header.stamp    = ros::Time::now();
    odom.header.frame_id = SOURCE_FRAME;
    odom.child_frame_id  = TARGET_FRAME;
    return odom;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "dead_reckoning");
    ros::NodeHandle handler;
    ros::Rate rate(nodeRate);

    ros::Publisher odom_pub = handler.advertise<nav_msgs::Odometry>("/odom", 10);
    ros::Publisher tf;
    ros::Subscriber wl = handler.subscribe("/wr", 10, receive_right_speed);
    ros::Subscriber wr = handler.subscribe("/wl", 10, receive_left_speed);

     float _WHEELBASE, _WHEELRADIUS;
    // Get robot parameters
    ros::param::get("/wheel_base", _WHEELBASE);
    ros::param::get("/wheel_radius", _WHEELRADIUS);

    nav_msgs::Odometry odom;
    float velocity, angular_vel;

    float robot_x = 0;
    float robot_y = 0;
    float robot_theta = 0;
    float dt = 0;

    while(ros::ok){
        chrono::steady_clock::time_point t = chrono::steady_clock::now();

        velocity = _WHEELRADIUS*(right_speed + left_speed)/2;
        angular_vel = _WHEELRADIUS*(right_speed - left_speed)/_WHEELBASE;

        robot_x = robot_x + ((velocity * cos(robot_theta))*dt);
        robot_y = robot_y + ((velocity * sin(robot_theta))*dt);
        robot_theta = robot_theta + (angular_vel * dt);
        
        if(robot_theta >  M_PI) robot_theta = robot_theta - 2*M_PI;
        else if(robot_theta < - M_PI) robot_theta = robot_theta + 2*M_PI;

        odom = get_odom_message(robot_x, robot_y, robot_theta, velocity, angular_vel);
        broadcast_odom_tf(odom);

        odom_pub.publish(odom);

        ros::spinOnce();
        rate.sleep();     
        dt = (chrono::duration_cast<chrono::microseconds> (chrono::steady_clock::now() - t).count())/1000000.0; // Time difference in seconds
     
     }
    
}