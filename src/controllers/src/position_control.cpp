#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <chrono>

using namespace std;

int nodeRate = 100;
nav_msgs::Path path;
geometry_msgs::PoseWithCovarianceStamped position;
float right_speed , left_speed;
bool new_path = false;

void receive_path(const nav_msgs::Path &received_path){
    path = received_path;
}

void receive_position(const geometry_msgs::PoseWithCovarianceStamped &received_pose){
    position = received_pose; 
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle handler;    
    ros::Subscriber systemFeedback = handler.subscribe("/path", 10, receive_path);
    ros::Subscriber positionSub = handler.subscribe("/filtered_pose", 10, receive_position);

    ros::Publisher controllerOutput = handler.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    float kpr, kit, kpt, wheelbase, wheel_radius;
    ros::param::get("/rotational_constant", kpr);
    ros::param::get("/translational_constant", kpt);
    ros::param::get("/translational_integral_constant", kit);
    
    ros::param::get("/wheel_base", wheelbase);
    ros::param::get("/wheel_radius", wheel_radius);
    ros::Rate rate(nodeRate);
    geometry_msgs::Twist output;
    //float dt = 0;
    float robot_x = 0;
    float robot_y = 0;
    float robot_orientation = 0;
    double roll = 0, pitch = 0, yaw = 0;

    float w_max = 2;
    float v_max = (w_max*wheel_radius)*0.5;
    float angularV_max = 2;

    float integral_trr = 0;
    float integral_rr = 0;

    while(ros::ok){
        auto local_path = path.poses;
        for (auto coord : local_path){
            float desired_x = coord.pose.position.x;
            float desired_y = coord.pose.position.y;
            float distance = 0, desired_angle = 0, angle_error = 0;

            distance = sqrt(pow(desired_y-robot_y,2)+pow(desired_x-robot_x,2));
            // cout << "Heading to: " << "x :" << desired_x << " y :" << desired_y << endl;

            while (distance>0.08 && ros::ok){
                chrono::steady_clock::time_point t = chrono::steady_clock::now();
                if(local_path != path.poses){
                    cout << "Path changes, aborting control\n";
                    break;
                }

                robot_x = position.pose.pose.position.x;
                robot_y = position.pose.pose.position.y;
                tf::Quaternion quat;
                tf::quaternionMsgToTF(position.pose.pose.orientation, quat);
                tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
                robot_orientation = yaw;

                desired_angle = atan2((desired_y-robot_y),(desired_x -robot_x));
                distance = sqrt(pow(desired_y-robot_y,2)+pow(desired_x-robot_x,2));
                
                angle_error = (desired_angle - robot_orientation);
                if(angle_error > M_PI) angle_error = angle_error - 2*M_PI;
                else if(angle_error < - M_PI)angle_error = angle_error + 2*M_PI;
                
                // cout << "Distance" << distance << endl;
                // cout << "Angle error" << angle_error << endl;

                float angularVelocity = kpr*angle_error;
                if(angularVelocity > angularV_max) angularVelocity = angularV_max;
                else if (angularVelocity < -angularV_max) angularVelocity = -angularV_max;
                
                float velocity = kpt*distance + kit*integral_trr; 
                velocity = v_max * tanh(velocity);
                integral_trr+=distance;

        
                output.linear.x = velocity;
                output.linear.y = 0;
                output.linear.z = 0;

                output.angular.x = 0;
                output.angular.y = 0;
                output.angular.z = angularVelocity;

                controllerOutput.publish(output);

                ros::spinOnce();
                rate.sleep();
                
            }
        }
        integral_trr = 0;
        ros::spinOnce();
        rate.sleep();
        
        output.linear.x = 0;
        output.linear.y = 0;
        output.linear.z = 0;

        output.angular.x = 0;
        output.angular.y = 0;
        output.angular.z = 0;
        controllerOutput.publish(output);
    }
}