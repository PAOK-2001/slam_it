#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <cmath>
#include <chrono>
#include <string>
#include <map>

using namespace std;

int nodeRate = 100;
nav_msgs::Path path;
float right_speed , left_speed;
bool isReceiving = false;


void receive_path(const nav_msgs::Path &received_path){
    path = received_path;
}



void receive_right_speed(const std_msgs::Float32 &received_speed){
    right_speed = received_speed.data;
    isReceiving = true;
}

void receive_left_speed(const std_msgs::Float32 &received_speed){
    left_speed = received_speed.data;
    isReceiving = true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle handler;
    bool hasFinished = false;
    
    ros::Subscriber systemFeedback = handler.subscribe("/path", 10, receive_path);
    ros::Subscriber wl = handler.subscribe("/wl", 10, receive_right_speed);
    ros::Subscriber wr = handler.subscribe("/wr", 10, receive_left_speed);

    ros::Publisher controllerOutput = handler.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher estimated_pose = handler.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 10);

    float kpr, kit, kpt, wheelbase, wheel_radius;
    ros::param::get("/rotational_constant", kpr);
    ros::param::get("/translational_constant", kpt);
    ros::param::get("/translational_integral_constant", kit);
    
    ros::param::get("/wheel_base", wheelbase);
    ros::param::get("/wheel_radius", wheel_radius);
    ros::Rate rate(nodeRate);
    geometry_msgs::Twist output;
    geometry_msgs::PoseStamped puzzlePose;
    float dt = 0;
    float robot_x = 0;
    float robot_y = 0;
    float robot_orientation = 0;

    float w_max = 8;
    float v_max = (w_max*wheel_radius)*0.5;
    float angularV_max = 1;

    float integral_trr = 0;
    float integral_rr = 0;

    while(ros::ok){
        if(!hasFinished){
            for (auto coord : path.poses){
                float desired_x = coord.pose.position.x;
                float desired_y = coord.pose.position.y;
                float desired_angle;

                cout << "X Goal :" << desired_x << endl;
                cout << "Y Goal :" << desired_y << endl;
                cout << "Theta :" << desired_angle << endl;

                float distance = sqrt(pow(desired_y-robot_y,2)+pow(desired_x-robot_x,2));

                while (distance>0.02){
                    chrono::steady_clock::time_point t = chrono::steady_clock::now();
                    //usleep(100000);
                    rate.sleep();
                    desired_angle = atan2(-(desired_y-robot_y),(desired_x -robot_x));
                    distance = sqrt(pow(desired_y-robot_y,2)+pow(desired_x-robot_x,2));
                    float angle_error = (robot_orientation - desired_angle);
                    if(angle_error > M_PI) angle_error = angle_error - 2*M_PI;
                    else if(angle_error < - M_PI)angle_error = angle_error + 2*M_PI;
                    
                    float angularVelocity = kpr*angle_error*traffic_light_multiplier;
                    if(angularVelocity > angularV_max) angularVelocity = angularV_max;
                    else if (angularVelocity < -angularV_max) angularVelocity = -angularV_max;
                    

                    float velocity = (kpt*distance + kit*integral_trr)*traffic_light_multiplier; 
                    velocity = v_max * tanh(velocity);
                    integral_trr+=distance;
            
                    output.linear.x = velocity;
                    output.linear.y = 0;
                    output.linear.z = 0;

                    output.angular.x = 0;
                    output.angular.y = 0;
                    output.angular.z = angularVelocity;

                    dt = (chrono::duration_cast<chrono::microseconds> (chrono::steady_clock::now() - t).count())/1000000.0;
                               
                    float real_linear_velocity =  ((right_speed + left_speed)/2) * wheel_radius;
                    float real_angular_velocity =  (right_speed - left_speed)/wheelbase*wheel_radius;

                    //cout << real_linear_velocity << " " << real_angular_velocity << endl;

                    robot_orientation += real_angular_velocity*dt;
                    
                    if(robot_orientation < 0) robot_orientation = robot_orientation + 2*M_PI;

                    robot_x += real_linear_velocity*dt*cos(robot_orientation);
                    robot_y -= real_linear_velocity*dt*sin(robot_orientation);

                    puzzlePose.pose.position.x = robot_x;
                    puzzlePose.pose.position.y = robot_y;
                    puzzlePose.pose.orientation.z = robot_orientation;

                    old_last_color = last_color;
                    controllerOutput.publish(output);
                    estimated_pose.publish(puzzlePose);
                    ros::spinOnce();
                    
                }
            }
            integral_trr = 0;
            ros::spinOnce();
            rate.sleep();
            hasFinished = robot_x || robot_y;
        }
        output.linear.x = 0;
        output.linear.y = 0;
        output.linear.z = 0;

        output.angular.x = 0;
        output.angular.y = 0;
        output.angular.z = 0;
        controllerOutput.publish(output);
    }
}