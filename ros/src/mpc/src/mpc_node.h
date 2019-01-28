#pragma once

#include <vector>
#include <math.h> /* floor */
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

#include "Eigen-3.3/Eigen/Core"
#include "MPC.h"


class MPCControllerNode {
private:

    ///* Time variables used to "benchmark" the callbacks and to keep track of
    // how long has it been since the last contact with the master control
    ros::Time time_, old_time_;
    double last_stop_msg_ts;

    ///* ROS-related
    ros::NodeHandle nh;

    ///* Publisher for the ESC, and the second one: for the servo
    ros::Publisher pub_angle;
    ros::Publisher pub_throttle;
    ros::Publisher pub_next_pos;

    ///* Subscribers for the readings of the lidar, and the second one: for an emergency stop signal
    ros::Subscriber sub_centerline;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_pf_pose_odom;

    ///* The Model Predictive controller
    MPC controller;

    ///* Callbacks
    void centerline_cb(const visualization_msgs::Marker & data);

    void odom_cb(const nav_msgs::Odometry & data);

    void pf_pose_odom_cb(const nav_msgs::Odometry & data);

    ///* Other methods
    visualization_msgs::Marker get_marker(const std::vector<double> & mpc_xvals, const std::vector<double> & mpc_yvals);

    ///* Non-ROS members
    std::vector<double> pts_x;
    std::vector<double> pts_y;
    bool pts_OK;

    double pos_x;
    double pos_y;
    bool pos_OK;

    double speed;
    bool speed_OK;

    double psi;
    bool psi_OK;


    // TODO(MD): declare as NaNs and then use `isnan`
    double steer;
    double throttle;

    ///* Other member attributes
    double latency;


public:
    MPCControllerNode(const ros::NodeHandle & nodehandle, const Params & p);

    void loop();

};
