#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>


class MPCControllerNode {
private:

    ///* Degree of the polynomial fit to the waypoints
    static constexpr int POLY_DEGREE = 3;

    ///* Time variables used to "benchmark" the callbacks and to keep track of
    ///* how long has it been since the last contact with the master control
    ros::Time time_, old_time_;
    double last_stop_msg_ts;

    ///* ROS-related
    ros::NodeHandle nh;

    ///* Publisher for the steering angle, throttle, and for publishing predicted
    ///* future positions
    ros::Publisher pub__mpc_angle;
    ros::Publisher pub__mpc_throttle;
    ros::Publisher pub__mpc_next_pos;

    ///* Subscribers for the readings of the waypoints,
    ros::Subscriber sub__centerline_numpy;
    ros::Subscriber sub__pf_pose_odom;
    ros::Subscriber sub__odom;


public:
    ///* TODO: pass also logger as argument
    MPCControllerNode(ros::NodeHandle * nodehandle, int steps_ahead, double dt);

    ///* Callbacks
    /**
     * Callback for collecting Marker data that contains the waypoints
     * @param data Marker containing waypoints for the MPC to follow
     */
    void centerline_cb(const visualization_msg::Marker & data);

    /**
     * Callback for receiving the: position, orientation, and calculating the speed
     * (although there is a better source of speed information, namely: the VESC)
     * @param data Odometry (coming from the Particle Filter)
     */
    void pf_pose_odom_cb(const nav_msgs::Odometry & data);

    /**
     * Callback for receiving odometry readings from the VESC
     * @param data Odometry (coming from the Particle Filter)
     */
    void odom_cb(const nav_msgs::Odometry & data);
};
