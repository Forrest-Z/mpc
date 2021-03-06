#pragma once

#include <vector>
#include <math.h> /* floor */
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

#include "Eigen-3.3/Eigen/Core"
#include "MPC.h"


class MPCControllerNode {
private:

    ///* Time variables used to "benchmark" the callbacks and to keep track of
    ///* how long has it been since the last contact with the master control
    ros::Time m_time;
    ros::Time m_old_time;
    double m_last_stop_msg_ts;

    ///* ROS-related
    ros::NodeHandle m_nodehandle;

    ///* Publisher for the ESC, and the second one: for the servo
    ros::Publisher m_pub_commands_servo_position;
    ros::Publisher m_pub_commands_motor_speed;
    // Publishers for debugging
    ros::Publisher m_pub_next_pos;
    ros::Publisher m_pub_poly;
    ros::Publisher m_pub_closest;

    ///* Subscribers for the readings of the lidar, and the second one: for an emergency stop signal
    ros::Subscriber m_sub_centerline;
    ros::Subscriber m_sub_odom;
    ros::Subscriber m_sub_pf_pose_odom;
    ros::Subscriber m_sub_signal_go;

    ///* The Model Predictive controller
    MPC m_controller;

    ///* Callbacks
    void centerline_cb(const visualization_msgs::Marker & data);

    void odom_cb(const nav_msgs::Odometry & data);

    void pf_pose_odom_cb(const nav_msgs::Odometry & data);

    void signal_go_cb(const std_msgs::UInt16 & data);

    ///* Other methods
    visualization_msgs::Marker get_marker(const std::vector<double> & vars, double px_lat, double py_lat, double sin_psi_lat, double cos_psi_lat, float red, float green, float blue);

    int find_closest(const std::vector<double> & pts_x, const std::vector<double> & pts_y, double pos_x, double pos_y);

    ///* Non-ROS members
    std::vector<double> m_pts_x;
    std::vector<double> m_pts_y;
    bool m_pts_OK;

    double m_pos_x;
    double m_pos_y;
    bool m_pos_OK;

    double m_speed;
    bool m_speed_OK;

    double m_psi;
    bool m_psi_OK;

    double m_steer;
    double m_rpm;

    double m_ref_v;
    double m_ref_v_alpha;

    size_t m_poly_degree;
    size_t m_num_steps_poly;

    ///* Other member attributes
    bool m_debug;
    bool m_go_flag;
    double m_latency;

    ///* When fitting a degree=3 polynomial to the waypoints we're using
    ///* (STEPS_POLY * 3) points ahead to fit it (impacts smoothness)
    static constexpr int STEP_POLY = 1;

    ///* If you see: "coeffs: nan   nan   nan   nan" in the logger,
    ///* it means that polyfit was unable to fit a polynomial which
    ///* may be caused by the `X_DELTA_MIN_VALUE` being too low
    static constexpr double X_DELTA_MIN_VALUE = 0.001;

    static constexpr int NUM_STEPS_BACK = 5;

    ///* The value of the steering angle that means "go straight" in Dzik
    static constexpr double CENTER_IN_DZIK = 0.56;
    ///* Dzik's wheel radius
    static constexpr double WHEEL_RADIUS_IN_DZIK = 0.055; // [m]


public:
    MPCControllerNode(const ros::NodeHandle & nodehandle, const Params & params);

    void loop();
};
