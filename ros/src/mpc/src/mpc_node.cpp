#include <math.h> /* floor, abs */
#include <cmath> /* atan2 */

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include "mpc_node.h"
#include "MPC.h"


MPCControllerNode::MPCControllerNode(const ros::NodeHandle & nodehandle, const Params & params)
        : controller(params)
{

    this->nh = nodehandle;
    this->old_time_ = ros::Time::now();
    this->last_stop_msg_ts = ros::Time::now().toSec();

    this->pts_OK = false;
    this->pos_OK = false;
    this->speed_OK = false;
    this->psi_OK = false;

    // Actuators
    this->steer = 0; // TODO: get steering angle from VESC
    this->throttle = 0; // TODO: get throttle from VESC

    // Advertisers
    this->pub_angle = nh.advertise<std_msgs::Float32>(
            "/mpc/angle",
            1
    );
    this->pub_throttle = nh.advertise<std_msgs::Float32>(
            "/mpc/throttle",
            1
    );
    this->pub_next_pos = nh.advertise<visualization_msgs::Marker>(
            "/mpc/next_pos",
            1
    );

    // Subscribers
    this->sub_centerline = nh.subscribe(
            "/centerline",
            1,
            &MPCControllerNode::centerline_cb,
            this
    );
    this->sub_odom = nh.subscribe(
            "/odom",
            1,
            &MPCControllerNode::odom_cb,
            this
    );
    this->sub_pf_pose_odom = nh.subscribe(
            "/pf/pose/odom",
            1,
            &MPCControllerNode::pf_pose_odom_cb,
            this
    );
}


void MPCControllerNode::centerline_cb(const visualization_msgs::Marker & data) {
    int num_points = data.points.size();

    this->pts_x = std::vector<double>();
    this->pts_x.reserve(num_points);

    this->pts_y = std::vector<double>();
    this->pts_y.reserve(num_points);

    for (auto & p : data.points) {
        this->pts_x.push_back(p.x);
        this->pts_y.push_back(p.y);
    }
    this->pts_OK = true;
}


visualization_msgs::Marker MPCControllerNode::get_marker(const std::vector<double> & mpc_xvals, const std::vector<double> & mpc_yvals) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.color.a = 0.7;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;

    marker.lifetime = ros::Duration();

    for (int i=0; mpc_xvals.size(); i++) {
        geometry_msgs::Point p;
        p.x = mpc_xvals[i];
        p.y = mpc_yvals[i];
        p.z = 0.0f;
        marker.points.push_back(p);
    }

    return marker;
}


void MPCControllerNode::odom_cb(const nav_msgs::Odometry & data) {
    this->speed = data.twist.twist.linear.x;
    this->speed_OK = true;
}


void MPCControllerNode::pf_pose_odom_cb(const nav_msgs::Odometry & data) {
    this->pos_x = data.pose.pose.position.x;
    this->pos_y = data.pose.pose.position.y;
    this->pos_OK = true;

    // Calculate the psi Euler angle
    // (source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
    auto o = data.pose.pose.orientation;
    double siny_cosp = 2.0 * (o.w * o.z + o.x * o.y);
    double cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z);
    this->psi = atan2(siny_cosp, cosy_cosp);
    this->psi_OK = true;
}


void MPCControllerNode::loop() {
    while (this->nh.ok()) {
        this->time_ = ros::Time::now();

        if (this->pts_OK and this->speed_OK and this->pos_OK and this->psi_OK) {
            double v_lat = this->speed + this->latency * this->throttle;
            double psi_lat = this->psi - this->latency * (v_lat * this->steer / Lf());
            double pos_x_lat = this->pos_x  + this->latency * (v_lat * cos(psi_lat));
            double pos_y_lat = this->pos_y  + this->latency * (v_lat * sin(psi_lat));

            // Before we get the actuators, we need to calculate points in car's
            // coordinate system; these will be passed later on to polyfit
            Eigen::VectorXd xvals(pts_x.size());
            Eigen::VectorXd yvals(pts_y.size());
            for (size_t i = 0; i < pts_x.size(); i++) {
                double dx = pts_x[i] - pos_x_lat;
                double dy = pts_y[i] - pos_y_lat;

                // Rotation around the origin
                xvals[i] = dx * cos(-psi_lat) - dy * sin(-psi_lat);
                yvals[i] = dx * sin(-psi_lat) + dy * cos(-psi_lat);
            }

            // Here we calculate the fit to the points in *car's coordinate system*
            Eigen::VectorXd coeffs = polyfit(xvals, yvals, 3);

            // Now, we can calculate the cross track error
            double cte = polyeval(coeffs, 0);

            // ... and psi's error
            double epsi = -atan(coeffs[1]);

            // And now we're ready to calculate the actuators using the MPC
            Eigen::VectorXd state(6);
            state << 0, 0, 0, v_lat, cte, epsi;
            auto vars = this->controller.Solve(state, coeffs);

            // Extract the actuator values
            steer = vars[0];
            throttle = vars[1];

            double delta_between_callbacks = 1000 * (this->time_.toSec() - this->old_time_.toSec());
            double delta_within_callback = 1000 * (ros::Time::now().toSec() - this->time_.toSec());
            ROS_WARN(
                    "dt_bet_cb: %.1f[ms] dt_in_cb: %.1f[ms]",
                    delta_between_callbacks, delta_within_callback
            );
        }

        ROS_WARN("No optimization");

        old_time_ = time_;
        ros::spinOnce();
    }
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "mpc_node_cpp");

    Params params;

    int num_expected_args = 11;

    if (argc == num_expected_args ) {
        params.steps_ahead = atoi(argv[1]);
        params.dt = atof(argv[2]);

        params.latency = atof(argv[3]);

        params.cte_coeff = atof(argv[4]);
        params.epsi_coeff = atof(argv[5]);
        params.speed_coeff = atof(argv[6]);
        params.acc_coeff = atof(argv[7]);
        params.steer_coeff = atof(argv[8]);

        params.consec_acc_coeff = atof(argv[9]);
        params.consec_steer_coeff = atof(argv[10]);

    } else if (argc > num_expected_args ) {
        std::cout << "Too many arguments passed to main\n";
    } else {
        std::cout << "Too few arguments passed to main\n";
    }

    std::cout << "steps_ahead: " << params.steps_ahead
              << " dt: " << params.dt
              << " latency: " << params.latency << "[ms]"
              << " cte_coeff: " << params.cte_coeff
              << " epsi_coeff: " << params.epsi_coeff
              << " speed_coeff: " << params.speed_coeff
              << " acc_coeff: " << params.acc_coeff
              << " steer_coeff: " << params.steer_coeff
              << " consec_acc_coeff" << params.consec_acc_coeff
              << " consec_steer_coeff: " << params.consec_steer_coeff
              << std::endl;

    ros::NodeHandle nh;
    MPCControllerNode mpc_node(nh, params);

    ros::Rate loop_rate(100);

    mpc_node.loop();

    return 0;
}
