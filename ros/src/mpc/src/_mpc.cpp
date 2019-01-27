// THIS IS ACTUALLY A TEMPLATE -- I WILL USE IT TO CREATE AN MPC NODE IN CPP
#include <vector>
#include <math.h> /* floor, abs */
#include <assert.h>
#include <algorithm> /* min, min_element, max_element */

#include "ros/ros.h"
#include "ros/console.h"
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>

#include "_mpc.h"


MPCController::MPCController(double target_speed, int steps_ahead, double dt) {
    this->target_speed = target_speed;
    this->steps_ahead = steps_ahead;
    this->dt = dt;
}


void MPCController::control(const std_msgs::UInt16 &data) {
    last_stop_msg_ts = ros::Time::now().toSec();
    if (data.data == 0) {
        ROS_WARN("Emergency stop!");
        exec_estop();
    } else if (data.data == 2309) {
        ROS_WARN("GO!");
        estart = true;
    }
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "ultimate_racer_in_cpp4");

    ros::NodeHandle nh;
    int slow_esc;
    int medium_esc;
    int fast_esc;
    float drive_medium_thr;
    float drive_fast_thr;
    int drive_fast_angle_margin;
    float yaw_multiplier;
    int num_points_below_sod3_to_stop;
    float karols_modifier;
    float margin_drive_slow;
    float margin_drive_fast;
    int scan_range_deleted;

    if (argc == 13) {
        slow_esc = atoi(argv[1]);
        medium_esc = atoi(argv[2]);
        fast_esc = atoi(argv[3]);
        drive_medium_thr = atof(argv[4]);
        drive_fast_thr = atof(argv[5]);
        drive_fast_angle_margin = atoi(argv[6]);
        yaw_multiplier = atof(argv[7]);
        num_points_below_sod3_to_stop = atoi(argv[8]);
        karols_modifier = atof(argv[9]);
        margin_drive_slow = atof(argv[10]);
        margin_drive_fast = atof(argv[11]);
        scan_range_deleted = atoi(argv[12]);
    } else {
        slow_esc = 1555;
        medium_esc = 1560;
        fast_esc = 1565;
        drive_medium_thr = 3.5;
        drive_fast_thr = 5.0;
        drive_fast_angle_margin = 60;
        yaw_multiplier = 1.2;
        num_points_below_sod3_to_stop = 20;
        karols_modifier = 50;
        margin_drive_slow = 0.3;
        margin_drive_fast = 0.5;
        scan_range_deleted = 170;
    }

    std::cout << "slow_esc: " << slow_esc
              << " medium_esc: " << medium_esc
              << " fast_esc: " << fast_esc
              << " drive_medium_thr: " << drive_medium_thr
              << " drive_fast_thr: " << drive_fast_thr
              << " drive_fast_angle_margin: " << drive_fast_angle_margin
              << " yaw_multiplier: " << yaw_multiplier
              << " num_points_below_sod3_to_stop: " << num_points_below_sod3_to_stop
              << " karols_modifier: " << karols_modifier
              << " margin_drive_slow: " << margin_drive_slow
              << " margin_drive_fast: " << margin_drive_fast
              << " scan_range_deleted: " << scan_range_deleted
              << std::endl;

    MPCControllerNode racer(
            &nh,
            slow_esc, medium_esc, fast_esc,
            drive_medium_thr, drive_fast_thr,
            drive_fast_angle_margin,
            yaw_multiplier,
            num_points_below_sod3_to_stop,
            karols_modifier,
            margin_drive_slow, margin_drive_fast,
            scan_range_deleted
    );

    ros::Rate loop_rate(40);

    ros::spin();

    return 0;
}
