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

#include "mpc.h"




MPCControllerNode::MPCControllerNode(ros::NodeHandle * nodehandle, int steps_ahead, double dt){

    nh = *nodehandle;
    this->steps_ahead = slow_esc;
    this->dt = dt;

    old_time_ = ros::Time::now();
    last_stop_msg_ts = ros::Time::now().toSec();

    pub_esc = nh.advertise<std_msgs::UInt16>(
            "/esc",
            1
    );

    pub_servo = nh.advertise<std_msgs::UInt16>(
            "/servo",
            1
    );

    sub_scan = nh.subscribe(
            "/scan",
            1,
            &UltimateRacer::scan_cb,
            this,
            ros::TransportHints().tcpNoDelay()
    );

    sub_estop = nh.subscribe(
            "/eStop",
            10,
            &UltimateRacer::estop_cb,
            this
    );
}



void UltimateRacer::estop_cb(const std_msgs::UInt16 &data) {
    last_stop_msg_ts = ros::Time::now().toSec();
    if (data.data == 0) {
        ROS_WARN("Emergency stop!");
        exec_estop();
    } else if (data.data == 2309) {
        ROS_WARN("GO!");
        estart = true;
    }
}


void UltimateRacer::exec_estop() {
    estop = true;
    yaw = YAW_MID;
    throttle = ESC_BRAKE;
    tmp_uint16.data = throttle;
    pub_esc.publish(tmp_uint16);
    tmp_uint16.data = yaw;
    pub_servo.publish(tmp_uint16);
}


float UltimateRacer::steerMAX(std::vector<float> &scan, float width_margin) const {
    float min_scan = *std::min_element(scan.begin(), scan.end());
    if (min_scan <= SAFE_OBSTACLE_DIST2)
        return -1;

    // The following actually copies the vector `scan`
    std::vector<float> scan2(scan);
    int idx = -1;
    bool is_reachable = false;
    int scan_size = scan.size();

    // First, prepare `segs`
    std::vector<int> segs = {180, scan_size - 180};
    for (auto i = 1; i < scan_size; i++)
        if (std::abs(scan[i] - scan[i - 1]) > NON_CONT_DIST) {
            segs.push_back(i);
            segs.push_back(i - 1);
        }

    // We're disregarding outskirts of the scan so that the car won't go backwards
    // (this happened when during a U-turn the car chose to go back because the
    // scan readings were higher than those down the track)
    for (auto i = 0; i < scan_range_deleted; i++)
        scan2[i] = -1;
    for (auto i = scan2.size() - scan_range_deleted; i < scan2.size(); i++)
        scan2[i] = -1;

    while (!is_reachable) {
        float max_value = *std::max_element(scan2.begin(), scan2.end());
        if (max_value <= 0)
            break;

        // Search for argmax (https://en.cppreference.com/w/cpp/algorithm/max_element)
        idx = std::distance(
                scan2.begin(),
                std::max_element(scan2.begin(), scan2.end())
        );

        for (auto s : segs) {
            if (s != idx) {
                bool could_be_reached = check_if_reachable(
                        scan[idx],
                        scan[s],
                        std::abs(s - idx),
                        width_margin
                );

                if (!could_be_reached) {
                    int left_limit = std::max(0, idx - 5);
                    int right_limit = std::min(idx + 5, int(scan2.size() - 1));
                    for (int i = left_limit; i < right_limit; i++)
                        scan2[i] = -1;
                    break;
                }
            }
        }
        // Instead of comparing to -1 (remember, we're dealing with floats),
        //  we check whether it's non-negative
        if (scan2[idx] >= 0)
            is_reachable = true;
    }

    if (is_reachable == false)
        idx = -1;

    return idx;
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "MPC_node_in_cpp");

    ros::NodeHandle nh;
    int steps_ahead;
    double dt;

    if (argc == 13) {
        steps_ahead = atoi(argv[1]);
        dt = atoi(argv[2]);
    } else {
        cout << "No arguments passed or passed incorrectly, using defaults\n";
        steps_ahead = 10;
        dt = 0.1;
    }

    std::cout << "steps_ahead: " << steps_ahead
              << " dt: " << medium_esc
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

    UltimateRacer racer(
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
