#!/usr/bin/env python
import numpy as np
import sys
from scipy.linalg import expm

import rospy
from rospy.numpy_msg import numpy_msg
from tf.transformations import (
    euler_from_quaternion,
    unit_vector,
    quaternion_multiply,
    quaternion_conjugate,
)

# Message types
from std_msgs.msg import Float32, Time, UInt16
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rospy_tutorials.msg import Floats

from _pid_controller import PIDController

CENTER_IN_DZIK = 0.56


class PIDControllerNode:
    def __init__(self, debug, pid_controller, common_queue_size=None, which_speed='VESC'):
        self.debug = debug

        self.which_speed = which_speed

        self.go_flag = False

        self.points = None
        self.speed_from_pf = None
        self.speed_from_vesc = None
        self.position = None
        self.psi = None
        self.orient_euler = None

        self.pid_controller = pid_controller

        queue_size = common_queue_size or 1

        # Subscribers
        self.subscribers = {}
        #  This gives us the waypoints
        self.subscribers['/centerline_numpy'] = rospy.Subscriber(
            '/centerline_numpy',
            numpy_msg(Floats),
            self.centerline_cb,
            queue_size=queue_size,
        )
        #  This gives us the position and orientation
        self.subscribers['/pf/pose/odom'] = rospy.Subscriber(
            '/pf/pose/odom',
            Odometry,
            self.pf_pose_odom_cb,
            queue_size=queue_size,
        )
        #  This gives us the speed (from VESC)
        self.subscribers['/odom'] = rospy.Subscriber(
            '/odom',
            Odometry,
            self.odom_cb,
            queue_size=queue_size,
        )
        #  This gives us the signal to go / stop
        self.subscribers['/signal/go'] = rospy.Subscriber(
            '/signal/go',
            UInt16,
            self.signal_go_cb,
            queue_size=queue_size,
        )

        # Publishers
        self.publishers = {}
        #  For publishing the steering angle
        self.publishers['/commands/servo/position'] = rospy.Publisher(
            '/commands/servo/position',
            Float32,
            queue_size=queue_size,
        )

        if self.debug:
            #  For publishing predicted next positions
            self.publishers['/pid/next_pos'] = rospy.Publisher(
                '/pid/next_pos',
                Marker,
                queue_size=queue_size,
            )
            #  For publishing polyeval points
            self.publishers['/pid/poly'] = rospy.Publisher(
                '/pid/poly',
                Marker,
                queue_size=queue_size,
            )

        rospy.init_node('~pid_node_python')

        self.loop()

    def loop(self):
        rate = rospy.Rate(100)
        start_time = 0

        while not rospy.is_shutdown():
            self.speed = {
                'VESC': self.speed_from_vesc,
                'PF': self.speed_from_pf,
            }[self.which_speed]

            points_OK = self.points is not None
            speed_OK = self.speed is not None
            position_OK = self.position is not None
            psi_OK = self.psi is not None

            if points_OK and speed_OK and position_OK and psi_OK:
                pid_results = self.pid_controller.control(self.points, self.speed, self.position, self.psi)

                steer = pid_results['steer']
                steer = CENTER_IN_DZIK - steer

                if steer < 0:
                    steer = 0
                elif steer > 1:
                    steer = 1

                if not self.go_flag:
                    steer = CENTER_IN_DZIK

                self.publishers['/commands/servo/position'].publish(steer)

                if self.debug and pid_results['next_pos'] is not None:
                    next_pos = pid_results['next_pos']
                    rot_fn = lambda v: self.rot_euler(v, self.orient_euler)
                    rotated_next_pos = np.apply_along_axis(rot_fn, 1, next_pos)
                    marker = self.get_marker(rotated_next_pos + self.position, 1.0, 0.0, 0.0)
                    self.publishers['/pid/next_pos'].publish(marker)

                    poly = pid_results['poly']
                    rot_fn = lambda v: self.rot_euler(v, self.orient_euler)
                    rotated_next_pos = np.apply_along_axis(rot_fn, 1, poly)
                    marker = self.get_marker(rotated_next_pos + self.position, 1.0, 1.0, 0.0)
                    self.publishers['/pid/poly'].publish(marker)

                elapsed = rospy.Time.now().to_sec() - start_time
                rospy.loginfo('This took {:.4f}s'.format(elapsed))
                start_time = rospy.Time.now().to_sec()

    @staticmethod
    def get_marker(next_pos, red, green, blue):
        # TODO: an almost identical function is in markers_mode.py
        marker = Marker()
        marker.header.frame_id = '/map'
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # marker color
        marker.color.a = 0.5
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # marker line points
        marker.points = []

        for i in range(next_pos.shape[0]):
            p = Point()
            p.x = next_pos[i, 0]
            p.y = next_pos[i, 1]
            p.z = 0.0
            marker.points.append(p)

        return marker

    @staticmethod
    def rot_euler(v, xyz):
        '''Rotate vector v (or array of vectors) by the euler angles xyz '''
        # https://stackoverflow.com/questions/6802577/python-rotation-of-3d-vector
        v = np.r_[v, 0]
        for theta, axis in zip(xyz, np.eye(3)):
            v = np.dot(np.array(v), expm(np.cross(np.eye(3), axis*-theta)))
        return v[:2]

    ### Callbacks ###
    def signal_go_cb(self, data):
        if data.data == 0:
            self.go_flag = False
        elif data.data == 2309:
            self.go_flag = True

    def centerline_cb(self, data):
        #
        # There are: x, y, yaw, speed in self.points -- I'm only interested in
        #  the first two
        self.points = data.data.reshape(-1, 4)[:, :2]  # TODO: magic number 4

    def pf_pose_odom_cb(self, data):
        position = data.pose.pose.position
        self.position = np.array([position.x, position.y])

        # Psi is a bit more complicated
        orient = data.pose.pose.orientation
        self.orient_euler = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        self.psi = self.orient_euler[-1]  # The last, third Euler angle is psi

        # FIXME
        self.speed_from_pf = 0.5  # TODO: consult with Karol how to determine speed

    def odom_cb(self, data):
        self.speed_from_vesc = data.twist.twist.linear.x



if __name__ == '__main__':
    num_arguments = 10
    if len(sys.argv) < num_arguments + 1:
        print('Need {} arguments to run the PID node'.format(num_arguments))
        sys.exit()

    debug = True if sys.argv[1] == 'True' else False

    Kp_cte = float(sys.argv[2])
    Ki_cte = float(sys.argv[3])
    Kd_cte = float(sys.argv[4])

    Kp_ePsi = float(sys.argv[5])
    Ki_ePsi = float(sys.argv[6])
    Kd_ePsi = float(sys.argv[7])

    Lf = float(sys.argv[8])

    poly_degree = int(sys.argv[9])
    poly_steps = int(sys.argv[10])

    pid_controller = PIDController(
        debug,
        Kp_cte, Ki_cte, Kd_cte,
        Kp_ePsi, Ki_ePsi, Kd_ePsi,
        Lf,
        poly_degree, poly_steps,
        rospy.loginfo
    )

    try:
        PIDControllerNode(debug, pid_controller)
    except rospy.ROSInterruptException:
        rospy.logerr('PIDControllerNode failed')
