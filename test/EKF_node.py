import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
import transforms3d.quaternions as quat

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class EKF_node(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.zed_odom_sub = self.create_subscription(Odometry, '/zed2i/zed_node/odom', self.odom_callback, 10) # this odom will be changed by gps_odom
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.pub = self.create_publisher(TFMessage, '/tf', 10)

        self.x = 0
        self.y = 0
        self.z = 0
        self.ox = 0
        self.oy = 0
        self.oz = 0
        self.ow = 0

        self.is_init = True
        self.init_q = np.zeros((4,))
        self.current_q = np.zeros((4,))

        self.localizer = ExtendedKalmanFilter(dim_x=7, dim_z=7)
        self.localizer.F = np.eye(7)
        self.localizer.Q = np.eye(7)*0.01
        self.h = lambda x: x
        self.H = lambda x: np.eye(7)
        self.R = 1 * 1 * np.eye(7)

    def imu_callback(self, data):
        if (self.is_init==True):
            self.init_q[0] = data.orientation.w
            self.init_q[1] = data.orientation.x
            self.init_q[2] = data.orientation.y
            self.init_q[3] = data.orientation.z
            self.is_init = False

        self.current_q[0] = data.orientation.w
        self.current_q[1] = data.orientation.x
        self.current_q[2] = data.orientation.y
        self.current_q[3] = data.orientation.z

        delta_q = quat.qmult(quat.qinverse(self.init_q), self.current_q)

        print('current_q : ', self.current_q)
        print('delta_q : ', delta_q, '\n')
        self.ox = delta_q[1]
        self.oy = delta_q[2]
        self.oz = delta_q[3]
        self.ow = delta_q[0]

        z = np.array([[self.x, self.y, self.z, self.ox, self.oy, self.oz, self.ow]]).T
        self.localizer.update(z, HJacobian=self.H, Hx=self.h)
        print(self.localizer.x)

        msg = TransformStamped()
        msg.header.stamp = data.header.stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.transform.rotation.x = float(self.localizer.x[3])
        msg.transform.rotation.y = float(self.localizer.x[4])
        msg.transform.rotation.z = float(self.localizer.x[5])
        msg.transform.rotation.w = float(self.localizer.x[6])
        msg.transform.translation.x = float(self.localizer.x[0])
        msg.transform.translation.y = float(self.localizer.x[1])
        msg.transform.translation.z = float(self.localizer.x[2])

        tf_msg = TFMessage()
        tf_msg.transforms = [msg]
        self.pub.publish(tf_msg)
        
        self.localizer.predict()


    def odom_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z

def main():
    rclpy.init()
    node = EKF_node()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
