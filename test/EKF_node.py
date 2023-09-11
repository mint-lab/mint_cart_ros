import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise
import transforms3d.quaternions as quat
from tf_transformations import euler_from_quaternion

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from numpy.linalg import inv
from scipy.spatial.transform import Rotation
from pyproj import Transformer

class EKF_node(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.fix_callback,
            10)
        
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        self.pub = self.create_publisher(TFMessage, '/tf', 10)

        self.positions = np.empty((0,3))
        self.init_east = 0
        self.init_north = 0
        self.init_alt = 0
        self.relative_matrix = 0
        self.is_fix_init = True
        self.is_imu_init = True

        self.init_quart = np.zeros(4)

        self.localizer = ExtendedKalmanFilter(dim_x=7, dim_z=7)
        self.localizer.F = np.eye(7)
        self.localizer.Q = np.eye(7)*0.01
        self.h = lambda x: x
        self.H = lambda x: np.eye(7)
        self.R = 1 * 1 * np.eye(7)

        self.x = 0
        self.y = 0
        self.z = 0

    def fix_callback(self, data):
        if (self.is_imu_init == False):
            latitude = data.latitude
            longitude = data.longitude
            altitude = data.altitude

            utm_e, utm_n = self.lat_lon_to_utm(latitude, longitude)

            if (self.is_fix_init == True):
                self.init_east = utm_e
                self.init_north = utm_n
                self.init_alt = altitude
                self.is_fix_init=False

            diff_e = -(utm_e-self.init_east)
            diff_n = utm_n-self.init_north
            diff_a = altitude-self.init_alt

            pos_array = np.array([diff_n, diff_e, diff_a])
            filtered_position = inv(self.relative_matrix) @ pos_array
            print(filtered_position)
            self.x = filtered_position[0]
            self.y = filtered_position[1]
            self.z = filtered_position[2]

    def lat_lon_to_utm(self, latitude, longitude):
        transformer = Transformer.from_crs('EPSG:4326', 'EPSG:32752')
        utm_easting, utm_northing = transformer.transform(latitude, longitude)
        return utm_easting, utm_northing

    def imu_callback(self, data):
        q_x = data.orientation.x
        q_y = data.orientation.y
        q_z = data.orientation.z
        q_w = data.orientation.w

        if (self.is_imu_init == True):

            self.init_quart = np.array([q_w, q_x, q_y, q_z])

            self.relative_matrix = Rotation.from_quat([q_x, q_y, q_z, q_w]).as_matrix()
            print(self.relative_matrix)
            self.is_imu_init=False

        current_q = np.array([q_w, q_x, q_y, q_z])

        delta_q = quat.qmult(quat.qinverse(self.init_quart), current_q)

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


def main():
    rclpy.init()
    node = EKF_node()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
