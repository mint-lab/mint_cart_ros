import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from pyproj import Transformer
from sensor_msgs.msg import Imu
import numpy as np
from tf_transformations import euler_from_quaternion
from numpy.linalg import inv
from scipy.spatial.transform import Rotation

class FixtoUtmNode(Node):
    def __init__(self):
        super().__init__('fixtoutm')
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
        
        self.positions = np.empty((0,3))
        self.init_east = 0
        self.init_north = 0
        self.init_alt = 0
        self.relative_matrix = 0
        self.is_fix_init = True
        self.is_imu_init = True
        

    def fix_callback(self, msg):
        if (self.is_imu_init == False):
            latitude = msg.latitude
            longitude = msg.longitude
            altitude = msg.altitude

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

            # print('utm_e : ', utm_e, ', diff_n : ', diff_n, ', alt : ', altitude)
            # print('diff_e : ', diff_e, ', diff_n : ', diff_n, ', diff_a : ', diff_a)
            self.positions = np.vstack((self.positions, filtered_position))

    def lat_lon_to_utm(self, latitude, longitude):
        transformer = Transformer.from_crs('EPSG:4326', 'EPSG:32752')
        utm_easting, utm_northing = transformer.transform(latitude, longitude)
        return utm_easting, utm_northing
    
    def imu_callback(self, msg):
        if (self.is_imu_init == True):
            q_x = msg.orientation.x
            q_y = msg.orientation.y
            q_z = msg.orientation.z
            q_w = msg.orientation.w

            self.relative_matrix = Rotation.from_quat([q_x, q_y, q_z, q_w]).as_matrix()
            print(self.relative_matrix)
            self.is_imu_init=False

            self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    fixtutm = FixtoUtmNode()

    try:
        rclpy.spin(fixtutm)
    except KeyboardInterrupt:
        pass

    np.save('UTM_position', fixtutm.positions)
    print('Save Complete!')

    fixtutm.destroy_node()


if __name__ == '__main__':
    main()
