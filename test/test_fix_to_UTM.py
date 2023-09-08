import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from pyproj import Transformer
import numpy as np

class FixtoUtmNode(Node):
    def __init__(self):
        super().__init__('fixtoutm')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.fix_callback,
            10)
        
        self.positions = np.empty((0,3))
        self.init_east = 0
        self.init_north = 0
        self.init_alt = 0
        self.isinit = True
        

    def fix_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude

        utm_e, utm_n = self.lat_lon_to_utm(latitude, longitude)

        if (self.isinit == True):
            self.init_east = utm_e
            self.init_north = utm_n
            self.init_alt = altitude
            self.isinit=False

        diff_e = utm_e-self.init_east
        diff_n = utm_n-self.init_north
        diff_a = altitude-self.init_alt

        print('utm_e : ', utm_e, ', diff_n : ', diff_n, ', alt : ', altitude)
        print('diff_e : ', diff_e, ', diff_n : ', diff_n, ', diff_a : ', diff_a)
        self.positions = np.vstack((self.positions, np.array([diff_e,
                                                              diff_n,
                                                              diff_a])))

    def lat_lon_to_utm(self, latitude, longitude):
        transformer = Transformer.from_crs('EPSG:4326', 'EPSG:32752')
        utm_easting, utm_northing = transformer.transform(latitude, longitude)
        return utm_easting, utm_northing

def main(args=None):
    rclpy.init(args=args)
    fixtutm = FixtoUtmNode()

    try:
        rclpy.spin(fixtutm)
    except KeyboardInterrupt:
        pass

    np.save('UTM_position', fixtutm.positions)

    fixtutm.destroy_node()


if __name__ == '__main__':
    main()
