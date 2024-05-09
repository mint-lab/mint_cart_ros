import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
import csv

class BarometerDataLogger(Node):
    def __init__(self):
        super().__init__('barometer_data_logger')
        self.subscription = self.create_subscription(
            FluidPressure,
            '/zed2/zed_node/atm_press',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.csv_file = open('barometer_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Timestamp', 'Pressure', 'Variance'])

    def listener_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        fluid_pressure = msg.fluid_pressure
        variance = msg.variance
        self.get_logger().info(f'Received pressure: {fluid_pressure}, variance: {variance}')
        self.csv_writer.writerow([timestamp, fluid_pressure, variance])

    def stop_logging(self):
        self.csv_file.close()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    barometer_logger = BarometerDataLogger()
    try:
        rclpy.spin(barometer_logger)
    except KeyboardInterrupt:
        print(f'Stopping the [{barometer_logger.get_name()}] node.')
        barometer_logger.stop_logging()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
