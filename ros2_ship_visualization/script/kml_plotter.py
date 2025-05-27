#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import simplekml
from sensor_msgs.msg import NavSatFix
import rospkg

class Plotter(Node):
    def __init__(self):
        super().__init__('kml_plotter_node')
        self.kml = simplekml.Kml()
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('ros_ship_visualization')
        self.is_start_time = True
        self.last_recieved_data = None
        self.fix_sub = self.create_subscription(NavSatFix, '/fix', self.fix_callback, 10)

    def fix_callback(self, data):
        self.last_recieved_data = data
        # simplekml的coords顺序是 (经度, 纬度)
        self.kml.newpoint(name="", coords=[(data.longitude, data.latitude)])
        if self.is_start_time:
            self.kml.newpoint(name="start point", coords=[(data.longitude, data.latitude)])
            self.is_start_time = False

    def save(self):
        if self.last_recieved_data is not None:
            self.kml.newpoint(name="finished point", coords=[(self.last_recieved_data.longitude, self.last_recieved_data.latitude)])
            save_path = self.package_path + "/data/gps_log.kml"
            self.kml.save(save_path)
            self.get_logger().info(f"gps log was written in {save_path}")
        else:
            self.get_logger().warn("No GPS data received, nothing to save.")

def main(args=None):
    rclpy.init(args=args)
    kml_plotter = Plotter()
    try:
        rclpy.spin(kml_plotter)
    except KeyboardInterrupt:
        pass
    finally:
        kml_plotter.save()
        kml_plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
