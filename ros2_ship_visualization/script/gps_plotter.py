#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Image
from std_msgs.msg import String
import rospkg
import requests
import cv2
from cv_bridge import CvBridge

class Plotter(Node):
    def __init__(self):
        super().__init__('gps_plotter_node')

        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('ros_ship_visualization')

        # 声明参数，提供默认值
        self.declare_parameter("google_statc_map_api_key", "")
        self.declare_parameter("maptype", "roadmap")
        self.declare_parameter("zoom", 14)
        self.declare_parameter("size/x", 640)
        self.declare_parameter("size/y", 480)
        self.declare_parameter("wam_v_marker_color", "blue")
        self.declare_parameter("start_point_marker_color", "green")
        self.declare_parameter("messages_per_plot", 10)
        self.declare_parameter("path_color", "0xff0000ff")

        self.google_statc_map_api_key = self.get_parameter("google_statc_map_api_key").get_parameter_value().string_value
        self.maptype = self.get_parameter("maptype").get_parameter_value().string_value
        self.zoom = self.get_parameter("zoom").get_parameter_value().integer_value
        self.size_x = self.get_parameter("size/x").get_parameter_value().integer_value
        self.size_y = self.get_parameter("size/y").get_parameter_value().integer_value
        self.wam_v_marker_color = self.get_parameter("wam_v_marker_color").get_parameter_value().string_value
        self.start_point_marker_color = self.get_parameter("start_point_marker_color").get_parameter_value().string_value
        self.messages_per_plot = self.get_parameter("messages_per_plot").get_parameter_value().integer_value
        self.path_color = self.get_parameter("path_color").get_parameter_value().string_value

        self.latitudes = []
        self.longitudes = []
        self.gps_seq = 0
        self.map_image_file = self.package_path + "/data/map_image.png"
        self.bridge = CvBridge()

        # 创建发布者
        self.image_pub = self.create_publisher(Image, self.get_name() + "/map_image", 1)
        self.google_statc_map_api_request_pub = self.create_publisher(String, self.get_name() + "/google_statc_map_api_request", 1)

        # 创建订阅者
        self.fix_sub = self.create_subscription(NavSatFix, '/fix', self.fix_callback, 10)

    def fix_callback(self, data):
        if self.gps_seq % self.messages_per_plot == 0:
            self.latitudes.append(data.latitude)
            self.longitudes.append(data.longitude)
            self.build_request()
        self.gps_seq += 1

    def build_request(self):
        url = "https://maps.googleapis.com/maps/api/staticmap?"
        center_request = "&center=" + str(self.longitudes[-1]) + "," + str(self.latitudes[-1])
        zoom_request = "&zoom=" + str(self.zoom)
        size_request = "&size=" + str(self.size_x) + "x" + str(self.size_y)
        maptype_request = "&maptype=" + self.maptype
        wam_v_marker_request = "&markers=color:" + self.wam_v_marker_color + "%7Clabel:C%7C" + str(self.longitudes[-1]) + "," + str(self.latitudes[-1])
        start_point_marker_request = "&markers=color:" + self.start_point_marker_color + "%7Clabel:S%7C" + str(self.longitudes[0]) + "," + str(self.latitudes[0])
        path_request = self.build_path_request()
        api_key_request = "&key=" + self.google_statc_map_api_key
        request_url = url + center_request + zoom_request + size_request + maptype_request + path_request + start_point_marker_request + wam_v_marker_request + api_key_request

        request_msg = String()
        request_msg.data = request_url
        self.google_statc_map_api_request_pub.publish(request_msg)

        with open(self.map_image_file, 'wb') as f:
            f.write(requests.get(request_url).content)

        self.publish_map_image()

    def build_path_request(self):
        num_points = len(self.longitudes)
        path_request = "&path=color:" + self.path_color + "|weight:5"
        for i in range(num_points):
            path_request += "|" + str(self.longitudes[i]) + "," + str(self.latitudes[i])
        return path_request

    def publish_map_image(self):
        map_image = cv2.imread(self.map_image_file)
        if map_image is not None:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(map_image, "bgr8"))
        else:
            self.get_logger().error("Failed to read map image file: " + self.map_image_file)

def main(args=None):
    rclpy.init(args=args)
    gps_plotter = Plotter()
    rclpy.spin(gps_plotter)
    gps_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
