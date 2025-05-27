#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_ship_visualization.srv import PlotCharacteristicCurve
from matplotlib import pyplot as plt
import rospkg
import numpy as np

class PlotCharacteristicCurveServer(Node):
    def __init__(self):
        super().__init__('plot_characteristic_curve_server')
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('ros_ship_visualization')
        self.srv = self.create_service(PlotCharacteristicCurve, '/plot_characteristic_curve', self.plot)

    def plot(self, request, response):
        file_path = self.package_path + "/data/" + request.file_name
        self.set_param(request.fluid_density, request.turning_radius, request.k0, request.k1, request.k2)
        inflow_rates = np.arange(request.min_inflow_rate, request.max_inflow_rate, request.resolution_inflow_rate)
        rotational_speeds = np.arange(request.min_rotational_speed, request.max_rotational_speed, request.resolution_rotational_speed)
        for inflow_rate in inflow_rates:
            thrusts = []
            for rotational_speed in rotational_speeds:
                thrusts.append(self.get_thrust(rotational_speed, inflow_rate))
            plt.plot(rotational_speeds, thrusts, label=("ship speed ="+str(inflow_rate)+"[m/s]"))
        plt.legend(loc='upper left')
        plt.xlabel("rotational speed [rad/s]")
        plt.ylabel("thrust [N]")
        plt.grid()
        plt.savefig(file_path+".jpg")
        plt.savefig(file_path+".eps")
        plt.close()
        self.get_logger().info(f"save characteristic curve server to {file_path}.jpg and .eps")

        return response  # ROS2 服务回调返回response对象

    def set_param(self, fluid_density, turning_radius, k0, k1, k2):
        self.fluid_density = fluid_density
        self.turning_radius = turning_radius
        self.k0 = k0
        self.k1 = k1
        self.k2 = k2

    def get_thrust(self, rotational_speed, inflow_rate):
        if rotational_speed == 0:
            return 0
        Js = inflow_rate / rotational_speed * self.turning_radius
        Kt = self.k2 * Js * Js + self.k1 * Js + self.k0
        if rotational_speed > 0:
            thrust = self.fluid_density * rotational_speed**2 * self.turning_radius**4 * Kt
        else:
            thrust = -self.fluid_density * rotational_speed**2 * self.turning_radius**4 * Kt
        return thrust

def main(args=None):
    rclpy.init(args=args)
    node = PlotCharacteristicCurveServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
