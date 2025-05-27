#!/usr/bin/env python3
import rospkg
import requests

def download_map(request_url):
    rospack = rospkg.RosPack()
    map_image_path = rospack.get_path('ros2_ship_visualization') + "/data/map.png"
    try:
        with open(map_image_path, 'wb') as f:
            f.write(requests.get(request_url).content)
        return 0
    except Exception as e:
        print(f"Failed to download map image: {e}")
        return -1

if __name__ == '__main__':
    # ROS2中一般不写全局执行逻辑，留空或做简单测试
    pass
