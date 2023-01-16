#! /usr/bin/env python3

import rclpy
import yaml
import os
from ament_index_python import get_package_share_directory
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Vector3

UPDATE_PERIOD = 1
CONFIG_FILE = os.path.join(get_package_share_directory("riptide_rviz"), "config", "markers.yaml")

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__("MarkerPublisher")
        self.get_logger().info("Marker Publisher node started.")
        self.timer = self.create_timer(UPDATE_PERIOD, self.timerCB)
        
        stream = open(CONFIG_FILE, 'r')
        docRoot = yaml.safe_load(stream)["marker_publisher"]
        self.meshPkg = docRoot["mesh_pkg"]
        self.meshDir = docRoot["mesh_directory"]
        self.markerTopic = docRoot["marker_topic"]
        self.markers = docRoot["markers"]
        self.markerNames = list(self.markers.keys()) #so swe can iterate through the list with numerical ids
        
        self.markerPub = self.create_publisher(MarkerArray, self.markerTopic, 10)
        
        
    def timerCB(self):
        array = MarkerArray()
        for i in range(0, len(self.markerNames)):
            propName = self.markerNames[i]
            frame = self.markers[propName]["frame"]
            mesh = self.markers[propName]["mesh"]
            
            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "mapping_markers"
            marker.id = i
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.MODIFY
            marker.scale = Vector3(x=1.0, y=1.0, z=1.0)
            marker.lifetime = Duration().to_msg() #forever
            marker.frame_locked = True
            marker.mesh_resource = "file://" + os.path.join(get_package_share_directory(self.meshPkg), self.meshDir, mesh, "model.dae")
            marker.mesh_use_embedded_materials = True
            
            array.markers.append(marker)
        
        self.markerPub.publish(array)


def main(args = None):
    rclpy.init(args = args)
    rclpy.spin(MarkerPublisher())
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Marker Pub Keyboard-Interrupted")
