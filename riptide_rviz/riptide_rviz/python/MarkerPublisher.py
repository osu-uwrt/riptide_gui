#! /usr/bin/env python3

import os

import rclpy
import yaml
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from riptide_msgs2.msg import ControllerCommand
from std_msgs.msg import ColorRGBA
from transforms3d.euler import euler2quat
from visualization_msgs.msg import Marker, MarkerArray

UPDATE_PERIOD = 0.25
CONFIG_FILE = os.path.join(get_package_share_directory("riptide_rviz"), "config", "markers.yaml")
LINEAR_CMD_TOPIC = "controller/linear"
ANGULAR_CMD_TOPIC = "controller/angular"


def toPoint(v: Vector3) -> Point:
    p = Point(
        x = v.x,
        y = v.y,
        z = v.z
    )
    
    return p

def poseFromArr(d) -> Pose:
    (w, x, y, z) = euler2quat(d[3], d[4], d[5])
    p = Pose(
        position = Point(
            x = float(d[0]),
            y = float(d[1]),
            z = float(d[2])
        ),
        orientation = Quaternion(
            w = w,
            x = x,
            y = y,
            z = z
        )
    )
    
    return p
    

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__("MarkerPublisher")
        self.get_logger().info("Marker Publisher node started.")
        self.timer = self.create_timer(UPDATE_PERIOD, self.timerCB)
        
        #configure yaml vars
        stream = open(CONFIG_FILE, 'r')
        docRoot = yaml.safe_load(stream)["marker_publisher"]
        self.meshPkg = docRoot["mesh_pkg"]
        self.meshDir = docRoot["mesh_directory"]
        self.markerTopic = docRoot["marker_topic"]
        self.markers = docRoot["markers"]
        self.markerNames = list(self.markers.keys()) #so swe can iterate through the list with numerical ids
        
        #configure ghost tempest vars
        self.latestLinearCmd = ControllerCommand()
        self.latestAngularCmd = ControllerCommand()
        self.latestOdom = Odometry()
        
        #configure ros pub subs
        self.markerPub = self.create_publisher(MarkerArray, self.markerTopic, 10)
        self.linearSub = self.create_subscription(ControllerCommand, LINEAR_CMD_TOPIC, self.linearCB, 10)
        self.angularSub = self.create_subscription(ControllerCommand, ANGULAR_CMD_TOPIC, self.angularCB, 10)
        self.odomSub = self.create_subscription(Odometry, "odometry/filtered", self.odomCB, 10)
        
        
    def linearCB(self, msg):
        self.latestLinearCmd = msg
        
        
    def angularCB(self, msg):
        self.latestAngularCmd = msg
        
        
    def odomCB(self, msg):
        self.latestOdom = msg
        
        
    def timerCB(self):
        array = MarkerArray()
        for i in range(0, len(self.markerNames)):
            propName = self.markerNames[i]
            frame = self.markers[propName]["frame"]
            mesh = self.markers[propName]["mesh"]
            pose = self.markers[propName]["pose"]
            scale = float(self.markers[propName]["scale"])
            
            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "mapping_markers"
            marker.id = i
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.MODIFY
            marker.pose = poseFromArr(pose)
            marker.scale = Vector3(x=scale, y=scale, z=scale)
            marker.lifetime = Duration().to_msg() #forever
            marker.frame_locked = True
            marker.mesh_resource = "file://" + os.path.join(get_package_share_directory(self.meshPkg), self.meshDir, mesh, "model.dae")
            marker.mesh_use_embedded_materials = True
            
            array.markers.append(marker)
            
        #publish ghost tempest
        tempest = Marker()
        tempest.header.frame_id = "world"
        tempest.header.stamp = self.get_clock().now().to_msg()
        tempest.ns = "tempest"
        tempest.id = 0
        tempest.type = Marker.MESH_RESOURCE
        tempest.pose = self.latestOdom.pose.pose
        tempest.scale = Vector3(x=1.0, y=1.0, z=1.0)
        tempest.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.5)
        tempest.lifetime = Duration().to_msg()
        tempest.mesh_resource = "file://" + os.path.join(get_package_share_directory(self.meshPkg), self.meshDir, "tempest", "model.dae")
        tempest.mesh_use_embedded_materials = False
        
        linearActive = self.latestLinearCmd.mode == ControllerCommand.POSITION
        angularActive = self.latestAngularCmd.mode == ControllerCommand.POSITION
        if not (linearActive or angularActive):
            tempest.action = Marker.DELETE
        else:
            tempest.action = Marker.MODIFY
            
        if linearActive:
            tempest.pose.position = toPoint(self.latestLinearCmd.setpoint_vect)
        
        if angularActive:
            tempest.pose.orientation = self.latestAngularCmd.setpoint_quat
        
        array.markers.append(tempest)
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
