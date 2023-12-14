#! /usr/bin/env python3

import os

import rclpy
import enum
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from transforms3d.euler import euler2quat
from visualization_msgs.msg import Marker, MarkerArray

class ControllerType(enum.Enum):
    OLD = 1
    SMC = 2
    PID = 3
    
CONTROLLER_TYPE = ControllerType.PID

# import controller msgs and set topic names
if CONTROLLER_TYPE == ControllerType.OLD:
    from riptide_msgs2.msg import ControllerCommand
    LINEAR_CMD_TOPIC = "controller/linear"
    ANGULAR_CMD_TOPIC = "controller/angular"
elif CONTROLLER_TYPE == ControllerType.SMC:
    pass
elif CONTROLLER_TYPE == ControllerType.PID:
    PID_SETPT_TOPIC = "pid/target_position"

    

CONFIG_FILE = os.path.join(get_package_share_directory("riptide_rviz"), "config", "markers.yaml")
ODOMETRY_TOPIC = "odometry/filtered"
SIM_TOPIC = "simulator/state"


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


class MarkerInfo:
    def __init__(self, mesh: str, frame: str, scale: float, pose: 'list[float]'):
        self.mesh   = mesh
        self.frame  = frame
        self.scale  = scale
        self.pose   = pose
    

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__("marker_publisher")
        self.readParameters()
        if self.updatePeriod == 0:
            self.get_logger().error(f"Failed to read parameters or update rate is 0.")
            exit(1)
        
        self.timer = self.create_timer(self.updatePeriod, self.timerCB)

        #configure ghost tempest vars
        self.latestOdom = Odometry()
        self.latestSimPose = Pose()
        
        #configure ros pub subs
        self.markerPub = self.create_publisher(MarkerArray, self.markerTopic, 10)
        self.odomSub = self.create_subscription(Odometry, ODOMETRY_TOPIC, self.odomCB, 10)
        self.simSub = self.create_subscription(Pose, SIM_TOPIC, self.simCB, 10)
    
        #controller command subs
        if CONTROLLER_TYPE == ControllerType.OLD:
            self.linearSub = self.create_subscription(ControllerCommand, LINEAR_CMD_TOPIC, self.linearCB, 10)
            self.angularSub = self.create_subscription(ControllerCommand, ANGULAR_CMD_TOPIC, self.angularCB, 10)
            self.latestLinearCmd = ControllerCommand()
            self.latestAngularCmd = ControllerCommand()
        elif CONTROLLER_TYPE == ControllerType.SMC:
            pass
        elif CONTROLLER_TYPE == ControllerType.PID:
            self.setptSub = self.create_subscription(Pose, PID_SETPT_TOPIC, self.setptCb, 10)
            self.latestSetpt = Pose()
        
        self.get_logger().info("Marker Publisher node started.")
        
    
    def declareAndReceiveParam(self, name: str, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value
    
    
    def hasParametersForMarker(self, idx):
        return \
            self.markers[idx].mesh != "" and \
            self.markers[idx].frame != "" and \
            self.markers[idx].scale > 0
    
    
    def readParameters(self):
        #declare parameters
        self.updatePeriod   = self.declareAndReceiveParam("update_period", 0.0)
        self.meshPkg        = self.declareAndReceiveParam("mesh_pkg", "")
        self.meshDir        = self.declareAndReceiveParam("mesh_directory", "")
        self.markerTopic    = self.declareAndReceiveParam("marker_topic", "")
        self.robot          = self.declareAndReceiveParam("robot", "")
        
        markerIdx = 0
        self.markers: 'list[MarkerInfo]' = [
            MarkerInfo(
                self.declareAndReceiveParam("markers.marker0.mesh", ""),
                self.declareAndReceiveParam("markers.marker0.frame", ""),
                self.declareAndReceiveParam("markers.marker0.scale", 0.0),
                self.declareAndReceiveParam("markers.marker0.pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            )
        ]
        
        while(self.hasParametersForMarker(markerIdx)):
            markerIdx += 1
            self.markers.append(
                MarkerInfo(
                    self.declareAndReceiveParam(f"markers.marker{markerIdx}.mesh", ""),
                    self.declareAndReceiveParam(f"markers.marker{markerIdx}.frame", ""),
                    self.declareAndReceiveParam(f"markers.marker{markerIdx}.scale", 0.0),
                    self.declareAndReceiveParam(f"markers.marker{markerIdx}.pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                )
            )

        # because of how the while executes, the last marker in self.markers will be bad. delete it
        self.markers.remove(self.markers[-1])
    
    # setpoint callbacks
    if CONTROLLER_TYPE == ControllerType.OLD:
        def linearCB(self, msg):
            self.latestLinearCmd = msg
            
            
        def angularCB(self, msg):
            self.latestAngularCmd = msg
    elif CONTROLLER_TYPE == ControllerType.SMC:
        pass
    elif CONTROLLER_TYPE == ControllerType.PID:
        def setptCb(self, msg):
            self.latestSetpt = msg
        
    def simCB(self, msg):
        self.latestSimPose = msg

    def odomCB(self, msg):
        self.latestOdom = msg
        
        
    def timerCB(self):
        array = MarkerArray()
        for i in range(0, len(self.markers)):
            markerInfo = self.markers[i]
            marker = Marker()
            marker.header.frame_id = markerInfo.frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "mapping_markers"
            marker.id = i
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.MODIFY
            marker.pose = poseFromArr(markerInfo.pose)
            marker.scale = Vector3(x=markerInfo.scale, y=markerInfo.scale, z=markerInfo.scale)
            marker.lifetime = Duration().to_msg() #forever
            marker.frame_locked = True
            marker.mesh_resource = "file://" + os.path.join(get_package_share_directory(self.meshPkg), self.meshDir, markerInfo.mesh, "model.dae")
            marker.mesh_use_embedded_materials = True
            
            array.markers.append(marker)
        

        #publish ghost talos of sim true position
        simGhost = Marker()
        simGhost.header.frame_id = "world"
        simGhost.header.stamp = self.get_clock().now().to_msg()
        simGhost.ns = "simGhost"
        simGhost.id = 0
        simGhost.type = Marker.MESH_RESOURCE
        simGhost.scale = Vector3(x=1.0, y=1.0, z=1.0)
        simGhost.color = ColorRGBA(r=0.0, g=0.0, b=255.0, a=0.5)
        simGhost.lifetime = Duration().to_msg()
        simGhost.mesh_resource = "file://" + os.path.join(get_package_share_directory(self.meshPkg), self.meshDir, self.robot, "model.dae")
        simGhost.mesh_use_embedded_materials = False
        simGhost.action = Marker.MODIFY
        simGhost.pose = self.latestSimPose
        
        array.markers.append(simGhost)
        #publish ghost robot for odom goal point
        ghost = Marker()
        ghost.header.frame_id = "world"
        ghost.header.stamp = self.get_clock().now().to_msg()
        ghost.ns = "ghost"
        ghost.id = 0
        ghost.type = Marker.MESH_RESOURCE
        ghost.pose = self.latestOdom.pose.pose
        ghost.scale = Vector3(x=1.0, y=1.0, z=1.0)
        ghost.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.5)
        ghost.lifetime = Duration().to_msg()
        ghost.mesh_resource = "file://" + os.path.join(get_package_share_directory(self.meshPkg), self.meshDir, self.robot, "model.dae")
        ghost.mesh_use_embedded_materials = False
        
        if CONTROLLER_TYPE == ControllerType.OLD:
            linearActive = self.latestLinearCmd.mode == ControllerCommand.POSITION
            angularActive = self.latestAngularCmd.mode == ControllerCommand.POSITION
            if not (linearActive or angularActive):
                ghost.action = Marker.DELETE
            else:
                ghost.action = Marker.MODIFY
                
            if linearActive:
                ghost.pose.position = toPoint(self.latestLinearCmd.setpoint_vect)
            
            if angularActive:
                ghost.pose.orientation = self.latestAngularCmd.setpoint_quat
        elif CONTROLLER_TYPE == ControllerType.SMC:
            pass
        elif CONTROLLER_TYPE == ControllerType.PID:
            ghost.action = Marker.MODIFY
            ghost.pose = self.latestSetpt
        
        ghost.pose.position.x = ghost.pose.position.x #- 0.148
        ghost.pose.position.y = ghost.pose.position.y #+ 0.040
        ghost.pose.position.z = ghost.pose.position.z #- 0.071
        array.markers.append(ghost)
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
