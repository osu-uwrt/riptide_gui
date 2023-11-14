#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.time import Duration
from transforms3d.euler import quat2euler, euler2quat
from interactive_markers import InteractiveMarkerServer
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, TransformStamped, Point, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from enum import Enum
from math import pi

ODOM_TOPIC_HZ = 30
LINEAR_SNAP_WIDTH = 1 # meters
ANGULAR_SNAP_WIDTH = 45 # degrees

# quaternions ordered xyzw
AXIS_ROTATIONS = [
    (0, 0, 0, 1), # X
    (0, 0, 1, 1), # Y
    (0, 1, 0, 1)  # Z
]

# values correspond with array indices
class AxisName(Enum):
    X_AXIS = 0
    Y_AXIS = 1
    Z_AXIS = 2


def makeControlForAxis(axis: AxisName, interactMode):
    control = InteractiveMarkerControl()
    control.name = axis.name
    
    orient = AXIS_ROTATIONS[axis.value]
    control.orientation.x = float(orient[0])
    control.orientation.y = float(orient[1])
    control.orientation.z = float(orient[2])
    control.orientation.w = float(orient[3])
    control.interaction_mode = interactMode
    
    return control


def pointToVector3(pt: Point):
    vec = Vector3()
    vec.x = pt.x
    vec.y = pt.y
    vec.z = pt.z
    
    return vec


def snap(value, toNearest):
    return round(value / toNearest) * toNearest


class OdometrySpoofer(Node):
    pose = Pose()
    
    def __init__(self):
        super().__init__("odometry_spoofer")
        self.timer = self.create_timer(1 / ODOM_TOPIC_HZ, self.timerCb)
        self.markerServer = InteractiveMarkerServer(self, "spoofer")
        self.odomPub = self.create_publisher(Odometry, "odometry/filtered", qos_profile_system_default)
        self.textOutPub = self.create_publisher(Marker, "spoofer/text_out", qos_profile_system_default)
        self.transformBroadcaster = TransformBroadcaster(self)
        ns = self.get_namespace() + "/"
        self.robotName = ns[1 : ns.find('/', 2)] #starts find after leading slash. if find fails, -1 will wrap to back of string and cut the trailing slash
        self.robotBaseLink = self.robotName + "/base_link"
        self.get_logger().info(f"Using robot name \"{self.robotName}\"")
        
        now = self.get_clock().now().to_msg()
        
        self.interactiveMarker = InteractiveMarker()
        self.interactiveMarker.header.stamp = now
        self.interactiveMarker.header.frame_id = "world"
        self.interactiveMarker.name = self.robotName
        
        #visual of object in RViz
        self.objectVisualMarker = Marker()
        self.objectVisualMarker.type = Marker.CUBE
        self.objectVisualMarker.color.r = 1.0
        self.objectVisualMarker.color.g = 1.0
        self.objectVisualMarker.color.b = 1.0
        self.objectVisualMarker.color.a = 1.0
        self.objectVisualMarker.scale.x = 0.1
        self.objectVisualMarker.scale.y = 0.1
        self.objectVisualMarker.scale.z = 0.1
        
        objectVisualControl = InteractiveMarkerControl()
        objectVisualControl.always_visible = True
        objectVisualControl.markers.append(self.objectVisualMarker)
        self.interactiveMarker.controls.append(objectVisualControl)
        
        #text output marker
        self.textOutputMarker = Marker()
        self.textOutputMarker.header.stamp = now
        self.textOutputMarker.header.frame_id = self.robotBaseLink
        self.textOutputMarker.type = Marker.TEXT_VIEW_FACING
        self.textOutputMarker.color.r = 1.0
        self.textOutputMarker.color.g = 1.0
        self.textOutputMarker.color.b = 1.0
        self.textOutputMarker.color.a = 1.0
        self.textOutputMarker.scale.x = 0.3
        self.textOutputMarker.scale.y = 0.3
        self.textOutputMarker.scale.z = 0.3
        self.textOutputMarker.pose.position.z = 1.0
        self.textOutputMarker.action = Marker.ADD
        self.textOutputMarker.ns = "spoofer"
        self.textOutputMarker.id = 0
        self.textOutputMarker.lifetime = Duration().to_msg()
        self.textOutputMarker.text = "Spoofer"
        
        #control of one degree of freedom
        for axistup in enumerate(AxisName):
            axis = axistup[1]
            self.interactiveMarker.controls.append(makeControlForAxis(axis, InteractiveMarkerControl.MOVE_AXIS))
            self.interactiveMarker.controls.append(makeControlForAxis(axis, InteractiveMarkerControl.ROTATE_AXIS))
        
        #add interactive marker to server
        self.markerServer.insert(self.interactiveMarker, feedback_callback=self.markerStateChanged)
        self.markerServer.applyChanges()
        
        self.get_logger().info("Spoofer started.")
        
    
    def timerCb(self):
        # header to use in Odometry msg and TransformStamped
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world"        
        
        # publish odometry
        odomToPub = Odometry()
        odomToPub.header = header
        odomToPub.child_frame_id = self.robotBaseLink
        odomToPub.pose.pose = self.pose
        self.odomPub.publish(odomToPub)
        
        # publish transform
        transform = TransformStamped()
        transform.header = header
        transform.child_frame_id = self.robotBaseLink
        transform.transform.translation = pointToVector3(self.pose.position)
        transform.transform.rotation = self.pose.orientation
        self.transformBroadcaster.sendTransform(transform)
                    
    
    def markerStateChanged(self, msg: InteractiveMarkerFeedback):
        #snap odometry in accordance to consts at the top of the file
        self.pose.position.x = float(snap(msg.pose.position.x, LINEAR_SNAP_WIDTH))
        self.pose.position.y = float(snap(msg.pose.position.y, LINEAR_SNAP_WIDTH))
        self.pose.position.z = float(snap(msg.pose.position.z, LINEAR_SNAP_WIDTH))
        
        orient = msg.pose.orientation
        (r, p, y) = quat2euler((orient.w, orient.x, orient.y, orient.z), axes="sxyz")
        angularSnapRads = ANGULAR_SNAP_WIDTH * pi / 180.0
        r = snap(r, angularSnapRads)
        p = snap(p, angularSnapRads)
        y = snap(y, angularSnapRads)
        
        snappedQuat = euler2quat(r, p, y, axes="sxyz")
        orient.w = snappedQuat[0]
        orient.x = snappedQuat[1]
        orient.y = snappedQuat[2]
        orient.z = snappedQuat[3]
        self.pose.orientation = orient
        
        # publish text output marker
        self.textOutputMarker.header.stamp = self.get_clock().now().to_msg()
        self.textOutputMarker.text = f"XYZ: ({self.pose.position.x}, {self.pose.position.y}, {self.pose.position.z})\n" \
                                    + f"RPY: ({r * 180/pi}, {p * 180/pi}, {y * 180/pi})"
        self.textOutPub.publish(self.textOutputMarker)
        

def main(args = None):
    rclpy.init(args = args)
    spoofer = OdometrySpoofer()
    rclpy.spin(spoofer)
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
