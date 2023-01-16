#! /usr/bin/env python3

import os

import rclpy
import rclpy.time
import yaml
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import (Point, PointStamped, Quaternion,
                               TransformStamped, Vector3)
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_point
from tf2_ros import (Buffer, TransformBroadcaster, TransformException,
                     TransformListener)
from transforms3d.euler import euler2quat, quat2euler

UPDATE_PERIOD = 0.125 #seconds
CONFIG_FILE = os.path.join(get_package_share_directory("riptide_rviz"), "config", "visual_frames.yaml")

class VisualFramePublisher(Node):
    def __init__(self):
        super().__init__("visual_frame_publisher")
        self.get_logger().info("Visual Frame Publisher started.")
        self.timer = self.create_timer(UPDATE_PERIOD, self.timerCB)
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        self.tfPub = TransformBroadcaster(self)
        
        #resolve frames to publish
        self.frames = None
        stream = open(CONFIG_FILE, 'r')
        self.frames = yaml.safe_load(stream)["visual_frames"]
        
                
    def parseFrameCoordinate(self, s: str) -> 'tuple(float, bool)':
        space = s.find(' ')
        if space > -1:
            numStr = s[ : space]
            absStr = s[space + 1 : ]
            
            num = 0
            try:
                num = float(numStr)
            except ValueError:
                self.get_logger().error(f"Could not convert \"{num}\" to a float.")
            
            absolute = absStr == "absolute"
            return num, absolute
        
        #no space in the provided string
        return 0, False
    
    
    def quatToVector3(self, quat: Quaternion) -> Vector3:
        (r, p, y) = quat2euler((quat.w, quat.x, quat.y, quat.z))
        return Vector3(x=r, y=p, z=y)
    
    
    def vector3ToQuat(self, vec3: Vector3) -> Quaternion:
        (w, x, y, z)  = euler2quat(vec3.x, vec3.y, vec3.z)
        return Quaternion(w=w, x=x, y=y, z=z)

    
    def timerCB(self):
        if self.frames is not None:
            for frameName in self.frames:
                frame = self.frames[frameName]
                parent = frame["parent"]
                
                #read data to see what the values for xyzrpy are and if they are absolute (world frame) or relative (parent frame)
                (x, xAbs) = self.parseFrameCoordinate(frame["x"])
                (y, yAbs) = self.parseFrameCoordinate(frame["y"])
                (z, zAbs) = self.parseFrameCoordinate(frame["z"])
                
                (roll, rollAbs) = self.parseFrameCoordinate(frame["roll"])
                (pitch, pitchAbs) = self.parseFrameCoordinate(frame["pitch"])
                (yaw, yawAbs) = self.parseFrameCoordinate(frame["yaw"])
                
                #look up parent transform and create the new transform based off of it
                try:
                    #look up parent transform and convert it to xyzrpy
                    parentToWorld = self.tfBuffer.lookup_transform("world", parent, rclpy.time.Time())
                    parentRotation = self.quatToVector3(parentToWorld.transform.rotation)
                    
                    #compute child xyz
                    childTranslationRelative = PointStamped(
                        point = Point(
                            x = 0.0 if xAbs else x,
                            y = 0.0 if xAbs else y,
                            z = 0.0 if zAbs else z
                        )
                    )
                                    
                    childTranslationPt = do_transform_point(childTranslationRelative, parentToWorld)
                    childTranslationVec3 = Vector3(
                        x = x if xAbs else childTranslationPt.point.x,
                        y = y if yAbs else childTranslationPt.point.y,
                        z = z if zAbs else childTranslationPt.point.z
                    )
                                        
                    #compute child rpy
                    childRotationVec3 = Vector3(
                        x = roll if rollAbs else roll + parentRotation.x,
                        y = pitch if pitchAbs else pitch + parentRotation.y,
                        z = yaw if yawAbs else yaw + parentRotation.z
                    )
                    
                    childRotationQuaternion = self.vector3ToQuat(childRotationVec3)
                    
                    childRotationVec3 = Vector3(
                        x = roll if rollAbs else roll + parentRotation.x,
                        y = pitch if pitchAbs else pitch + parentRotation.y,
                        z = yaw if yawAbs else yaw + parentRotation.z
                    )
                    
                    childRotationQuaternion = self.vector3ToQuat(childRotationVec3)
                    
                    childTransform = TransformStamped()
                    childTransform.header.frame_id = "world"
                    childTransform.header.stamp = self.get_clock().now().to_msg()
                    childTransform.child_frame_id = frameName
                    childTransform.transform.translation = childTranslationVec3
                    childTransform.transform.rotation = childRotationQuaternion
                    
                    self.tfPub.sendTransform(childTransform)
                    
                except TransformException as ex:
                    self.get_logger().warn(f"Could not look up transform from world to {parent}: {ex}")
        

def main(args = None):
    rclpy.init(args = args)
    rclpy.spin(VisualFramePublisher())
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Visual frame publisher keyboard-interrupted")
        