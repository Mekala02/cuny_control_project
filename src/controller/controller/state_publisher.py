import math
from geometry_msgs.msg import PoseArray, Point
from messages.msg import VehicleState # Import the custom message type for vehicle state
from visualization_msgs.msg import Marker # Import for marker messages
import rclpy
from rclpy.node import Node


class StatePublisher(Node):
    def __init__(self):
        super().__init__('gazebo_state_publisher')

        # Subscribe to /pose_info topic
        self.subscription = self.create_subscription(
            PoseArray,
            '/pose_info',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create publisher for the new topic
        self.publisher_ = self.create_publisher(VehicleState, '/gazebo/vehicle_state', 10)
        # Publisher for marker
        self.marker_publisher_ = self.create_publisher(Marker, '/car_position_marker', 10)

    def pose_callback(self, msg):
        current_pose = msg.poses[1]  # Assuming you are interested in the third pose
        current_position = current_pose.position
        current_orientation = current_pose.orientation

        current_x = current_position.x
        current_y = current_position.y
        current_z = current_position.z

        orientation_x = current_orientation.x
        orientation_y = current_orientation.y
        orientation_z = current_orientation.z
        orientation_w = current_orientation.w

        # Convert quaternion to yaw angle
        yaw = self.quaternion_to_yaw(orientation_x, orientation_y, orientation_z, orientation_w)

        # Calculate new positions
        rear_axle = self.move_point_along_line([current_x, current_y], yaw, -0.9)
        front_axle = self.move_point_along_line([current_x, current_y], yaw, 0.9)
        # Create a new message for the new topic
        new_msg = VehicleState()
        new_msg.center = Point(x=current_x, y=current_y, z=current_z)
        new_msg.rear_axle = Point(x=rear_axle[0], y=rear_axle[1], z=current_z)
        new_msg.front_axle = Point(x=front_axle[0], y=front_axle[1], z=current_z)
        new_msg.yaw = yaw
        new_msg.speed = 0.0  # Not implemented
        new_msg.orientation = current_orientation

        # Publish the new message
        self.publisher_.publish(new_msg)

        # Publish marker for visualization (Vehicle's position)
        marker = Marker()
        marker.header.frame_id = "map"  # Change to your frame ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = current_x
        marker.pose.position.y = current_y
        marker.pose.position.z = current_z
        marker.pose.orientation.x = orientation_x
        marker.pose.orientation.y = orientation_y
        marker.pose.orientation.z = orientation_z
        marker.pose.orientation.w = orientation_w
        marker.scale.x = 5.0  # Adjust the scale as needed
        marker.scale.y = 5.0
        marker.scale.z = 5.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_publisher_.publish(marker)

    def move_point_along_line(self, point, angle, distance):
        # Convert angle to unit vector (direction vector)
        dx = math.cos(angle)
        dy = math.sin(angle)
        # Calculate new point coordinates
        new_x = point[0] + distance * dx
        new_y = point[1] + distance * dy
        return new_x, new_y

    def quaternion_to_yaw(self, x, y, z, w):
        # Convert quaternion to yaw angle
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw
    
def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()