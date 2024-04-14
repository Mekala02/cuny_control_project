import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from messages.msg import VehicleState # Import the custom message type for vehicle state
from geometry_msgs.msg import PoseStamped
import math

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher_node')
        self.logger = self.get_logger()

        # Subscribe to vehicle state topic
        self.vehicle_state_subscriber = self.create_subscription(
            VehicleState,
            '/gazebo/vehicle_state',
            self.pose_callback,
            1)

        # Load waypoints from coordinates.txt file
        self.waypoints = self.load_waypoints_from_file('waypoints/sonoma_waypoints.txt')

        # Initialize current waypoint index
        self.current_waypoint_index = 0

        # Define a publisher for the path
        self.path_publisher = self.create_publisher(Path, '/vehicle_waypoints', 10)

    def load_waypoints_from_file(self, file_path):
        waypoints = []
        with open(file_path, 'r') as file:
            for line in file:
                parts = line.split(',')
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                waypoints.append((x, y, z))
        return waypoints

    def find_closest_waypoint_index(self, x, y, z):
        closest_distance = float('inf')
        closest_index = 0
        for i, waypoint in enumerate(self.waypoints):
            distance = math.sqrt((x - waypoint[0]) ** 2 + (y - waypoint[1]) ** 2 + (z - waypoint[2]) ** 2)
            if distance < closest_distance:
                closest_distance = distance
                closest_index = i
            if i == 500:  # Limiting to 500 waypoints for efficiency
                break
        return closest_index
    

    def pose_callback(self, msg):
        current_pose = msg.center
        current_x = current_pose.x
        current_y = current_pose.y
        current_z = current_pose.z
        current_orientation = msg.orientation
        
        self.current_waypoint_index = self.find_closest_waypoint_index(current_x, current_y, current_z)
        self.waypoints = self.waypoints[self.current_waypoint_index:]

        # Reset current waypoint index
        self.current_waypoint_index = 0

        # Calculate distance to current waypoint (For debugging)
        distance_to_waypoint = (current_x - self.waypoints[self.current_waypoint_index][0]) ** 2 + (current_y - self.waypoints[self.current_waypoint_index][1]) ** 2
        # self.logger.info(f"Distance: {distance_to_waypoint}")

        # Publish the waypoints as a path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"  # Adjust frame id if needed
        for i, waypoint in enumerate(self.waypoints):
            pose = PoseStamped()
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.orientation.w = 1.0  # Orientation is not relevant for waypoints
            path_msg.poses.append(pose)
            if i == 500:
                break

        self.path_publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()