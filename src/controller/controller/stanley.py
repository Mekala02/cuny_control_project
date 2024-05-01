import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path
from messages.msg import VehicleState  # Import the custom message type for vehicle state
import math

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        self.logger = self.get_logger()
        self.waypoints = []
        self.epsilon = 10e-5

        # Vehicle Parameters
        self.wheelbase = 1.8

        # Tunable parameters
        self.k = 1  # Placeholder for proportional gain
        self.linear_velocity = 3.0  # Placeholder for constant linear velocity

        # Subscribe to /gazebo/vehicle_state topic
        self.subscription = self.create_subscription(
            VehicleState,
            '/gazebo/vehicle_state',
            self.pose_callback,
            10)

        # Subscribe to the path published by WaypointPublisher node
        self.path_subscriber = self.create_subscription(
            Path,
            '/vehicle_waypoints',
            self.path_callback,
            10)

        # Publisher for cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.logger.info("Stanley Controller node started")

    def path_callback(self, path_msg):
        # Update the waypoints when a new path is received
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]

    def pose_callback(self, msg):
        if not self.waypoints:
            # If no waypoints available, do nothing
            return

        current_pose = msg.front_axle
        current_x = current_pose.x
        current_y = current_pose.y
        yaw = msg.yaw

        # Calculate and publish cmd_vel message
        steering_angle = self.generate_control_output(current_x, current_y, yaw)
        # Publish cmd_vel message
        twist = Twist()
        twist.linear.x = self.linear_velocity  # constant linear velocity
        twist.angular.z = self.steering_angle_to_angular_velocity(steering_angle)
        self.cmd_vel_publisher.publish(twist)

    def steering_angle_to_angular_velocity(self, steering_angle):
        angular_velocity = math.tan(steering_angle) * self.linear_velocity / self.wheelbase
        return angular_velocity

    def generate_control_output(self, current_x, current_y, yaw):
        if not self.waypoints:
            return 0.0

        closest_dist = float('inf')
        closest_index = 0
        for i, waypoint in enumerate(self.waypoints):
            dist = math.sqrt((current_x - waypoint[0])**2 + (current_y - waypoint[1])**2)
            if dist < closest_dist:
                closest_dist = dist
                closest_index = i

        # Calculate cross track error 
        waypoint_x, waypoint_y = self.waypoints[closest_index]
        heading_vector = [math.cos(yaw), math.sin(yaw)]
        waypoint_vector = [waypoint_x - current_x, waypoint_y - current_y]
        cross_track_error = math.cross(heading_vector, waypoint_vector)

        desired_heading = math.atan2(waypoint_y - current_y, waypoint_x - current_x)
        heading_error = desired_heading - yaw
        steering_angle = heading_error + math.atan2(self.k * cross_track_error, self.linear_velocity)

        return steering_angle

def main(args=None):
    rclpy.init(args=args)
    controller = StanleyController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
