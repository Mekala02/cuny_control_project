import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path
from messages.msg import VehicleState  # Import the custom message type for vehicle state
import math


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.logger = self.get_logger()
        self.waypoints = []

        # Vehicle Parameters
        self.wheelbase = 1.8

        # Tunable parameters
        self.k = 1.0
        self.linear_velocity = 3.0
        self.lookahead_distance = 9.0

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
        self.logger.info("Pure Pursuit Controller node started")

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

    def distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def steering_angle_to_angular_velocity(self, steering_angle):
        angular_velocity = math.tan(steering_angle) * self.linear_velocity / self.wheelbase
        return angular_velocity

    def generate_control_output(self, current_x, current_y, yaw):
        if not self.waypoints:
            raise NotImplementedError
    
        # The closest waypoint to the vehicle
        closest_dist = float('inf')
        closest_index = 0
        for i, waypoint in enumerate(self.waypoints):
            dist = self.distance((current_x, current_y), waypoint)
            if dist < closest_dist:
                closest_dist = dist
                closest_index = i
        
        # The lookahead waypoint index
        lookahead_index = closest_index
        while lookahead_index < len(self.waypoints) - 1:
            if self.distance(self.waypoints[closest_index], self.waypoints[lookahead_index]) > self.lookahead_distance:
                break
            lookahead_index += 1
        
        # The steering angle using the Pure Pursuit algorithm
        lookahead_point = self.waypoints[lookahead_index]
        alpha = math.atan2(lookahead_point[1] - current_y, lookahead_point[0] - current_x) - yaw
        steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), self.lookahead_distance)
        
        # Apply proportional gain
        steering_angle *= self.k
        
        return steering_angle
    
        raise NotImplementedError


def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
