import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path
from messages.msg import VehicleState # Import the custom message type for vehicle state
import math

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        self.logger = self.get_logger()
        self.waypoints = []
        self.epsilon = 10e-5

        # Vehicle Parameters
        self.wheelbase = 1.8

        # Tunable parameters
        self.k = 1
        self.linear_velocity = 3.0

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
        """
        Calculates the steering angle required for path following (Use Stanley Algorithm).

        Args:
            current_x (float): Current x-coordinate of the vehicle.
            current_y (float): Current y-coordinate of the vehicle.
            yaw (float): Current yaw angle (orientation) of the vehicle.

        Returns:
            float: Steering angle in radians.

        This function is called by pose_callback() to determine the necessary
        steering angle based on the vehicle's current position and orientation,
        as well as the recent waypoints automatically updated and stored in
        self.waypoints. The waypoints represent the path that the vehicle
        should follow, and the controller uses them to calculate the optimal
        steering angle to stay on track.

        Additional Notes:
            The self.waypoints list automatically updates to reflect the planned trajectory of
            the autonomous vehicle. This list serves as a repository of coordinates, representing
            waypoints along the vehicle's trajectory. As the vehicle progresses, the list is
            continuously updated based on its position, ensuring it contains only the waypoints
            relevant to its current location and future path. The first index in the list
            corresponds to the closest waypoint to the vehicle, prioritizing navigation towards
            nearby points. Additionally, the list excludes waypoints that are behind the vehicle,
            focusing solely on waypoints ahead to streamline navigation planning and decision-making
            processes for control algorithms.
        """

        raise NotImplementedError
    

def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()