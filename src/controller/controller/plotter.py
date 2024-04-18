import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from messages.msg import (
    VehicleState,
)  # Import the custom message type for vehicle state
import math

class Plotter(Node):
    def __init__(self):
        super().__init__("plotter")
        self.state = None
        # Subscribe to /gazebo/vehicle_state topic
        self.subscription = self.create_subscription(
            VehicleState, "/gazebo/vehicle_state", self.pose_callback, 10
        )

        # Subscribe to the path published by WaypointPublisher node
        self.path_subscriber = self.create_subscription(
            Path, "/vehicle_waypoints", self.path_callback, 10
        )
        self.x_error_publisher = self.create_publisher(Float32, "/tracking_error/x", 10)
        self.y_error_publisher = self.create_publisher(Float32, "/tracking_error/y", 10)
        self.euc_error_publisher = self.create_publisher(
            Float32, "/tracking_error/euc", 10
        )

        timer = self.create_timer(0.1, self.calc_error)

    def path_callback(self, path_msg):
        # Update the waypoints when a new path is received
        self.waypoints = [
            (pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses
        ]

    def pose_callback(self, msg):
        self.state = msg

    def calc_error(self):
        # calculate the error between the current pose and the target waypoint
        if self.state is None or not self.waypoints:
            return

        x, y = self.state.center.x, self.state.center.y
        error_x = self.waypoints[2][0] - x
        error_y = self.waypoints[2][1] - y
        euc_error = math.sqrt(error_x**2 + error_y**2)
        self.x_error_publisher.publish(Float32(data=error_x))
        self.y_error_publisher.publish(Float32(data=error_y))
        self.euc_error_publisher.publish(Float32(data=euc_error))
        # print(f"X Error: {error_x}, Y Error: {error_y}, Euclidean Error: {euc_error}")


def main(args=None):
    rclpy.init(args=args)
    controller = Plotter()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
