import rclpy
from rclpy.node import Node
from messages.msg import VehicleState
import time

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        # Subscribe to the /gazebo/vehicle_state topic
        self.subscription = self.create_subscription(
            VehicleState,
            '/gazebo/vehicle_state',
            self.vehicle_state_callback,
            10)
        self.subscription

        # Open file for writing waypoints
        self.waypoints_file = open('waypoints.txt', 'w')

        # Time tracking for recording waypoints
        self.last_waypoint_time = time.time()

    def vehicle_state_callback(self, msg):
        # Extract vehicle coordinates from the message
        x = msg.center.x
        y = msg.center.y
        z = msg.center.z

        # Check if a second has passed since the last recording
        current_time = time.time()
        # Recording @10 hz
        if current_time - self.last_waypoint_time >= 0.1:
            self.record_waypoint(x, y, z)
            self.last_waypoint_time = current_time

    def record_waypoint(self, x, y, z):
        # Record the waypoint to the file
        self.waypoints_file.write(f"{x},{y},{z}\n")

    def __del__(self):
        # Close the waypoints file when the node is destroyed
        self.waypoints_file.close()


def main(args=None):
    rclpy.init(args=args)
    waypoint_recorder = WaypointRecorder()
    rclpy.spin(waypoint_recorder)
    waypoint_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()