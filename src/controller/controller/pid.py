import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path
from messages.msg import VehicleState # Import the custom message type for vehicle state
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker # Import for marker messages
import math
import time

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.logger = self.get_logger()
        self.waypoint = [303.34893051869386,-154.93960140432998,2.1951846162325]
        self.error = 0
        # For graph
        self.data = []
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], lw=2)
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Data')
        self.ax.set_title('Topic vs Time')

        # PID parameters
        self.k = 4.0
        self.Kp = 0.1
        self.Ki = 0.0
        self.Kd = 1
        self.I_max = 0
        self.current_time = 0
        self.previous_time = 0
        self.current_error = 0
        self.previous_error = 0
        self.P = 0
        self.I = 0
        self.D = 0

        # Subscribe to /gazebo/vehicle_state topic
        self.subscription = self.create_subscription(
            VehicleState,
            '/gazebo/vehicle_state',
            self.pose_callback,
            10)

        # Publisher for cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_publisher_ = self.create_publisher(Marker, '/waypoint', 10)

        # Publish marker for visualization (Vehicle's position)
        marker = Marker()
        marker.header.frame_id = "map"  # Change to your frame ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.waypoint[0]
        marker.pose.position.y = self.waypoint[1]
        marker.pose.position.z = self.waypoint[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.0
        marker.scale.x = 5.0  # Adjust the scale as needed
        marker.scale.y = 5.0
        marker.scale.z = 5.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker = marker

        self.logger.info("PID Controller node started")

    def pose_callback(self, msg):
        current_pose = msg.center
        current_x = current_pose.x
        current_y = current_pose.y
        yaw = msg.yaw

        self.error = self.calc_error(self.waypoint, (current_x, current_y))
        self.data.append(self.error)
        self.update_plot()
        self.marker_publisher_.publish(self.marker)
        # Calculate and publish cmd_vel message
        self.generate_control_output()

    def calc_error(self, desired_point, actual_position):
        x_diff = desired_point[0] - actual_position[0]
        y_diff = desired_point[1] - actual_position[1]
        error_length = math.sqrt(x_diff ** 2 + y_diff ** 2)
        error_sign = 1 if x_diff >= 0 and y_diff <= 0 else -1
        return error_length*error_sign

    def steering_angle_to_angular_velocity(self, steering_angle):
        angular_velocity = math.tan(steering_angle) * self.linear_velocity / self.wheelbase
        return angular_velocity

    def update_plot(self):
        self.line.set_data(range(len(self.data)), self.data)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.001)  # Pause briefly to allow the plot to update

    def generate_control_output(self):
        # Write the controller here
        speed = 0
        pass

        # Publish cmd_vel message
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()