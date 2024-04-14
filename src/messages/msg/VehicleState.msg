# VehicleState.msg
geometry_msgs/Point center    # Position of the center of the vehicle (Unit: meter)
geometry_msgs/Point rear_axle # Position of the rear axle of the vehicle (Unit: meter)
geometry_msgs/Point front_axle # Position of the front axle of the vehicle (Unit: meter)
float32 yaw        # Yaw angle of the vehicle (Unit: radians)
float32 speed      # Speed of the vehicle (direction: yaw, Unit: m/s)
geometry_msgs/Quaternion orientation # Quaternion orientation of the vehicle
