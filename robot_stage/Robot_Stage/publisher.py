import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class GoalNavigator(Node):
    def __init__(self):
        super().__init__('goal_navigator')
        self.subscription_laser = self.create_subscription(
            LaserScan,
            'base_scan',
            self.laser_callback,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            'ground_truth',
            self.odom_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()

        self.current_position = (0.0, 0.0)
        self.current_orientation = 0.0
        self.target_positions = [(5.0, 5.0), (4.0, -4.0)]  # Lista de objetivos
        self.current_target_index = 0
        
        self.closest_distance = float('inf')
        self.distance_threshold = 0.3  # Distance to start avoiding obstacles
        self.goal_tolerance = 0.3  # Tolerance distance to the goal

    def laser_callback(self, scan):
        self.closest_distance = min(scan.ranges)
        self.get_logger().info(f"Laser distancia más cercana: {self.closest_distance}")

        # Si se detecta un obstáculo cerca, detener el robot
        if self.closest_distance < self.distance_threshold:
            self.get_logger().info("Obstáculo detectado, deteniendo el robot.")
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.avoid_obstacle(scan)
        else:
            self.move_towards_goal()

        self.publisher_.publish(self.cmd)

    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_orientation = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.get_logger().info(f"Posición actual del robot: x={self.current_position[0]}, y={self.current_position[1]}")

    def move_towards_goal(self):
        target_position = self.target_positions[self.current_target_index]
        distance_to_goal = math.sqrt((target_position[0] - self.current_position[0]) ** 2 + 
                                     (target_position[1] - self.current_position[1]) ** 2)
        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info(f"Llegamos al objetivo {self.current_target_index + 1}")
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            
            # Cambiar al siguiente objetivo
            if self.current_target_index < len(self.target_positions) - 1:
                self.current_target_index += 1
            else:
                self.get_logger().info("¡Hemos llegado a todos los objetivos!")
                rclpy.shutdown()  # Detener el nodo
        else:
            target_angle = math.atan2(target_position[1] - self.current_position[1], target_position[0] - self.current_position[0])
            angle_diff = target_angle - self.current_orientation
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize angle to [-pi, pi]

            self.get_logger().info(f"Ángulo hacia el objetivo: {math.degrees(target_angle)} grados")
            self.get_logger().info(f"Diferencia de ángulo: {math.degrees(angle_diff)} grados")

            self.cmd.linear.x = 0.2  # Forward speed
            self.cmd.angular.z = 0.5 * angle_diff  # Proportional control for angular speed

    def avoid_obstacle(self, scan):
        # Divide the scan ranges into three sections: right, front, left
        right_ranges = scan.ranges[:90]
        front_ranges = scan.ranges[90:180]
        left_ranges = scan.ranges[180:270]

        # Calculate the average distances for each section
        right_avg = sum(right_ranges) / len(right_ranges)
        front_avg = sum(front_ranges) / len(front_ranges)
        left_avg = sum(left_ranges) / len(left_ranges)

        self.get_logger().info(f"Promedio de distancias: derecha={right_avg}, frente={front_avg}, izquierda={left_avg}")

        # Determine the direction with the farthest average distance
        if right_avg > left_avg:
            turn_direction = -1.0  # Turn right
        else:
            turn_direction = 1.0  # Turn left

        # Rotate the robot towards the direction with the farthest average distance
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.5 * turn_direction
        self.publisher_.publish(self.cmd)

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    goal_navigator = GoalNavigator()
    
    try:
        rclpy.spin(goal_navigator)
    except KeyboardInterrupt:
        pass
    finally:
        goal_navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()