import rclpy
from rclpy.node import Node 
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math


class MoveScara(Node):
    def __init__(self):
        super().__init__("move_scara")
        self.publisher = self.create_publisher(JointTrajectory, "set_joint_trajectory", 10)
        self.get_logger().info("move_scara has been started!!")
        self.a1 = 600.0
        self.a2 = 600.0

        self.timer = self.create_timer(1, self.test)
        self.x = 10
        self.y = 10

    
    def test(self):
        theta1, theta2 = self.inverse_kinematics(self.x, self.y)
        self.get_logger().info("inverse_kinematics")
        self.get_logger().info(f"x:{self.x}, y:{self.y}, theta1:{theta1}, theta2:{theta2}")

        x, y = self.forward_kinematics(theta1, theta2)
        self.get_logger().info("forward_kinematics")
        self.get_logger().info(f"x:{x}, y:{y}, theta1:{theta1}, theta2:{theta2}")

        self.x += 10
        self.y += 10


    def inverse_kinematics(self, x, y):
        r = math.sqrt(x*x + y*y)
        beta = math.acos((self.a1*self.a1 + self.a2*self.a2 - r*r)/(2 * self.a1 * self.a2))
        theta2 = math.pi - beta

        phi = math.atan2(y, x)
        alpha = math.atan2(self.a2 * math.sin(theta2),self.a1 + self.a2*math.cos(theta2))
        theta1 = phi - alpha

        return theta1, theta2
    
    def forward_kinematics(self, theta1, theta2):
        y_d = self.a1 * math.sin(theta1)
        x_d = self.a1 * math.cos(theta1)
        y_dd = self.a2 * math.sin(theta1+theta2)
        x_dd = self.a2 * math.cos(theta1+theta2)
        x = x_d + x_dd
        y = y_d + y_dd
        return x, y


def main(args=None):
    rclpy.init(args=args)
    node = MoveScara()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()