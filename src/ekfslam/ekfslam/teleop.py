import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class teleop(Node):
    def __init__(self):
        super().__init__("teleop_ps4")
        self.sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
    def joy_callback(self, msg: Joy):
        cmd_vel = Twist()
        cmd_vel.linear.x = msg.axes[1]
        cmd_vel.angular.z = msg.axes[3]
        self.pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = teleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
