import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
import math as m

def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = m.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = m.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = m.atan2(t3, t4)
    return roll, pitch, yaw

def quaternion_from_euler(roll, pitch, yaw):
    qx = m.sin(roll/2) * m.cos(pitch/2) * m.cos(yaw/2) - m.cos(roll/2) * m.sin(pitch/2) * m.sin(yaw/2)
    qy = m.cos(roll/2) * m.sin(pitch/2) * m.cos(yaw/2) + m.sin(roll/2) * m.cos(pitch/2) * m.sin(yaw/2)
    qz = m.cos(roll/2) * m.cos(pitch/2) * m.sin(yaw/2) - m.sin(roll/2) * m.sin(pitch/2) * m.cos(yaw/2)
    qw = m.cos(roll/2) * m.cos(pitch/2) * m.cos(yaw/2) + m.sin(roll/2) * m.sin(pitch/2) * m.sin(yaw/2)
    return [qx, qy, qz, qw]

class ekflocdemoviz(Node):
    def __init__(self):
        from rclpy.parameter import Parameter
        super().__init__('ekflocdemoviz', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])
        self.create_subscription(PoseWithCovarianceStamped, '/filtered_odom', self.odom_callback, 10)
        self.create_subscription(Float32MultiArray, '/slam_state', self.state_callback, 10)
        self.path_pub = self.create_publisher(Path, '/slam_path', 10)
        self.landmark_pub = self.create_publisher(MarkerArray, '/slam_landmarks', 10)
        self.path = Path()
        self.path.header.frame_id = "map"
        self.get_logger().info("viz node started")
        
    def odom_callback(self, msg: PoseWithCovarianceStamped):
        posestamped = PoseStamped()
        posestamped.header = msg.header
        posestamped.pose = msg.pose.pose
        self.path.poses.append(posestamped)
        self.path_pub.publish(self.path)
        
    def state_callback(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < 3:
            self.get_logger().warn('state data is empty')
            return 
            
        if len(data) >= 3:
            robot_x = data[0]
            robot_y = data[1]
            robot_yaw = data[2]
            self.get_logger().info(f'Odometry -> x: {robot_x:.3f}, y: {robot_y:.3f}, yaw: {robot_yaw * (180/m.pi):.3f}')
            
        landmarks = [(data[i], data[i+1], data[i+2]) for i in range(3, len(data), 3)]

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for idx, (x, y, r) in enumerate(landmarks):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = now 
            marker.ns = "slam_landmarks"
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.5
            marker.scale.x = r * 2.0
            marker.scale.y = r * 2.0
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker_array.markers.append(marker)
            
        if marker_array.markers:
            self.landmark_pub.publish(marker_array)
            self.get_logger().info("published landmark markers")
        else:
            self.get_logger().info("No landmarks published")

def main(args=None):
    rclpy.init(args=args)
    node = ekflocdemoviz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
