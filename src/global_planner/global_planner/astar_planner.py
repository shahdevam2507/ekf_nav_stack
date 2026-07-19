import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker
import yaml
import os
import math
import heapq

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        
        # Grid settings
        self.resolution = 0.025 # 2.5 cm per cell
        self.origin_x = -10.0
        self.origin_y = -10.0
        self.width = 800 # 20m wide
        self.height = 800 # 20m high
        
        # The inflation radius ensures the robot won't clip the cylinders.
        # Reduced to 0.15m for tight topological planning (CBF will handle exact collision avoidance).
        self.robot_radius = 0.15 
        
        self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
        self.landmarks = []
        
        self.current_pose = None
        
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/filtered_odom', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.path_marker_pub = self.create_publisher(Marker, '/planned_path_marker', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/global_costmap', 10)
        
        self.load_map()
        self.build_costmap()
        
        # Publish costmap periodically so RViz can display the inflated obstacles
        self.timer = self.create_timer(1.0, self.publish_costmap)
        
    def load_map(self):
        # find map
        save_dir = "/home/rover/custom_nav_stack/src/saved_maps"
        map_id = 1
        while os.path.exists(os.path.join(save_dir, f"saved_map_{map_id+1}.yaml")):
            map_id += 1
            
        map_file = os.path.join(save_dir, f"saved_map_{map_id}.yaml")
        if os.path.exists(map_file):
            with open(map_file, 'r') as f:
                data = yaml.safe_load(f)
                if 'landmarks' in data:
                    self.landmarks = data['landmarks']
            self.get_logger().info(f"Loaded {len(self.landmarks)} landmarks from {map_file}")
            
    def build_costmap(self):
        for y in range(self.height):
            for x in range(self.width):
                wx = self.origin_x + x * self.resolution + self.resolution/2.0
                wy = self.origin_y + y * self.resolution + self.resolution/2.0
                
                occupied = False
                for lm in self.landmarks:
                    lx = lm['position']['x']
                    ly = lm['position']['y']
                    # We default to 0.3 if radius is missing in legacy maps
                    lr = lm['position'].get('r', 0.3) 
                    
                    dist = math.sqrt((wx - lx)**2 + (wy - ly)**2)
                    # If distance to center is less than cylinder radius + robot radius
                    if dist <= (lr + self.robot_radius):
                        occupied = True
                        break
                        
                if occupied:
                    self.grid[y][x] = 100 # 100 means definitely occupied in standard OccupancyGrid
                else:
                    self.grid[y][x] = 0
                    
    def publish_costmap(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        
        # Flatten grid for 1D array message format
        flat_grid = []
        for y in range(self.height):
            for x in range(self.width):
                flat_grid.append(self.grid[y][x])
                
        msg.data = flat_grid
        self.map_pub.publish(msg)
        
    def pose_callback(self, msg):
        # Update current pose from EKF
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy
        
    def grid_to_world(self, gx, gy):
        x = self.origin_x + gx * self.resolution + self.resolution/2.0
        y = self.origin_y + gy * self.resolution + self.resolution/2.0
        return x, y
        
    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
    def goal_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().warn("Cannot plan: Unknown current pose (waiting for /filtered_odom)")
            return
            
        start_world = self.current_pose
        goal_world = (msg.pose.position.x, msg.pose.position.y)
        
        start_grid = self.world_to_grid(*start_world)
        goal_grid = self.world_to_grid(*goal_world)
        
        self.get_logger().info(f"Planning from {start_grid} to {goal_grid}")
        
        if not (0 <= start_grid[0] < self.width and 0 <= start_grid[1] < self.height):
            self.get_logger().warn("Start is outside the map")
            return
            
        if not (0 <= goal_grid[0] < self.width and 0 <= goal_grid[1] < self.height):
            self.get_logger().warn("Goal is outside the map")
            return
            
        if self.grid[goal_grid[1]][goal_grid[0]] == 100:
            self.get_logger().warn("Goal is inside an obstacle! Please pick a valid point.")
            return
            
        # Standard A* Search
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {}
        cost_so_far = {}
        came_from[start_grid] = None
        cost_so_far[start_grid] = 0
        
        found = False
        
        # 8-connected grid (horizontal, vertical, and diagonals)
        neighbors = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        
        while frontier:
            _, current = heapq.heappop(frontier)
            
            if current == goal_grid:
                found = True
                break
                
            for dx, dy in neighbors:
                nx, ny = current[0] + dx, current[1] + dy
                
                # Check bounds
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    # Check collision
                    if self.grid[ny][nx] == 100:
                        continue
                        
                    # Diagonal moves cost more than straight moves
                    move_cost = math.sqrt(dx**2 + dy**2)
                    new_cost = cost_so_far[current] + move_cost
                    
                    next_node = (nx, ny)
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        cost_so_far[next_node] = new_cost
                        priority = new_cost + self.heuristic(next_node, goal_grid)
                        heapq.heappush(frontier, (priority, next_node))
                        came_from[next_node] = current
                        
        if found:
            self.get_logger().info("Path found! Publishing...")
            path = []
            curr = goal_grid
            while curr != start_grid:
                path.append(curr)
                curr = came_from[curr]
            path.append(start_grid)
            path.reverse()
            
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'
            
            marker_msg = Marker()
            marker_msg.header = path_msg.header
            marker_msg.ns = "astar_path"
            marker_msg.id = 0
            marker_msg.type = Marker.LINE_STRIP
            marker_msg.action = Marker.ADD
            marker_msg.scale.x = 0.08 # 8cm thick line
            marker_msg.color.a = 1.0
            marker_msg.color.r = 0.1
            marker_msg.color.g = 1.0
            marker_msg.color.b = 0.1
            
            for p in path:
                wx, wy = self.grid_to_world(*p)
                
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = wx
                pose.pose.position.y = wy
                path_msg.poses.append(pose)
                
                point = Point()
                point.x = wx
                point.y = wy
                point.z = 0.1
                marker_msg.points.append(point)
                
            # Embed the final user-requested orientation into the very last waypoint
            if len(path_msg.poses) > 0:
                path_msg.poses[-1].pose.orientation = msg.pose.orientation
                
            self.path_pub.publish(path_msg)
            self.path_marker_pub.publish(marker_msg)
        else:
            self.get_logger().warn("No path found! Is the goal completely blocked?")

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
