import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32MultiArray
import numpy as np
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

def norm_angle(angle):
    return m.atan2(m.sin(angle), m.cos(angle))

import yaml

class ekflocnode(Node):
    def __init__(self):
        from rclpy.parameter import Parameter
        super().__init__('ekflocnode', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])

        self.odom_sub  = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub  = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub       = self.create_publisher(PoseWithCovarianceStamped, '/filtered_odom', 10)
        self.state_pub = self.create_publisher(Float32MultiArray, '/slam_state', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer     = self.create_timer(0.20, self.timer_callback)

        # Odometry deltas
        self.Dt   = 0.0
        self.Dr1  = 0.0
        self.Dr2  = 0.0
        
        # Latest raw odom pose (updated on every /odom message)
        self.odom_x      = None
        self.odom_y      = None
        self.odom_theta  = None

        # Pose snapshot taken at the last predict step
        self.pred_x      = None
        self.pred_y      = None
        self.pred_theta  = None

        self.m_x_prev    = None
        self.m_y_prev    = None
        self.m_theta_prev = None

        # EKF Matrices
        self.state_matrix  = None  # Shape (3 + 3N, 1)
        self.dimension     = 3
        
        self.sigma_x       = 0.05
        self.sigma_y       = 0.05
        self.sigma_theta   = 0.02
        
        self.covar_matrix      = np.eye(self.dimension) * 0.1
        self.pred_covar_matrix = np.eye(self.dimension) * 0.1
        
        # LiDAR configuration & processing
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.scan  = []
        self.diff  = []
        self.zm    = np.array([]).reshape(3, 0)
        
        # Tracking landmark match associations
        self.matched_landmarks = []
        self.new_landmarks     = []
        
        # Determine latest map file or use a specific one
        save_dir = "/home/jival/ekf_nav_stack/src/saved_maps"
        map_id = 1
        while os.path.exists(os.path.join(save_dir, f"saved_map_{map_id+1}.yaml")):
            map_id += 1
        map_file = os.path.join(save_dir, f"saved_map_{map_id}.yaml")
        
        self.load_map(map_file)

    def load_map(self, filename):
        self.get_logger().info(f"Loading map from {filename}")
        try:
            with open(filename, 'r') as f:
                map_data = yaml.safe_load(f)
            
            landmarks = map_data.get('landmarks', [])
            num_landmarks = len(landmarks)
            
            self.dimension = 3 + 3 * num_landmarks
            self.state_matrix = np.zeros((self.dimension, 1))
            self.covar_matrix = np.eye(self.dimension) * 1e-6
            
            for i, lm in enumerate(landmarks):
                lm_idx = 3 + 3 * i
                self.state_matrix[lm_idx, 0] = lm['position']['x']
                self.state_matrix[lm_idx+1, 0] = lm['position']['y']
                self.state_matrix[lm_idx+2, 0] = lm['position'].get('r', 0.3)
                
            self.get_logger().info(f"Loaded {num_landmarks} landmarks.")
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")

    def odom_callback(self, msg: Odometry):
        m_x = msg.pose.pose.position.x
        m_y = msg.pose.pose.position.y
        ort = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ort.x, ort.y, ort.z, ort.w])
        m_theta = norm_angle(yaw)

        # Cache the latest raw pose; deltas are computed once per tick at predict time
        self.odom_x     = m_x
        self.odom_y     = m_y
        self.odom_theta = m_theta

        # Initialize snapshot + EKF state on the first message
        if self.pred_x is None:
            self.pred_x     = m_x
            self.pred_y     = m_y
            self.pred_theta = m_theta
            if self.state_matrix is None:
                self.state_matrix = np.array([[m_x], [m_y], [m_theta]])

        # Initialize tracking on the very first frame
        if self.m_x_prev is None:
            self.m_x_prev = m_x
            self.m_y_prev = m_y
            self.m_theta_prev = m_theta
            # Initialize EKF state at first known odom pose
            if self.state_matrix is not None:
                self.state_matrix[0, 0] = m_x
                self.state_matrix[1, 0] = m_y
                self.state_matrix[2, 0] = m_theta
            return

        # Calculate control deltas (Odometry Motion Model)
        self.Dt  = m.sqrt((m_x - self.m_x_prev)**2 + (m_y - self.m_y_prev)**2)
        if self.Dt > 0.001:
            self.Dr1 = norm_angle(m.atan2(m_y - self.m_y_prev, m_x - self.m_x_prev) - self.m_theta_prev)
            self.Dr2 = norm_angle(m_theta - self.m_theta_prev - self.Dr1)
        else:
            self.Dr1 = 0.0
            self.Dr2 = norm_angle(m_theta - self.m_theta_prev)

        self.m_x_prev     = m_x
        self.m_y_prev     = m_y
        self.m_theta_prev = m_theta

    def compute_motion(self):
        # One odometry-motion-model delta over the whole tick interval:
        # snapshot (last predict) -> latest cached odom pose.
        if self.odom_x is None or self.pred_x is None:
            self.Dt = 0.0
            self.Dr1 = 0.0
            self.Dr2 = 0.0
            return

        dx = self.odom_x - self.pred_x
        dy = self.odom_y - self.pred_y
        self.Dt = m.sqrt(dx * dx + dy * dy)

        if self.Dt > 0.001:
            self.Dr1 = norm_angle(m.atan2(dy, dx) - self.pred_theta)
            self.Dr2 = norm_angle(self.odom_theta - self.pred_theta - self.Dr1)
        else:
            self.Dr1 = 0.0
            self.Dr2 = norm_angle(self.odom_theta - self.pred_theta)

        # Advance snapshot so the next interval starts where this one ended
        self.pred_x     = self.odom_x
        self.pred_y     = self.odom_y
        self.pred_theta = self.odom_theta

    def predict_state(self):
        if self.state_matrix is None:
            return

        # Current estimated orientation
        theta = self.state_matrix[2, 0]

        # Apply standard odometry motion model updates to the robot pose slice
        self.state_matrix[0, 0] += self.Dt * m.cos(theta + self.Dr1)
        self.state_matrix[1, 0] += self.Dt * m.sin(theta + self.Dr1)
        self.state_matrix[2, 0] = norm_angle(theta + self.Dr1 + self.Dr2)


    def compute_Gt(self):  
        if self.state_matrix is None:
            return
            
        self.dimension = self.state_matrix.shape[0]
        theta = self.state_matrix[2, 0]
        
        # Jacobian with respect to the robot state
        Gtx = np.eye(3)
        Gtx[0, 2] = -self.Dt * m.sin(theta + self.Dr1)
        Gtx[1, 2] =  self.Dt * m.cos(theta + self.Dr1)

        # Map to the full system dimension matrix
        self.Gt_matrix = np.eye(self.dimension)
        self.Gt_matrix[0:3, 0:3] = Gtx

    def control_covar_matrix(self):
        if self.state_matrix is None:
            return

        # Mapping controls to state uncertainty space
        V = np.zeros((self.dimension, 3))
        theta = self.state_matrix[2, 0]
        
        V[0, 0] = m.cos(theta + self.Dr1)
        V[0, 1] = -self.Dt * m.sin(theta + self.Dr1)
        V[1, 0] = m.sin(theta + self.Dr1)
        V[1, 1] = self.Dt * m.cos(theta + self.Dr1)
        V[2, 1] = 1.0
        V[2, 2] = 1.0

        Rt = np.diag([self.sigma_x**2, self.sigma_y**2, self.sigma_theta**2])
        self.R = V @ Rt @ V.T

    def predict_covariance(self): 
        if self.state_matrix is None:
            return
        # Propagate uncertainty through the motion model step
        self.pred_covar_matrix = self.Gt_matrix @ self.covar_matrix @ self.Gt_matrix.T + self.R
        self.covar_matrix = self.pred_covar_matrix.copy()

    def scan_callback(self, msg: LaserScan):
        # Dynamically map incoming hardware laser frames instead of assuming fixed geometry
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        
        n = len(msg.ranges)
        self.scan = [0.0] * n   
        self.diff = [0.0] * n   

        for i in range(n):
            if m.isinf(msg.ranges[i]) or m.isnan(msg.ranges[i]):
                self.scan[i] = 5.0
            else:
                self.scan[i] = msg.ranges[i]

            if i > 0 and self.scan[i] > 0.3 and self.scan[i-1] > 0.3:
                diff_val = self.scan[i] - self.scan[i-1]
            else:
                diff_val = 0.0

            self.diff[i] = diff_val if abs(diff_val) > 0.15 else 0.0

    def landmark_detect(self):
        if not self.scan:
            return

        obj_detected = 0
        add_ranges   = 0.0
        indices      = 0
        add_index    = 0
        min_range    = 0.0
        lidar_range  = []
        lidar_angle  = []
        lidar_radius = []

        j = 0
        while j < len(self.diff):
            if self.diff[j] < 0 and obj_detected == 0:
                obj_detected = 1
                indices      = 1
                add_index    = j
                add_ranges   = self.scan[j]
                min_range    = self.scan[j]
            elif self.diff[j] < 0 and obj_detected == 1:
                obj_detected = 0
                indices      = 0
                add_index    = 0
                add_ranges   = 0.0
                j -= 1
            elif self.diff[j] == 0 and obj_detected == 1:
                indices   += 1
                add_ranges += self.scan[j]
                add_index  += j
                if self.scan[j] < min_range:
                    min_range = self.scan[j]
            elif self.diff[j] > 0 and obj_detected == 1:
                obj_detected = 0
                if indices > 0:
                    # Dynamically calculate true geometric angle base on incoming topic metadata
                    avg_idx = add_index / indices
                    mean_angle = norm_angle(self.angle_min + (avg_idx * self.angle_increment))
                    mean_range = add_ranges / indices
                    
                    # The total angle spanned by the cluster
                    theta_span = indices * self.angle_increment
                    
                    # Exact geometric radius calculation from tangents: sin(theta/2) = R / (D_min + R)
                    sin_half_theta = m.sin(theta_span / 2.0)
                    
                    if 0.0 < sin_half_theta < 0.99:
                        exact_radius = (min_range * sin_half_theta) / (1.0 - sin_half_theta)
                    else:
                        exact_radius = 0.0
                        
                    # Calculate approximated arc width for wall filtering
                    width = mean_range * theta_span
                    
                    # A cylinder has a width between 0.2 and 1.2 meters. 
                    # Walls will be much larger.
                    if 0.1 < min_range < 5.0 and 0.2 < width < 1.2:
                        # Bound the radius loosely to filter out mathematically erratic calculations on noisy borders
                        estimated_radius = min(max(exact_radius, 0.2), 0.6)
                        
                        lidar_range.append(min_range + estimated_radius)
                        lidar_angle.append(mean_angle)
                        lidar_radius.append(estimated_radius)
                indices   = 0
                add_index = 0
                add_ranges = 0.0
            j += 1

        if lidar_range:
            self.zm = np.array([lidar_range, lidar_angle, lidar_radius])
            self.get_logger().info(f"Landmarks detected: {self.zm.shape[1]}")
        else:
            self.zm = np.array([]).reshape(3, 0)

    def state_match(self):
        self.matched_landmarks = []
        self.new_landmarks     = []

        if self.zm.size == 0:
            return

        # If no landmarks map exists yet, everything found is treated as completely new
        if self.state_matrix is None or self.state_matrix.shape[0] <= 3:
            return

        matched_obs       = set()
        matched_landmarks = set()
        r_threshold       = 0.6
        phi_threshold     = 25.0 * m.pi / 180.0
        possible_matches  = []

        rb_x     = self.state_matrix[0, 0]
        rb_y     = self.state_matrix[1, 0]
        rb_theta = self.state_matrix[2, 0]

        # Scan global landscape estimates currently residing in state matrix space
        num_existing_landmarks = (self.state_matrix.shape[0] - 3) // 3
        
        for i in range(self.zm.shape[1]): # Iterating observations
            z_r = self.zm[0, i]
            z_phi = self.zm[1, i]
            
            for j in range(num_existing_landmarks): # Iterating established states
                lm_idx = 3 + 3 * j
                dx = self.state_matrix[lm_idx, 0] - rb_x
                dy = self.state_matrix[lm_idx + 1, 0] - rb_y
                
                pred_r = m.sqrt(dx**2 + dy**2)
                pred_phi = norm_angle(m.atan2(dy, dx) - rb_theta)
                
                r_diff   = abs(z_r - pred_r)
                phi_diff = abs(norm_angle(z_phi - pred_phi))
                
                # Convert the radial/angular error into a true Cartesian distance error
                # using the arc length (pred_r * phi_diff) for the angular component
                cartesian_distance_error = m.sqrt(r_diff**2 + (pred_r * phi_diff)**2)
                
                if r_diff < r_threshold and phi_diff < phi_threshold:
                    possible_matches.append((cartesian_distance_error, i, j))

        possible_matches.sort(key=lambda x: x[0])

        for distance, obs_idx, landmark_idx in possible_matches:
            if obs_idx not in matched_obs and landmark_idx not in matched_landmarks:
                self.matched_landmarks.append((obs_idx, landmark_idx))
                matched_obs.add(obs_idx)
                matched_landmarks.add(landmark_idx)

        # Localization ONLY: Do not add new landmarks to the map
        pass

    def add_new_landmarks(self):
        num_new_landmarks = len(self.new_landmarks)
        if num_new_landmarks == 0:
            return

        current_size = self.state_matrix.shape[0]
        new_size     = current_size + 3 * num_new_landmarks
        
        new_state = np.zeros((new_size, 1))
        new_state[:current_size, 0] = self.state_matrix[:, 0]

        robot_x     = self.state_matrix[0, 0]
        robot_y     = self.state_matrix[1, 0]
        robot_theta = self.state_matrix[2, 0]

        new_covar = np.zeros((new_size, new_size))
        new_covar[:current_size, :current_size] = self.covar_matrix
        landmark_uncertainty = 0.5  # Realistic initial observation mapping covariance error allocation

        for idx, obs_idx in enumerate(self.new_landmarks):
            lm_state_idx = current_size + 3 * idx
            range_val    = self.zm[0, obs_idx]
            phi          = self.zm[1, obs_idx]
            radius_val   = self.zm[2, obs_idx]
            
            # Non-linear observation translation projection backwards into Cartesian system space
            landmark_x   = robot_x + range_val * m.cos(robot_theta + phi)
            landmark_y   = robot_y + range_val * m.sin(robot_theta + phi)
            
            new_state[lm_state_idx,     0] = landmark_x
            new_state[lm_state_idx + 1, 0] = landmark_y
            new_state[lm_state_idx + 2, 0] = radius_val
            
            # Assigning diagonal initial variance maps
            new_covar[lm_state_idx,     lm_state_idx]     = landmark_uncertainty
            new_covar[lm_state_idx + 1, lm_state_idx + 1] = landmark_uncertainty
            new_covar[lm_state_idx + 2, lm_state_idx + 2] = 0.1

        self.state_matrix = new_state
        self.covar_matrix = new_covar
        self.dimension    = new_size

    def correction_step(self):
        if not hasattr(self, 'matched_landmarks') or not self.matched_landmarks:
            return

        R_sensor = np.diag([0.05**2, 0.02**2, 0.05**2])

        # Loop observations sequentially
        for obs_idx, landmark_idx in self.matched_landmarks:
            # Recompute parameters dynamically using up-to-date states mutated within this cycle loop
            robot_x     = self.state_matrix[0, 0]
            robot_y     = self.state_matrix[1, 0]
            robot_theta = self.state_matrix[2, 0]

            landmark_state_idx = 3 + 3 * landmark_idx
            landmark_x = self.state_matrix[landmark_state_idx,     0]
            landmark_y = self.state_matrix[landmark_state_idx + 1, 0]   
            landmark_r = self.state_matrix[landmark_state_idx + 2, 0]

            del_x  = landmark_x - robot_x
            del_y  = landmark_y - robot_y
            q      = del_x**2 + del_y**2                
            sqrt_q = m.sqrt(q)

            if q < 1e-5:
                continue

            # Expected measurement h calculation computed fresh
            pred_r = sqrt_q
            pred_phi = norm_angle(m.atan2(del_y, del_x) - robot_theta)
            h = np.array([[pred_r], [pred_phi], [landmark_r]])

            z = np.array([[self.zm[0, obs_idx]], [self.zm[1, obs_idx]], [self.zm[2, obs_idx]]])
            y      = z - h
            y[1, 0] = norm_angle(y[1, 0])

            # Recalculate full-dimension Jacobian mapping space coordinates
            H = np.zeros((3, self.dimension))
            H[0, 0]                     = -del_x / sqrt_q
            H[0, 1]                     = -del_y / sqrt_q
            H[0, 2]                     =  0.0
            H[0, landmark_state_idx]    =  del_x / sqrt_q
            H[0, landmark_state_idx+1]  =  del_y / sqrt_q
            H[1, 0]                     =  del_y / q
            H[1, 1]                     = -del_x / q
            H[1, 2]                     = -1.0
            H[1, landmark_state_idx]    = -del_y / q
            H[1, landmark_state_idx+1]  =  del_x / q
            H[2, landmark_state_idx+2]  =  1.0

            # Kalman gain calculation steps
            S = H @ self.covar_matrix @ H.T + R_sensor
            K = self.covar_matrix @ H.T @ np.linalg.inv(S)

            # Apply immediate localized updates to systemic map array matrices
            self.state_matrix = self.state_matrix + K @ y
            self.state_matrix[2, 0] = norm_angle(self.state_matrix[2, 0]) # Keep heading normalized
            
            I = np.eye(self.dimension)
            self.covar_matrix = (I - K @ H) @ self.covar_matrix

    def timer_callback(self):  
        # Wait until the map is loaded AND the first odometry frame is received
        if self.state_matrix is None or self.odom_x is None:
            return

        try:
            # 1. EKF State and Covariance Prediction (Motion Model Execution)
            self.compute_motion()
            self.predict_state()
            self.compute_Gt()          
            self.control_covar_matrix()
            self.predict_covariance()  
            
            # 2. Sensor Processing & Measurement Integration (Correction Phase)
            self.landmark_detect()
            self.state_match()          
            self.correction_step()

            # 3. Publish Updated Odometry Metrics
            pmsg = PoseWithCovarianceStamped()
            pmsg.header.stamp = self.get_clock().now().to_msg()
            pmsg.header.frame_id = "map"
            pmsg.pose.pose.position.x = self.state_matrix[0, 0]
            pmsg.pose.pose.position.y = self.state_matrix[1, 0]
            
            ort = quaternion_from_euler(0, 0, self.state_matrix[2, 0])
            pmsg.pose.pose.orientation.x = ort[0]
            pmsg.pose.pose.orientation.y = ort[1]
            pmsg.pose.pose.orientation.z = ort[2]
            pmsg.pose.pose.orientation.w = ort[3]
            self.pub.publish(pmsg)              
            
            # Broadcast map -> odom transform for RViz visualization
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            
            del_theta = norm_angle(self.state_matrix[2, 0] - self.odom_theta)
            del_x = self.state_matrix[0, 0] - (self.odom_x * m.cos(del_theta) - self.odom_y * m.sin(del_theta))
            del_y = self.state_matrix[1, 0] - (self.odom_x * m.sin(del_theta) + self.odom_y * m.cos(del_theta))
            
            t.transform.translation.x = float(del_x)
            t.transform.translation.y = float(del_y)
            t.transform.translation.z = 0.0
            
            q = quaternion_from_euler(0, 0, del_theta)
            t.transform.rotation.x = float(q[0])
            t.transform.rotation.y = float(q[1])
            t.transform.rotation.z = float(q[2])
            t.transform.rotation.w = float(q[3])
            
            self.tf_broadcaster.sendTransform(t)

            # 4. Broadcast global tracking array definitions
            state_msg      = Float32MultiArray()
            state_msg.data = self.state_matrix.flatten().tolist()
            self.state_pub.publish(state_msg)  

            self.get_logger().info(
                f"State size: {self.state_matrix.shape[0]}  "
                f"Landmarks: {(self.state_matrix.shape[0]-3)//3}")

        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {str(e)}")

    def save_map(self):
        if self.state_matrix is None or self.state_matrix.shape[0] <= 3:
            self.get_logger().info("No landmarks to save.")
            return
        
        try:
            # Determine save directory
            save_dir = "/home/jival/ekf_nav_stack/src/saved_maps"
            os.makedirs(save_dir, exist_ok=True)
            
            # Find next available ID
            map_id = 1
            while os.path.exists(os.path.join(save_dir, f"saved_map_{map_id}.yaml")):
                map_id += 1
                
            filename = os.path.join(save_dir, f"saved_map_{map_id}.yaml")
            
            with open(filename, 'w') as f:
                f.write("landmarks:\n")
                num_landmarks = (self.state_matrix.shape[0] - 3) // 3
                for i in range(num_landmarks):
                    lm_idx = 3 + 3 * i
                    x = self.state_matrix[lm_idx, 0]
                    y = self.state_matrix[lm_idx + 1, 0]
                    r = self.state_matrix[lm_idx + 2, 0]
                    var_x = self.covar_matrix[lm_idx, lm_idx]
                    var_y = self.covar_matrix[lm_idx + 1, lm_idx + 1]
                    var_r = self.covar_matrix[lm_idx + 2, lm_idx + 2]
                    
                    f.write(f"  - id: {i}\n")
                    f.write(f"    position:\n")
                    f.write(f"      x: {x:.4f}\n")
                    f.write(f"      y: {y:.4f}\n")
                    f.write(f"      r: {r:.4f}\n")
                    f.write(f"    variance:\n")
                    f.write(f"      x: {var_x:.6f}\n")
                    f.write(f"      y: {var_y:.6f}\n")
                    f.write(f"      r: {var_r:.6f}\n")
            self.get_logger().info(f"Successfully saved {num_landmarks} landmarks to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")

def main(args=None):
    rclpy.init(args=args)
    ekf_node = ekflocnode()   
    try:
        rclpy.spin(ekf_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        ekf_node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        ekf_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()