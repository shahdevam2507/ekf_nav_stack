#!/usr/bin/env python3
import os
import math
import numpy as np
from scipy.interpolate import splprep, splev, CubicSpline
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class ReferencePath:
    def __init__(self, waypoints_xy: np.ndarray):
        # Robust downsampling to ~10cm spacing for stable spline fitting
        pts_list = [waypoints_xy[0]]
        for pt in waypoints_xy[1:]:
            if np.hypot(pt[0] - pts_list[-1][0], pt[1] - pts_list[-1][1]) > 0.1:
                pts_list.append(pt)
        pts_list.append(waypoints_xy[-1]) # always keep the exact goal
        pts = np.array(pts_list)

        # Need at least 4 points for a cubic spline
        if len(pts) < 4:
            if len(pts) == 0:
                pts = np.array([[0,0], [1,0], [2,0], [3,0]])
            while len(pts) < 4:
                pts = np.vstack((pts, pts[-1] + [0.01, 0.01]))

        # Fit a standard non-periodic spline (s=0.0 forces strict interpolation)
        tck, _ = splprep([pts[:, 0], pts[:, 1]], s=0.0, k=3)
        u_fine = np.linspace(0, 1, max(100, len(pts)*10))
        x_fine, y_fine = splev(u_fine, tck)

        ds = np.hypot(np.diff(x_fine), np.diff(y_fine))
        self.s = np.concatenate(([0.0], np.cumsum(ds)))
        self.total_length = self.s[-1]

        self.cx = CubicSpline(self.s, x_fine, bc_type='natural')
        self.cy = CubicSpline(self.s, y_fine, bc_type='natural')

    def eval(self, s):
        s = np.clip(s, 0.0, self.total_length)
        return float(self.cx(s)), float(self.cy(s))

    def eval_d(self, s):
        s = np.clip(s, 0.0, self.total_length)
        return float(self.cx(s, 1)), float(self.cy(s, 1))

    def curvature(self, s):
        s = np.clip(s, 0.0, self.total_length)
        dx, dy = self.eval_d(s)
        ddx, ddy = float(self.cx(s, 2)), float(self.cy(s, 2))
        denom = (dx**2 + dy**2)**1.5
        if denom < 1e-6: return 0.0
        return abs(dx * ddy - dy * ddx) / denom

    def heading(self, s):
        dx, dy = self.eval_d(s)
        return math.atan2(dy, dx)

    def find_closest_s(self, x, y, s_guess, window=2.0, n_samples=40):
        s_min = max(0.0, s_guess - window)
        s_max = min(self.total_length, s_guess + window)
        ss = np.linspace(s_min, s_max, n_samples)
        xs = self.cx(ss)
        ys = self.cy(ss)
        dists = (xs - x)**2 + (ys - y)**2
        return float(ss[np.argmin(dists)])


class BasicMPCSolver:
    def __init__(self, ref_path: ReferencePath, N=10, dt=0.1):
        self.ref = ref_path
        self.N = N
        self.dt = dt
        self.v_max = 1.0        
        self.w_max = 1.0        
        self.a_lat_max = 3.5    
        self.u_prev = None

    def get_reference_trajectory(self, s0):
        refs = []
        s = s0
        for _ in range(self.N):
            v_ref = self.v_max
            
            
            dist_to_end = self.ref.total_length - s
            if dist_to_end < 0.5:
                v_ref *= max(0.0, dist_to_end / 0.5)

            s += v_ref * self.dt 
            
            x, y = self.ref.eval(s)
            th = self.ref.heading(s)
            refs.append((x, y, th, v_ref))
            
        return refs

    def solve(self, state0):
        x0, y0, th0 = state0[0], state0[1], state0[2]
        s0 = state0[5]
        
        refs = self.get_reference_trajectory(s0)
        n_vars = self.N * 2  
        
        if self.u_prev is not None:
            u0 = np.roll(self.u_prev.reshape(self.N, 2), -1, axis=0).flatten()
            u0[-2:] = u0[-4:-2]
        else:
            u0 = np.zeros(n_vars)
            for i in range(self.N):
                u0[i*2] = refs[i][3]
                
        bounds = [(0.0, self.v_max), (-self.w_max, self.w_max)] * self.N
            
        def cost_fn(u_flat):
            u_all = u_flat.reshape(self.N, 2)
            J = 0.0
            x, y, th = x0, y0, th0
            
            for i in range(self.N):
                v, w = u_all[i]
                
                x += v * math.cos(th) * self.dt
                y += v * math.sin(th) * self.dt
                th += w * self.dt
                th = wrap_angle(th)
                
                xr, yr, thr, vr = refs[i]
                J += 200.0 * ((x - xr)**2 + (y - yr)**2)
                dth = wrap_angle(th - thr)
                
                J += 100.0 * dth**2
                J += 10.0 * (v - vr)**2
                J += 5.0 * w**2
                
                if i > 0:
                    dv = v - u_all[i-1, 0]
                    dw = w - u_all[i-1, 1]
                    J += 10.0 * dv**2 + 50.0 * dw**2
            return J

        res = minimize(cost_fn, u0, method='SLSQP', bounds=bounds, options={'maxiter': 10, 'disp': False})
        self.u_prev = res.x
        
        v_opt, w_opt = res.x[0], res.x[1]
        return float(v_opt), float(w_opt)


class BasicMPCNode(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        self.declare_parameter('rate', 10.0)
        self.declare_parameter('horizon', 10)
        self.declare_parameter('dt', 0.1)

        rate = self.get_parameter('rate').value
        self.N = self.get_parameter('horizon').value
        self.dt = self.get_parameter('dt').value

        self.ref_path = None
        self.solver = None
        self.ego_state = None
        self.s_progress = 0.0
        self._s_initialized = False
        self.final_goal_yaw = None

        # Subscribes to accurate SLAM position
        self.create_subscription(PoseWithCovarianceStamped, '/filtered_odom', self._odom_cb, 10)
        # Subscribes to A* generated path
        self.create_subscription(Path, '/planned_path', self._path_cb, 10)

        # Publishes velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(1.0 / rate, self._control_tick)
        self.get_logger().info('MPC node started. Awaiting A* /planned_path...')

    def _path_cb(self, msg: Path):
        pts = []
        for p in msg.poses:
            pts.append([p.pose.position.x, p.pose.position.y])
        
        if len(pts) < 2:
            return
            
        waypoints = np.array(pts)
        self.ref_path = ReferencePath(waypoints)
        self.solver = BasicMPCSolver(self.ref_path, N=self.N, dt=self.dt)
        
        # Reset MPC tracking states on new path
        self.solver.u_prev = None
        self._s_initialized = False
        self.s_progress = 0.0
        self.final_goal_yaw = quat_to_yaw(msg.poses[-1].pose.orientation)
        self.get_logger().info(f'New reference trajectory active: length={self.ref_path.total_length:.1f}m, goal_yaw={self.final_goal_yaw:.2f}')

    def _odom_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q)
        # Initialize ego_state: [x, y, th, v, w]
        self.ego_state = np.array([p.x, p.y, yaw, 0.0, 0.0])

    def _control_tick(self):
        if self.ref_path is None or self.solver is None or self.ego_state is None:
            return

        x, y, th, v, w = self.ego_state

        if not self._s_initialized:
            self.s_progress = self.ref_path.find_closest_s(x, y, 0.0, window=self.ref_path.total_length, n_samples=100)
            self._s_initialized = True
        else:
            self.s_progress = self.ref_path.find_closest_s(x, y, self.s_progress, window=2.0)

        # Stop command if we've reached the end of the trajectory
        dist_to_end = self.ref_path.total_length - self.s_progress
        if dist_to_end < 0.15:
            # Reached goal position, now turn to align with goal orientation
            if self.final_goal_yaw is not None:
                yaw_error = wrap_angle(self.final_goal_yaw - th)
                if abs(yaw_error) > 0.05: # roughly 3 degrees tolerance
                    cmd = Twist()
                    cmd.angular.z = np.clip(1.5 * yaw_error, -self.solver.w_max, self.solver.w_max)
                    self.cmd_pub.publish(cmd)
                    return
                    
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.ref_path = None # Clear path so we don't keep tracking
            self.get_logger().info('Goal reached and aligned! Awaiting new path.')
            return

        # State Machine: If heading error is huge, turn in place before launching MPC
        # This prevents SLSQP from falling into a v=0 local minimum
        _, _, path_yaw, _ = self.solver.get_reference_trajectory(self.s_progress)[0]
        initial_yaw_err = wrap_angle(path_yaw - th)
        
        if abs(initial_yaw_err) > math.pi / 4.0: # 45 degrees
            cmd = Twist()
            cmd.angular.z = np.clip(1.5 * initial_yaw_err, -self.solver.w_max, self.solver.w_max)
            self.cmd_pub.publish(cmd)
            self.solver.u_prev = None # Reset MPC guess
            return

        state = np.array([x, y, th, v, w, self.s_progress])
        v_cmd, w_cmd = self.solver.solve(state)
        
        # Log to terminal so we can guarantee the solver is outputting a command!
        self.get_logger().info(f"MPC Command -> v: {v_cmd:.2f}, w: {w_cmd:.2f} | Progress: {self.s_progress:.2f}/{self.ref_path.total_length:.2f}m")

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BasicMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
