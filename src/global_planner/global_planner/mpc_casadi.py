#!/usr/bin/env python3
import os
import math
import numpy as np
from scipy.interpolate import splprep, splev, CubicSpline
import casadi as ca
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
        pts_list = [waypoints_xy[0]]
        for pt in waypoints_xy[1:]:
            if np.hypot(pt[0] - pts_list[-1][0], pt[1] - pts_list[-1][1]) > 0.1:
                pts_list.append(pt)
        pts_list.append(waypoints_xy[-1])
        pts = np.array(pts_list)

        if len(pts) < 4:
            if len(pts) == 0:
                pts = np.array([[0,0], [1,0], [2,0], [3,0]])
            while len(pts) < 4:
                pts = np.vstack((pts, pts[-1] + [0.01, 0.01]))

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
        self._build_solver()

    def _build_solver(self):
        N, dt = self.N, self.dt
        opti = ca.Opti()
        V = opti.variable(N)
        W = opti.variable(N)

        x0 = opti.parameter()
        y0 = opti.parameter()
        th0 = opti.parameter()
        xref = opti.parameter(N)
        yref = opti.parameter(N)
        thref = opti.parameter(N)
        vref = opti.parameter(N)

        x, y, th = x0, y0, th0
        J = 0
        for i in range(N):
            x = x + V[i] * ca.cos(th) * dt
            y = y + V[i] * ca.sin(th) * dt
            th = th + W[i] * dt
            dth = ca.atan2(ca.sin(th - thref[i]), ca.cos(th - thref[i]))
            J += 200.0 * ((x - xref[i])**2 + (y - yref[i])**2)
            J += 100.0 * dth ** 2
            J += 10.0 * (V[i] - vref[i]) ** 2
            J += 5.0 * W[i] ** 2
            if i > 0:
                J += 10.0 * (V[i] - V[i-1]) ** 2
                J += 50.0 * (W[i] - W[i-1]) ** 2

        opti.minimize(J)
        opti.subject_to(opti.bounded(0.0, V, self.v_max))
        opti.subject_to(opti.bounded(-self.w_max, W, self.w_max))
        for i in range(N):
            opti.subject_to(opti.bounded(-self.a_lat_max, V[i] * W[i], self.a_lat_max))

        p_opts = {"expand": True, "print_time": False}
        s_opts = {"max_iter": 10, "print_level": 0, "sb": "yes",
                   "warm_start_init_point": "yes"}
        opti.solver('ipopt', p_opts, s_opts)

        self.opti = opti
        self.V, self.W = V, W
        self.x0, self.y0, self.th0 = x0, y0, th0
        self.xref, self.yref, self.thref, self.vref = xref, yref, thref, vref

    def get_reference_trajectory(self, s0):
        refs = []
        s = s0
        for _ in range(self.N):
            dist_to_end = self.ref.total_length - s
            vref = self.v_max
            if dist_to_end < 0.5:
                vref = self.v_max * max(0.0, dist_to_end / 0.5)
            s = s + vref * self.dt
            x, y = self.ref.eval(s)
            th = self.ref.heading(s)
            refs.append((x, y, th, vref))
        return refs

    def solve(self, state):
        x0, y0, th0 = state[0], state[1], state[2]
        s0 = state[5]
        refs = self.get_reference_trajectory(s0)
        xr = [r[0] for r in refs]
        yr = [r[1] for r in refs]
        thr = [r[2] for r in refs]
        vr = [r[3] for r in refs]

        self.opti.set_value(self.x0, x0)
        self.opti.set_value(self.y0, y0)
        self.opti.set_value(self.th0, th0)
        self.opti.set_value(self.xref, xr)
        self.opti.set_value(self.yref, yr)
        self.opti.set_value(self.thref, thr)
        self.opti.set_value(self.vref, vr)

        if self.u_prev is not None:
            v_ini = np.roll(self.u_prev[:, 0], -1)
            v_ini[-1] = v_ini[-2]
            w_ini = np.roll(self.u_prev[:, 1], -1)
            w_ini[-1] = w_ini[-2]
        else:
            v_ini = np.array(vr)
            w_ini = np.zeros(self.N)

        self.opti.set_initial(self.V, v_ini)
        self.opti.set_initial(self.W, w_ini)

        try:
            sol = self.opti.solve()
            v_sol = sol.value(self.V)
            w_sol = sol.value(self.W)
        except RuntimeError:
            v_sol = self.opti.debug.value(self.V)
            w_sol = self.opti.debug.value(self.W)

        self.u_prev = np.stack([np.atleast_1d(v_sol), np.atleast_1d(w_sol)], axis=1)
        return float(np.atleast_1d(v_sol)[0]), float(np.atleast_1d(w_sol)[0])


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
        self.bot_state = None
        self.s_progress = 0.0
        self._s_initialized = False
        self.final_goal_yaw = None

        self.create_subscription(PoseWithCovarianceStamped, '/filtered_odom', self._odom_cb, 10)
        self.create_subscription(Path, '/planned_path', self._path_cb, 10)
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

        self.solver.u_prev = None
        self._s_initialized = False
        self.s_progress = 0.0
        self.final_goal_yaw = quat_to_yaw(msg.poses[-1].pose.orientation)
        self.get_logger().info(f'New reference trajectory active: length={self.ref_path.total_length:.1f}m, goal_yaw={self.final_goal_yaw:.2f}')

    def _odom_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q)
        self.bot_state = np.array([p.x, p.y, yaw, 0.0, 0.0])

    def _control_tick(self):
        if self.ref_path is None or self.solver is None or self.bot_state is None:
            return

        x, y, th, v, w = self.bot_state

        if not self._s_initialized:
            self.s_progress = self.ref_path.find_closest_s(x, y, 0.0, window=self.ref_path.total_length, n_samples=100)
            self._s_initialized = True
        else:
            self.s_progress = self.ref_path.find_closest_s(x, y, self.s_progress, window=2.0)

        dist_to_end = self.ref_path.total_length - self.s_progress
        if dist_to_end < 0.15:
            if self.final_goal_yaw is not None:
                yaw_error = wrap_angle(self.final_goal_yaw - th)
                if abs(yaw_error) > 0.05:
                    cmd = Twist()
                    cmd.angular.z = np.clip(1.5 * yaw_error, -self.solver.w_max, self.solver.w_max)
                    self.cmd_pub.publish(cmd)
                    return

            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.ref_path = None
            self.get_logger().info('Goal reached and aligned! Awaiting new path.')
            return
        _, _, path_yaw, _ = self.solver.get_reference_trajectory(self.s_progress)[0]
        initial_yaw_err = wrap_angle(path_yaw - th)

        if abs(initial_yaw_err) > math.pi / 4.0:
            cmd = Twist()
            cmd.angular.z = np.clip(1.5 * initial_yaw_err, -self.solver.w_max, self.solver.w_max)
            self.cmd_pub.publish(cmd)
            self.solver.u_prev = None
            return

        state = np.array([x, y, th, v, w, self.s_progress])
        v_cmd, w_cmd = self.solver.solve(state)

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