
import math
from typing import Optional, Deque, List, Tuple
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.msg import BehaviorTreeLog
from nav2_msgs.action import NavigateToPose, Spin, BackUp, ComputePathToPose
from builtin_interfaces.msg import Duration as RosDuration
from std_srvs.srv import Empty  # for costmap clearing


@dataclass
class Params:
    mode: str = "explore"                  # "loop" or "explore"
    # Stuck detection
    stuck_window_sec: float = 3.0
    min_progress_m: float = 0.05
    # Recovery
    backup_distance_m: float = 0.15
    backup_speed_mps: float = 0.05
    spin_angle_rad: float = 2.0 * math.pi
    action_time_allowance_sec: int = 10
    max_recovery_attempts_per_goal: int = 2
    # Goal timeout
    nav_timeout_sec: float = 45.0
    # Exploration cadence/quality
    explore_tick_sec: float = 1.0
    initial_explore_delay_sec: float = 2.0
    blacklist_radius_m: float = 0.25
    min_frontier_separation_m: float = 0.35
    min_plan_length_m: float = 0.30
    # Frontier goal “sanitization”
    safe_snap_radius_cells: int = 1
    # Endpoint backoff along global path
    backoff_poses_base: int = 5
    backoff_poses_step: int = 4
    # Spin throttling
    min_seconds_between_spins: float = 3.0
    # Retry delay
    retry_delay_sec: float = 0.5


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class WaypointRecoveryCommander(Node):
    def __init__(self):
        super().__init__('waypoint_recovery_commander')

        self.declare_parameter('mode', 'loop')
        self.params = Params(mode=self.get_parameter('mode').get_parameter_value().string_value)

        qos_bt = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_odom = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(BehaviorTreeLog, 'behavior_tree_log', self.bt_log_callback, qos_bt)
        self.create_subscription(Odometry, 'odom', self.odom_callback, qos_odom)
        self.create_subscription(OccupancyGrid, 'map', self.map_cb, 10)
        self.latest_map: Optional[OccupancyGrid] = None

        self.nav_client    = ActionClient(self, NavigateToPose,    'navigate_to_pose')
        self.plan_client   = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.spin_client   = ActionClient(self, Spin,              'spin')
        self.backup_client = ActionClient(self, BackUp,            'backup')

        self.clear_local  = self.create_client(Empty, '/local_costmap/clear_entirely_local_costmap')
        self.clear_global = self.create_client(Empty, '/global_costmap/clear_entirely_global_costmap')

        # loop-mode demo waypoints
        self.waypoints: List[PoseStamped] = []
        p0 = PoseStamped(); p0.header.frame_id='map'; p0.pose.position.x= 1.7; p0.pose.position.y= 0.5; p0.pose.orientation.w=1.0
        p1 = PoseStamped(); p1.header.frame_id='map'; p1.pose.position.x=-0.6; p1.pose.position.y= 1.8; p1.pose.orientation.w=1.0
        self.waypoints += [p0, p1]

        # state
        self.current_idx = 0
        self.current_goal: Optional[PoseStamped] = None
        self.goal_active = False
        self.recovering = False
        self.recovery_attempts = 0
        self._odom_buf: Deque[tuple[float, float, float]] = deque(maxlen=40)

        self._goal_timeout_timer = None
        self._retry_timer = None          # <-- one-shot retry timer handle
        self._last_spin_end_time = 0.0

        self.blacklist: List[Tuple[float, float]] = []
        self.planning = False
        self.pending_candidate: Optional[PoseStamped] = None

        self.create_timer(0.2, self._check_stuck)

        self.get_logger().info(f"Mode: {self.params.mode}")
        self.get_logger().info('Waiting for Nav2 action servers...')
        self.nav_client.wait_for_server(); self.plan_client.wait_for_server()
        self.spin_client.wait_for_server(); self.backup_client.wait_for_server()
        self.get_logger().info('Servers ready.')

        if self.params.mode == "explore":
            self.get_logger().info(f"Explorer arms in {self.params.initial_explore_delay_sec:.1f}s…")
            self._start_explore_once = self.create_timer(self.params.initial_explore_delay_sec, self._start_explore_after_delay)
        else:
            self._send_current_waypoint()

    # helpers
    def _start_explore_after_delay(self):
        self._start_explore_once.cancel()
        self.create_timer(self.params.explore_tick_sec, self._explore_tick)
        self.get_logger().info("Explorer active.")

    def _stamp(self, p: PoseStamped) -> PoseStamped:
        out = PoseStamped(); out.header.frame_id = p.header.frame_id
        out.header.stamp = self.get_clock().now().to_msg(); out.pose = p.pose
        return out

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _start_goal_timeout(self):
        if self._goal_timeout_timer:
            self._goal_timeout_timer.cancel()
        def _timeout():
            if self.goal_active and not self.recovering:
                self.get_logger().warn(f'Nav goal timed out after {self.params.nav_timeout_sec:.0f}s → recovery')
                self._trigger_recovery('nav_timeout')
        self._goal_timeout_timer = self.create_timer(self.params.nav_timeout_sec, _timeout)

    def _cancel_goal_timeout(self):
        if self._goal_timeout_timer:
            self._goal_timeout_timer.cancel()
            self._goal_timeout_timer = None

    def _cancel_retry_timer(self):
        if self._retry_timer:
            self._retry_timer.cancel()
            self._retry_timer = None

    # send goals 
    def _same_goal(self, a: PoseStamped, b: PoseStamped, tol: float = 1e-3) -> bool:
        return (abs(a.pose.position.x - b.pose.position.x) < tol and
                abs(a.pose.position.y - b.pose.position.y) < tol)

    def _send_nav_to(self, goal_pose: PoseStamped, *, reset_attempts: bool):
        # prevent double-send of the same goal while one is already active
        if self.goal_active and self.current_goal is not None and self._same_goal(self.current_goal, goal_pose):
            return

        if reset_attempts:
            self.recovery_attempts = 0
        self.goal_active = True
        self.recovering = False
        self.current_goal = self._stamp(goal_pose)
        self._start_goal_timeout()

        goal = NavigateToPose.Goal(); goal.pose = self.current_goal
        self.get_logger().info(f"NavigateToPose → ({goal.pose.pose.position.x:.2f}, {goal.pose.pose.position.y:.2f})")
        self.nav_client.send_goal_async(goal, feedback_callback=self._nav_feedback_cb)\
                       .add_done_callback(self._on_nav_goal_sent)

    def _send_current_waypoint(self):
        self._send_nav_to(self._stamp(self.waypoints[self.current_idx]), reset_attempts=True)

    def _advance_waypoint(self):
        self.current_idx = (self.current_idx + 1) % len(self.waypoints)
        self._send_current_waypoint()

    # Nav2 callbacks 
    def _on_nav_goal_sent(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('NavigateToPose goal REJECTED → recovery')
            self._trigger_recovery('nav_goal_rejected'); return
        gh.get_result_async().add_done_callback(self._on_nav_result)

    def _nav_feedback_cb(self, _):
        pass

    def _on_nav_result(self, future):
        self.goal_active = False
        self._cancel_goal_timeout()
        status = future.result().status  # 4=SUCCEEDED, 6=ABORTED, etc.

        if status == 4:
            self.get_logger().info('Goal reached')
            self.recovering = False
            self.recovery_attempts = 0
            if self.params.mode == "explore" and self.current_goal is not None:
                gx, gy = self.current_goal.pose.position.x, self.current_goal.pose.position.y
                self.blacklist.append((gx, gy))
            if self.params.mode == "explore": self._explore_tick()
            else:                             self._advance_waypoint()
        else:
            self.get_logger().warn(f'NavigateToPose ended with status={status} → recovery')
            self._trigger_recovery(f'nav_status_{status}')

    # BT and Odom
    def bt_log_callback(self, msg: BehaviorTreeLog):
        if not self.goal_active or self.recovering:
            return 
        
        for e in msg.event_log:
            if not e.node_name.endswith('NavigateRecovery'):
                continue

            prev = getattr(e, 'previous_status', '')
            curr = getattr(e, 'current_status', '')

            try:
                et = e.timestamp.sec + e.timestamp.nanosec / 1e9
            except Exception:
                et = now

            if (curr == 'FAILURE' and prev == 'RUNNING' and (now - et) < 0.5):
                self.get_logger().info('BT NavigateRecovery transition RUNNING→FAILURE → trigger recovery')
                self._trigger_recovery('bt_navrecov_failed')
                return 

    def odom_callback(self, msg: Odometry):
        now = self._now_sec()
        p = msg.pose.pose.position
        self._odom_buf.append((now, p.x, p.y))

    def _check_stuck(self):
        if not self.goal_active or self.recovering or len(self._odom_buf) < 2:
            return
        now = self._now_sec()
        oldest = None
        for (t, x, y) in self._odom_buf:
            if now - t >= self.params.stuck_window_sec:
                oldest = (t, x, y); break
        if oldest is None: return
        _, x0, y0 = oldest; _, x1, y1 = self._odom_buf[-1]
        if math.hypot(x1 - x0, y1 - y0) < self.params.min_progress_m:
            self.get_logger().warn(f'No progress in {self.params.stuck_window_sec:.1f}s → recovery')
            self._trigger_recovery('stuck_no_progress')

    # recovery
    def _trigger_recovery(self, reason: str):
        if self.recovering: return
        if self.recovery_attempts >= self.params.max_recovery_attempts_per_goal:
            self.get_logger().error('Max recoveries reached.')
            self.recovering = False
            self._cancel_retry_timer()
            # avoid retrying same blacklisted spot
            if self.params.mode == "explore" and self.current_goal is not None:
                gx, gy = self.current_goal.pose.position.x, self.current_goal.pose.position.y
                self.blacklist.append((gx, gy))
                self.get_logger().warn(f'Blacklisting unreachable goal ({gx:.2f}, {gy:.2f}).')
                self.current_goal = None  # important: drop it so we don't reuse
                self._explore_tick()
            else:
                self._advance_waypoint()
            return

        self._clear_costmaps()

        self.recovering = True
        self.recovery_attempts += 1
        self.get_logger().info(f'Starting recovery #{self.recovery_attempts} ({reason})')
        self._do_backup()

    def _clear_costmaps(self):
        if self.clear_local.service_is_ready():
            self.clear_local.call_async(Empty.Request())
        if self.clear_global.service_is_ready():
            self.clear_global.call_async(Empty.Request())

    def _do_backup(self):
        goal = BackUp.Goal()
        goal.target = Point(x=-self.params.backup_distance_m, y=0.0, z=0.0)
        goal.speed = self.params.backup_speed_mps
        goal.time_allowance = RosDuration(sec=self.params.action_time_allowance_sec)
        self.backup_client.send_goal_async(goal).add_done_callback(self._on_backup_sent)

    def _on_backup_sent(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('BackUp goal REJECTED → try spin')
            self._do_spin(); return
        gh.get_result_async().add_done_callback(self._on_backup_done)

    def _on_backup_done(self, future):
        if future.result().status == 4: self.get_logger().info('BackUp complete')
        else:                            self.get_logger().warn(f'BackUp failed (status={future.result().status})')
        self._do_spin()

    def _do_spin(self):
        if self._now_sec() - self._last_spin_end_time < self.params.min_seconds_between_spins:
            self.get_logger().warn("Skipping spin (throttled) → retry nav")
            self._finish_recovery(); return
        goal = Spin.Goal()
        goal.target_yaw = self.params.spin_angle_rad
        goal.time_allowance = RosDuration(sec=self.params.action_time_allowance_sec)
        self.spin_client.send_goal_async(goal).add_done_callback(self._on_spin_sent)

    def _on_spin_sent(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('Spin goal REJECTED → retry nav anyway')
            self._finish_recovery(); return
        gh.get_result_async().add_done_callback(self._on_spin_done)

    def _on_spin_done(self, future):
        if future.result().status == 4: self.get_logger().info('Spin complete (360°)')
        else:                            self.get_logger().warn(f'Spin failed (status={future.result().status})')
        self._last_spin_end_time = self._now_sec()
        self._finish_recovery()

    def _finish_recovery(self):
        self.recovering = False
        self.get_logger().info('Retrying current goal after recovery…')
        self._cancel_retry_timer()

        # one-shot retry timer (cancel itself before running)
        def _retry_once():
            if self._retry_timer:
                self._retry_timer.cancel()
                self._retry_timer = None
            if self.current_goal is not None:
                self._send_nav_to(self.current_goal, reset_attempts=False)
            else:
                if self.params.mode == "explore": self._explore_tick()
                else:                             self._send_current_waypoint()

        self._retry_timer = self.create_timer(self.params.retry_delay_sec, _retry_once)

    # exploration
    def map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg

    def _explore_tick(self):
        if self.params.mode != "explore": return
        if self.goal_active or self.recovering or self.planning: return

        goal = self._pick_frontier_goal()
        if goal is None:
            self.get_logger().info("No strict frontiers. 360° sweep then try relaxed…")
            def _retry_relaxed():
                g2 = self._pick_frontier_goal_relaxed()
                if g2 is None:
                    self.get_logger().info("No frontiers (relaxed). Exploration may be complete.")
                    return
                self._validate_goal_async(g2)
            self._spin_sweep_then(_retry_relaxed)
            return
        self._validate_goal_async(goal)

    def _is_blacklisted(self, x: float, y: float) -> bool:
        return any(math.hypot(x - bx, y - by) < self.params.blacklist_radius_m for (bx, by) in self.blacklist)

    def _pick_frontier_goal(self) -> Optional[PoseStamped]:
        if self.latest_map is None:
            self.get_logger().info("Waiting for map...")
            return None
        grid = self.latest_map.data; w = self.latest_map.info.width; h = self.latest_map.info.height
        res  = self.latest_map.info.resolution
        ox   = self.latest_map.info.origin.position.x
        oy   = self.latest_map.info.origin.position.y
        def idx(xi:int, yi:int) -> int: return yi * w + xi
        def world(xi:int, yi:int) -> Tuple[float,float]: return ox + (xi + 0.5)*res, oy + (yi + 0.5)*res

        fr_ij: List[Tuple[int,int]] = []
        for y in range(1, h-1):
            base = y * w
            for x in range(1, w-1):
                if grid[base + x] != 0: continue
                if (grid[idx(x, y-1)] == -1 or grid[idx(x, y+1)] == -1 or
                    grid[idx(x-1, y)] == -1 or grid[idx(x+1, y)] == -1):
                    fr_ij.append((x, y))
        if not fr_ij: return None

        filt: List[Tuple[int,int]] = []
        if self.current_goal is not None:
            lgx, lgy = self.current_goal.pose.position.x, self.current_goal.pose.position.y
            for (xi, yi) in fr_ij:
                wx, wy = world(xi, yi)
                if not self._is_blacklisted(wx, wy) and math.hypot(wx - lgx, wy - lgy) >= self.params.min_frontier_separation_m:
                    filt.append((xi, yi))
        else:
            for (xi, yi) in fr_ij:
                wx, wy = world(xi, yi)
                if not self._is_blacklisted(wx, wy):
                    filt.append((xi, yi))
        if not filt: return None

        if self.current_goal is not None:
            lgx, lgy = self.current_goal.pose.position.x, self.current_goal.pose.position.y
            best = min(filt, key=lambda ij: math.hypot(world(*ij)[0] - lgx, world(*ij)[1] - lgy))
        else:
            best = filt[len(filt)//2]

        safe = self._snap_inward_to_safe(best[0], best[1], grid, w, h)
        if safe is None:
            wx, wy = world(*best); self.blacklist.append((wx, wy))
            return None

        fx, fy = world(*safe)
        goal = PoseStamped(); goal.header.frame_id='map'
        goal.pose.position.x = fx; goal.pose.position.y = fy; goal.pose.orientation.w = 1.0
        return goal

    def _snap_inward_to_safe(self, xi:int, yi:int, grid, w:int, h:int) -> Optional[Tuple[int,int]]:
        r = self.params.safe_snap_radius_cells
        def is_free(x:int,y:int)->bool: return 0 <= x < w and 0 <= y < h and grid[y*w + x] == 0
        def near_unknown(x:int,y:int)->bool:
            if y-1>=0 and grid[(y-1)*w + x] == -1: return True
            if y+1<h and grid[(y+1)*w + x] == -1:  return True
            if x-1>=0 and grid[y*w + (x-1)] == -1: return True
            if x+1<w and grid[y*w + (x+1)] == -1:  return True
            return False
        cands: List[Tuple[int,int]] = []
        for dy in range(-r, r+1):
            for dx in range(-r, r+1):
                cx, cy = xi+dx, yi+dy
                if not is_free(cx, cy): continue
                if near_unknown(cx, cy): continue
                cands.append((cx, cy))
        if not cands: return None
        cands.sort(key=lambda ij: (ij[0]-xi)**2 + (ij[1]-yi)**2)
        return cands[0]

    def _pick_frontier_goal_relaxed(self) -> Optional[PoseStamped]:
        if self.latest_map is None:
            return None
        grid = self.latest_map.data; w = self.latest_map.info.width; h = self.latest_map.info.height
        res  = self.latest_map.info.resolution
        ox   = self.latest_map.info.origin.position.x
        oy   = self.latest_map.info.origin.position.y
        def idx(xi, yi): return yi * w + xi
        def world(xi, yi): return ox + (xi + 0.5)*res, oy + (yi + 0.5)*res

        neigh = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
        fr: List[Tuple[int,int]] = []
        for y in range(1, h-1):
            base = y * w
            for x in range(1, w-1):
                if grid[base + x] != 0: continue
                if any(grid[idx(x+dx, y+dy)] == -1 for dx,dy in neigh):
                    fr.append((x,y))
        if not fr: return None

        if self.current_goal is not None:
            lgx, lgy = self.current_goal.pose.position.x, self.current_goal.pose.position.y
            cand = [(xi,yi) for (xi,yi) in fr
                    if not self._is_blacklisted(*world(xi,yi))
                    and math.hypot(world(xi,yi)[0]-lgx, world(xi,yi)[1]-lgy) >= max(0.35, self.params.min_frontier_separation_m*0.75)]
        else:
            cand = [(xi,yi) for (xi,yi) in fr if not self._is_blacklisted(*world(xi,yi))]
        if not cand: return None

        if self.current_goal is not None:
            lgx, lgy = self.current_goal.pose.position.x, self.current_goal.pose.position.y
            xi, yi = min(cand, key=lambda ij: math.hypot(world(*ij)[0]-lgx, world(*ij)[1]-lgy))
        else:
            xi, yi = cand[len(cand)//2]

        fx, fy = world(xi, yi)
        goal = PoseStamped(); goal.header.frame_id='map'
        goal.pose.position.x = fx; goal.pose.position.y = fy; goal.pose.orientation.w = 1.0
        return goal

    def _spin_sweep_then(self, cont_fn):
        if self._now_sec() - self._last_spin_end_time < self.params.min_seconds_between_spins:
            cont_fn(); return
        goal = Spin.Goal()
        goal.target_yaw = 2.0 * math.pi
        goal.time_allowance = RosDuration(sec=self.params.action_time_allowance_sec)
        def _after_sent(fut):
            gh = fut.result()
            if not gh.accepted:
                self.get_logger().warn("Sweep spin rejected; continuing anyway.")
                cont_fn(); return
            def _after_done(done):
                self.get_logger().info("Sweep spin complete; re-checking frontiers.")
                self._last_spin_end_time = self._now_sec()
                cont_fn()
            gh.get_result_async().add_done_callback(_after_done)
        self.spin_client.send_goal_async(goal).add_done_callback(_after_sent)

    # planner pre-check
    def _validate_goal_async(self, goal_pose: PoseStamped):
        if self.planning: return
        self.planning = True; self.pending_candidate = goal_pose

        req = ComputePathToPose.Goal()
        req.goal = self._stamp(goal_pose)
        req.use_start = False
        self.get_logger().info(f"Validating candidate ({req.goal.pose.position.x:.2f}, {req.goal.pose.position.y:.2f}) with planner...")
        self.plan_client.send_goal_async(req).add_done_callback(self._on_plan_goal_sent)

    def _on_plan_goal_sent(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("Planner REJECTED compute-path. Blacklisting and picking another.")
            self._blacklist_pending_and_continue(); return
        gh.get_result_async().add_done_callback(self._on_plan_result)

    def _on_plan_result(self, future):
        res = future.result().result; self.planning = False
        if res is None or not res.path.poses:
            self.get_logger().warn("Planner found NO path. Blacklisting and picking another frontier.")
            self._blacklist_pending_and_continue(); return

        path = res.path.poses
        path_len = self._path_length_m(res.path)
        if path_len < self.params.min_plan_length_m:
            self.get_logger().info(f"Planner path too short ({path_len:.2f} m) → treating as done; blacklist & next.")
            self._blacklist_pending_and_continue(); return

        cand = self.pending_candidate; self.pending_candidate = None
        if cand is None:
            self._explore_tick(); return

        attempts = self.recovery_attempts
        backoff = min(self.params.backoff_poses_base + attempts * self.params.backoff_poses_step,
                      max(1, len(path)-1))
        safe_pose = path[-backoff].pose
        cand.pose.position.x = safe_pose.position.x
        cand.pose.position.y = safe_pose.position.y

        if len(path) >= backoff + 1:
            p_next = path[-backoff+1].pose.position
            p_cur  = safe_pose.position
            yaw = math.atan2(p_next.y - p_cur.y, p_next.x - p_cur.x)
            cand.pose.orientation = yaw_to_quat(yaw)
        else:
            cand.pose.orientation = safe_pose.orientation

        self.get_logger().info(
            f"Planner path OK (poses={len(path)}, len={path_len:.2f} m). Sending nav goal with backoff={backoff}."
        )
        self._send_nav_to(cand, reset_attempts=True)

    def _path_length_m(self, path) -> float:
        if not path.poses or len(path.poses) < 2: return 0.0
        total = 0.0; prev = path.poses[0].pose.position
        for ps in path.poses[1:]:
            p = ps.pose.position; total += math.hypot(p.x - prev.x, p.y - prev.y); prev = p
        return total

    def _blacklist_pending_and_continue(self):
        self.planning = False
        if self.pending_candidate is not None:
            px, py = self.pending_candidate.pose.position.x, self.pending_candidate.pose.position.y
            self.blacklist.append((px, py))
        self.pending_candidate = None
