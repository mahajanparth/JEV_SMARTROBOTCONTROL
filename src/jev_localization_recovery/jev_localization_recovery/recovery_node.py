"""ROS transport and demo state machine. Pure algorithms live in sibling modules."""
import json
import math
import time
from concurrent.futures import Future
from dataclasses import replace
from threading import Thread

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker

from .localization_monitor import (Health, LocalizationMonitor, LocalizationState,
                                   Thresholds, classify, quaternion_yaw)
from .scan_map_score import Grid, scan_map_score, scan_clearance, transform_point
from .recovery_actions import ActionConfig, MotionHistory, MotionSample, RecoveryActions
from .recovery_selector import JevRecoverySelector, MockRecoverySelector, RecoveryAction, RecoveryDecision


DEFAULTS = dict(
    pose_topic='', scan_topic='', map_topic='', odom_topic='', cmd_vel_topic='',
    global_localization_service='', base_frame='base_link', odom_frame='odom',
    enable_recovery=False, beam_stride=8, occupied_threshold=65, cell_tolerance=2,
    minimum_valid_beams=12, max_scan_age=0.75, max_pose_age=30.0,
    pose_scan_tolerance=0.5, tf_time_tolerance=0.05, log_interval=2.0,
    bad_hold_time=1.5, cooldown=3.0, max_attempts=3, evaluation_time=8.0,
    healthy_hold_time=1.0, score_improvement=0.1, uncertainty_improvement=0.1,
    selector_provider='mock', jev_model='jev-latest', jev_api_key_file='',
    jev_timeout=3.0, decision_timeout=4.0, jev_min_confidence=0.2,
)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def tf_pose(transform):
    t = transform.transform
    return t.translation.x, t.translation.y, quaternion_yaw(t.rotation)


def compose(a, b):
    return (*transform_point(b[0], b[1], a), a[2] + b[2])


def inverse(pose):
    x, y = transform_point(-pose[0], -pose[1], (0, 0, -pose[2]))
    return x, y, -pose[2]


class RecoveryNode(Node):
    def __init__(self):
        super().__init__('jev_recovery')
        params = dict(DEFAULTS, **vars(Thresholds()), **vars(ActionConfig()))
        for key, value in params.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in params}
        self.validate_parameters()
        self.thresholds = Thresholds(**{key: self.p[key] for key in vars(Thresholds())})
        self.monitor = LocalizationMonitor(self.thresholds)
        self.grid = self.scan = self.pose_message = self.odom = None
        self.scan_at = self.odom_at = -math.inf
        self.map_frame = ''
        self.bindings = {}
        self.subscriptions_by_role = {}
        self.tf = Buffer(cache_time=Duration(seconds=self.p['max_pose_age'] + 5.0))
        self.tf_listener = TransformListener(self.tf, self)
        self.state_publisher = self.create_publisher(String, '~/state', 10)
        self.marker_publisher = self.create_publisher(Marker, '~/markers', 10)
        self.event_publisher = self.create_publisher(String, '~/events', 10)
        self.velocity_publisher = None
        self.velocity_type = Twist
        self.global_client = None
        self.global_service = ''
        self.motion_sample = None
        self.motion_history = MotionHistory()
        self.worsened_while_moving = False
        self.previous_health = None
        if self.p['selector_provider'] == 'jev':
            self.selector = JevRecoverySelector(self.p['jev_model'], self.p['jev_timeout'],
                self.p['jev_api_key_file'], {k: self.p[k] for k in ('spin_clearance', 'rear_clearance', 'max_attempts')})
        else:
            self.selector = MockRecoverySelector(self.p['spin_clearance'], self.p['rear_clearance'],
                                                 self.p['max_attempts'])
        self.decision_future = None
        self.last_decision = None
        self.actions = RecoveryActions(self.publish_velocity, self.request_global,
                                      ActionConfig(**{key: self.p[key] for key in vars(ActionConfig())}))
        self.attempts = dict(spin_attempts=0, global_relocalization_attempts=0, backtrack_attempts=0)
        self.bad_since = self.healthy_since = None
        self.cooldown_until = 0.0
        self.before = None
        self.reset_service = self.create_service(Trigger, '~/reset_stop', self.reset_stop)
        self.machine = 'MONITORING'
        self.active_action = 'NONE'
        self.last_log = -math.inf
        self.last_status = None
        self.last_reason = 'Waiting for graph discovery'
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0, self.discover, clock=self.steady_clock)
        self.create_timer(0.1, self.tick, clock=self.steady_clock)
        self.get_logger().info('Jev monitor starting; discovering actual ROS graph topics.')
        self.get_logger().info('Recovery motion {}'.format('ENABLED' if self.p['enable_recovery'] else 'DISABLED (monitor only)'))
        self.get_logger().info('Decision provider: ' + self.p['selector_provider'].upper())

    def validate_parameters(self):
        for name, value in self.p.items():
            if isinstance(value, (int, float)) and not math.isfinite(value):
                raise ValueError(name + ' must be finite')
        for name in ('beam_stride', 'minimum_valid_beams', 'max_scan_age',
                     'max_pose_age', 'pose_scan_tolerance', 'log_interval', 'max_odom_age',
                     'action_timeout', 'service_timeout', 'evaluation_time', 'healthy_hold_time',
                     'max_attempts', 'spin_clearance', 'rear_clearance', 'backtrack_distance',
                     'jev_timeout', 'decision_timeout'):
            if not math.isfinite(self.p[name]) or self.p[name] <= 0:
                raise ValueError(name + ' must be positive and finite')
        if not 1 <= self.p['occupied_threshold'] <= 100 or self.p['cell_tolerance'] < 0:
            raise ValueError('occupied_threshold must be 1..100; cell_tolerance must be nonnegative')
        if not 0 <= self.p['score_lost'] < self.p['score_healthy'] <= 1:
            raise ValueError('Require 0 <= score_lost < score_healthy <= 1')
        if not 0 < self.p['uncertainty_degraded'] < self.p['uncertainty_lost']:
            raise ValueError('Require 0 < uncertainty_degraded < uncertainty_lost')
        for name in ('yaw_weight', 'jump_distance', 'jump_yaw', 'jump_hold_sec'):
            if not math.isfinite(self.p[name]) or self.p[name] < 0:
                raise ValueError(name + ' must be finite and nonnegative')
        for name in ('bad_hold_time', 'cooldown', 'global_settle_time', 'tf_time_tolerance',
                     'score_improvement', 'uncertainty_improvement'):
            if not math.isfinite(self.p[name]) or self.p[name] < 0:
                raise ValueError(name + ' must be finite and nonnegative')
        if not 0 < self.p['spin_speed'] <= 0.6 or not 0 < self.p['backtrack_speed'] <= 0.15:
            raise ValueError('Demo limits: spin_speed in (0,0.6], backtrack_speed in (0,0.15]')
        if not 0 < self.p['spin_angle'] <= 2 * math.pi or self.p['backtrack_distance'] > 1.0:
            raise ValueError('Demo limits: spin_angle in (0,2*pi], backtrack_distance <= 1 m')
        if self.p['rear_clearance'] < self.p['spin_clearance']:
            raise ValueError('rear_clearance must be at least spin_clearance')
        if self.p['tf_time_tolerance'] > 0.1:
            raise ValueError('tf_time_tolerance cannot exceed 0.1 seconds')
        if self.p['selector_provider'] not in ('mock', 'jev'):
            raise ValueError('selector_provider must be mock or jev')
        if not 0 <= self.p['jev_min_confidence'] <= 1:
            raise ValueError('jev_min_confidence must be in [0,1]')
        if self.p['decision_timeout'] < self.p['jev_timeout']:
            raise ValueError('decision_timeout must be at least jev_timeout')

    def resolve_topic(self, role, type_name, preferred):
        topics = self.get_topic_names_and_types()
        requested = self.p[role + '_topic']
        if requested:
            return requested if any(n == requested and type_name in ts for n, ts in topics) else None
        candidates = [n for n, ts in topics if type_name in ts and self.count_publishers(n) > 0]
        named = [n for n in candidates if n.rsplit('/', 1)[-1] in preferred]
        return named[0] if len(named) == 1 else None

    def discover(self):
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for role, cls, typ, names, callback, qos in [
            ('pose', PoseWithCovarianceStamped, 'geometry_msgs/msg/PoseWithCovarianceStamped',
             ['amcl_pose'], self.on_pose, qos_profile_sensor_data),
            ('scan', LaserScan, 'sensor_msgs/msg/LaserScan', ['scan'], self.on_scan, qos_profile_sensor_data),
            ('map', OccupancyGrid, 'nav_msgs/msg/OccupancyGrid', ['map'], self.on_map, latched),
            ('odom', Odometry, 'nav_msgs/msg/Odometry', ['odom'], self.on_odom, qos_profile_sensor_data),
        ]:
            if role in self.bindings:
                continue
            topic = self.resolve_topic(role, typ, names)
            if topic:
                if role == 'pose':
                    infos = self.get_publishers_info_by_topic(topic)
                    if infos and all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                                     for i in infos):
                        qos = latched
                # Accept both live volatile maps and map-server latched maps.
                if role == 'map':
                    infos = self.get_publishers_info_by_topic(topic)
                    if infos and any(i.qos_profile.durability == DurabilityPolicy.VOLATILE for i in infos):
                        qos = qos_profile_sensor_data
                self.subscriptions_by_role[role] = self.create_subscription(cls, topic, callback, qos)
                self.bindings[role] = topic
                self.get_logger().info('Discovered {}: {} [{}]'.format(role, topic, typ))
        self.discover_actuators()

    def discover_actuators(self):
        if self.velocity_publisher is None:
            candidates = []
            for name, types in self.get_topic_names_and_types():
                requested = self.p['cmd_vel_topic']
                if (name == requested if requested else name.rsplit('/', 1)[-1] == 'cmd_vel'):
                    if len(types) == 1 and types[0] in ('geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped'):
                        candidates.append((name, types[0]))
            if len(candidates) == 1:
                name, typ = candidates[0]
                self.velocity_type = TwistStamped if typ.endswith('TwistStamped') else Twist
                self.velocity_publisher = self.create_publisher(self.velocity_type, name, 10)
                self.bindings['cmd_vel'] = name
                self.get_logger().info('Discovered velocity command: {} [{}]'.format(name, typ))
        if self.global_client is None:
            requested = self.p['global_localization_service']
            choices = [name for name, types in self.get_service_names_and_types()
                       if 'std_srvs/srv/Empty' in types and
                       (name == requested if requested else name.rsplit('/', 1)[-1] in
                        ('reinitialize_global_localization', 'global_localization'))]
            if len(choices) == 1:
                self.global_service = choices[0]
                self.global_client = self.create_client(Empty, choices[0])
                self.get_logger().info('Discovered AMCL service: {} [std_srvs/srv/Empty]'.format(choices[0]))

    def request_global(self):
        if self.global_client is None or not self.global_client.service_is_ready():
            return None
        self.get_logger().info('Calling ' + self.global_service)
        return self.global_client.call_async(Empty.Request())

    def publish_velocity(self, linear, angular):
        if not self.p['enable_recovery'] or self.velocity_publisher is None:
            return
        msg = self.velocity_type()
        twist = msg
        if isinstance(msg, TwistStamped):
            msg.header.frame_id = self.p['base_frame']
            msg.header.stamp = self.get_clock().now().to_msg()
            twist = msg.twist
        twist.linear.x, twist.angular.z = float(linear), float(angular)
        self.velocity_publisher.publish(msg)

    def command_problem(self):
        topic = self.bindings.get('cmd_vel')
        if not topic or self.count_subscribers(topic) == 0:
            return 'No discovered velocity command subscriber'
        publishers = self.get_publishers_info_by_topic(topic)
        if len(publishers) != 1 or any(p.node_name != self.get_name() or
                                     p.node_namespace != self.get_namespace() for p in publishers):
            return 'Velocity topic has another publisher; recovery requires exclusive command ownership'
        return ''

    def reset_stop(self, request, response):
        if self.decision_future is not None and not self.decision_future.done():
            response.success, response.message = False, 'Wait for the pending Jev request to finish'
            return response
        if self.machine != 'STOPPED':
            response.success, response.message = False, 'Reset applies only to a latched STOP'
            return response
        self.machine, self.active_action = 'MONITORING', 'NONE'
        self.decision_future = None
        self.attempts = {key: 0 for key in self.attempts}
        self.bad_since = None
        self.worsened_while_moving = False
        self.cooldown_until = time.monotonic() + self.p['cooldown']
        self.publish_velocity(0, 0)
        response.success, response.message = True, 'Stop latch cleared; monitoring resumes after cooldown'
        return response

    def on_pose(self, msg):
        self.pose_message = msg
        p = msg.pose.pose
        self.monitor.update(p.position.x, p.position.y, quaternion_yaw(p.orientation),
                            msg.pose.covariance, time.monotonic())

    def on_map(self, msg):
        p = msg.info.origin
        if (not msg.header.frame_id or msg.info.width <= 0 or msg.info.height <= 0 or
                not math.isfinite(msg.info.resolution) or msg.info.resolution <= 0 or
                not all(math.isfinite(v) for v in (p.position.x, p.position.y, quaternion_yaw(p.orientation))) or
                len(msg.data) != msg.info.width * msg.info.height):
            self.grid = None
            return
        self.map_frame = msg.header.frame_id
        self.grid = Grid(msg.info.width, msg.info.height, msg.info.resolution,
                         (p.position.x, p.position.y, quaternion_yaw(p.orientation)), msg.data)

    def on_scan(self, msg):
        self.scan, self.scan_at = msg, time.monotonic()

    def on_odom(self, msg):
        self.odom, self.odom_at = msg, time.monotonic()
        p = msg.pose.pose
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp_seconds(msg.header.stamp)
        values = p.position.x, p.position.y, quaternion_yaw(p.orientation)
        if (msg.header.frame_id != self.p['odom_frame'] or msg.child_frame_id != self.p['base_frame'] or
                not all(math.isfinite(v) for v in values) or age < -0.1 or age > self.p['max_odom_age']):
            self.motion_sample = None
        else:
            self.motion_sample = MotionSample(*values, self.odom_at)
        self.motion_history.add(self.motion_sample)

    def scan_geometry(self, now):
        if self.scan is None or now - self.scan_at > self.p['max_scan_age']:
            raise ValueError('Missing or stale LaserScan')
        scan = self.scan
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        age = ros_now - stamp_seconds(scan.header.stamp)
        if age < -0.1 or age > self.p['max_scan_age']:
            raise ValueError('LaserScan timestamp is stale or in the future')
        if (not scan.ranges or not math.isfinite(scan.angle_min) or
                not math.isfinite(scan.range_min) or scan.range_min < 0 or
                not math.isfinite(scan.angle_increment) or scan.angle_increment == 0 or
                not math.isfinite(scan.range_max) or scan.range_max <= scan.range_min):
            raise ValueError('Invalid LaserScan geometry')
        laser = tf_pose(self.tf.lookup_transform(self.p['base_frame'], scan.header.frame_id,
                                                 Time.from_msg(scan.header.stamp)))
        return laser

    def snapshot(self, now):
        state = LocalizationState(pose_uncertainty=self.monitor.uncertainty, **self.attempts)
        state.global_service_available = self.global_client is not None and self.global_client.service_is_ready()
        state.backtrack_available = self.motion_history.forward_distance(now) >= self.p['backtrack_distance']
        state.worsened_while_moving = self.worsened_while_moving
        state.pose_jump_distance, state.pose_jump_yaw = self.monitor.jumps(now)
        try:
            laser = self.scan_geometry(now)
            scan = self.scan
            args = (scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max, laser)
            state.min_obstacle_distance = scan_clearance(*args)
            state.rear_clearance = scan_clearance(*args, rear=True)
            if self.grid is None or self.pose_message is None or self.monitor.pose is None:
                raise ValueError('Waiting for map and AMCL pose')
            if self.pose_message.header.frame_id != self.map_frame:
                raise ValueError('AMCL pose and occupancy grid frames differ')
            pose_age = self.get_clock().now().nanoseconds * 1e-9 - stamp_seconds(self.pose_message.header.stamp)
            if pose_age < -0.1 or pose_age > self.p['max_pose_age'] or self.monitor.uncertainty is None:
                raise ValueError('AMCL pose/covariance unavailable or stale')
            pose = self.monitor.pose
            try:
                old_odom = tf_pose(self.tf.lookup_transform(self.p['odom_frame'], self.p['base_frame'],
                                                          Time.from_msg(self.pose_message.header.stamp)))
                try:
                    new_odom = tf_pose(self.tf.lookup_transform(self.p['odom_frame'], self.p['base_frame'],
                                                              Time.from_msg(scan.header.stamp)))
                except TransformException:
                    # Gazebo scan and TF callbacks can arrive a few milliseconds
                    # apart. Use the newest TF only within a bounded timestamp gap.
                    latest = self.tf.lookup_transform(self.p['odom_frame'], self.p['base_frame'], Time())
                    if abs(stamp_seconds(latest.header.stamp) - stamp_seconds(scan.header.stamp)) > self.p['tf_time_tolerance']:
                        raise
                    new_odom = tf_pose(latest)
                pose = compose(compose(pose, inverse(old_odom)), new_odom)
            except TransformException:
                if abs(stamp_seconds(scan.header.stamp) - stamp_seconds(self.pose_message.header.stamp)) > self.p['pose_scan_tolerance']:
                    raise ValueError('Cannot align AMCL pose with scan using odometry TF')
            result = scan_map_score(self.grid, scan.ranges, scan.angle_min, scan.angle_increment,
                                    scan.range_min, scan.range_max, pose, laser,
                                    self.p['beam_stride'], self.p['occupied_threshold'],
                                    self.p['cell_tolerance'], self.p['minimum_valid_beams'])
            state.scan_map_score = result.score
            state.data_ready = result.score is not None
            state.status, state.reason = classify(state, self.thresholds)
            if result.reason:
                state.reason = result.reason
        except (ValueError, TransformException) as exc:
            state.reason = 'UNKNOWN: ' + str(exc).split('\n')[0]
        return state

    def tick(self):
        now = time.monotonic()
        state = self.snapshot(now)
        if state.data_ready:
            if (self.previous_health == Health.HEALTHY and state.status != Health.HEALTHY
                    and self.motion_history.moving_forward(now)):
                self.worsened_while_moving = state.worsened_while_moving = True
            self.previous_health = state.status
        if self.p['enable_recovery']:
            self.step_recovery(state, now)
        self.publish_state(state)
        if state.status != self.last_status or now - self.last_log >= self.p['log_interval']:
            f = lambda value: 'UNKNOWN' if value is None else '{:.3f}'.format(value)
            self.get_logger().info(
                '\n==============================\nLOCALIZATION HEALTH\n'
                'Pose uncertainty:      {}\nScan-map consistency:  {}\n'
                'Pose jump:             {:.2f} m / {:.2f} rad\nObstacle distance:     {} m\n'
                'STATUS: {} | {}\n{}'.format(f(state.pose_uncertainty), f(state.scan_map_score),
                    state.pose_jump_distance, state.pose_jump_yaw, f(state.min_obstacle_distance),
                    state.status.value, self.machine, state.reason))
            self.last_status, self.last_log = state.status, now

    def latch_stop(self, reason):
        self.actions.stop(reason, safety=True)
        self.machine, self.active_action = 'STOPPED', 'STOP'
        self.get_logger().error('STOP LATCHED: {}. Reset with /jev_recovery/reset_stop.'.format(reason))
        self.event_publisher.publish(String(data=json.dumps({'event': 'STOP', 'reason': reason})))

    def step_recovery(self, state, now):
        if self.machine == 'STOPPED':
            self.publish_velocity(0, 0)
            return
        if self.machine in ('MONITORING', 'LOCALIZATION_BAD'):
            if not state.data_ready or state.status == Health.HEALTHY:
                self.machine, self.bad_since = 'MONITORING', None
                return
            if self.bad_since is None:
                self.bad_since = now
            self.machine = 'LOCALIZATION_BAD'
            if now >= self.cooldown_until and now - self.bad_since >= self.p['bad_hold_time']:
                self.machine = 'SELECT_RECOVERY'
            return
        if self.machine == 'SELECT_RECOVERY':
            self.publish_velocity(0, 0)
            if now < self.cooldown_until:
                return
            if state.status == Health.HEALTHY and state.data_ready:
                self.machine, self.bad_since = 'MONITORING', None
                return
            if sum(self.attempts.values()) >= self.p['max_attempts']:
                self.latch_stop('Recovery attempt limit reached (deterministic override)')
                return
            if self.p['selector_provider'] == 'jev':
                problem = self.decision_safety_problem(state, now)
                if problem:
                    self.latch_stop(problem)
                    return
                self.start_jev_request(state, now)
            else:
                try:
                    chosen = self.selector.choose(state)
                except Exception:
                    self.latch_stop('Invalid recovery decision: provider failed')
                    return
                self.apply_decision(chosen, state, now)
        elif self.machine == 'WAITING_DECISION':
            self.publish_velocity(0, 0)
            problem = self.decision_safety_problem(state, now)
            if problem:
                self.latch_stop(problem)
                return
            if now - self.decision_started >= self.p['decision_timeout']:
                self.latch_stop('Jev decision deadline exceeded; late responses will not execute')
                return
            if not self.decision_future.done():
                return
            try:
                chosen = self.decision_future.result()
            except Exception as exc:
                # Adapter exceptions are sanitized; do not log arbitrary provider text.
                self.latch_stop('Jev request failed ({})'.format(type(exc).__name__))
                return
            original = self.decision_state
            changed = (state.status != original.status or
                       abs(state.scan_map_score - original.scan_map_score) > 0.15 or
                       abs(state.pose_uncertainty - original.pose_uncertainty) > 0.5)
            self.decision_future = None
            if changed:
                self.get_logger().info('Discarded Jev decision: localization changed while waiting')
                self.machine, self.active_action, self.bad_since = 'MONITORING', 'NONE', None
                return
            self.apply_decision(chosen, state, now)
        elif self.machine == 'EXECUTING_RECOVERY':
            problem = self.command_problem()
            if problem:
                self.actions.stop(problem, safety=True)
            else:
                self.actions.tick(state, self.motion_sample, now)
            if self.actions.result is not None:
                if not self.actions.result:
                    self.finish_recovery(False, state, self.actions.reason, now)
                    if self.actions.safety_failure:
                        self.latch_stop(self.actions.reason)
                else:
                    self.machine = 'EVALUATING'
                    self.evaluation_started = now
                    self.healthy_since = None
        elif self.machine == 'EVALUATING':
            self.publish_velocity(0, 0)
            fresh = (self.scan_at > self.evaluation_started and self.pose_message is not None and
                     stamp_seconds(self.pose_message.header.stamp) > self.before_pose_stamp)
            score_better = (state.scan_map_score is not None and self.before.scan_map_score is not None and
                            state.scan_map_score - self.before.scan_map_score >= self.p['score_improvement'])
            uncertainty_better = (state.pose_uncertainty is not None and self.before.pose_uncertainty is not None and
                                 self.before.pose_uncertainty - state.pose_uncertainty >= self.p['uncertainty_improvement'])
            recovered = fresh and state.data_ready and state.status == Health.HEALTHY and (score_better or uncertainty_better)
            if recovered:
                if self.healthy_since is None:
                    self.healthy_since = now
                if now - self.healthy_since >= self.p['healthy_hold_time']:
                    self.finish_recovery(True, state, 'Fresh healthy observations and improved metrics', now)
            else:
                self.healthy_since = None
            if self.machine == 'EVALUATING' and now - self.evaluation_started >= self.p['evaluation_time']:
                self.finish_recovery(False, state, 'Health or metrics did not recover within evaluation window', now)

    def decision_safety_problem(self, state, now):
        if not state.data_ready or state.scan_map_score is None or state.pose_uncertainty is None:
            return 'Localization evidence became unavailable during decision selection'
        return self.command_problem() or self.actions.safety_reason(state, self.motion_sample, now)

    def start_jev_request(self, state, now):
        self.decision_started, self.decision_state = now, replace(state)
        future = Future()
        self.decision_future = future
        selector, snapshot = self.selector, self.decision_state

        def infer():
            future.set_running_or_notify_cancel()
            try:
                future.set_result(selector.choose(snapshot))
            except Exception as exc:
                future.set_exception(exc)

        # One outstanding request; daemon cannot hold robot shutdown hostage to DNS.
        Thread(target=infer, name='jev-decision', daemon=True).start()
        self.machine, self.active_action = 'WAITING_DECISION', 'AWAITING_JEV'
        self.get_logger().info('Requesting TypeSafe Jev decision; robot remains stopped')

    def apply_decision(self, chosen, state, now):
        try:
            decision = RecoveryDecision(chosen.selected_action, chosen.confidence,
                chosen.probabilities, chosen.explanation, getattr(chosen, 'model', ''))
        except Exception:
            self.latch_stop('Invalid recovery decision: response validation failed')
            return
        event = dict(event='DECISION', provider=self.p['selector_provider'], model=decision.model,
                     selected_action=decision.selected_action.value, confidence=decision.confidence,
                     probabilities={a.value: p for a, p in decision.probabilities.items()},
                     explanation=decision.explanation)
        self.last_decision = event
        self.event_publisher.publish(String(data=json.dumps(event)))
        self.get_logger().info('\nRECOVERY DECISION ({})\n'.format(self.p['selector_provider'].upper()) + '\n'.join(
            '{:22s} {:.2f} {}'.format(a.value, decision.probabilities[a],
             '<-- SELECTED' if a == decision.selected_action else '') for a in RecoveryAction)
             + '\nConfidence: {:.3f} | Model: {}'.format(decision.confidence, decision.model or 'mock'))
        if decision.selected_action == RecoveryAction.STOP:
            self.latch_stop(decision.explanation)
            return
        if self.p['selector_provider'] == 'jev' and decision.confidence < self.p['jev_min_confidence']:
            self.latch_stop('Jev confidence below configured threshold')
            return
        if sum(self.attempts.values()) >= self.p['max_attempts']:
            self.latch_stop('Recovery attempt limit reached (deterministic override)')
            return
        if not state.data_ready:
            self.latch_stop('Cannot execute a decision without fresh localization evidence')
            return
        if decision.selected_action == RecoveryAction.GLOBAL_RELOCALIZE and not state.global_service_available:
            self.latch_stop('Requested AMCL global service is unavailable')
            return
        if decision.selected_action == RecoveryAction.BACKTRACK_AND_SPIN and not (
                state.backtrack_available and state.worsened_while_moving):
            self.latch_stop('Backtrack requires verified history and worsening while moving')
            return
        problem = self.command_problem()
        if problem:
            self.latch_stop(problem)
            return
        self.before = state
        self.before_pose_stamp = stamp_seconds(self.pose_message.header.stamp)
        self.active_action = decision.selected_action.value
        counter = {RecoveryAction.SPIN: 'spin_attempts',
                   RecoveryAction.GLOBAL_RELOCALIZE: 'global_relocalization_attempts',
                   RecoveryAction.BACKTRACK_AND_SPIN: 'backtrack_attempts'}[decision.selected_action]
        self.attempts[counter] += 1
        self.actions.start(decision.selected_action, state, self.motion_sample, now)
        self.machine = 'EXECUTING_RECOVERY'
        self.get_logger().info('Executing ' + self.active_action)

    def finish_recovery(self, success, state, reason, now):
        event = 'RECOVERY SUCCESS' if success else 'RECOVERY FAILED'
        self.get_logger().info('\n==============================\nRECOVERY EVALUATION\n'
            'Before scan score: {}\nAfter scan score: {}\nBefore uncertainty: {}\n'
            'After uncertainty: {}\n{}: {}'.format(self.before.scan_map_score, state.scan_map_score,
                self.before.pose_uncertainty, state.pose_uncertainty, event, reason))
        self.event_publisher.publish(String(data=json.dumps(dict(event=event, reason=reason,
            action=self.active_action, provider=self.p['selector_provider'], decision=self.last_decision,
            before=self.before.to_dict(), after=state.to_dict()), allow_nan=False)))
        self.cooldown_until = now + self.p['cooldown']
        if success:
            self.machine, self.active_action, self.bad_since = 'MONITORING', 'NONE', None
            self.attempts = {key: 0 for key in self.attempts}
            self.worsened_while_moving = False
        else:
            self.machine = 'SELECT_RECOVERY'

    def publish_state(self, state):
        payload = dict(state.to_dict(), machine=self.machine, current_action=self.active_action,
                       topics=self.bindings, selector_provider=self.p['selector_provider'],
                       last_decision=self.last_decision)
        self.state_publisher.publish(String(data=json.dumps(payload, allow_nan=False)))
        if self.monitor.pose is None or not self.map_frame:
            return
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id = 'jev_recovery', 0
        marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        marker.pose.position.x, marker.pose.position.y = self.monitor.pose[:2]
        marker.pose.position.z = 0.8
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0 if state.status == Health.HEALTHY else 1.0
        marker.color.g = 0.0 if state.status == Health.LOST else 1.0
        score = 'UNKNOWN' if state.scan_map_score is None else '{:.2f}'.format(state.scan_map_score)
        marker.text = '{} | score {}\n{}'.format(state.status.value, score, self.active_action)
        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_velocity(0, 0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
