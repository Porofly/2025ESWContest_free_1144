# ros2 run your_pkg yolo_ground_localizer.py  (예시)
import math
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseArray, PointStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import Monitoring
from collections import deque
import numpy as np

from yolov8_msgs.msg import Yolov8Inference  # pip/ros msg as in your env

def rot_x(a):
    ca, sa = math.cos(a), math.sin(a)
    return np.array([[1,0,0],[0,ca,-sa],[0,sa,ca]], dtype=float)

def rot_y(a):
    ca, sa = math.cos(a), math.sin(a)
    return np.array([[ca,0,sa],[0,1,0],[-sa,0,ca]], dtype=float)

def rot_z(a):
    ca, sa = math.cos(a), math.sin(a)
    return np.array([[ca,-sa,0],[sa,ca,0],[0,0,1]], dtype=float)

class MovingAverage2D:
    def __init__(self, window=11):
        self.N = window
        self.bufx = deque(maxlen=window)
        self.bufy = deque(maxlen=window)

    def push(self, x, y):
        self.bufx.append(float(x)); self.bufy.append(float(y))
        # 평균 (창이 덜 찼을 때도 현재까지 평균)
        return (float(np.mean(self.bufx)), float(np.mean(self.bufy)))

class EMAFilter2D:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.x = None
        self.y = None

    def push(self, x, y):
        if self.x is None:
            self.x = float(x)
            self.y = float(y)
        else:
            self.x = (1 - self.alpha) * self.x + self.alpha * float(x)
            self.y = (1 - self.alpha) * self.y + self.alpha * float(y)
        return (self.x, self.y)
    
class YoloGroundLocalizer(Node):
    def __init__(self):
        super().__init__('yolo_ground_localizer')

        # --- Camera intrinsics (params you provided)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fx', 678.8712179620)
        # 655.68563818)
        self.declare_parameter('fy', 676.5923040326)
        # 656.8373932213)
        self.declare_parameter('cx', 600.7451721112)
        # 630.0668311460)
        self.declare_parameter('cy', 363.7283523432)
        # 366.3269405158)
        self.declare_parameter('sigma_x', 15)
        self.declare_parameter('sigma_y', 15)

        self.declare_parameter('cam_pitch_deg', -30.0)  # downward tilt
        self.declare_parameter('cam_roll_deg', 0.0)     # given 0
        # yaw follows drone → relative yaw 0

        # --- Topics
        self.declare_parameter('yolo_topic', '/Yolov8_Inference_1')  # 확실하지 않음: 네 실제 토픽으로
        self.declare_parameter('monitor_topic', '/drone1/fmu/out/monitoring')

        # --- Monitoring fields (string names to getattr)
        #    Example from your snippet:
        self.declare_parameter('field_pos_x', 'pos_x')
        self.declare_parameter('field_pos_y', 'pos_y')
        self.declare_parameter('field_pos_z', 'pos_z')
        self.declare_parameter('field_roll', 'roll')
        self.declare_parameter('field_pitch', 'pitch')
        self.declare_parameter('field_yaw', 'head')  # radians

        # --- RViz frame id
        self.declare_parameter('frame_id', 'map')  # 확실하지 않음: 'ned' or 'map'
        
        self.declare_parameter('roi_enabled', False)
        
        self.declare_parameter('roi_min_u', 100)
        self.declare_parameter('roi_max_u', 1180)
        self.declare_parameter('roi_min_v', 200)
        self.declare_parameter('roi_max_v', 700)

        self.roi_enabled = self.get_parameter('roi_enabled').value
        self.roi_min_u = float(self.get_parameter('roi_min_u').value)
        self.roi_max_u = float(self.get_parameter('roi_max_u').value)
        self.roi_min_v = float(self.get_parameter('roi_min_v').value)
        self.roi_max_v = float(self.get_parameter('roi_max_v').value)
        
        # Params to vars
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value

        self.sigma_x = float(self.get_parameter('sigma_x').value)
        self.sigma_y = float(self.get_parameter('sigma_y').value)
        self.sigma_uv = np.array([[self.sigma_x*self.sigma_x, 0], [0, self.sigma_y*self.sigma_y]])

        self.cam_pitch = math.radians(self.get_parameter('cam_pitch_deg').value)
        self.cam_roll = math.radians(self.get_parameter('cam_roll_deg').value)

        self.yolo_topic = self.get_parameter('yolo_topic').value
        self.monitor_topic = self.get_parameter('monitor_topic').value

        self.field_pos_x = self.get_parameter('field_pos_x').value
        self.field_pos_y = self.get_parameter('field_pos_y').value
        self.field_pos_z = self.get_parameter('field_pos_z').value
        self.field_roll = self.get_parameter('field_roll').value
        self.field_pitch = self.get_parameter('field_pitch').value
        self.field_yaw = self.get_parameter('field_yaw').value

        self.frame_id = self.get_parameter('frame_id').value
        
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.prev_point = []
        self.prev_pose = PoseArray()
        self.prev_pose.header = Header()

        # State from monitor
        self.C_ned = np.zeros(3, dtype=float)   # [x,y,z]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Filters
        # self.ma = MovingAverage2D(window=11)
        # self.ma = MovingAverage2D(window=20)  # Change MA window size

        # self.ma = EMAFilter2D(alpha=0.3)      # Another Filter (EMA)
        
        # Delay
        self.declare_parameter('detection_delay_sec', 0.0)
        self.delay_sec = float(self.get_parameter('detection_delay_sec').value)
        self.state_buf = deque(maxlen=600)
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'detections_markers', 10)
        self.pose_pub = self.create_publisher(PoseArray, 'detections_posearray', 10)
        self.point_pub = self.create_publisher(PointStamped, 'detection_point', 10)
        self.box_cov = self.create_publisher(PoseWithCovarianceStamped, 'box_covariance', 10)

        # Subscribers
        self.create_subscription(Yolov8Inference, self.yolo_topic, self.on_yolo, reliable_qos)

        # NOTE: 실제 모니터링 메시지 타입을 모름(확실하지 않음). 아래는 예시용 lamdba로 교체.
        # 네 환경의 msg type으로 바꿔서 같은 콜백 로직을 사용해줘.
        qos = QoSProfile(depth=10)

        # ---- 예시: Generic subscription via topic_tools/relay가 아니라면, 실제 타입을 넣어야 함
        self.create_subscription(Monitoring, self.monitor_topic, self.on_monitor, best_effort)

        # 임시 타이머 경고 (실제 타입 연결 필요)
        self.warned_monitor = False
        self.create_timer(2.0, self._warn_monitor_once)

        # Precompute camera->body alignment (no tilt): b_x<-c_z, b_y<-c_x, b_z<-c_y
        self.R_b_c_align = np.array([[0,0,1],
                                     [1,0,0],
                                     [0,1,0]], dtype=float)

        # Fixed gimbal pitch/roll relative to body
        self.R_b_cam_fixed = rot_y(self.cam_pitch) @ self.R_b_c_align
        # self.R_b_cam_fixed = self.R_b_c_align @ rot_x(self.cam_pitch)

        self.get_logger().info('yolo_ground_localizer ready.')

    def _warn_monitor_once(self):
        if not self.warned_monitor:
            self.get_logger().warn('Monitoring topic subscriber not set (message type unknown). '
                                   'Replace on_monitor subscription with your real msg type.', throttle_duration_sec=0)
            self.warned_monitor = True

    # ---- Replace this with a real callback once you know the message type
    def on_monitor(self, msg):
        # Example mapping based on your snippet:
        self.C_ned[0] = float(getattr(msg, self.field_pos_x))
        self.C_ned[1] = float(getattr(msg, self.field_pos_y))
        self.C_ned[2] = float(getattr(msg, self.field_pos_z))
        self.roll     = float(getattr(msg, self.field_roll))
        self.pitch    = float(getattr(msg, self.field_pitch))
        self.yaw      = float(getattr(msg, self.field_yaw))
        
        t = self._now_sec()
        self.state_buf.append((t, self.C_ned[0], self.C_ned[1], self.C_ned[2],
                            self.roll, self.pitch, self.yaw))
    
    def in_roi(self, u, v):
        if not self.roi_enabled:
            return True
        return (self.roi_min_u <= u <= self.roi_max_u) and (self.roi_min_v <= v <= self.roi_max_v)

    def on_yolo(self, msg: Yolov8Inference):
        # Build body->NED rotation (Z-Y-X)
        # R_n_b = rot_z(self.yaw) @ rot_y(self.pitch) @ rot_x(self.roll)

        t_now = self._now_sec()
        t_query = t_now - self.delay_sec
        C_used, yaw_used, roll_used, pitch_used, ok = self._interp_state(t_query)
        t_query = t_now - 0.1
        C_prev, yaw_prev, roll_prev, pitch_prev, ok_ = self._interp_state(t_query)
        if not ok or not ok_:
            # 버퍼가 아직 비었거나 보간 실패 → 최신 상태 fallback
            C_used = self.C_ned.copy()
            C_prev = self.C_ned.copy()
            yaw_used = self.yaw
            yaw_prev = self.yaw
            # 필요시 로그
            self.get_logger().warn('State buffer not ready; using latest state.')

        now = self.get_clock().now().to_msg()

        poses = PoseArray()
        poses.header = Header()
        poses.header.frame_id = self.frame_id
        poses.header.stamp = now

        covariance = PoseWithCovarianceStamped()
        covariance.header = Header()
        covariance.header = poses.header

        self.prev_pose.header = poses.header

        curr_point = []
        match = {}

        markers = MarkerArray()
        m_id = 0

        # 드론 포인트(시각화도 과거상태 기준으로 표시 권장)
        drone_marker = Marker()
        drone_marker.header.frame_id = self.frame_id
        drone_marker.header.stamp = now
        drone_marker.ns = "drone"
        drone_marker.id = m_id; m_id += 1
        drone_marker.type = Marker.SPHERE
        drone_marker.pose.position.x = float(C_used[0])
        drone_marker.pose.position.y = float(C_used[1])
        drone_marker.pose.position.z = -float(C_used[2])  # RViz 프레임 부호 일관성은 환경에 맞게
        drone_marker.scale.x = drone_marker.scale.y = drone_marker.scale.z = 0.2
        drone_marker.color.r = 0.2; drone_marker.color.g = 0.4; drone_marker.color.b = 1.0; drone_marker.color.a = 0.9
        markers.markers.append(drone_marker)

        # Process each detection
        for det in msg.yolov8_inference:
            if det.left > det.right:
                # self.get_logger().warn(f'Invalid detection: left {det.left} > right {det.right}. Skipping.')
                continue
            
            if det.top > det.bottom:
                # self.get_logger().warn(f'Invalid detection: top {det.top} < bottom {det.bottom}. Skipping.')
                continue
            
            u = (det.left + det.right) * 0.5
            v = (det.top  + det.bottom) * 0.5
            # v = float(det.bottom)
            
            if not self.in_roi(u, v):
                self.get_logger().warn(f"No In ROI u, v")
                continue

            curr_point.append((u, v))

            # self.get_logger().info("successful detection: ")
            # Ray in camera frame (pinhole; no distortion)
            x = (u - self.cx) / self.fx
            y = (v - self.cy) / self.fy
            r_cam = np.array([x, y, 1.0], dtype=float)
            # r_cam /= np.linalg.norm(r_cam)

            J1 = np.array([[1/self.fx, 0], [0, 1/self.fy], [0, 0]])

            # Camera->Body (alignment + fixed gimbal pitch/roll)
            r_body = self.R_b_cam_fixed @ r_cam
            # r_body /= np.linalg.norm(r_body)

            J2 = self.R_b_cam_fixed

            # Body->NED
            # r_ned = R_n_b @ r_body
            
            # R_n_yaw = rot_z(yaw_used)
            # R_cam_fixed = rot_y(self.cam_pitch) @ self.R_b_c_align
            # r_ned = R_n_yaw @ r_body # (R_cam_fixed @ r_cam)

            # Body -> NED
            r_nb = rot_z(yaw_used)
            r_ned = r_nb @ r_body
            norm_r_ned = np.linalg.norm(r_ned)
            r_ned_norm = r_ned / norm_r_ned

            J3 = r_nb

            if norm_r_ned < 1e-9:
                J_norm = np.identity(r_ned.shape[0])
            else:
                I = np.identity(r_ned.shape[0])
                outer_v_norm = np.outer(r_ned_norm, r_ned_norm)
                
                J_norm = (I - outer_v_norm) / norm_r_ned

            # Intersect with ground plane z=0
            Cz = float(self.C_ned[2])
            rx = float(r_ned_norm[0])
            ry = float(r_ned_norm[1])
            rz = float(r_ned_norm[2])

            if abs(rz) < 1e-6:
                self.get_logger().warn('Ray nearly parallel to ground; skipping this detection.')
                continue

            J4 = np.array([
                [-Cz/rz, 0,      Cz*rx/(rz**2)],
                [0,      -Cz/rz, Cz*ry/(rz**2)],
                [0,      0,      0]
            ])

            J_total = J4 @ J_norm @ J3 @ J2 @ J1
            Sigma_P = J_total @ self.sigma_uv @ J_total.T

            covariance.pose.covariance[0] = Sigma_P[0, 0]
            covariance.pose.covariance[1] = Sigma_P[0, 1]
            covariance.pose.covariance[6] = Sigma_P[1, 0]
            covariance.pose.covariance[7] = Sigma_P[1, 1]

            t = (0.0 - Cz) / rz

            if t <= 0.0:
                # If this triggers consistently, your z sign convention likely differs.
                self.get_logger().warn('Intersection behind camera (t<=0). '
                                       'Check NED z sign (is z down positive?). Skipping.')
                continue

            P = C_used + t * r_ned_norm  # [x,y,z], z≈0
            # P = r_ned
            # Px_raw, Py_raw = float(P[0]), float(P[1])

            # Px_avg_tmp, Py_avg_tmp = self.ma.push(Px_raw, Py_raw)
            # if abs(Px_raw - Px_avg_tmp) > 3.0: Px_raw = Px_avg_tmp
            # if abs(Py_raw - Py_avg_tmp) > 3.0: Py_raw = Py_avg_tmp

            # 최종 이동평균
            # Px_ma, Py_ma = self.ma.push(Px_raw, Py_raw)

            # Use EMA Filter
            # Px_ma, Py_ma = self.ma.push(Px_raw, Py_raw)

            # P[0] = Px_ma
            # P[1] = Py_ma

            # Pose for PoseArray (orientation set identity)
            pose = Pose()
            pose.position.x = float(P[0])
            pose.position.y = float(P[1])
            pose.position.z = float(P[2])
            poses.poses.append(pose)
            covariance.pose.pose = pose
            self.box_cov.publish(covariance)


            # PointStamped (first one only, optional)
            pt = PointStamped()
            pt.header.frame_id = self.frame_id
            pt.header.stamp = now
            pt.point = Point(x=float(P[0]), y=float(P[1]), z=float(P[2]))
            self.point_pub.publish(pt)

            # Marker: point
            mk = Marker()
            mk.header.frame_id = self.frame_id
            mk.header.stamp = now
            mk.ns = "detections"
            mk.id = m_id; m_id += 1
            mk.type = Marker.SPHERE
            mk.pose.position.x = float(P[0])
            mk.pose.position.y = float(P[1])
            mk.pose.position.z = float(P[2])
            mk.scale.x = mk.scale.y = mk.scale.z = 0.25
            mk.color.r = 1.0; mk.color.g = 0.2; mk.color.b = 0.2; mk.color.a = 0.95
            markers.markers.append(mk)

            # Marker: label
            txt = Marker()
            txt.header.frame_id = self.frame_id
            txt.header.stamp = now
            txt.ns = "labels"
            txt.id = m_id; m_id += 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.text = getattr(det, 'class_name', 'obj')
            txt.pose.position.x = float(P[0])
            txt.pose.position.y = float(P[1])
            txt.pose.position.z = float(P[2]) + 0.5
            txt.scale.z = 0.4
            txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 1.0; txt.color.a = 0.9
            markers.markers.append(txt)

            # Marker: line from drone to object
            ln = Marker()
            ln.header.frame_id = self.frame_id
            ln.header.stamp = now
            ln.ns = "rays"
            ln.id = m_id; m_id += 1
            ln.type = Marker.LINE_LIST
            ln.scale.x = 0.03
            ln.color.r = 0.3; ln.color.g = 1.0; ln.color.b = 0.3; ln.color.a = 0.9
            ln.points = [Point(x=float(self.C_ned[0]), y=float(self.C_ned[1]), z=-float(self.C_ned[2])),
                         Point(x=float(P[0]), y=float(P[1]), z=float(P[2]))]
            markers.markers.append(ln)

        # Publish
        if not self.prev_point or len(self.prev_point) <= len(curr_point):
            self.get_logger().info(f"이전 point 길이: {len(self.prev_point)}, 현재 Point 길이: {len(curr_point)}")
            self.prev_point = curr_point
            self.prev_pose.poses = poses.poses
        else:
            self.get_logger().info(f"이전 point 길이: {len(self.prev_point)}, 현재 Point 길이: {len(curr_point)}")
            self.get_logger().info("Start Match Position")
            match = self._match(curr_point, poses, C_prev, yaw_prev, pitch_prev, roll_prev)
            if match == -2:
                self.prev_point = curr_point
                self.prev_pose.poses = poses.poses
                self.pose_pub.publish(poses)
                self.marker_pub.publish(markers)
                return
            match_success = []
            for i, j in match.items():
                # self.get_logger().info(f"Successfully matched previous object indices: {i}")
                if j != -1:
                    match_success.append((i, j))
            if not match_success:
                self.get_logger().warn("No Match Position")
                self.pose_pub.publish(poses)
                return
            
            for i, j in match.items():
                if j == -1:
                    predict_pose = Pose()
                    d_x = self.prev_pose.poses[i].position.x - self.prev_pose.poses[match_success[0][0]].position.x
                    d_y = self.prev_pose.poses[i].position.y - self.prev_pose.poses[match_success[0][0]].position.y
                    self.get_logger().info(f"Success Match Position: dx - {d_x}, dy - {d_y}")
                    predict_pose.position.x = poses.poses[match_success[0][1]].position.x + d_x
                    predict_pose.position.y = poses.poses[match_success[0][1]].position.y + d_y
                    predict_pose.position.z = poses.poses[match_success[0][1]].position.z
                    poses.poses.append(predict_pose)
        self.get_logger().info(f"Publishing PoseArray with {len(poses.poses)} poses:")
        for i, p in enumerate(poses.poses):
            self.get_logger().info(
                f"  - Pose[{i}]: x={p.position.x:.2f}, y={p.position.y:.2f}, z={p.position.z:.2f}"
            )
        self.pose_pub.publish(poses)
        self.marker_pub.publish(markers)

    def _match(self, curr_points, curr_poses, C_prev, yaw_prev, pitch_prev, roll_prev):
        back_project = []
        back_project = self._back_projection(curr_poses, C_prev, yaw_prev, pitch_prev, roll_prev)
        score_board = []
        min_score = 10
        matches = {}
        for i, proj_point in enumerate(back_project):
            proj_np = np.array(proj_point)
            curr_np = np.array(curr_points[i])
            measure_motion_vec = curr_np - proj_np
            measure_motion_norm = np.linalg.norm(measure_motion_vec)
        for i, curr_point in enumerate(curr_points):
            best_i = -1
            for j, prev_point in enumerate(self.prev_point):
                if i == 0:
                    matches[j] = -1
                prev_np = np.array(prev_point)
                curr_np = np.array(curr_point)
                real_motion_vec = curr_np - prev_np
                real_motion_norm = np.linalg.norm(real_motion_vec)
                # 방향 점수 (코사인 유사도 기반, 0~1)
                dir_score = 1.0 # 기본값 (움직임이 없을 때)
                if real_motion_norm > 1e-6 and measure_motion_norm > 1e-6:
                    cos_sim = np.dot(real_motion_vec, measure_motion_vec) / (real_motion_norm * measure_motion_norm)
                    dir_score = (1.0 - cos_sim) / 2.0 # 0(같은방향) ~ 1(다른방향)
                # 크기 점수 (움직인 거리 차이, 정규화 필요)
                magnitude_diff = abs(real_motion_norm - measure_motion_norm)
                if magnitude_diff >= 700.0:
                    self.get_logger().warn(f"image position difference : {magnitude_diff}")
                    return -2
                mag_score = min(magnitude_diff / 700.0, 1.0) # 0(크기유사) ~ 1(크기다름)
                w_dir, w_mag = (0.9, 0.5)
                total_score = w_dir * dir_score + w_mag * mag_score
                if total_score < min_score:
                    min_score = total_score
                    self.get_logger().info(f"이전 {j}대비 현재 {i}의 score {min_score}")
                    best_i = j
            score_board.append(best_i)

        for i, best_prev in enumerate(score_board):
            matches[best_prev] = i

        # self.prev_point.append(back_project)
        return matches

    def _back_projection(self, curr_poses, prev_C_ned, prev_yaw, prev_pitch, prev_roll):
        back_projected_coords = []

        # 1. 이전 시점 카메라의 전체 회전 행렬 계산 (NED -> Body -> Camera)
        R_n_b_prev = rot_z(prev_yaw) @ rot_y(prev_pitch) @ rot_x(prev_roll)
        R_b_n_prev = R_n_b_prev.T

        R_c_b_prev = self.R_b_cam_fixed.T
        
        R_c_n_prev = R_c_b_prev @ R_b_n_prev

        # 2. 현재 프레임에서 탐지된 각 3D 좌표에 대해 반복
        for pose in curr_poses.poses:
            # 현재 객체의 3D NED 좌표
            P_ned_current = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)

            # 3. 이전 시점 카메라 위치 기준의 상대 벡터 계산 (NED 좌표계)
            vec_ned = P_ned_current - prev_C_ned

            # 4. 상대 벡터를 NED -> 이전 시점 카메라 좌표계로 변환
            vec_cam = R_c_n_prev @ vec_ned

            # 5. 3D 카메라 좌표를 2D 이미지 평면에 투영
            x_cam, y_cam, z_cam = vec_cam[0], vec_cam[1], vec_cam[2]

            if z_cam <= 0.1:
                continue

            u = self.fx * (x_cam / z_cam) + self.cx
            v = self.fy * (y_cam / z_cam) + self.cy

            back_projected_coords.append((u, v))

        return back_projected_coords
        
    def _now_sec(self):
        # 노드의 steady clock 기준 (ROS2 Clock)
        return self.get_clock().now().nanoseconds * 1e-9
    
    def _interp_state(self, t_query):
        """
        state_buf에서 t_query를 둘러싸는 두 샘플 사이 선형보간.
        반환: (C_used (np.array[3]), yaw_used, roll_used, pitch_used, ok)
        ok=False면 보간 실패(버퍼 부족 등).
        """
        if len(self.state_buf) == 0:
            return None, None, None, None, False

        # 버퍼는 시계열로 append됨(오름차순 보장)
        # 1) 경계 처리
        t0, x0, y0, z0, r0, p0, y0aw = self.state_buf[0]
        tn, xn, yn, zn, rn, pn, ynaw = self.state_buf[-1]

        if t_query <= t0:
            # 너무 과거: 첫 샘플 사용
            C = np.array([x0, y0, z0], float)
            return C, y0aw, r0, p0, True
        if t_query >= tn:
            # 너무 미래: 마지막 샘플 사용
            C = np.array([xn, yn, zn], float)
            return C, ynaw, rn, pn, True

        # 2) 이분 탐색 (선형 탐색도 OK; 버퍼가 크면 이분이 유리)
        lo, hi = 0, len(self.state_buf)-1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if self.state_buf[mid][0] <= t_query:
                lo = mid
            else:
                hi = mid

        tA, xA, yA, zA, rA, pA, yAaw = self.state_buf[lo]
        tB, xB, yB, zB, rB, pB, yBaw = self.state_buf[hi]

        if tB == tA:
            C = np.array([xA, yA, zA], float)
            return C, yAaw, rA, pA, True

        alpha = (t_query - tA) / (tB - tA)
        # 위치 보간
        x = xA + alpha*(xB - xA)
        y = yA + alpha*(yB - yA)
        z = zA + alpha*(zB - zA)
        # 각도 보간(최단각)
        dyaw = _ang_diff(yAaw, yBaw)
        yaw = _wrap_to_pi(yAaw + alpha*dyaw)
        droll = _ang_diff(rA, rB)
        roll = _wrap_to_pi(rA + alpha*droll)
        dpitch = _ang_diff(pA, pB)
        pitch = _wrap_to_pi(pA + alpha*dpitch)

        C = np.array([x, y, z], float)
        return C, yaw, roll, pitch, True

@staticmethod
def _wrap_to_pi(a):
    # [-pi, pi]
    return (a + math.pi) % (2*math.pi) - math.pi

@staticmethod
def _ang_diff(a, b):
    # b - a (최단각)
    d = (b - a + math.pi) % (2*math.pi) - math.pi
    return d

def main():
    rclpy.init()
    node = YoloGroundLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
