import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
from ament_index_python.packages import get_package_share_directory
import os
import time
import threading
from collections import deque, Counter

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# MediaPipe Hand Landmark Indices
WRIST = 0
THUMB_CMC = 1
THUMB_MCP = 2
THUMB_IP = 3
THUMB_TIP = 4

INDEX_MCP = 5
INDEX_PIP = 6
INDEX_DIP = 7
INDEX_TIP = 8

MIDDLE_MCP = 9
MIDDLE_PIP = 10
MIDDLE_DIP = 11
MIDDLE_TIP = 12

RING_MCP = 13
RING_PIP = 14
RING_DIP = 15
RING_TIP = 16

PINKY_MCP = 17
PINKY_PIP = 18
PINKY_DIP = 19
PINKY_TIP = 20

FINGER_TIPS = [INDEX_TIP, MIDDLE_TIP, RING_TIP, PINKY_TIP]
FINGER_PIPS = [INDEX_PIP, MIDDLE_PIP, RING_PIP, PINKY_PIP]
FINGER_MCPS = [INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP]

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),         # Thumb
    (0, 5), (5, 6), (6, 7), (7, 8),         # Index
    (5, 9), (9, 10), (10, 11), (11, 12),    # Middle
    (9, 13), (13, 14), (14, 15), (15, 16),  # Ring
    (13, 17), (17, 18), (18, 19), (19, 20), # Pinky
    (0, 17)                                 # Palm base
]


def dist_3d(p1, p2):
    """Calculate 3D Euclidean distance between two MediaPipe landmark points."""
    return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2) ** 0.5


class ThreadedWebcam:
    """
    Asynchronous, non-blocking webcam thread.
    Prevents cv2.VideoCapture.read() from blocking ROS 2 event loop!
    """
    def __init__(self, src=0, width=640, height=480):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.grabbed, self.frame = self.cap.read()
        self.started = False
        self.read_lock = threading.Lock()

    def start(self):
        if self.started:
            return self
        self.started = True
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()
        return self

    def update(self):
        while self.started:
            grabbed, frame = self.cap.read()
            if not grabbed:
                time.sleep(0.005)
                continue
            with self.read_lock:
                self.grabbed = grabbed
                self.frame = frame
            time.sleep(0.001)

    def read(self):
        with self.read_lock:
            if not self.grabbed or self.frame is None:
                return False, None
            return True, self.frame.copy()

    def stop(self):
        self.started = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.cap.release()


class GestureNode(Node):
    def resolve_model_path(self):
        candidates = []

        # 1. Relative to current python script location
        file_dir = os.path.dirname(os.path.abspath(__file__))
        candidates.append(os.path.join(file_dir, 'models', 'hand_landmarker.task'))

        # 2. Package share directory
        try:
            share_dir = get_package_share_directory('gesture_control')
            candidates.append(os.path.join(share_dir, 'models', 'hand_landmarker.task'))
            candidates.append(os.path.abspath(os.path.join(share_dir, '..', '..', 'lib', 'gesture_control', 'models', 'hand_landmarker.task')))
        except Exception:
            pass

        # 3. Source directory fallback
        candidates.append('/home/sundar/gesture_robot_ws/src/gesture_control/gesture_control/models/hand_landmarker.task')

        for path in candidates:
            if os.path.exists(path):
                return path

        raise FileNotFoundError(f"Could not find 'hand_landmarker.task' in any candidate path: {candidates}")

    def __init__(self):
        super().__init__('gesture_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lock = threading.Lock()

        model_path = self.resolve_model_path()
        self.get_logger().info(f'Loading HandLandmarker model from: {model_path}')

        base_options = mp_python.BaseOptions(model_asset_path=model_path)
        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=1,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
            running_mode=vision.RunningMode.LIVE_STREAM,
            result_callback=self.on_result
        )
        self.landmarker = vision.HandLandmarker.create_from_options(options)

        # Non-blocking Threaded Webcam Reader (640x480 @ low latency)
        self.webcam = ThreadedWebcam(src=0, width=640, height=480).start()

        self.last_timestamp_ms = 0

        # State variables updated asynchronously by MediaPipe callback
        self.latest_landmarks = None
        self.latest_handedness = "None"
        self.latest_raw_count = 0
        self.latest_filtered_count = 0
        self.gesture_buffer = deque(maxlen=5)
        self.current_twist = Twist()

        self.last_time = time.time()
        self.timer = self.create_timer(1.0 / 30.0, self.loop)  # Smooth 30 FPS processing

    def count_fingers(self, landmarks):
        """
        Accurate, rotation-invariant finger counting using 3D relative distances.
        Works regardless of hand rotation, tilt, or orientation!
        """
        wrist = landmarks[WRIST]
        pinky_mcp = landmarks[PINKY_MCP]
        fingers_up = 0

        # 1. Thumb Check:
        thumb_tip = landmarks[THUMB_TIP]
        thumb_ip = landmarks[THUMB_IP]
        d_tip_pinky = dist_3d(thumb_tip, pinky_mcp)
        d_ip_pinky = dist_3d(thumb_ip, pinky_mcp)
        d_tip_wrist = dist_3d(thumb_tip, wrist)
        d_ip_wrist = dist_3d(thumb_ip, wrist)

        if (d_tip_pinky > d_ip_pinky * 1.05) and (d_tip_wrist > d_ip_wrist * 1.02):
            fingers_up += 1

        # 2. Four Fingers Check (Index, Middle, Ring, Pinky):
        for tip_idx, pip_idx, mcp_idx in zip(FINGER_TIPS, FINGER_PIPS, FINGER_MCPS):
            tip = landmarks[tip_idx]
            pip = landmarks[pip_idx]
            mcp = landmarks[mcp_idx]

            d_tip_w = dist_3d(tip, wrist)
            d_pip_w = dist_3d(pip, wrist)
            d_tip_m = dist_3d(tip, mcp)
            d_pip_m = dist_3d(pip, mcp)

            if (d_tip_w > d_pip_w) and (d_tip_m > d_pip_m * 1.05):
                fingers_up += 1

        return fingers_up

    def on_result(self, result: vision.HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
        """Asynchronous callback executed by MediaPipe C++ thread when detection completes."""
        with self.lock:
            if result.hand_landmarks and result.handedness:
                landmarks = result.hand_landmarks[0]
                handedness_str = result.handedness[0][0].category_name
                raw_count = self.count_fingers(landmarks)

                self.latest_landmarks = landmarks
                self.latest_handedness = handedness_str
                self.latest_raw_count = raw_count
            else:
                self.latest_landmarks = None
                self.latest_handedness = "None"
                self.latest_raw_count = 0

            self.gesture_buffer.append(self.latest_raw_count)
            filtered_count = Counter(self.gesture_buffer).most_common(1)[0][0]
            self.latest_filtered_count = filtered_count

            # Calculate Twist command
            twist = Twist()
            if filtered_count == 1:
                twist.linear.x = 0.2
            elif filtered_count == 2:
                twist.linear.x = -0.2
            elif filtered_count == 3:
                twist.angular.z = 0.5
            elif filtered_count == 4:
                twist.angular.z = -0.5

            self.current_twist = twist

    def loop(self):
        ret, frame = self.webcam.read()
        if not ret or frame is None:
            return

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        # Monotonically increasing real-time wall-clock timestamp
        now_ms = int(time.time() * 1000)
        if now_ms <= self.last_timestamp_ms:
            now_ms = self.last_timestamp_ms + 1
        self.last_timestamp_ms = now_ms

        # Non-blocking async inference dispatch (returns in ~0ms)
        try:
            self.landmarker.detect_async(mp_image, self.last_timestamp_ms)
        except Exception:
            pass

        # Thread-safe read of latest processed state
        with self.lock:
            landmarks = self.latest_landmarks
            handedness_str = self.latest_handedness
            raw_count = self.latest_raw_count
            filtered_count = self.latest_filtered_count
            twist = self.current_twist

        self.pub.publish(twist)

        # Draw Hand Skeleton Connections
        if landmarks is not None:
            for pt1_idx, pt2_idx in HAND_CONNECTIONS:
                pt1 = (int(landmarks[pt1_idx].x * w), int(landmarks[pt1_idx].y * h))
                pt2 = (int(landmarks[pt2_idx].x * w), int(landmarks[pt2_idx].y * h))
                cv2.line(frame, pt1, pt2, (255, 200, 0), 2)

            for idx, lm in enumerate(landmarks):
                cx, cy = int(lm.x * w), int(lm.y * h)
                color = (0, 255, 0) if idx in FINGER_TIPS or idx == THUMB_TIP else (0, 0, 255)
                cv2.circle(frame, (cx, cy), 5, color, -1)

        # Velocity Control Text
        gesture_text = "STOP"
        if filtered_count == 1:
            gesture_text = "FORWARD (1 Finger)"
        elif filtered_count == 2:
            gesture_text = "BACKWARD (2 Fingers)"
        elif filtered_count == 3:
            gesture_text = "TURN LEFT (3 Fingers)"
        elif filtered_count == 4:
            gesture_text = "TURN RIGHT (4 Fingers)"
        elif filtered_count == 5:
            gesture_text = "STOP (5 Fingers / Open Palm)"
        else:
            gesture_text = "STOP (No Hand / 0 Fingers)"

        # Calculate FPS
        current_time = time.time()
        fps = 1.0 / (current_time - self.last_time + 1e-6)
        self.last_time = current_time

        # Draw UI Overlay Box
        cv2.rectangle(frame, (10, 10), (450, 130), (0, 0, 0), -1)
        cv2.rectangle(frame, (10, 10), (450, 130), (0, 255, 255), 2)

        cv2.putText(frame, f'Hand: {handedness_str} | FPS: {int(fps)}', (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(frame, f'Raw Count: {raw_count} | Filtered: {filtered_count}', (20, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f'Cmd: {gesture_text}', (20, 105),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow('Accurate Gesture Control', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):  # Esc or 'q' to quit
            rclpy.shutdown()

    def destroy_node(self):
        self.webcam.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()