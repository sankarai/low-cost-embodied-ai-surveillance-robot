import os
import time
from datetime import datetime
from typing import Optional, Tuple, Dict, List

import cv2
import requests
from ultralytics import YOLO
import math
import sys
import select

# =========================
# CONFIG
# =========================
#ROBOT_IP = "192.168.1.127"
ROBOT_IP = "esp32robot.local"
CONTROL_URL = f"http://{ROBOT_IP}/control"
DISTANCE_URL = f"http://{ROBOT_IP}/distance"
STREAM_URL = f"http://{ROBOT_IP}:81/stream"
STATE_URL = f"http://{ROBOT_IP}/state"

FORWARD_DISTANCE_CM = {
    "forward_short": 13.0,
    "forward_medium": 35.0,
    "forward_long": 62.5,
}

X_EXIT_SEARCH_THRESHOLD_CM = 180
Y_EXIT_SEARCH_THRESHOLD_CM = 120
PATROL_DURATION_SEC = 10 * 60
OBSTACLE_THRESHOLD = 40

# Run intruder detection only every N patrol points
DETECTION_EVERY_N_POINTS = 20

YOLO_MODEL = "yolo11l-seg.pt"
PERSON_CLASS_ID = 0
CONF_THRESHOLD = 0.15
MIN_PERSON_AREA_RATIO = 0.005

# Mask/free-space navigation
MASK_NAV_AFTER_EXIT_MODE_ONLY = True

MASK_NAV_MIN_DIST_CM = 45
MASK_NAV_CONF = 0.15

MASK_CENTER_BAND_RATIO = 0.50
MASK_BOTTOM_REGION_RATIO = 0.55

MASK_TURN_LEFT_THRESHOLD = -0.18
MASK_TURN_RIGHT_THRESHOLD = 0.18

MASK_CLEAR_SCORE_THRESHOLD = 0.55

SAVE_DIR = "patrol_yolo_events"
os.makedirs(SAVE_DIR, exist_ok=True)

BURST_DURATION_SEC = 0.35
BURST_FPS = 3.0

# Camera freshness controls. ESP32 MJPEG streams can buffer old frames in
# OpenCV VideoCapture, so reopen/flush before important image capture.
CAMERA_SETTLE_SEC = 0.25
# Keep normal patrol photo capture light. Use heavier flushing only after pan/tilt scans
# or after stream recovery.
PHOTO_FLUSH_FRAMES = 4
SCAN_FLUSH_FRAMES = 6
RECONNECT_FLUSH_FRAMES = 10
REOPEN_STREAM_BEFORE_CAPTURE = False

# Frequent lightweight photo logging for doorway/data collection.
# This is separate from the heavier YOLO scan.
PHOTO_EVERY_N_PATROL_POINTS = 0       # 0 = no normal forward photos; images only during multiview scans
SAVE_PATROL_CANNY = True
AUTO_PAUSE_AFTER_PATROL_PHOTO = False # ignored when PHOTO_EVERY_N_PATROL_POINTS=0
ENABLE_KEYBOARD_CONTROL = True        # p=pause, r=resume, q=quit at safe checkpoints
PATROL_PHOTO_PREFIX = "patrol_photo"

# Reduce repeated servo commands and heavy scans, which can overload the ESP32.
PAN_TILT_CENTER_EVERY_N_PHOTOS = 10    # 0 disables periodic re-centering before normal photos
ENABLE_YOLO_SCAN = True                # keep multiview scan enabled in this version
ENABLE_YOLO_BURST_CONFIRMATION = False # False = run YOLO only on the fresh saved frame
FORCE_REOPEN_EACH_SCAN_VIEW = False    # True is freshest but heavier on ESP32
STOP_AFTER_FINAL_SURVEY_SCAN = False    # stop after the survey-completion multiview scan
POST_PHOTO_PAUSE_SEC = 0.20

MIN_CMD_GAP_SEC = 0.05
HTTP_TIMEOUT = 2.5
MAX_CONSECUTIVE_HTTP_FAILURES = 3
COMM_RECOVERY_WAIT_SEC = 8.0

_last_cmd_time = 0.0
_http_failure_count = 0

# Turn timings: tune on your floor
SMALL_TURN_SEC = 0.20
MEDIUM_TURN_SEC = 1.0
SWEEP_STEP_SEC = 0.30     # one heading step during sweep
REVERSE_SEC = 0.30

# Number of headings to sample during blocked escape
# 8 headings ~= full 360 in timed approximation
SWEEP_HEADINGS = 8
STUCK_COUNT_THRESHOLD = 2
TURN_STEP_SEC = 0.20
GOOD_ENOUGH_DISTANCE = 80     # tune this
TURN_STUCK_EPS_CM = 4
TURN_STUCK_REPEAT_THRESHOLD = 3
REVERSE_STEPS_NORMAL = 1
REVERSE_STEPS_HARD = 2
MIN_IMPROVEMENT = 3
OPEN_SPACE_IGNORE_CM = 260
PROGRESS_EPS_CM = 5
OPEN_SPACE_SCAN_MIN_CM = 160
LOOP_HISTORY_SIZE = 8
LOOP_SPREAD_CM = 20
LOOP_BLOCKED_THRESHOLD = 3
MIN_ESCAPE_SWEEP_STEPS = 3
BIG_TURN_SEC = 0.35

ESCAPE_TURN_DEG = 15
MAX_SWEEP_STEPS = 12
OPEN_SPACE_FOUND_CM = 295
MIN_SCAN_AREA_EXPANSION_CM = 80
MIN_SCAN_POINT_GAP = 12
BIG_SCAN_ANGLE_DEG = 30
BIG_SCAN_STEPS = 6
YAW_ALIGN_STEP_DEG = 10
SAVE_BINARY_MASKS = False
USE_MASK_NAVIGATION = False
YAW_ALIGN_TOLERANCE_DEG = 6.0
YAW_ALIGN_MAX_STEPS_LOCAL = 20
YAW_ALIGN_MAX_STEPS_BIG = 30
SAVE_SEGMENTED_IMAGES = False
SAVE_BINARY_MASKS = False
USE_MASK_NAVIGATION = False
REFINE_AFTER_YAW_ALIGN = False
REFINE_PROBE_STEPS = 3
REFINE_MIN_GAIN_CM = 8
YAW_FINE_STEP_DEG = 1
YAW_FINE_TOLERANCE_DEG = 2.0
YAW_FINE_MAX_STEPS = 2
ESCAPE_CMD_PAUSE_SEC = 0.12

# Escape-loop safety guards. If turn commands time out repeatedly or the
# measured yaw does not change, stop the escape loop instead of repeatedly
# trying all sweep steps while physically stuck.
MAX_FAILED_TURN_COMMANDS_IN_ESCAPE = 2
MAX_NO_YAW_CHANGE_IN_ESCAPE = 2
MIN_YAW_CHANGE_FOR_TURN_DEG = 2.0
# Generic yaw-stall thresholds used by escape, yaw alignment, and big scan recovery.
YAW_NO_CHANGE_THRESHOLD_DEG = MIN_YAW_CHANGE_FOR_TURN_DEG
YAW_NO_CHANGE_LIMIT = MAX_NO_YAW_CHANGE_IN_ESCAPE

# Survey completion guardrails
# The x/y span is only approximate odometry, so do not stop too early.
MIN_PATROL_POINTS_BEFORE_SURVEY_STOP = 12
MIN_SCANS_BEFORE_SURVEY_STOP = 1
MIN_TOTAL_DISTANCE_CM_BEFORE_STOP = 350.0

# =========================
# ROBOT CONTROL
# =========================
def send_cmd(cmd: str, timeout: float = HTTP_TIMEOUT, retries: int = 1) -> bool:
    global _last_cmd_time, _http_failure_count

    now = time.time()
    gap = now - _last_cmd_time
    if gap < MIN_CMD_GAP_SEC:
        time.sleep(MIN_CMD_GAP_SEC - gap)

    for attempt in range(retries + 1):
        try:
            r = requests.get(CONTROL_URL, params={"cmd": cmd}, timeout=timeout)
            _last_cmd_time = time.time()

            print(f"[CMD] {cmd} -> {r.status_code} {r.text.strip()[:60]}")

            if r.ok:
                _http_failure_count = 0
                return True
        except requests.RequestException as e:
            if attempt < retries:
                time.sleep(0.25)
            else:
                _http_failure_count += 1
                print(f"[WARN] command failed: {cmd} -> {e} (http_failures={_http_failure_count})")
                return False

def stop() -> None:
    send_cmd("stop", retries=1)

def forward_short() -> bool:
    return send_cmd("forward_short")

def forward_medium() -> bool:
    return send_cmd("forward_medium")

def forward_long() -> bool:
    return send_cmd("forward_long")

def back(duration: float = REVERSE_SEC) -> None:
    send_cmd("back")
    time.sleep(duration)
    stop()

def left(duration: float = SMALL_TURN_SEC) -> None:
    send_cmd("left")
    time.sleep(duration)
    stop()

def right(duration: float = SMALL_TURN_SEC) -> None:
    send_cmd("right")
    time.sleep(duration)
    stop()

def reverse_steps(n: int = 2, duration: float = REVERSE_SEC) -> None:
    for _ in range(n):
        back(duration=duration)
        time.sleep(0.02)

def buzzer(times: int = 3) -> None:
    for i in range(times):
        ok = send_cmd("buzzer", retries=1)
        if not ok:
            print(f"[WARN] buzzer command {i+1} failed")
        time.sleep(0.45)

def pan_center() -> None:
    send_cmd("pan_center")
    time.sleep(0.15)

def pan_left_once() -> None:
    send_cmd("pan_left")
    time.sleep(0.15)

def pan_right_once() -> None:
    send_cmd("pan_right")
    time.sleep(0.15)

def tilt_center() -> None:
    send_cmd("tilt_center")
    time.sleep(0.15)

def tilt_up_once() -> None:
    send_cmd("tilt_up")
    time.sleep(0.15)

def measure_distance() -> int:
    ok = send_cmd("measure_distance", retries=0)
    if not ok:
        return -1

    time.sleep(0.03)

    try:
        r = requests.get(DISTANCE_URL, timeout=HTTP_TIMEOUT)
        r.raise_for_status()
        return int(r.json().get("distance_cm", -1))
    except Exception as e:
        print(f"[WARN] distance read failed -> {e}")
        return -1

def get_state():
    try:
        r = requests.get(STATE_URL, timeout=HTTP_TIMEOUT)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        print(f"[WARN] state read failed -> {e}")
        return None

def wrap_angle_deg(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def update_odometry_from_forward(
    move_cmd: str,
    before_state,
    after_state,
    x_cm: float,
    y_cm: float,
    max_x_cm: float,
    min_x_cm: float,
    max_y_cm: float,
    min_y_cm: float,
):
    move_cm = FORWARD_DISTANCE_CM.get(move_cmd, 0.0)

    if before_state is not None and after_state is not None:
        yaw_before = wrap_angle_deg(float(before_state.get("yaw", 0.0)))
        yaw_after = wrap_angle_deg(float(after_state.get("yaw", yaw_before)))
        yaw_deg = wrap_angle_deg((yaw_before + yaw_after) / 2.0)
    elif after_state is not None:
        yaw_deg = wrap_angle_deg(float(after_state.get("yaw", 0.0)))
    else:
        yaw_deg = 0.0

    dx = move_cm * math.cos(math.radians(yaw_deg))
    dy = move_cm * math.sin(math.radians(yaw_deg))

    x_cm += dx
    y_cm += dy

    max_x_cm = max(max_x_cm, x_cm)
    min_x_cm = min(min_x_cm, x_cm)
    max_y_cm = max(max_y_cm, y_cm)
    min_y_cm = min(min_y_cm, y_cm)

    span_x = max_x_cm - min_x_cm
    span_y = max_y_cm - min_y_cm

    print(
        f"[ODOM] cmd={move_cmd}, move={move_cm:.1f}cm, yaw={yaw_deg:.1f}, "
        f"x={x_cm:.1f}, y={y_cm:.1f}, "
        f"x_span={span_x:.1f}, y_span={span_y:.1f}, "
        f"x_range=({min_x_cm:.1f},{max_x_cm:.1f}), "
        f"y_range=({min_y_cm:.1f},{max_y_cm:.1f})"
    )

    return x_cm, y_cm, max_x_cm, min_x_cm, max_y_cm, min_y_cm

def get_yaw():
    state = get_state()
    if state is None:
        return None
    return state.get("yaw", None)

def turn_left_angle(deg=15):
    ok = send_cmd(f"turn_left_{deg}", timeout=7.0)
    time.sleep(0.05)
    return ok

def turn_right_angle(deg=15):
    ok = send_cmd(f"turn_right_{deg}", timeout=7.0)
    time.sleep(0.05)
    return ok
_escape_toggle = "right"

def get_next_escape_direction():
    global _escape_toggle
    d = _escape_toggle
    _escape_toggle = "left" if _escape_toggle == "right" else "right"
    return d
# =========================
# CAMERA
# =========================
class CameraClient:
    def __init__(self, stream_url: str) -> None:
        self.stream_url = stream_url
        self.cap: Optional[cv2.VideoCapture] = None
        self.fail_count = 0

    def open(self) -> bool:
        if self.cap is not None and self.cap.isOpened():
            return True
        self.cap = cv2.VideoCapture(self.stream_url)
        self.fail_count = 0
        return self.cap.isOpened()

    def reopen(self) -> bool:
        self.release()
        time.sleep(0.5)
        return self.open()

    def read(self):
        if not self.open():
            return None

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.fail_count += 1
            if self.fail_count >= 3:
                print("[CAMERA] reopening stream...")
                self.reopen()
            return None

        self.fail_count = 0
        return frame

    def flush(self, n: int = PHOTO_FLUSH_FRAMES) -> int:
        """Discard buffered MJPEG frames so the next read is closer to current view."""
        if not self.open():
            return 0

        valid = 0
        for _ in range(max(0, n)):
            ret, frame = self.cap.read()
            if ret and frame is not None:
                valid += 1
            time.sleep(0.03)

        print(f"[CAMERA] flushed {n} frames, valid={valid}")
        return valid

    def capture_fresh(self, settle_sec: float = CAMERA_SETTLE_SEC, flush_frames: int = PHOTO_FLUSH_FRAMES, force_reopen: bool = False):
        """Return a fresh frame after robot/camera has settled.

        This avoids saving an old buffered frame after pan/tilt/turn/move.
        """
        time.sleep(settle_sec)

        if force_reopen or REOPEN_STREAM_BEFORE_CAPTURE:
            self.reopen()

        self.flush(flush_frames)

        # Read a few times and keep the last valid frame.
        frame = None
        for _ in range(3):
            f = self.read()
            if f is not None:
                frame = f.copy()
            time.sleep(0.05)

        if frame is None:
            print("[CAMERA] fresh capture failed")
        return frame

    def release(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

# =========================
# YOLO DETECTOR
# =========================
class PersonDetector:
    def __init__(self, model_path: str = YOLO_MODEL, conf: float = CONF_THRESHOLD) -> None:
        self.model = YOLO(model_path)
        self.conf = conf

    def predict(self, frame, conf: Optional[float] = None):
        c = self.conf if conf is None else conf
        results = self.model.predict(source=frame, conf=c, verbose=False)
        if not results:
            return None
        return results[0]

    def detect_person_from_result(self, result, frame) -> Tuple[bool, list]:
        boxes = []

        if result is None or result.boxes is None:
            return False, boxes

        h, w = frame.shape[:2]
        frame_area = w * h

        for box in result.boxes:
            cls_id = int(box.cls[0].item())
            if cls_id != PERSON_CLASS_ID:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            area = max(0, (x2 - x1)) * max(0, (y2 - y1))

            if area < MIN_PERSON_AREA_RATIO * frame_area:
                continue

            boxes.append((int(x1), int(y1), int(x2), int(y2)))

        return len(boxes) > 0, boxes
# =========================
# SAVE / ALERT
# =========================
def timestamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")

def save_frame(frame, prefix: str) -> str:
    path = os.path.join(SAVE_DIR, f"{prefix}_{timestamp()}.jpg")
    cv2.imwrite(path, frame)
    return path


def save_canny_frame(frame, prefix: str) -> str:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 60, 160)
    path = os.path.join(SAVE_DIR, f"{prefix}_canny_{timestamp()}.jpg")
    cv2.imwrite(path, edges)
    return path


def poll_keyboard_command():
    """Non-blocking keyboard command. Works in the Linux terminal used for the robot."""
    if not ENABLE_KEYBOARD_CONTROL:
        return None
    try:
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if readable:
            cmd = sys.stdin.readline().strip().lower()
            if cmd:
                return cmd[0]
    except Exception:
        return None
    return None


def pause_until_resume(context: str = "manual pause") -> bool:
    """Pause safely. Return False when user requests quit."""
    stop()
    print(f"[CTRL] paused: {context}")
    print("[CTRL] type r + Enter to resume, q + Enter to quit safely")
    while True:
        try:
            cmd = sys.stdin.readline().strip().lower()
        except KeyboardInterrupt:
            return False
        if cmd == "r":
            print("[CTRL] resuming")
            return True
        if cmd == "q":
            print("[CTRL] quit requested")
            return False
        print("[CTRL] waiting for r or q")


def handle_keyboard_checkpoint(context: str = "checkpoint") -> bool:
    """Return False when the patrol should stop."""
    cmd = poll_keyboard_command()
    if cmd == "p":
        return pause_until_resume(context)
    if cmd == "q":
        print("[CTRL] quit requested")
        stop()
        return False
    return True


def save_forward_patrol_photo(camera: CameraClient, step: int, patrol_points: int, x_cm: float, y_cm: float, front_dist: int):
    """Save a fresh forward-facing image and Canny edge image without running YOLO."""
    stop()

    # Re-centering pan/tilt before every photo adds several HTTP commands per step.
    # Do it only periodically; otherwise assume the camera is already forward-facing.
    if PAN_TILT_CENTER_EVERY_N_PHOTOS and patrol_points % PAN_TILT_CENTER_EVERY_N_PHOTOS == 0:
        pan_center()
        tilt_center()
        time.sleep(CAMERA_SETTLE_SEC)

    yaw = get_yaw()
    if yaw is None:
        yaw = 0.0

    frame = camera.capture_fresh(flush_frames=PHOTO_FLUSH_FRAMES, force_reopen=False)
    if frame is None:
        print("[PHOTO] no fresh frame captured")
        return None

    label = (
        f"{PATROL_PHOTO_PREFIX}_step_{step:04d}_point_{patrol_points:04d}_"
        f"yaw_{norm360(float(yaw)):.1f}_dist_{front_dist}_"
        f"x_{x_cm:.1f}_y_{y_cm:.1f}"
    )
    img_path = save_frame(frame, label)
    print(f"[PHOTO] saved {img_path}")

    canny_path = None
    if SAVE_PATROL_CANNY:
        canny_path = save_canny_frame(frame, label)
        print(f"[PHOTO] saved canny {canny_path}")

    time.sleep(POST_PHOTO_PAUSE_SEC)

    if AUTO_PAUSE_AFTER_PATROL_PHOTO:
        ok = pause_until_resume(f"after patrol photo {img_path}")
        if not ok:
            raise KeyboardInterrupt

    return img_path

def draw_boxes(frame, boxes):
    out = frame.copy()
    for x1, y1, x2, y2 in boxes:
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            out,
            "person",
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
    return out

def alert_person_detected(frame, boxes, label: str) -> None:
    annotated = draw_boxes(frame, boxes)
    path = save_frame(annotated, f"detected_{label}")
    print(f"[ALERT] Person detected. Saved: {path}")
    buzzer(times=3)

# =========================
# STARTUP HEALTH CHECK
# =========================
def startup_health_check(camera: CameraClient, max_attempts: int = 5) -> bool:
    print("[STARTUP] Running health check...")

    for attempt in range(1, max_attempts + 1):
        print(f"[STARTUP] Attempt {attempt}/{max_attempts}")

        ok_stop = send_cmd("stop", retries=1)
        time.sleep(0.2)

        ok_pan = send_cmd("pan_center", retries=1)
        time.sleep(0.2)

        ok_tilt = send_cmd("tilt_center", retries=1)
        time.sleep(0.2)

        dist = measure_distance()
        frame = camera.read()

        ok_dist = dist != -1
        ok_frame = frame is not None

        print(f"[STARTUP] stop={ok_stop} pan={ok_pan} tilt={ok_tilt} dist={dist} frame={ok_frame}")

        if ok_stop and ok_pan and ok_tilt and ok_dist and ok_frame:
            print("[STARTUP] Health check passed.")
            return True

        print("[STARTUP] Health check failed, waiting before retry...")
        camera.reopen()
        time.sleep(1.0)

    print("[STARTUP] Health check failed after retries.")
    return False

# =========================
# NAVIGATION
# =========================
def choose_forward_command(distance_cm: int) -> Optional[str]:
    if distance_cm < 0:
        return None
    if distance_cm < OBSTACLE_THRESHOLD:
        return None
    if distance_cm < 55:
        return "forward_short"
    if distance_cm < 90:
        return "forward_medium"
    return "forward_long"

def execute_forward(cmd: str) -> None:
    if cmd == "forward_short":
        forward_short()
    elif cmd == "forward_medium":
        forward_medium()
    elif cmd == "forward_long":
        forward_long()

def heading_sweep() -> Tuple[int, List[int]]:
    """
    Approximate 360-degree scan by rotating in fixed timed steps
    and measuring front distance at each heading.
    Returns:
        best_index, distances
    """
    print("[ESCAPE] Starting heading sweep...")
    distances: List[int] = []

    stop()
    time.sleep(0.15)
    pan_center()
    tilt_center()
    time.sleep(0.15)

    for i in range(SWEEP_HEADINGS):
        d = measure_distance()
        distances.append(d)
        print(f"[ESCAPE] heading {i}: distance={d}")

        # turn to next heading except after last sample
        if i < SWEEP_HEADINGS - 1:
            send_cmd("right")
            time.sleep(SWEEP_STEP_SEC)
            stop()
            time.sleep(0.12)

    valid = [d if d >= 0 else -1 for d in distances]
    best_index = max(range(len(valid)), key=lambda k: valid[k])

    print(f"[ESCAPE] sweep distances = {distances}")
    print(f"[ESCAPE] best heading index = {best_index}, distance = {distances[best_index]}")
    return best_index, distances

def rotate_to_heading(best_index: int) -> None:
    """
    After the sweep, robot is near the final heading.
    We turn further right to wrap around to the chosen heading.
    """
    if best_index == SWEEP_HEADINGS - 1:
        return

    steps_to_move = (best_index + 1) % SWEEP_HEADINGS
    # after the sweep we are near the last heading; wrap to desired heading
    wrap_steps = (best_index + 1) % SWEEP_HEADINGS

    # Better approximation: continue turning right until heading wraps to best
    remaining_steps = (best_index + 1) % SWEEP_HEADINGS
    if remaining_steps == 0:
        return

    print(f"[ESCAPE] rotating to chosen heading with {remaining_steps} extra steps")
    for _ in range(remaining_steps):
        send_cmd("right")
        time.sleep(SWEEP_STEP_SEC)
        stop()
        time.sleep(0.12)

def is_turn_stuck(prev_d, curr_d, repeat_count):
    """
    Detect whether the robot is not effectively changing heading while turning.
    """
    if prev_d <= 0 or curr_d <= 0:
        return repeat_count

    if abs(curr_d - prev_d) < TURN_STUCK_EPS_CM:
        repeat_count += 1
    else:
        repeat_count = 0

    return repeat_count

def small_left_step():
    return turn_left_angle(ESCAPE_TURN_DEG)

def small_right_step():
    return turn_right_angle(ESCAPE_TURN_DEG)

def opposite_turn(turn):
    return "left" if turn == "right" else "right"

def escape_search_one_direction(search_dir: str) -> bool:
    step_fn = small_left_step if search_dir == "left" else small_right_step

    start_distance = measure_distance()
    if start_distance < 0:
        start_distance = 0

    start_yaw = get_yaw()
    if start_yaw is None:
        start_yaw = 0.0

    print(f"[ESCAPE] initial distance = {start_distance}, yaw={start_yaw:.1f}")

    best_distance = start_distance
    best_yaw = start_yaw
    prev_distance = start_distance
    prev_yaw = start_yaw
    found_improvement = False
    yaw_no_change_count = 0
    failed_turn_count = 0

    for step in range(1, MAX_SWEEP_STEPS + 1):
        ok = step_fn()

        if not ok:
            failed_turn_count += 1
            print(f"[ESCAPE] turn command failed count={failed_turn_count}")

            if failed_turn_count >= MAX_FAILED_TURN_COMMANDS_IN_ESCAPE:
                print("[ESCAPE] repeated turn command failures -> aborting direction")
                stop()
                return False
        else:
            failed_turn_count = 0

        time.sleep(ESCAPE_CMD_PAUSE_SEC)

        d = measure_distance()
        if d < 0:
            d = 0

        yaw = get_yaw()
        if yaw is None:
            yaw = prev_yaw

        yaw_delta = abs(angle_diff_deg(yaw, prev_yaw))

        print(
            f"[ESCAPE] step={step}, dir={search_dir}, "
            f"distance={d}, yaw={yaw:.1f}, yaw_delta={yaw_delta:.2f}"
        )

        if yaw_delta < YAW_NO_CHANGE_THRESHOLD_DEG:
            yaw_no_change_count += 1
            print(
                f"[ESCAPE] yaw not changing sufficiently "
                f"(delta={yaw_delta:.2f}) "
                f"count={yaw_no_change_count}"
            )
        else:
            yaw_no_change_count = 0

        if yaw_no_change_count >= YAW_NO_CHANGE_LIMIT:
            print("[ESCAPE] likely physically stuck/carpet trap -> hard reverse")

            reverse_steps(n=2, duration=0.45)
            time.sleep(0.15)

            yaw_after_reverse = get_yaw()
            if yaw_after_reverse is None:
                print("[ESCAPE] yaw unavailable after hard reverse -> aborting direction")
                stop()
                return False

            reverse_yaw_delta = abs(angle_diff_deg(yaw_after_reverse, yaw))
            print(
                f"[ESCAPE] yaw after hard reverse={yaw_after_reverse:.1f}, "
                f"reverse_yaw_delta={reverse_yaw_delta:.2f}"
            )

            if reverse_yaw_delta < YAW_NO_CHANGE_THRESHOLD_DEG:
                print("[ESCAPE] still no yaw change after reverse -> aborting direction")
                stop()
                return False

            yaw_no_change_count = 0
            prev_yaw = yaw_after_reverse
            prev_distance = measure_distance()
            if prev_distance < 0:
                prev_distance = d
            continue

        if d >= OPEN_SPACE_FOUND_CM:
            print("[ESCAPE] open space found -> stop search and continue")
            return True

        if d > best_distance + MIN_IMPROVEMENT:
            best_distance = d
            best_yaw = yaw
            found_improvement = True

        if step >= MIN_ESCAPE_SWEEP_STEPS and found_improvement and d < prev_distance:
            print(f"[ESCAPE] passed peak -> aligning to best yaw {best_yaw:.1f}")
            align_ok = align_to_yaw(
                best_yaw,
                tolerance_deg=YAW_ALIGN_TOLERANCE_DEG,
                max_steps=YAW_ALIGN_MAX_STEPS_LOCAL,
            )

            if not align_ok:
                print("[ESCAPE] yaw alignment failed after peak search")
                return False

            if REFINE_AFTER_YAW_ALIGN:
                refine_heading_by_distance()

            return True

        prev_distance = d
        prev_yaw = yaw

    if best_distance >= GOOD_ENOUGH_DISTANCE:
        print(f"[ESCAPE] good enough direction found -> align yaw {best_yaw:.1f}")
        align_ok = align_to_yaw(
            best_yaw,
            tolerance_deg=YAW_ALIGN_TOLERANCE_DEG,
            max_steps=YAW_ALIGN_MAX_STEPS_LOCAL,
        )

        if not align_ok:
            print("[ESCAPE] yaw alignment failed for good-enough direction")
            return False

        if REFINE_AFTER_YAW_ALIGN:
            refine_heading_by_distance()

        return True

    print(f"[ESCAPE] {search_dir} failed, best_distance={best_distance}")
    return False


    start_distance = measure_distance()
    if start_distance < 0:
        start_distance = 0

    start_yaw = get_yaw()
    if start_yaw is None:
        start_yaw = 0.0

    print(f"[ESCAPE] initial distance = {start_distance}, yaw={start_yaw:.1f}")

    best_distance = start_distance
    best_yaw = start_yaw
    prev_distance = start_distance
    prev_yaw = start_yaw
    found_improvement = False
    failed_turn_commands = 0
    no_yaw_change_count = 0

    for step in range(1, MAX_SWEEP_STEPS + 1):
        yaw_before_turn = get_yaw()
        if yaw_before_turn is None:
            yaw_before_turn = prev_yaw

        ok_turn = step_fn()
        time.sleep(ESCAPE_CMD_PAUSE_SEC)

        d = measure_distance()
        if d < 0:
            d = 0

        yaw = get_yaw()
        if yaw is None:
            yaw = prev_yaw

        yaw_change = abs(angle_diff_deg(yaw, yaw_before_turn))

        print(
            f"[ESCAPE] step={step}, dir={search_dir}, distance={d}, "
            f"yaw={yaw:.1f}, turn_ok={ok_turn}, yaw_change={yaw_change:.1f}"
        )

        # A timed-out HTTP command can still occasionally execute on the ESP32.
        # Therefore do not abort on timeout alone. Abort only when the command
        # failed AND the yaw also did not meaningfully change.
        if not ok_turn and yaw_change < MIN_YAW_CHANGE_FOR_TURN_DEG:
            failed_turn_commands += 1
            print(
                f"[ESCAPE] turn command failed and yaw did not change "
                f"({failed_turn_commands}/{MAX_FAILED_TURN_COMMANDS_IN_ESCAPE})"
            )
        else:
            failed_turn_commands = 0

        if yaw_change < MIN_YAW_CHANGE_FOR_TURN_DEG:
            no_yaw_change_count += 1
            print(
                f"[ESCAPE] yaw not changing during escape "
                f"({no_yaw_change_count}/{MAX_NO_YAW_CHANGE_IN_ESCAPE})"
            )
        else:
            no_yaw_change_count = 0

        if failed_turn_commands >= MAX_FAILED_TURN_COMMANDS_IN_ESCAPE:
            print("[ESCAPE] aborting this direction: repeated failed turn commands")
            stop()
            return False

        if no_yaw_change_count >= MAX_NO_YAW_CHANGE_IN_ESCAPE:
            print("[ESCAPE] aborting this direction: robot/IMU yaw not changing")
            stop()
            return False

        if d >= OPEN_SPACE_FOUND_CM:
            print("[ESCAPE] open space found -> stop search and continue")
            return True

        if d > best_distance + MIN_IMPROVEMENT:
            best_distance = d
            best_yaw = yaw
            found_improvement = True

        if step >= MIN_ESCAPE_SWEEP_STEPS and found_improvement and d < prev_distance:
            print(f"[ESCAPE] passed peak -> aligning to best yaw {best_yaw:.1f}")
            aligned = align_to_yaw(
                best_yaw,
                tolerance_deg=YAW_ALIGN_TOLERANCE_DEG,
                max_steps=YAW_ALIGN_MAX_STEPS_LOCAL,
            )

            if not aligned:
                print("[ESCAPE] alignment to best yaw failed")
                return False

            if REFINE_AFTER_YAW_ALIGN:
                refine_heading_by_distance()

            return True

        prev_distance = d
        prev_yaw = yaw

    if best_distance >= GOOD_ENOUGH_DISTANCE:
        print(f"[ESCAPE] good enough direction found -> align yaw {best_yaw:.1f}")
        aligned = align_to_yaw(
            best_yaw,
            tolerance_deg=YAW_ALIGN_TOLERANCE_DEG,
            max_steps=YAW_ALIGN_MAX_STEPS_LOCAL,
        )

        if not aligned:
            print("[ESCAPE] good-enough yaw alignment failed")
            return False

        if REFINE_AFTER_YAW_ALIGN:
            refine_heading_by_distance()

        return True

    print(f"[ESCAPE] {search_dir} failed, best_distance={best_distance}")
    return False

def refine_heading_by_distance(max_probe_steps=REFINE_PROBE_STEPS):
    """
    After yaw alignment, refine locally using sonar distance.
    Tests headings around current direction and aligns to the best measured yaw.
    """
    stop()
    time.sleep(0.05)

    readings = []

    center_yaw = get_yaw()
    if center_yaw is None:
        print("[REFINE] yaw unavailable")
        return measure_distance()

    center_d = measure_distance()
    if center_d < 0:
        center_d = 0

    readings.append((0, center_d, center_yaw))
    print(f"[REFINE] offset=0, distance={center_d}, yaw={center_yaw:.1f}")

    # Probe left: -1, -2, -3
    for offset in range(-1, -max_probe_steps - 1, -1):
        turn_left_angle(YAW_ALIGN_STEP_DEG)
        d = measure_distance()
        yaw = get_yaw()

        if d < 0:
            d = 0
        if yaw is None:
            yaw = center_yaw

        readings.append((offset, d, yaw))
        print(f"[REFINE] offset={offset}, distance={d}, yaw={yaw:.1f}")

    # Return to center
    for _ in range(max_probe_steps):
        turn_right_angle(YAW_ALIGN_STEP_DEG)

    # Probe right: +1, +2, +3
    for offset in range(1, max_probe_steps + 1):
        turn_right_angle(YAW_ALIGN_STEP_DEG)
        d = measure_distance()
        yaw = get_yaw()

        if d < 0:
            d = 0
        if yaw is None:
            yaw = center_yaw

        readings.append((offset, d, yaw))
        print(f"[REFINE] offset={offset}, distance={d}, yaw={yaw:.1f}")

    best_offset, best_d, best_yaw = max(readings, key=lambda x: x[1])

    print(
        f"[REFINE] best_offset={best_offset}, "
        f"best_distance={best_d}, best_yaw={best_yaw:.1f}"
    )

    # Only move to refined heading if it is meaningfully better
    if best_d >= center_d + REFINE_MIN_GAIN_CM:
        print("[REFINE] applying refined heading")
        align_to_yaw(
            best_yaw,
            tolerance_deg=YAW_ALIGN_TOLERANCE_DEG,
            max_steps=YAW_ALIGN_MAX_STEPS_LOCAL,
        )
    else:
        print("[REFINE] no meaningful gain -> returning to center yaw")
        align_to_yaw(
            center_yaw,
            tolerance_deg=YAW_ALIGN_TOLERANCE_DEG,
            max_steps=YAW_ALIGN_MAX_STEPS_LOCAL,
        )

    verified = measure_distance()
    print(f"[REFINE] final verified distance={verified}")

    return verified

def choose_yaw_step(abs_diff):
    if abs_diff > 100:
        return 90
    if abs_diff > 45:
        return 30
    if abs_diff > 18:
        return 15
    if abs_diff > 10:
        return 10
    if abs_diff > 5:
        return 5
    if abs_diff > 1:
        return 1
    return 0

def smart_escape(last_turn: str) -> str:
    print("[PATROL] Blocked -> peak search escape")
    stop()
    time.sleep(0.05)

    reverse_steps(n=REVERSE_STEPS_NORMAL, duration=0.30)
    time.sleep(0.05)

    preferred_dir = get_next_escape_direction()

    for search_dir in [preferred_dir, opposite_turn(preferred_dir)]:
        print(f"[ESCAPE] trying {search_dir}")

        success = escape_search_one_direction(search_dir)

        if success:
            return search_dir

    print("[ESCAPE] both directions failed -> hard reverse recovery and continue")
    reverse_steps(n=REVERSE_STEPS_HARD, duration=0.40)
    stop()
    return opposite_turn(preferred_dir)

def execute_wandering_step(dist: int, last_turn: str) -> str:
    move_cmd = choose_forward_command(dist)

    if move_cmd is None:
        return smart_escape(last_turn)

    print(f"[PATROL] Executing {move_cmd}")
    execute_forward(move_cmd)
    stop()
    time.sleep(0.05)

    return last_turn
# =========================
# VIDEO BURST DETECTION
# =========================
def capture_video_burst(camera: CameraClient, duration_sec: float = BURST_DURATION_SEC, fps_limit: float = BURST_FPS):
    """Capture a short burst after flushing stale MJPEG frames.

    The first returned frame should now correspond much more closely to the
    current pan/tilt/robot heading.
    """
    def grab_once():
        frames = []
        start = time.time()
        min_interval = 1.0 / fps_limit
        last_capture = 0.0

        # Discard stale frames before burst capture.
        camera.flush(SCAN_FLUSH_FRAMES)

        while time.time() - start < duration_sec:
            now = time.time()
            if now - last_capture < min_interval:
                time.sleep(0.01)
                continue

            frame = camera.read()
            if frame is not None:
                frames.append(frame.copy())
                last_capture = now
            else:
                time.sleep(0.03)

        return frames

    if REOPEN_STREAM_BEFORE_CAPTURE:
        camera.reopen()
        time.sleep(CAMERA_SETTLE_SEC)

    frames = grab_once()
    if frames:
        return frames

    print("[CAMERA] Burst empty, retrying once...")
    camera.reopen()
    time.sleep(0.5)
    return grab_once()

def detect_person_in_burst(detector: PersonDetector, frames):
    hit_count = 0
    best_frame = None
    best_boxes = []
    best_result = None

    for frame in frames:
        result = detector.predict(frame)
        found, boxes = detector.detect_person_from_result(result, frame)

        if found:
            hit_count += 1
            best_frame = frame
            best_boxes = boxes
            best_result = result

    return hit_count >= 1, best_frame, best_boxes, best_result, hit_count
# =========================
# DETECTION SCAN
# =========================
def get_detection_positions():
    """
    Richer scan coverage:
    3 left, 3 right, 3 up
    """
    return [
        ("center_center", pan_center, tilt_center),

        ("left1_center", pan_left_once, None),
        ("left2_center", pan_left_once, None),
        ("left3_center", pan_left_once, None),

        ("center_center_again", pan_center, tilt_center),

        ("right1_center", pan_right_once, None),
        ("right2_center", pan_right_once, None),
        ("right3_center", pan_right_once, None),

        ("center_up1", pan_center, tilt_up_once),
        ("center_up2", None, tilt_up_once),
        ("center_up3", None, tilt_up_once),
    ]
def build_obstacle_mask_from_result(result, frame_shape):
    """
    Returns binary obstacle mask:
    255 = detected object / occupied
    0   = free/background
    """
    h, w = frame_shape[:2]

    if result is None or result.masks is None:
        return None

    masks = result.masks.data.cpu().numpy()

    if len(masks) == 0:
        return None

    combined = None

    for m in masks:
        m = cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)
        m = (m > 0.5).astype("uint8")

        if combined is None:
            combined = m
        else:
            combined = cv2.bitwise_or(combined, m)

    return combined * 255


def score_free_space_from_mask(obstacle_mask):
    """
    Uses lower half of image because floor/opening matters most for navigation.
    Returns:
        direction_score:
            negative -> more free space left
            positive -> more free space right
            near zero -> center
        clear_score:
            fraction of free pixels in bottom region
    """
    h, w = obstacle_mask.shape[:2]

    y0 = int(h * MASK_BOTTOM_REGION_RATIO)
    roi = obstacle_mask[y0:h, :]

    free = (roi == 0).astype("uint8")

    left = free[:, :w // 3].mean()
    center = free[:, w // 3: 2 * w // 3].mean()
    right = free[:, 2 * w // 3:].mean()

    clear_score = max(left, center, right)

    direction_score = right - left

    print(
        f"[MASK NAV] free left={left:.2f}, center={center:.2f}, "
        f"right={right:.2f}, direction={direction_score:.2f}, clear={clear_score:.2f}"
    )

    return direction_score, clear_score, left, center, right


def save_mask_debug(frame, result, label):
    """
    Saves:
    - YOLO annotated image
    - binary obstacle mask
    """
    if result is None:
        return

    annotated = result.plot()
    path = save_frame(annotated, f"masknav_annotated_{label}")
    print(f"[MASK NAV] saved annotated {path}")

    obstacle_mask = build_obstacle_mask_from_result(result, frame.shape)
    if SAVE_BINARY_MASKS and obstacle_mask is not None:
        mask_path = save_frame(obstacle_mask, f"mask_binary_{label}")
        print(f"[SEG] saved binary mask {mask_path}")

def mask_based_navigation_step(camera: CameraClient, detector: PersonDetector, front_dist: int) -> bool:
    """
    One visual+sonar navigation step.
    Returns True if it handled movement/turning.
    """
    if front_dist < MASK_NAV_MIN_DIST_CM:
        print("[MASK NAV] too close -> skip mask nav")
        return False

    frame = camera.read()
    if frame is None:
        print("[MASK NAV] no frame")
        return False

    result = detector.predict(frame, conf=MASK_NAV_CONF)

    save_mask_debug(frame, result, "nav")

    obstacle_mask = build_obstacle_mask_from_result(result, frame.shape)

    if obstacle_mask is None:
        print("[MASK NAV] no masks found -> treat as visually open")
        if front_dist >= 90:
            execute_forward("forward_long")
        elif front_dist >= 55:
            execute_forward("forward_medium")
        else:
            execute_forward("forward_short")
        stop()
        time.sleep(0.01)
        return True

    direction_score, clear_score, left, center, right = score_free_space_from_mask(obstacle_mask)

    if clear_score < MASK_CLEAR_SCORE_THRESHOLD:
        print("[MASK NAV] poor visual clearance -> let sonar recovery handle it")
        return False

    if direction_score <= MASK_TURN_LEFT_THRESHOLD:
        print("[MASK NAV] free space biased LEFT -> turn left")
        turn_left_angle(10)
        return True

    if direction_score >= MASK_TURN_RIGHT_THRESHOLD:
        print("[MASK NAV] free space biased RIGHT -> turn right")
        turn_right_angle(10)
        return True

    print("[MASK NAV] free space centered -> move forward")

    if front_dist >= 90:
        execute_forward("forward_long")
    elif front_dist >= 55:
        execute_forward("forward_medium")
    else:
        execute_forward("forward_short")

    stop()
    time.sleep(0.01)
    return True


def scan_and_detect(camera: CameraClient, detector: PersonDetector, save_segmented: bool = False):
    positions = get_detection_positions()
    detected_any = False

    pan_center()
    tilt_center()
    time.sleep(0.2)

    # Reopen once at the start of the scan to clear stale MJPEG buffering,
    # instead of reopening for every pan/tilt view.
    camera.reopen()

    for label, pan_fn, tilt_fn in positions:
        if pan_fn is not None:
            pan_fn()
        if tilt_fn is not None:
            tilt_fn()

        time.sleep(CAMERA_SETTLE_SEC)

        # Capture a fresh still frame for saving/debugging. This prevents a
        # saved image from belonging to the previous pan/tilt/heading.
        fresh_frame = camera.capture_fresh(flush_frames=SCAN_FLUSH_FRAMES, force_reopen=FORCE_REOPEN_EACH_SCAN_VIEW)
        if fresh_frame is None:
            print(f"[SCAN] No fresh frame captured for {label}")
            continue

        saved_img = save_frame(fresh_frame, f"scan_{label}")
        print(f"[SCAN] saved fresh image {saved_img}")

        # YOLO can be run either on the fresh saved frame only, or on a short
        # burst for confirmation. Burst is more robust but much heavier on the
        # ESP32 stream and laptop inference.
        frames = [fresh_frame]
        if ENABLE_YOLO_BURST_CONFIRMATION:
            frames.extend(capture_video_burst(camera))

        found, best_frame, best_boxes, best_result, hit_count = detect_person_in_burst(detector, frames)
        print(f"[YOLO] view={label}, hits={hit_count}, detected={found}")

        if SAVE_SEGMENTED_IMAGES and save_segmented:
            result0 = detector.predict(frames[0])
            if result0 is not None:
                annotated = result0.plot()
                seg_path = save_frame(annotated, f"segmented_{label}")
                print(f"[SEG] saved segmented image {seg_path}")

        if SAVE_BINARY_MASKS:
            result0 = detector.predict(frames[0])
            obstacle_mask = build_obstacle_mask_from_result(result0, frames[0].shape)
            if obstacle_mask is not None:
                mask_path = save_frame(obstacle_mask, f"mask_binary_{label}")
                print(f"[SEG] saved binary mask {mask_path}")

        if found and best_frame is not None:
            alert_person_detected(best_frame, best_boxes, label)

            detected_any = True

    pan_center()
    tilt_center()
    return detected_any
# =========================
# PATROL LOOP
# =========================


def update_stuck_counter(before, after, counter):
    """
    Only treat low distance change as 'stuck' when robot is not in open space.
    """
    if before <= 0 or after <= 0:
        return counter

    # Ignore low-progress check in open space
    if before < OPEN_SPACE_IGNORE_CM or after < OPEN_SPACE_IGNORE_CM:
        change = abs(after - before)
        print(f"[STUCK CHECK] open-space before={before}, after={after}, change={change}")

        if change < PROGRESS_EPS_CM:
            counter += 1
            print(f"[STUCK] No progress even in open space. Counter = {counter}")
        else:
            counter = 0

        return counter

    change = abs(after - before)
    print(f"[STUCK CHECK] before={before}, after={after}, change={change}")


    if change < PROGRESS_EPS_CM:
        counter += 1
        print(f"[STUCK] No progress detected. Counter = {counter}")
    else:
        counter = 0

    return counter

def should_run_scan(front_dist: int, patrol_points: int, last_scan_point: int) -> bool:
    """
    Run scan only if:
    1. enough patrol points have passed since last scan
    2. robot is in sufficiently open space for good detection
    """
    if front_dist < 0:
        return False

    if patrol_points - last_scan_point < DETECTION_EVERY_N_POINTS:
        return False

    if front_dist < OPEN_SPACE_SCAN_MIN_CM or front_dist >= 280:
        return False

    return True

def detect_local_loop(recent_distances, blocked_events):
    """
    Detect whether robot is cycling in the same trapped region.
    """
    if len(recent_distances) < LOOP_HISTORY_SIZE:
        return False

    dmin = min(recent_distances)
    dmax = max(recent_distances)
    spread = dmax - dmin

    print(f"[LOOP CHECK] distances={recent_distances}, spread={spread}, blocked_events={blocked_events}")

    return spread <= LOOP_SPREAD_CM and blocked_events >= LOOP_BLOCKED_THRESHOLD

def norm360(angle):
    return angle % 360.0


def angle_diff_deg(target, current):
    """
    Shortest signed angle from current to target.
    Positive means target is counter-clockwise in math sense,
    but actual robot command direction is tested adaptively.
    """
    target = norm360(target)
    current = norm360(current)
    return (target - current + 180.0) % 360.0 - 180.0


def align_to_yaw(target_yaw, tolerance_deg=YAW_ALIGN_TOLERANCE_DEG, max_steps=30):
    current_yaw = get_yaw()
    if current_yaw is None:
        print("[YAW ALIGN] yaw unavailable at start")
        return False

    diff = angle_diff_deg(target_yaw, current_yaw)
    abs_diff = abs(diff)

    print(
        f"[YAW ALIGN] start target={target_yaw:.1f}({norm360(target_yaw):.1f}), "
        f"current={current_yaw:.1f}({norm360(current_yaw):.1f}), diff={diff:.2f}"
    )

    if abs_diff <= tolerance_deg:
        print("[YAW ALIGN] aligned within tolerance")
        stop()
        return True

    use_left = diff > 0

    for i in range(max_steps):
        step_deg = choose_yaw_step(abs_diff)

        if step_deg == 0:
            print("[YAW ALIGN] aligned")
            stop()
            return True

        old_abs_diff = abs_diff
        attempted_reverse = False

        while True:
            if use_left:
                cmd_name = f"left_{step_deg}"
                ok = turn_left_angle(step_deg)
            else:
                cmd_name = f"right_{step_deg}"
                ok = turn_right_angle(step_deg)

            if not ok:
                print(f"[YAW ALIGN] turn command failed: {cmd_name}")
                stop()
                return False

            time.sleep(ESCAPE_CMD_PAUSE_SEC)

            new_yaw = get_yaw()
            if new_yaw is None:
                print("[YAW ALIGN] yaw unavailable after command")
                stop()
                return False

            yaw_delta = abs(angle_diff_deg(new_yaw, current_yaw))

            if yaw_delta < YAW_NO_CHANGE_THRESHOLD_DEG:
                print(
                    f"[YAW ALIGN] yaw not changing enough "
                    f"(delta={yaw_delta:.2f}) -> likely physical stall"
                )
                stop()
                return False

            new_diff = angle_diff_deg(target_yaw, new_yaw)
            new_abs_diff = abs(new_diff)

            print(
                f"[YAW ALIGN] step={i+1}, cmd={cmd_name}, "
                f"target={norm360(target_yaw):.1f}, "
                f"current={norm360(new_yaw):.1f}, "
                f"diff={new_diff:.2f}, yaw_delta={yaw_delta:.2f}"
            )

            if new_abs_diff < old_abs_diff:
                current_yaw = new_yaw
                diff = new_diff
                abs_diff = new_abs_diff
                use_left = diff > 0
                break

            if not attempted_reverse:
                print("[YAW ALIGN] correction worsened -> reversing once")
                use_left = not use_left
                attempted_reverse = True
                continue

            print("[YAW ALIGN] reverse correction also failed")
            stop()
            return False

        if abs_diff <= tolerance_deg:
            print("[YAW ALIGN] aligned within tolerance")
            stop()
            return True

    print("[YAW ALIGN] max correction steps reached")
    stop()
    return False


    diff = angle_diff_deg(target_yaw, current_yaw)
    abs_diff = abs(diff)

    print(
        f"[YAW ALIGN] start target={target_yaw:.1f}({norm360(target_yaw):.1f}), "
        f"current={current_yaw:.1f}({norm360(current_yaw):.1f}), diff={diff:.2f}"
    )

    if abs_diff <= tolerance_deg:
        print("[YAW ALIGN] aligned within tolerance")
        stop()
        return True

    use_left = diff > 0

    for i in range(max_steps):
        step_deg = choose_yaw_step(abs_diff)

        if step_deg == 0:
            print("[YAW ALIGN] aligned")
            stop()
            return True

        old_abs_diff = abs_diff
        attempted_reverse = False

        while True:
            if use_left:
                cmd_name = f"left_{step_deg}"
                turn_left_angle(step_deg)
            else:
                cmd_name = f"right_{step_deg}"
                turn_right_angle(step_deg)

            time.sleep(ESCAPE_CMD_PAUSE_SEC)

            new_yaw = get_yaw()
            if new_yaw is None:
                print("[YAW ALIGN] yaw unavailable after command")
                stop()
                return False

            new_diff = angle_diff_deg(target_yaw, new_yaw)
            new_abs_diff = abs(new_diff)

            print(
                f"[YAW ALIGN] step={i+1}, cmd={cmd_name}, "
                f"target={norm360(target_yaw):.1f}, "
                f"current={norm360(new_yaw):.1f}, diff={new_diff:.2f}"
            )

            if new_abs_diff < old_abs_diff:
                current_yaw = new_yaw
                diff = new_diff
                abs_diff = new_abs_diff
                use_left = diff > 0
                break

            if not attempted_reverse:
                print("[YAW ALIGN] correction worsened -> reversing once")
                use_left = not use_left
                attempted_reverse = True
                continue

            print("[YAW ALIGN] reverse correction also failed")
            stop()
            return False

        if abs_diff <= tolerance_deg:
            print("[YAW ALIGN] aligned within tolerance")
            stop()
            return True

    print("[YAW ALIGN] max correction steps reached")
    stop()
    return False

def fine_align_to_yaw(target_yaw, tolerance_deg=YAW_FINE_TOLERANCE_DEG, max_steps=YAW_FINE_MAX_STEPS):
    current_yaw = get_yaw()
    if current_yaw is None:
        print("[FINE ALIGN] yaw unavailable")
        return False

    diff = angle_diff_deg(target_yaw, current_yaw)
    abs_diff = abs(diff)

    for i in range(max_steps):
        print(
            f"[FINE ALIGN] step={i+1}, "
            f"target={norm360(target_yaw):.1f}, "
            f"current={norm360(current_yaw):.1f}, diff={diff:.2f}"
        )

        if abs_diff <= tolerance_deg:
            print("[FINE ALIGN] aligned")
            return True

        old_abs_diff = abs_diff
        use_left = diff > 0
        attempted_reverse = False

        while True:
            if use_left:
                cmd_name = f"left_{YAW_FINE_STEP_DEG}"
                ok = turn_left_angle(YAW_FINE_STEP_DEG)
            else:
                cmd_name = f"right_{YAW_FINE_STEP_DEG}"
                ok = turn_right_angle(YAW_FINE_STEP_DEG)

            if not ok:
                print(f"[FINE ALIGN] turn command failed: {cmd_name}")
                stop()
                return False

            time.sleep(ESCAPE_CMD_PAUSE_SEC)

            new_yaw = get_yaw()
            if new_yaw is None:
                print("[FINE ALIGN] yaw unavailable after command")
                stop()
                return False

            yaw_delta = abs(angle_diff_deg(new_yaw, current_yaw))

            if yaw_delta < YAW_NO_CHANGE_THRESHOLD_DEG:
                print(
                    f"[FINE ALIGN] yaw not changing enough "
                    f"(delta={yaw_delta:.2f}) -> likely physical stall"
                )
                stop()
                return False

            new_diff = angle_diff_deg(target_yaw, new_yaw)
            new_abs_diff = abs(new_diff)

            print(
                f"[FINE ALIGN] cmd={cmd_name}, "
                f"new_current={norm360(new_yaw):.1f}, "
                f"new_diff={new_diff:.2f}, yaw_delta={yaw_delta:.2f}"
            )

            if new_abs_diff < old_abs_diff:
                current_yaw = new_yaw
                diff = new_diff
                abs_diff = new_abs_diff
                break

            if not attempted_reverse:
                print("[FINE ALIGN] worsened -> trying opposite once")
                use_left = not use_left
                attempted_reverse = True
                continue

            print("[FINE ALIGN] opposite also failed -> stopping fine alignment")
            stop()
            return False

    print("[FINE ALIGN] max steps reached")
    stop()
    return False


    diff = angle_diff_deg(target_yaw, current_yaw)
    abs_diff = abs(diff)

    for i in range(max_steps):
        print(
            f"[FINE ALIGN] step={i+1}, "
            f"target={norm360(target_yaw):.1f}, "
            f"current={norm360(current_yaw):.1f}, diff={diff:.2f}"
        )

        if abs_diff <= tolerance_deg:
            print("[FINE ALIGN] aligned")
            return True

        old_abs_diff = abs_diff
        use_left = diff > 0
        attempted_reverse = False

        while True:
            if use_left:
                cmd_name = f"left_{YAW_FINE_STEP_DEG}"
                turn_left_angle(YAW_FINE_STEP_DEG)
            else:
                cmd_name = f"right_{YAW_FINE_STEP_DEG}"
                turn_right_angle(YAW_FINE_STEP_DEG)

            time.sleep(ESCAPE_CMD_PAUSE_SEC)

            new_yaw = get_yaw()
            if new_yaw is None:
                print("[FINE ALIGN] yaw unavailable after command")
                return False

            new_diff = angle_diff_deg(target_yaw, new_yaw)
            new_abs_diff = abs(new_diff)

            print(
                f"[FINE ALIGN] cmd={cmd_name}, "
                f"new_current={norm360(new_yaw):.1f}, new_diff={new_diff:.2f}"
            )

            if new_abs_diff < old_abs_diff:
                current_yaw = new_yaw
                diff = new_diff
                abs_diff = new_abs_diff
                break

            if not attempted_reverse:
                print("[FINE ALIGN] worsened -> trying opposite once")
                use_left = not use_left
                attempted_reverse = True
                continue

            print("[FINE ALIGN] opposite also failed -> stopping fine alignment")
            stop()
            return False

    print("[FINE ALIGN] max steps reached")
    stop()
    return False

def turn_for_big_scan(search_dir):
    """
    Slower, more stable turn handling for big direction scans.
    """

    if search_dir == "left":
        ok = turn_left_angle(BIG_SCAN_ANGLE_DEG)
    else:
        ok = turn_right_angle(BIG_SCAN_ANGLE_DEG)

    # allow robot + IMU to settle
    time.sleep(0.25)

    stop()

    time.sleep(0.20)

    return ok

def big_direction_scan(last_turn: str) -> str:
    """
    Strong recovery:
    - reverse several times
    - scan using larger yaw steps
    - detect command failure or physical stall
    - align to the yaw with maximum reliable distance
    """
    print("[NAV] Performing big direction scan")

    stop()
    time.sleep(0.05)

    initial = measure_distance()

    if initial >= OPEN_SPACE_FOUND_CM:
        print("[BIG SCAN] already open space -> skipping big scan")
        return last_turn

    reverse_steps(n=3, duration=0.40)
    time.sleep(0.10)

    # small forward/back wiggle to free wheels from carpet edge
    send_cmd("forward_short")
    stop()
    time.sleep(0.10)

    reverse_steps(n=1, duration=0.40)
    time.sleep(0.05)

    search_dir = get_next_escape_direction()

    records = []
    best_distance = -1
    best_yaw = None
    best_step = 0

    previous_yaw = get_yaw()
    if previous_yaw is None:
        previous_yaw = 0.0

    yaw_no_change_count = 0

    for step in range(1, BIG_SCAN_STEPS + 1):
        ok = turn_for_big_scan(search_dir)

        if not ok:
            print("[BIG SCAN] turn command failed -> aborting big scan")
            stop()
            return last_turn

        d = measure_distance()
        if d < 0:
            d = 0

        yaw = get_yaw()
        if yaw is None:
            print("[BIG SCAN] yaw unavailable -> aborting big scan")
            stop()
            return last_turn

        yaw_delta = abs(angle_diff_deg(yaw, previous_yaw))

        print(
            f"[BIG SCAN] step={step}, dir={search_dir}, "
            f"distance={d}, yaw={yaw:.1f}, yaw_delta={yaw_delta:.2f}"
        )

        if yaw_delta < YAW_NO_CHANGE_THRESHOLD_DEG:
            yaw_no_change_count += 1
            print(
                f"[BIG SCAN] yaw not changing sufficiently "
                f"(delta={yaw_delta:.2f}) count={yaw_no_change_count}"
            )
        else:
            yaw_no_change_count = 0

        if yaw_no_change_count >= YAW_NO_CHANGE_LIMIT:
            print("[BIG SCAN] likely carpet/mechanical stall -> hard reverse")

            reverse_steps(n=2, duration=0.45)
            time.sleep(0.15)

            yaw_after_reverse = get_yaw()
            if yaw_after_reverse is None:
                print("[BIG SCAN] yaw unavailable after hard reverse")
                stop()
                return last_turn

            reverse_delta = abs(angle_diff_deg(yaw_after_reverse, yaw))
            print(
                f"[BIG SCAN] yaw after reverse={yaw_after_reverse:.1f}, "
                f"reverse_delta={reverse_delta:.2f}"
            )

            if reverse_delta < YAW_NO_CHANGE_THRESHOLD_DEG:
                print("[BIG SCAN] still stalled after hard reverse -> aborting")
                stop()
                return last_turn

            yaw_no_change_count = 0
            previous_yaw = yaw_after_reverse
            continue

        records.append((d, yaw))

        if d > best_distance:
            best_distance = d
            best_yaw = yaw
            best_step = step

        if d >= OPEN_SPACE_FOUND_CM:
            print("[BIG SCAN] open direction found -> stopping scan early")
            break

        previous_yaw = yaw

    if best_yaw is None:
        print("[BIG SCAN] no reliable yaw/distance record -> aborting")
        stop()
        return last_turn

    print(
        f"[BIG SCAN] best_distance={best_distance}, "
        f"best_step={best_step}, best_yaw={best_yaw:.1f}"
    )

    if best_distance >= OPEN_SPACE_FOUND_CM:
        print("[BIG SCAN] open space found -> no yaw refinement needed")
        return search_dir

    align_ok = align_to_yaw(
        best_yaw,
        tolerance_deg=YAW_ALIGN_TOLERANCE_DEG,
        max_steps=YAW_ALIGN_MAX_STEPS_BIG,
    )

    if not align_ok:
        print("[BIG SCAN] alignment to best yaw failed")
        stop()
        return last_turn

    if REFINE_AFTER_YAW_ALIGN:
        verified = refine_heading_by_distance()
    else:
        verified = measure_distance()

    print(f"[BIG SCAN] aligned/refined distance={verified}")

    return search_dir


    stop()
    time.sleep(0.05)

    initial = measure_distance()

    if initial >= OPEN_SPACE_FOUND_CM:
        print("[BIG SCAN] already open space -> skipping big scan")
        return last_turn

    reverse_steps(n=3, duration=0.40)
    time.sleep(0.10)

    # small forward/back wiggle to free wheels from carpet edge
    send_cmd("forward_short")
    stop()
    time.sleep(0.10)

    reverse_steps(n=1, duration=0.40)
    time.sleep(0.05)

    search_dir = get_next_escape_direction()

    if search_dir == "left":
        step_fn = lambda: turn_left_angle(BIG_SCAN_ANGLE_DEG)
    else:
        step_fn = lambda: turn_right_angle(BIG_SCAN_ANGLE_DEG)

    records = []  # each item: (distance, yaw)

    best_distance = -1
    best_yaw = None
    best_step = 0

    for step in range(1, BIG_SCAN_STEPS + 1):
        ok = turn_for_big_scan(search_dir)
        d = measure_distance()
        if d < 0:
            d = 0

        yaw = get_yaw()
        if yaw is None:
            yaw = 0.0

        records.append((d, yaw))

        print(
            f"[BIG SCAN] step={step}, dir={search_dir}, "
            f"distance={d}, yaw={yaw:.1f}"
        )

        # Treat 300 as valid open direction, but avoid over-rotating forever.
        if d > best_distance:
            best_distance = d
            best_yaw = yaw
            best_step = step

        # Early stop if clear open space found
        if d >= OPEN_SPACE_FOUND_CM:
            print("[BIG SCAN] open direction found -> stopping scan early")
            break

    print(
        f"[BIG SCAN] best_distance={best_distance}, "
        f"best_step={best_step}, best_yaw={best_yaw:.1f}"
    )

    if best_distance >= OPEN_SPACE_FOUND_CM:
        print("[BIG SCAN] open space found -> no yaw refinement needed")
        return search_dir

    if best_yaw is not None:
        align_to_yaw(
            best_yaw,
            tolerance_deg=YAW_ALIGN_TOLERANCE_DEG,
            max_steps=YAW_ALIGN_MAX_STEPS_BIG,
        )

        if REFINE_AFTER_YAW_ALIGN:
            verified = refine_heading_by_distance()
        else:
            verified = measure_distance()

        print(f"[BIG SCAN] aligned/refined distance={verified}")

    return search_dir


def recover_communications(camera: CameraClient) -> bool:
    """Try to recover after repeated ESP32 HTTP timeouts.

    Returns True when the robot is responsive again. Returns False when the
    user should reset/restart and the patrol should stop/pause.
    """
    global _http_failure_count

    if _http_failure_count < MAX_CONSECUTIVE_HTTP_FAILURES:
        time.sleep(0.6)
        return True

    print("[COMM] repeated HTTP failures may indicate WiFi/server issues OR the robot being physically stuck")
    print(f"[COMM] { _http_failure_count } consecutive HTTP failures -> releasing camera and waiting")
    camera.release()
    time.sleep(COMM_RECOVERY_WAIT_SEC)

    print("[COMM] running short health check after communication failure")
    ok = startup_health_check(camera, max_attempts=2)
    if ok:
        print("[COMM] communication recovered")
        _http_failure_count = 0
        return True

    print("[COMM] still not responsive. Entering safe pause. Reset ESP32 if needed.")
    return pause_until_resume("communication failure")

def patrol():
    detector = PersonDetector()
    camera = CameraClient(STREAM_URL)

    print("Starting patrol_photo_v3_multiview: fast IMU patrol + frequent photos + safer multiview scan...")
    print("[CTRL] Keyboard controls at checkpoints: p=pause, r=resume, q=quit")

    send_cmd("reset_global_yaw", retries=1)
    time.sleep(0.2)

    if not startup_health_check(camera):
        print("[STARTUP] Please reset/power-cycle the ESP32 and try again.")
        return

    last_turn = "right"
    patrol_points = 0
    stuck_counter = 0
    last_scan_point = -DETECTION_EVERY_N_POINTS

    recent_front_distances = []
    recent_blocked_events = 0
    x_cm = 0.0
    y_cm = 0.0
    max_x_cm = 0.0
    min_x_cm = 0.0
    max_y_cm = 0.0
    min_y_cm = 0.0
    exit_search_mode = False
    last_scan_x_span = 0
    last_scan_y_span = 0
    force_exit_scan = False
    final_scan_pending = False
    scan_count = 0
    total_forward_distance_cm = 0.0

    try:
        pan_center()
        tilt_center()
        stop()
        time.sleep(0.4)

        step = 0
        while True:
            step += 1
            print(f"\n[STEP {step}]")

            if not handle_keyboard_checkpoint(f"before step {step}"):
                break

            before_dist = measure_distance()
            print(f"[PATROL] front distance = {before_dist} cm")

            if before_dist == -1:
                print("[PATROL] Distance unavailable -> communication recovery path")
                if not recover_communications(camera):
                    break
                continue

            # immediate blocked case
            if before_dist < OBSTACLE_THRESHOLD:
                recent_blocked_events += 1
                last_turn = smart_escape(last_turn)
                after_dist = measure_distance()
                print(f"[PATROL] front distance after move = {after_dist} cm")
            else:
                handled_by_mask_nav = False

                if (
                        USE_MASK_NAVIGATION
                        and (exit_search_mode or not MASK_NAV_AFTER_EXIT_MODE_ONLY)
                ):
                    handled_by_mask_nav = mask_based_navigation_step(camera, detector, before_dist)

                if handled_by_mask_nav:
                    move_cmd = "mask_nav"
                    before_state = get_state()
                    after_dist = measure_distance()
                    after_state = get_state()
                else:
                    move_cmd = choose_forward_command(before_dist)
                    print(f"[PATROL] Executing {move_cmd}")

                    before_state = get_state()

                    execute_forward(move_cmd)
                    stop()
                    time.sleep(0.01)

                    after_dist = measure_distance()
                    after_state = get_state()

                print(f"[PATROL] front distance after move = {after_dist} cm")

                if move_cmd in FORWARD_DISTANCE_CM:
                    total_forward_distance_cm += FORWARD_DISTANCE_CM[move_cmd]
                    print(f"[SURVEY] total_forward_distance_cm={total_forward_distance_cm:.1f}")

                    x_cm, y_cm, max_x_cm, min_x_cm, max_y_cm, min_y_cm = update_odometry_from_forward(
                        move_cmd,
                        before_state,
                        after_state,
                        x_cm,
                        y_cm,
                        max_x_cm,
                        min_x_cm,
                        max_y_cm,
                        min_y_cm,
                    )
                else:
                    print("[ODOM] mask_nav action -> odometry update skipped")

                x_span = max_x_cm - min_x_cm
                y_span = max_y_cm - min_y_cm

                if (
                        not exit_search_mode
                        and x_span >= X_EXIT_SEARCH_THRESHOLD_CM
                        and y_span >= Y_EXIT_SEARCH_THRESHOLD_CM
                        and patrol_points >= MIN_PATROL_POINTS_BEFORE_SURVEY_STOP
                        and total_forward_distance_cm >= MIN_TOTAL_DISTANCE_CM_BEFORE_STOP
                ):
                    exit_search_mode = True
                    final_scan_pending = True
                    force_exit_scan = True
                    print(
                        "[SURVEY] X/Y span reached with enough patrol coverage "
                        "-> forcing final scan before stopping "
                        f"(points={patrol_points}, scans={scan_count}, "
                        f"distance={total_forward_distance_cm:.1f}, "
                        f"x_span={x_span:.1f}, y_span={y_span:.1f})"
                    )
            # update recent distance history
            if after_dist > 0:
                recent_front_distances.append(after_dist)
                if len(recent_front_distances) > LOOP_HISTORY_SIZE:
                    recent_front_distances.pop(0)

            # progress-based stuck logic
            stuck_counter = update_stuck_counter(before_dist, after_dist, stuck_counter)

            if stuck_counter >= STUCK_COUNT_THRESHOLD:
                print("[NAV] Low progress -> invoking recovery")

                recent_blocked_events += 1

                if before_dist < OPEN_SPACE_IGNORE_CM or after_dist < OPEN_SPACE_IGNORE_CM:
                    print("[NAV] Open-space no-progress -> big direction scan")
                    last_turn = big_direction_scan(last_turn)
                else:
                    last_turn = smart_escape(last_turn)

                stuck_counter = 0

                after_dist = measure_distance()
                print(f"[PATROL] front distance after recovery = {after_dist} cm")

                if after_dist > 0:
                    recent_front_distances.append(after_dist)
                    if len(recent_front_distances) > LOOP_HISTORY_SIZE:
                        recent_front_distances.pop(0)

            # if robot escaped into more open space, relax blocked counter
            if after_dist >= 80:
                recent_blocked_events = 0

            # detect loop and do stronger recovery
            if detect_local_loop(recent_front_distances, recent_blocked_events):
                print("[NAV] Local loop detected -> big scan recovery")

                last_turn = big_direction_scan(last_turn)

                recent_front_distances.clear()
                recent_blocked_events = 0
                stuck_counter = 0

                after_dist = measure_distance()
                print(f"[NAV] distance after big scan recovery = {after_dist}")
                continue

            patrol_points += 1

            # Optional forward-facing photo collection.
            # In this scan-only version, PHOTO_EVERY_N_PATROL_POINTS is 0 by default,
            # so the ESP32 camera is used only during the multiview scan.
            if PHOTO_EVERY_N_PATROL_POINTS > 0 and patrol_points % PHOTO_EVERY_N_PATROL_POINTS == 0:
                save_forward_patrol_photo(camera, step, patrol_points, x_cm, y_cm, after_dist)
                if not handle_keyboard_checkpoint(f"after patrol photo at step {step}"):
                    break

            # run detection scan only in open space and with enough patrol gap
            x_span = max_x_cm - min_x_cm
            y_span = max_y_cm - min_y_cm

            area_expanded = (
                    (x_span - last_scan_x_span) >= MIN_SCAN_AREA_EXPANSION_CM
                    or
                    (y_span - last_scan_y_span) >= MIN_SCAN_AREA_EXPANSION_CM
            )

            enough_point_gap = patrol_points - last_scan_point >= MIN_SCAN_POINT_GAP

            fallback_trigger = patrol_points - last_scan_point >= DETECTION_EVERY_N_POINTS

            should_run_yolo_scan = force_exit_scan or (
                    after_dist >= OPEN_SPACE_SCAN_MIN_CM
                    and after_dist < 280
                    and before_dist >= OPEN_SPACE_SCAN_MIN_CM
                    and enough_point_gap
                    and (area_expanded or fallback_trigger)
            )

            if should_run_yolo_scan:
                force_exit_scan = False

                if not ENABLE_YOLO_SCAN:
                    print("[SCAN] YOLO/multiview scan skipped because ENABLE_YOLO_SCAN=False")
                    final_scan_pending = False
                    last_scan_point = patrol_points
                    last_scan_x_span = x_span
                    last_scan_y_span = y_span
                    continue

                print(f"[SCAN] Triggered in open space: before={before_dist}, after={after_dist} cm")

                stop()
                time.sleep(0.30)

                detected = scan_and_detect(camera, detector, save_segmented=exit_search_mode)
                scan_count += 1
                print(f"[SURVEY] scan_count={scan_count}")

                if detected:
                    print("[PATROL] Detection occurred during this scan")

                last_scan_point = patrol_points
                last_scan_x_span = x_span
                last_scan_y_span = y_span
                time.sleep(0.25)

                if (
                        final_scan_pending
                        and scan_count >= MIN_SCANS_BEFORE_SURVEY_STOP
                ):
                    if STOP_AFTER_FINAL_SURVEY_SCAN:
                        print("[SURVEY] final scan completed -> stopping patrol")
                        stop()
                        return

                    print("[SURVEY] final scan completed -> continuing patrol")
                    final_scan_pending = False
                    force_exit_scan = False

    except KeyboardInterrupt:
        print("\n[PATROL] Keyboard interrupt received.")

    finally:
        stop()
        camera.release()
        print("[PATROL] Stopped safely.")

if __name__ == "__main__":
    patrol()