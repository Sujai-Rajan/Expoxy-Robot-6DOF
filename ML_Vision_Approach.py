import cv2
import time
import math
import numpy as np
from collections import deque
from xarm.wrapper import XArmAPI

# =========================
# USER SETTINGS
# =========================
ROBOT_IP = "192.168.1.196"
CAM_INDEX = 0

# Safe fixed pose
SAFE_Z = 180.0
SAFE_ROLL = -180.0
SAFE_PITCH = 0.0
SAFE_YAW = 0.0

# Safe start XY
START_X = 350.0
START_Y = 0.0

# Workspace clamp
X_MIN, X_MAX = 250.0, 500.0
Y_MIN, Y_MAX = -200.0, 200.0

# Camera-to-robot mapping
# Tune these from a quick calibration:
# Move robot +10 mm in X and measure image target pixel shift in X.
MM_PER_PIXEL_X = 0.10
MM_PER_PIXEL_Y = 0.10

# Flip these if robot moves opposite of what you expect
ROBOT_X_SIGN = 1.0
ROBOT_Y_SIGN = 1.0

# Servo gains / limits
KP = 0.55                     # proportional gain on image error -> robot correction
MAX_STEP_MM = 8.0             # max XY command per update
MIN_STEP_MM = 0.15            # do not send tiny useless updates
CONTROL_DT = 0.06             # seconds between motion updates

# Green detection in HSV
# Start with these and tune with your lighting
LOWER_GREEN = np.array([35, 50, 40], dtype=np.uint8)
UPPER_GREEN = np.array([95, 255, 255], dtype=np.uint8)
MIN_CONTOUR_AREA = 250

# Center acceptance box
CENTER_BOX_W = 80
CENTER_BOX_H = 80

# Lost target handling
LOST_FRAME_LIMIT = 12         # continue briefly using last command
COMMAND_SMOOTHING = 4         # moving average over last few commands

SHOW_DEBUG = True


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def init_robot(ip: str) -> XArmAPI:
    arm = XArmAPI(ip, is_radian=False)
    arm.connect()
    arm.motion_enable(enable=True)
    arm.clean_error()
    arm.clean_warn()
    arm.set_mode(7)   # Cartesian online trajectory planning
    arm.set_state(0)
    time.sleep(1)
    return arm


def get_pose(arm: XArmAPI):
    code, pose = arm.get_position(is_radian=False)
    if code != 0:
        raise RuntimeError(f"get_position failed, code={code}")
    return pose  # [x, y, z, roll, pitch, yaw]


def set_pose_online(arm: XArmAPI, x: float, y: float):
    x = clamp(x, X_MIN, X_MAX)
    y = clamp(y, Y_MIN, Y_MAX)

    code = arm.set_position(
        x=x,
        y=y,
        z=SAFE_Z,
        roll=SAFE_ROLL,
        pitch=SAFE_PITCH,
        yaw=SAFE_YAW,
        speed=200,
        mvacc=2000,
        is_radian=False,
        wait=False,   # critical for online updates
    )
    if code != 0:
        print(f"Warning: set_position returned code={code}")


def detect_green_target(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < MIN_CONTOUR_AREA:
        return None, mask

    x, y, w, h = cv2.boundingRect(largest)
    cx = x + w / 2.0
    cy = y + h / 2.0

    return {
        "bbox": (x, y, w, h),
        "center": (cx, cy),
        "area": area,
        "contour": largest,
    }, mask


def vector_clip(dx, dy, max_norm):
    norm = math.hypot(dx, dy)
    if norm <= max_norm or norm == 0:
        return dx, dy
    scale = max_norm / norm
    return dx * scale, dy * scale


def main():
    arm = init_robot(ROBOT_IP)
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera")

    # Go to safe start
    set_pose_online(arm, START_X, START_Y)
    time.sleep(1.5)

    last_cmd_dx = 0.0
    last_cmd_dy = 0.0
    lost_count = 0
    cmd_hist = deque(maxlen=COMMAND_SMOOTHING)
    last_update = 0.0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            h, w = frame.shape[:2]
            img_cx = w / 2.0
            img_cy = h / 2.0

            box_x1 = int(img_cx - CENTER_BOX_W / 2)
            box_y1 = int(img_cy - CENTER_BOX_H / 2)
            box_x2 = int(img_cx + CENTER_BOX_W / 2)
            box_y2 = int(img_cy + CENTER_BOX_H / 2)

            target, mask = detect_green_target(frame)
            pose = get_pose(arm)
            cur_x, cur_y = float(pose[0]), float(pose[1])

            status_text = "Searching"
            inside_box = False
            cmd_dx_mm = 0.0
            cmd_dy_mm = 0.0

            if target is not None:
                lost_count = 0

                tx, ty = target["center"]
                x, y, bw, bh = target["bbox"]

                err_px_x = tx - img_cx
                err_px_y = ty - img_cy

                inside_box = (
                    box_x1 <= tx <= box_x2 and
                    box_y1 <= ty <= box_y2
                )

                if inside_box:
                    status_text = "Target centered"
                    cmd_dx_mm = 0.0
                    cmd_dy_mm = 0.0
                else:
                    # Direct shortest-path correction in image space
                    raw_dx_mm = -ROBOT_X_SIGN * KP * err_px_x * MM_PER_PIXEL_X
                    raw_dy_mm = -ROBOT_Y_SIGN * KP * err_px_y * MM_PER_PIXEL_Y

                    # Move along the direct error vector, clipped by max step
                    cmd_dx_mm, cmd_dy_mm = vector_clip(raw_dx_mm, raw_dy_mm, MAX_STEP_MM)

                    # Ignore tiny noise
                    if math.hypot(cmd_dx_mm, cmd_dy_mm) < MIN_STEP_MM:
                        cmd_dx_mm = 0.0
                        cmd_dy_mm = 0.0

                    status_text = f"Track ex={err_px_x:.1f}px ey={err_px_y:.1f}px"

                if SHOW_DEBUG:
                    cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                    cv2.circle(frame, (int(tx), int(ty)), 5, (0, 0, 255), -1)
                    cv2.putText(
                        frame,
                        f"target=({int(tx)},{int(ty)})",
                        (x, max(y - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2,
                    )
            else:
                lost_count += 1
                if lost_count <= LOST_FRAME_LIMIT:
                    # Keep moving briefly in last known direction to reacquire
                    cmd_dx_mm = last_cmd_dx
                    cmd_dy_mm = last_cmd_dy
                    status_text = f"Target lost, continuing {lost_count}/{LOST_FRAME_LIMIT}"
                else:
                    cmd_dx_mm = 0.0
                    cmd_dy_mm = 0.0
                    status_text = "Target lost, holding"

            # Smooth command to reduce jitter
            cmd_hist.append((cmd_dx_mm, cmd_dy_mm))
            avg_dx = float(np.mean([c[0] for c in cmd_hist]))
            avg_dy = float(np.mean([c[1] for c in cmd_hist]))

            # Send online motion updates at fixed rate
            now = time.time()
            if now - last_update >= CONTROL_DT:
                next_x = cur_x + avg_dx
                next_y = cur_y + avg_dy
                set_pose_online(arm, next_x, next_y)
                last_update = now
                last_cmd_dx = avg_dx
                last_cmd_dy = avg_dy

            # Draw center box
            cv2.rectangle(frame, (box_x1, box_y1), (box_x2, box_y2), (255, 0, 0), 2)
            cv2.circle(frame, (int(img_cx), int(img_cy)), 4, (255, 0, 0), -1)

            cv2.putText(
                frame,
                status_text,
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 255),
                2,
            )

            cv2.putText(
                frame,
                f"cmd_dx={avg_dx:.2f} mm cmd_dy={avg_dy:.2f} mm",
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
            )

            if SHOW_DEBUG:
                cv2.imshow("Green Target Servo", frame)
                cv2.imshow("Mask", mask if target is not None else np.zeros((h, w), dtype=np.uint8))

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break

            if inside_box:
                # Hold centered and stop commanding motion
                time.sleep(0.05)

    finally:
        cap.release()
        cv2.destroyAllWindows()
        arm.disconnect()


if __name__ == "__main__":
    main()