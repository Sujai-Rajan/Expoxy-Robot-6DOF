import cv2
import time
import numpy as np
from xarm.wrapper import XArmAPI

# =========================
# USER SETTINGS
# =========================
ROBOT_IP = "192.168.1.196"
CAM_INDEX = 0

# Safe working pose/orientation
SAFE_Z = 180.0
SAFE_ROLL = -180.0
SAFE_PITCH = 0.0
SAFE_YAW = 0.0

MOVE_SPEED = 40
MOVE_ACC = 500

# Motion increments based on distance
STEP_X_MM_LARGE = 5.0      # Large steps when far
STEP_Y_MM_LARGE = 5.0
STEP_X_MM_SMALL = 1.5      # Small steps when close
STEP_Y_MM_SMALL = 1.5
DISTANCE_THRESHOLD = 100   # pixels - switch from large to small steps

# Flip these if robot moves the wrong way
ROBOT_X_SIGN = -1   # try 1, if wrong set to -1
ROBOT_Y_SIGN = -1   # try 1, if wrong set to -1

# Green detection HSV ranges
LOWER_GREEN = np.array([40, 50, 50])
UPPER_GREEN = np.array([80, 255, 255])
MIN_CONTOUR_AREA = 100

# Image center acceptance box (pixels)
CENTER_BOX_W = 80
CENTER_BOX_H = 80

# Workspace clamp
X_MIN, X_MAX = 250.0, 500.0
Y_MIN, Y_MAX = -200.0, 200.0

SHOW_DEBUG = True


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def init_robot(ip: str) -> XArmAPI:
    arm = XArmAPI(ip, is_radian=False)
    arm.motion_enable(enable=True)
    arm.clean_error()
    arm.clean_warn()
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(1)
    return arm


def get_pose_xy(arm: XArmAPI):
    code, pose = arm.get_position(is_radian=False)
    if code != 0:
        raise RuntimeError(f"get_position failed, code={code}")
    # pose = [x, y, z, roll, pitch, yaw]
    return float(pose[0]), float(pose[1]), pose


def move_xy(arm: XArmAPI, x: float, y: float):
    x = clamp(x, X_MIN, X_MAX)
    y = clamp(y, Y_MIN, Y_MAX)
    code = arm.set_position(
        x=x,
        y=y,
        z=SAFE_Z,
        roll=SAFE_ROLL,
        pitch=SAFE_PITCH,
        yaw=SAFE_YAW,
        speed=MOVE_SPEED,
        mvacc=MOVE_ACC,
        is_radian=False,
        wait=True,
    )
    if code != 0:
        raise RuntimeError(f"set_position failed, code={code}")


def detect_green_target(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

    # Clean noise
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
    cx = x + w // 2
    cy = y + h // 2

    return {
        "bbox": (x, y, w, h),
        "center": (cx, cy),
        "area": area,
        "contour": largest,
    }, mask


def calculate_shortest_path(err_x, err_y, step_x, step_y):
    """
    Calculate movement that reduces euclidean distance most efficiently.
    Uses proportional movement based on error magnitude.
    """
    # Normalize errors to unit direction
    error_magnitude = np.sqrt(err_x**2 + err_y**2)
    
    if error_magnitude == 0:
        return 0, 0
    
    # Calculate unit vector towards center
    unit_x = err_x / error_magnitude
    unit_y = err_y / error_magnitude
    
    # Scale steps proportionally
    move_x = unit_x * step_x
    move_y = unit_y * step_y
    
    return move_x, move_y


def draw_coordinate_system(frame, img_cx, img_cy):
    """
    Draw X and Y axes with quadrants on the frame for debugging.
    """
    h, w = frame.shape[:2]
    
    # Draw axes
    axis_length = 100
    line_thickness = 2
    
    # X-axis (red) - horizontal
    cv2.line(frame, (img_cx - axis_length, img_cy), (img_cx + axis_length, img_cy), (0, 0, 255), line_thickness)
    cv2.putText(frame, "+X", (img_cx + axis_length + 10, img_cy - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(frame, "-X", (img_cx - axis_length - 40, img_cy - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    
    # Y-axis (green) - vertical
    cv2.line(frame, (img_cx, img_cy - axis_length), (img_cx, img_cy + axis_length), (0, 255, 0), line_thickness)
    cv2.putText(frame, "+Y", (img_cx + 5, img_cy - axis_length - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(frame, "-Y", (img_cx + 5, img_cy + axis_length + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # Draw quadrant labels
    quad_offset = 60
    cv2.putText(frame, "Q1 (+X,+Y)", (img_cx + quad_offset, img_cy - quad_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    cv2.putText(frame, "Q2 (-X,+Y)", (img_cx - quad_offset - 80, img_cy - quad_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    cv2.putText(frame, "Q3 (-X,-Y)", (img_cx - quad_offset - 80, img_cy + quad_offset + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    cv2.putText(frame, "Q4 (+X,-Y)", (img_cx + quad_offset, img_cy + quad_offset + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    
    # Draw center dot
    cv2.circle(frame, (img_cx, img_cy), 6, (255, 255, 255), -1)
    
    return frame


def main():
    arm = init_robot(ROBOT_IP)
    cap = cv2.VideoCapture(CAM_INDEX)

    if not cap.isOpened():
        raise RuntimeError("Could not open camera")

    # Move robot to a known start pose first
    move_xy(arm, 350.0, 0.0)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Camera frame failed")
                continue

            # Resize frame to 400x400
            frame = cv2.resize(frame, (400, 400))

            h, w = frame.shape[:2]
            img_cx = w // 2
            img_cy = h // 2

            box_x1 = img_cx - CENTER_BOX_W // 2
            box_y1 = img_cy - CENTER_BOX_H // 2
            box_x2 = img_cx + CENTER_BOX_W // 2
            box_y2 = img_cy + CENTER_BOX_H // 2

            target, mask = detect_green_target(frame)

            status_text = "Searching..."
            inside_box = False

            if target is not None:
                tx, ty = target["center"]
                x, y, bw, bh = target["bbox"]

                err_x = tx - img_cx
                err_y = ty - img_cy

                # Calculate euclidean distance
                distance = np.sqrt(err_x**2 + err_y**2)

                inside_box = (box_x1 <= tx <= box_x2) and (box_y1 <= ty <= box_y2)

                if inside_box:
                    status_text = "Target centered"
                else:
                    cur_x, cur_y, _ = get_pose_xy(arm)

                    # Choose step size based on distance
                    if distance > DISTANCE_THRESHOLD:
                        step_x = STEP_X_MM_LARGE
                        step_y = STEP_Y_MM_LARGE
                    else:
                        step_x = STEP_X_MM_SMALL
                        step_y = STEP_Y_MM_SMALL

                    # Calculate shortest path using proportional movement
                    move_x, move_y = calculate_shortest_path(err_x, err_y, step_x, step_y)

                    # Apply sign corrections and move robot
                    next_x = cur_x - ROBOT_X_SIGN * move_x
                    next_y = cur_y - ROBOT_Y_SIGN * move_y

                    status_text = f"Dist={distance:.1f} err_x={err_x:.1f}, err_y={err_y:.1f}"
                    move_xy(arm, next_x, next_y)

                if SHOW_DEBUG:
                    cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                    cv2.circle(frame, (tx, ty), 5, (0, 0, 255), -1)
                    cv2.putText(frame, f"Target ({tx},{ty})", (x, max(y - 10, 20)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Draw center box
            cv2.rectangle(frame, (box_x1, box_y1), (box_x2, box_y2), (255, 0, 0), 2)
            cv2.circle(frame, (img_cx, img_cy), 4, (255, 0, 0), -1)

            # Draw coordinate system with quadrants
            frame = draw_coordinate_system(frame, img_cx, img_cy)

            cv2.putText(frame, status_text, (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            if SHOW_DEBUG:
                cv2.imshow("Robot Camera", frame)
                cv2.resizeWindow("Robot Camera", 400, 400)
                cv2.imshow("Green Mask", mask)
                cv2.resizeWindow("Green Mask", 400, 400)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC to quit
                break

            # If centered, hold for operator
            if inside_box:
                print("Green target is inside center box.")
                time.sleep(0.2)

    finally:
        cap.release()
        cv2.destroyAllWindows()
        arm.disconnect()


if __name__ == "__main__":
    main()