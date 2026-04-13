import time
import numpy as np

from xarm.wrapper import XArmAPI

# Optional ML dependency:
# pip install scikit-learn xarm-python-sdk
from sklearn.neural_network import MLPRegressor

ROBOT_IP = "192.168.1.196"

# Keep these conservative and safe
SAFE_Z = 180.0
SAFE_ROLL = -180.0
SAFE_PITCH = 0.0
SAFE_YAW = 0.0
MOVE_SPEED = 40
STEP_LIMIT_MM = 10.0
GOAL_TOL_MM = 2.0
MAX_ITERS = 20

# Choose a very safe workspace for testing
X_MIN, X_MAX = 250.0, 450.0
Y_MIN, Y_MAX = -150.0, 150.0


def clamp(val, low, high):
    return max(low, min(high, val))


def build_training_data(n=2000):
    """
    Synthetic training set:
    inputs  = [x, y, goal_x, goal_y]
    outputs = [dx, dy]
    Policy is intentionally simple and safe:
    move 20% toward goal, capped per step.
    """
    rng = np.random.default_rng(42)
    X = []
    y = []

    for _ in range(n):
        x = rng.uniform(X_MIN, X_MAX)
        yy = rng.uniform(Y_MIN, Y_MAX)
        gx = rng.uniform(X_MIN, X_MAX)
        gy = rng.uniform(Y_MIN, Y_MAX)

        dx = 0.2 * (gx - x)
        dy = 0.2 * (gy - yy)

        dx = np.clip(dx, -STEP_LIMIT_MM, STEP_LIMIT_MM)
        dy = np.clip(dy, -STEP_LIMIT_MM, STEP_LIMIT_MM)

        X.append([x, yy, gx, gy])
        y.append([dx, dy])

    return np.array(X), np.array(y)


def train_model():
    X, y = build_training_data()
    model = MLPRegressor(
        hidden_layer_sizes=(32, 32),
        activation="relu",
        max_iter=500,
        random_state=0,
    )
    model.fit(X, y)
    return model


def get_xy_position(arm):
    code, pose = arm.get_position(is_radian=False)
    if code != 0:
        raise RuntimeError(f"get_position failed, code={code}")
    # pose = [x, y, z, roll, pitch, yaw]
    return float(pose[0]), float(pose[1]), pose


def move_to_pose(arm, x, y):
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
        is_radian=False,
        wait=True,
    )
    if code != 0:
        raise RuntimeError(f"set_position failed, code={code}")


def main():
    model = train_model()

    arm = XArmAPI(ROBOT_IP, is_radian=False)
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(1)

    # Move to a known safe start first
    move_to_pose(arm, 320.0, 0.0)

    # Example target inside safe box
    goal_x = 400.0
    goal_y = 80.0

    for i in range(MAX_ITERS):
        cur_x, cur_y, _ = get_xy_position(arm)
        err = np.hypot(goal_x - cur_x, goal_y - cur_y)
        print(f"iter={i}, current=({cur_x:.1f}, {cur_y:.1f}), error={err:.2f} mm")

        if err < GOAL_TOL_MM:
            print("Goal reached.")
            break

        pred = model.predict([[cur_x, cur_y, goal_x, goal_y]])[0]
        dx = float(np.clip(pred[0], -STEP_LIMIT_MM, STEP_LIMIT_MM))
        dy = float(np.clip(pred[1], -STEP_LIMIT_MM, STEP_LIMIT_MM))

        next_x = cur_x + dx
        next_y = cur_y + dy

        move_to_pose(arm, next_x, next_y)

    # Return to a safe position
    move_to_pose(arm, 320.0, 0.0)
    arm.disconnect()


if __name__ == "__main__":
    main()