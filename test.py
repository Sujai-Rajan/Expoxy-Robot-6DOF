from xarm.wrapper import XArmAPI
import time


XARM_IP = "192.168.1.196"
GRIPPER_INPUT_1_CO = 0  # Control box CO0 -> gripper input pin 6
GRIPPER_INPUT_2_CO = 1  # Control box CO1 -> gripper input pin 7
PULSE_SECONDS = 1.0
PAUSE_SECONDS = 1.0


def set_co(arm, channel, value):
    code = arm.set_cgpio_digital(channel, int(value), delay_sec=0, sync=True)
    print(f"set_cgpio_digital({channel}, {int(value)}) -> {code}")
    return code


def set_gripper_lines(arm, first_value, second_value, hold_seconds):
    set_co(arm, GRIPPER_INPUT_1_CO, first_value)
    set_co(arm, GRIPPER_INPUT_2_CO, second_value)
    time.sleep(hold_seconds)


arm = XArmAPI(XARM_IP)
arm.connect()

try:
    arm.clean_warn()
    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.5)

    # Reset both gripper control inputs low before starting.
    set_gripper_lines(arm, 0, 0, 0.5)

    # Close once: input1 high, input2 low.
    print("closing gripper")
    set_gripper_lines(arm, 1, 0, PULSE_SECONDS)
    set_gripper_lines(arm, 0, 0, 0.2)

    time.sleep(PAUSE_SECONDS)

    # Open once: input1 low, input2 high.
    print("opening gripper")
    set_gripper_lines(arm, 0, 1, PULSE_SECONDS)
    set_gripper_lines(arm, 0, 0, 0.2)
finally:
    arm.disconnect()
