def map1(input_val, power=2.0, deadzone=0.05):
    """
    input_val: -1.0 to 1.0
    power: Curve steepness (2.0 = quadratic, 3.0 = cubic)
    deadzone: Input range to ignore (e.g., 0.05 ignores -0.05 to 0.05)
    """
    # 1. Immediate exit for the deadzone
    if abs(input_val) < deadzone:
        return 0

    # 2. Rescale input to start from 0 AFTER the deadzone
    # This prevents a 'jump' from 0 to a high value immediately
    scaled_input = (abs(input_val) - deadzone) / (1.0 - deadzone)

    # 3. Apply the Exponential Curve
    curve_val = scaled_input ** power
    output = 20000 * curve_val

    return int(output)

# --- Quick Test ---
# print(motor_mapper_with_deadzone(0.02))  # Returns 0 (Inside deadzone)
# print(motor_mapper_with_deadzone(0.06))  # Returns ~1 (Very slow start)
# print(motor_mapper_with_deadzone(-0.06)) # Returns ~10001 (Very slow reverse)