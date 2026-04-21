def map1(input_val, power=2.0, deadzone=0.05):
    if abs(input_val) < deadzone:
        return 0
    scaled_input = (abs(input_val) - deadzone) / (1.0 - deadzone)
    curve_val = scaled_input ** power
    output = 20000 * curve_val
    return int(output)

class ButtonRampController:
    def __init__(self, smoothing=0.1):
        self.smoothing = smoothing
        self.current_pwm = 0.0

    def process(self, is_pressed):
        # If button is pressed, target is 1.0 (Full), else 0.0
        # We pass 1.0 into map1 to get the max value (20,000)
        target_pwm = float(map1(1.0)) if is_pressed else 0.0
        
        error = target_pwm - self.current_pwm
        self.current_pwm += (error * self.smoothing)
        return int(self.current_pwm)