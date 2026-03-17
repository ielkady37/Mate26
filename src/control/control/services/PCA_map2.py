class ExponentialReachController:
    def __init__(self, deadzone=0.05, smoothing=0.1):
        """
        deadzone: ignores small stick movements.
        smoothing: How fast it reaches target (0.01 = very slow, 0.5 = very fast).
        """
        self.deadzone = deadzone
        self.smoothing = smoothing
        self.current_pwm = 0.0

    def process(self, input_val):
        # --- PHASE 1: LINEAR MAPPING WITH DEADZONE ---
        if abs(input_val) < self.deadzone:
            # If in deadzone, set target to the "Stop" values
            target_pwm = 0.0 if input_val >= 0 else 10000.0
        else:
            if input_val > 0:
                # Forward: Linear 0 to 1 -> 0 to 10,000
                target_pwm = input_val * 10000.0
            else:
                # Reverse: Linear -1 to 0 -> 20,000 to 10,000
                # Formula: 10,000 + (abs(input) * 10,000)
                # Note: -1 input = 20,000, -0.01 input = ~10,100
                target_pwm = 10000.0 + (abs(input_val) * 10000.0)

        # --- PHASE 2: EXPONENTIAL REACH ---
        # Calculate the gap (error) between where we are and where we want to be
        error = target_pwm - self.current_pwm
        
        # Move a percentage of that gap. 
        # This is what creates the exponential 'easing' effect.
        self.current_pwm += (error * self.smoothing)

        return int(self.current_pwm)

# Example Usage:
# controller = ExponentialReachController(smoothing=0.05) # Slow, smooth reach
# while True:
#    raw_joy = get_joystick() 
#    pwm_to_send = controller.process(raw_joy)
#    pca.duty_cycle = pwm_to_send