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
            target_pwm = 0.0
        else:
            # Forward or Reverse: Linear mapping from 0 to 1 -> 0 to 20,000
            target_pwm = abs(input_val) * 20000.0

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