import time
import board
import busio
import adafruit_bno08x
import math
import statistics

#pip install 
# ANSI escape codes for colors
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"

class IMUSensor:
    def __init__(self, i2c, retries=3):
        self.bno = None
        self.gyro_bias = (0.0, 0.0, 0.0)
        self.last_euler = None

        attempt = 0
        while attempt < retries and self.bno is None:
            try:
                self.bno = adafruit_bno08x.BNO08X_I2C(i2c)
                self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
                self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
                self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
                self.bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
                print(GREEN + "BNO085 initialized successfully" + RESET)
            except Exception as e:
                print(RED + f"Initialization error: {e}" + RESET)
                time.sleep(1)
                attempt += 1

        if self.bno is None:
            raise RuntimeError(RED + "Failed to initialize BNO085 after retries" + RESET)

    def calibrate_gyro(self, samples=200):
        gx, gy, gz = [], [], []
        print(YELLOW + "Calibrating gyro bias..." + RESET)
        for _ in range(samples):
            g = self.bno.gyro
            gx.append(g[0]); gy.append(g[1]); gz.append(g[2])
            time.sleep(0.01)
        self.gyro_bias = (statistics.mean(gx), statistics.mean(gy), statistics.mean(gz))
        print(GREEN + f"Gyro bias calibrated: {self.gyro_bias}" + RESET)

    def quaternion_to_euler(self, quat):
        # Reorder: library returns (i, j, k, real)
        w, x, y, z = quat[3], quat[0], quat[1], quat[2]
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return {
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
            "roll_rad": roll,
            "pitch_rad": pitch,
            "yaw_rad": yaw
        }

    def read(self):
        try:
            accel = self.bno.acceleration
            gyro = tuple(g - b for g, b in zip(self.bno.gyro, self.gyro_bias))
            mag = self.bno.magnetic
            quat = self.bno.quaternion
            euler = self.quaternion_to_euler(quat) if quat else None

            if self.last_euler and euler:
                euler = {k: 0.9*self.last_euler[k] + 0.1*euler[k] for k in euler}
            self.last_euler = euler

            return {"accel": accel, "gyro": gyro, "mag": mag, "quat": quat, "euler": euler}
        except Exception as e:
            print(RED + f"Sensor read error: {e}" + RESET)
            return {"accel": None, "gyro": None, "mag": None, "quat": None, "euler": None}

class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.integral = 0
        self.last_error = 0

    def compute(self, measurement, dt=0.1):
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.last_error = error
        return output

class ThrusterController:
    def __init__(self):
        # Placeholder: replace with GPIO/PWM driver code
        self.left_thruster = 0
        self.right_thruster = 0

    def apply_control(self, yaw_signal):
        # Simple differential thrust control
        self.left_thruster = 1500 + yaw_signal
        self.right_thruster = 1500 - yaw_signal
        print(GREEN + f"Thrusters -> Left: {self.left_thruster}, Right: {self.right_thruster}" + RESET)

def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    imu = IMUSensor(i2c)
    imu.calibrate_gyro(samples=300)

    yaw_pid = PID(kp=1.2, ki=0.05, kd=0.1, setpoint=0)  # target yaw = 0°
    thrusters = ThrusterController()

    last_time = time.time()
    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        data = imu.read()
        if data["euler"]:
            yaw = data["euler"]["yaw_deg"]
            control_signal = yaw_pid.compute(yaw, dt)
            thrusters.apply_control(control_signal)

            print(YELLOW + f"Yaw: {yaw:.2f}°, Control Signal: {control_signal:.2f}" + RESET)
        time.sleep(0.1)

if __name__ == "__main__":
    main()
