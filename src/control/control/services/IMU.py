#!/usr/bin/env python3
import time
import math
import board
import adafruit_bno08x as bno
from digitalio import DigitalInOut
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_extended_bus import ExtendedI2C
from digitalio import DigitalInOut, Direction


class IMU:

    # Notice we removed reset_pin from the arguments
    def __init__(self, address: int = 0x4b, debug: bool = False):
        """Initialize BNO085 sensor"""
        try:
            self.i2c = ExtendedI2C(4) # Still on Bus 4!
            
            max_retries = 5
            for attempt in range(max_retries):
                try:
                    # Explicitly tell the library reset=None so it doesn't touch the pin
                    self.bno = BNO08X_I2C(self.i2c, address=address, reset=None, debug=debug)
                    print(f"Connected to BNO085 on attempt {attempt + 1}")
                    break
                except Exception as e:
                    print(f"Init Attempt {attempt + 1} failed: {e}")
                    if attempt == max_retries - 1:
                        raise ValueError(f"BNO085 totally failed to initialize: {e}")
                    time.sleep(0.5) 

            time.sleep(1)

            # Only enable what we need
            self.enableFeature(bno.BNO_REPORT_ROTATION_VECTOR)

            # Flush old packets to avoid driver crash
            for _ in range(10):
                try:
                    _ = self.bno.quaternion
                except Exception:
                    pass
                time.sleep(0.05)

            print("BNO085 initialized successfully")

        except Exception as e:
            raise ValueError(f"BNO085 initialization failed: {e}")

    def enableFeature(self, feature: int) -> None:
        self.bno.enable_feature(feature)

    def getAcceleration(self) -> tuple:
        """Return acceleration (x,y,z)"""
        try:
            return self.bno.acceleration
        except Exception:
            return (0.0, 0.0, 0.0)

    def getGyro(self) -> tuple:
        """Return gyroscope data (x,y,z)"""
        try:
            return self.bno.gyro
        except Exception:
            return (0.0, 0.0, 0.0)

    def getMagnetometer(self) -> tuple:
        """Return magnetometer data (x,y,z)"""
        try:
            return self.bno.magnetic
        except Exception:
            return (0.0, 0.0, 0.0)

    def getQuaternion(self) -> tuple:
        """Return quaternion (i,j,k,real)"""
        try:
            return self.bno.quaternion
        except Exception as e:
            # Print the actual error to the console so you can see it!
            print(f"I2C Read Error: {e}") 
            # Raise it up to the ROS node so it logs a warning
            raise RuntimeError("IMU Desynced")

    def getEulerAngles(self) -> tuple:
        """Return Euler angles (roll, pitch, yaw) in radians"""

        q1, q2, q3, q0 = self.getQuaternion()

        roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q0 * q2 - q3 * q1))))
        yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))

        return roll, pitch, yaw

    def Calibrate(self, duration: int = 5) -> None:
        """
        Calibrate the sensor for a fixed duration
        """
        print("Calibration started")

        try:
            self.bno.begin_calibration()

            for i in range(duration, 0, -1):
                print(f"Calibrating... {i}s remaining", end="\r")
                time.sleep(1)

            self.bno.save_calibration_data()

            print("\nCalibration complete and saved")

        except Exception as e:
            raise ValueError(f"Calibration failed: {e}")

