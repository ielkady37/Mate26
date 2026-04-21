#!/usr/bin/env python3
import board
import busio
from adafruit_pca9685 import PCA9685
import PCA_map1 
class PCA:    
    def __init__(self, i2c_address=0x40, frequency=50):
        self.frequency = frequency
        self.address = i2c_address
        self.pca = None
        self._initializePCA(i2c_address, self.frequency)

    def _initializePCA(self, i2c_address, frequency):
        """
        Initialize the PCA9685 driver.
        """
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c, address=i2c_address)
            self.pca.frequency = frequency
        except Exception as e:
            raise ValueError(f"Failed to initialize PCA9685 at {hex(i2c_address)}: {e}")
                
    def _microsecondsToDutycycle(self, microseconds):
        period_us = 1_000_000 / self.frequency
        duty_cycle = int((microseconds / period_us) * 65535)
        return min(max(duty_cycle, 0), 65535)


    def PWMWrite(self, channel, channel1, ContollerInput):
        if not 0 <= channel <= 15:
            raise ValueError("Channel must be between 0 and 15.")
        
        if ContollerInput < -1.0 or ContollerInput > 1.0:
            raise ValueError("Controller input must be between -1.0 and 1.0.")

        if self.pca is not None and controller_input > 0:
            duty_cycle_value = PCA_map1.map1(ContollerInput)
            
            self.pca.channels[channel].duty_cycle = duty_cycle_value
            self.pca.channels[channel1].duty_cycle = 0
            
        elif self.pca is not None and controller_input < 0:
            duty_cycle_value = PCA_map1.map1(ContollerInput)
            self.pca.channels[channel].duty_cycle = 0
            self.pca.channels[channel1].duty_cycle = duty_cycle_value
        else:
            print(f"Error: PCA at {hex(self.address)} not initialized.")



    def stopAll(self):
        if self.pca is not None:
            for i in range(16):
                self.pca.channels[i].duty_cycle = 0

    def close(self):
        if self.pca is not None:
            self.pca.deinit()

# if __name__ == "__main__":
#     try:
#         import RPi.GPIO as GPIO
#         import time
#         GPIO.setmode(GPIO.BCM)
#         GPIO.setup(17, GPIO.OUT)
#         GPIO.setup(27, GPIO.OUT)
#         board_a = PCA(i2c_address=0x40)
#         board_b = PCA(i2c_address=0x41)
        
#         while True:
#             print("Board A servo")
#             board_a.PWMWrite(0, 0)
#             GPIO.output(17, GPIO.HIGH)
#             GPIO.output(27, GPIO.LOW)
#             time.sleep(2)

#             print("Board B servo")
#             board_b.PWMWrite(0, 0)
#             GPIO.output(17, GPIO.LOW)
#             GPIO.output(27, GPIO.HIGH)
#             time.sleep(2)


        
#         print("Moving Servo on Board B, Channel 0")

#     except Exception as e:
#         print(f"Test failed: {e}")