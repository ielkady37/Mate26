import board
import busio
import adafruit_bmp280
import time

class Depth:
    """
        handler for the BMP280 sensor on Raspberry Pi 4.
    """
    def __init__(self, sea_level_pa=1013.25):
        try:
            # Setup I2C bus for RPi 4
            self.i2c = busio.I2C(board.SCL, board.SDA)
            
            # Initialize sensor (Address 0x76 is common, try 0x77 if it fails)
            self.sensor = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c, address=0x76)
            
            # Essential for accurate altitude
            self.sensor.sea_level_pressure = sea_level_pa
            print(f"BMP280 initialized. Sea Level Pressure set to {sea_level_pa} hPa.")
            
        except ValueError:
            print("Sensor not found at 0x76, trying 0x77...")
            self.sensor = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c, address=0x77)
        except Exception as e:
            print(f"Critical Error: {e}")
            self.sensor = None

    def read_altitude(self):
        """Returns altitude in meters."""
        return round(self.sensor.altitude, 2) if self.sensor else None

    def read_pressure(self):
        """Returns pressure in hectopascals (hPa)."""
        return round(self.sensor.pressure, 2) if self.sensor else None

    def read_temperature(self):
        """Returns temperature in Celsius."""
        return round(self.sensor.temperature, 2) if self.sensor else None

   