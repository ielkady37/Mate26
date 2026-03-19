import board
import busio
import adafruit_bmp280
import time

class Depth:
    def __init__(self, sea_level_pa=1021.0):
        """
        Initializes the BMP280 sensor and applies initial calibration.
        """
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = None
        
        # Try both common I2C addresses
        for address in [0x76, 0x77]:
            try:
                self.sensor = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c, address=address)
                # Factory calibration is loaded automatically by the library here
                self.sensor.sea_level_pressure = sea_level_pa
                break
            except Exception:
                continue

        if not self.sensor:
            raise RuntimeError("Could not find BMP280 sensor. Check wiring!")

    def tare(self):
        """
        Sets the current atmospheric pressure as the 'zero' altitude point.
        """
        # Average 10 readings to ignore minor air fluctuations
        readings = [self.sensor.pressure for _ in range(10)]
        avg_pressure = sum(readings) / len(readings)
        self.sensor.sea_level_pressure = avg_pressure
        return avg_pressure

    def get_readings(self):
        """
        Returns a dictionary of the current sensor values.
        """
        return {
            "altitude": round(self.sensor.altitude, 2),
            "pressure": round(self.sensor.pressure, 2),
            "temperature": round(self.sensor.temperature, 2)
        }