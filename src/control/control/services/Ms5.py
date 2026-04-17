import ms5837
import time

class MS5837Service:
    def __init__(self, bus=1):
        """
        Initializes the MS5837 sensor. 
        Note: MS5837_30BA is the 30 Bar model (e.g., BlueRobotics Bar30). 
        If you have the 2 Bar model (Bar02), change this to ms5837.MS5837_02BA.
        """
        self.sensor = ms5837.MS5837_30BA(bus)
        
        # Initialize the I2C bus connection
        if not self.sensor.init():
            raise RuntimeError("MS5837 sensor could not be initialized. Check I2C wiring and bus number.")
        
        # Set fluid density (DENSITY_FRESHWATER = 997, DENSITY_SALTWATER = 1029)
        self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)

    def read_data(self):
        """
        Reads data from the sensor and returns a dictionary.
        """
        if self.sensor.read():
            return {
                'pressure': self.sensor.pressure(),       # mbar
                'temperature': self.sensor.temperature(), # Celsius
                'depth': self.sensor.depth()              # meters
            }
        else:
            return None