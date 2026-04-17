#!/usr/bin/env python3
import time

# Importing the local library as we fixed previously
from control.services import ms5837

class Ms5:
    def __init__(self, bus: int = 1, density: float = ms5837.DENSITY_FRESHWATER):
        """Initialize MS5837 sensor"""
        try:
            # Note: MS5837_30BA is the standard 30 Bar model.
            self.sensor = ms5837.MS5837_30BA(bus)
            
            max_retries = 5
            for attempt in range(max_retries):
                try:
                    if self.sensor.init():
                        print(f"Connected to MS5837 on attempt {attempt + 1}")
                        break
                    else:
                        raise RuntimeError("Sensor init returned False")
                except Exception as e:
                    print(f"Init Attempt {attempt + 1} failed: {e}")
                    if attempt == max_retries - 1:
                        raise ValueError(f"MS5837 totally failed to initialize: {e}")
                    time.sleep(0.5)

            time.sleep(0.5)
            
            # Set initial fluid density
            self.sensor.setFluidDensity(density)
            print("MS5837 initialized successfully")

        except Exception as e:
            raise ValueError(f"MS5837 initialization failed: {e}")

    def read_sensor(self) -> tuple:
        """
        Reads the sensor and returns all data at once:
        (temperature, pressure, depth, temp_param, p_param, fluid_density)
        """
        try:
            if self.sensor.read():
                # Read standard metrics
                temp = self.sensor.temperature()
                press = self.sensor.pressure()
                depth = self.sensor.depth()
                
                # Read internal parameters (mimicking your screenshot from earlier)
                # Using getattr prevents a crash if the library version varies slightly
                t_param = getattr(self.sensor, '_temp_param', 0)
                p_param = getattr(self.sensor, '_p_param', 0)
                fluid_density = getattr(self.sensor, '_fluidDensity', 0.0)
                
                return temp, press, depth, t_param, p_param, fluid_density
            else:
                raise RuntimeError("Sensor read returned False")
                
        except Exception as e:
            # Print the actual error to the console so you can see it!
            print(f"I2C Read Error: {e}") 
            # Raise it up to the ROS node so it logs a warning
            raise RuntimeError("MS5837 Desynced")