#!/usr/bin/env python3
import board
import adafruit_dht

class TemperatureSensor:
    def __init__(self, pin=board.D4):
        """Initialize HW-484 (DHT sensor) on a specific GPIO pin"""
        try:
            # HW-484 is a DHT11
            self.sensor = adafruit_dht.DHT22(pin)
            print(f"HW-484 initialized on pin {pin}")
        except Exception as e:
            raise ValueError(f"Failed to initialize HW-484: {e}")

    def get_data(self):
        """Returns (temperature_c, humidity)"""
        try:
            temperature_c = self.sensor.temperature
            humidity = self.sensor.humidity
            
            if temperature_c is not None and humidity is not None:
                return temperature_c, humidity
            else:
                raise RuntimeError("Sensor returned None")
                
        except RuntimeError as error:
            # Catch expected DHT timing errors
            raise RuntimeError(f"DHT Read Error: {error.args[0]}")
        except Exception as e:
            self.sensor.exit()
            raise e