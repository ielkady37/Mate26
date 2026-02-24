import time
import RPi.GPIO as GPIO
import spidev  # For analog input via MCP3008 ADC (if used)

class RaindropSensor:
    def __init__(self, digital_pin, adc_channel=None, spi_bus=0, spi_device=0):
        """
        Initialize Raindrop Sensor.
        :param digital_pin: GPIO pin connected to digital output (DO).
        :param adc_channel: Channel on MCP3008 for analog output (AO). None if unused.
        :param spi_bus: SPI bus for MCP3008.
        :param spi_device: SPI device for MCP3008.
        """
        self.digital_pin = digital_pin
        self.adc_channel = adc_channel

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.digital_pin, GPIO.IN)

        # Setup SPI if analog channel is provided
        if self.adc_channel is not None:
            self.spi = spidev.SpiDev()
            self.spi.open(spi_bus, spi_device)
            self.spi.max_speed_hz = 1350000
        else:
            self.spi = None

    def read_digital(self):
        """
        Read digital output (rain detected or not).
        Returns True if rain is detected, False otherwise.
        """
        return GPIO.input(self.digital_pin) == GPIO.LOW  # LOW = rain detected

    def read_analog(self):
        """
        Read analog output (rain intensity).
        Requires MCP3008 ADC.
        """
        if self.spi is None or self.adc_channel is None:
            raise RuntimeError("Analog channel not configured.")
        adc = self.spi.xfer2([1, (8 + self.adc_channel) << 4, 0])
        value = ((adc[1] & 3) << 8) + adc[2]
        return value

    def cleanup(self):
        """
        Clean up GPIO and SPI resources.
        """
        GPIO.cleanup()
        if self.spi:
            self.spi.close()

def main():
    # Example: digital pin 17, analog channel 0
    sensor = RaindropSensor(digital_pin=17, adc_channel=0)

    try:
        while True:
            rain_detected = sensor.read_digital()
            print("Rain detected:", rain_detected)

            intensity = sensor.read_analog()
            print("Rain intensity (0-1023):", intensity)

            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        sensor.cleanup()

if __name__ == "__main__":
    main()
