#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rov_msgs.msg import Ms5 as MS5837Data
from control.services.Ms5 import Ms5 as Ms5Driver


MAX_DEPTH_METERS = 1.0   # 100% = 1 meter below surface


class Ms5Node(Node):

    def __init__(self):
        super().__init__("ms5_node")

        self.ms5 = Ms5Driver(bus=4)
        self.get_logger().info("MS5837 Driver initialized")

        self.publisher = self.create_publisher(MS5837Data, "sensor/ms5/data", 10)

        # --- Calibration: capture surface pressure on startup ---
        self.surface_pressure = None          # raw pressure at surface (mbar)
        self.calibration_samples = []         # collect a few samples to average
        self.calibration_target = 10          # number of samples to average
        self.calibrated = False

        self.msg = MS5837Data()

        # Timer at 10 Hz
        self.timer = self.create_timer(0.1, self.run)
        self.get_logger().info("Collecting surface calibration samples...")

    # ------------------------------------------------------------------
    # Calibration helper
    # ------------------------------------------------------------------
    def _try_calibrate(self, raw_pressure: float) -> bool:
        """
        Collect samples until we have enough, then set surface_pressure.
        Returns True once calibration is complete.
        """
        self.calibration_samples.append(raw_pressure)

        if len(self.calibration_samples) >= self.calibration_target:
            self.surface_pressure = sum(self.calibration_samples) / len(self.calibration_samples)
            self.calibrated = True
            self.get_logger().info(
                f"Calibration complete. Surface pressure = {self.surface_pressure:.4f} mbar"
            )
            return True

        self.get_logger().info(
            f"Calibrating... ({len(self.calibration_samples)}/{self.calibration_target})"
        )
        return False

    # ------------------------------------------------------------------
    # Depth normalisation  0 (surface) → 100 (MAX_DEPTH_METERS)
    # ------------------------------------------------------------------
    def _normalize_depth(self, raw_depth_meters: float) -> float:
        """
        raw_depth_meters: depth calculated by the driver (metres, 0 at surface).
        Returns a float clamped to [0.0, 100.0].
        """
        # Offset so that the exact water-entry point is 0
        corrected = raw_depth_meters - self._surface_depth_offset
        normalized = (corrected / MAX_DEPTH_METERS) * 100.0
        return max(0.0, min(100.0, normalized))   # clamp to valid range

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self):
        try:
            temp, press, depth, t_param, p_param, density = self.ms5.read_sensor()

            # Phase 1 – still calibrating: collect samples, publish raw zeros
            if not self.calibrated:
                if self._try_calibrate(float(press)):
                    # On the final calibration sample, also record the depth offset
                    # so that depth == 0 matches exactly the surface pressure moment
                    self._surface_depth_offset = float(depth)
                else:
                    return   # don't publish until calibrated

            # Phase 2 – normal operation
            normalized_depth = self._normalize_depth(float(depth))

            self.msg.temperature   = float(temp)
            self.msg.pressure      = float(press)
            self.msg.depth         = normalized_depth   # 0–100
            self.msg.temp_param    = int(t_param)
            self.msg.p_param       = int(p_param)
            self.msg.fluid_density = float(density)

            self.publisher.publish(self.msg)

        except Exception as e:
            self.get_logger().warn(f"MS5837 read failed, skipping frame: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Ms5Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()