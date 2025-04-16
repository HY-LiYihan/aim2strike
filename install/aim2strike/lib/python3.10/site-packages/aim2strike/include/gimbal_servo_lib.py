# gimbal_servo_lib.py
# -- coding: utf-8 --

import numpy as np
import time
from aim2strike.include.uart_servo import UartServoManager
# --- Attempt to import UartServoManager ---
# The user of this library is responsible for ensuring this path is correct
# or that UartServoManager is otherwise importable.
# try:
#     from src.GimbalServo.uart_servo import UartServoManager
#     # print("Debug: UartServoManager imported in library.") # Optional debug print
# except ImportError as e:
#     print(f"CRITICAL ERROR in gimbal_servo_lib: Failed to import UartServoManager.")
#     print(f"Import Error: {e}")
#     print("Ensure 'src/pans/uart_servo.py' is accessible in the Python path.")
#     # Re-raise the error so the main program knows initialization failed
#     raise ImportError("UartServoManager not found, cannot initialize GimbalServo library.") from e
# except Exception as e:
#     print(f"An unexpected error occurred during UartServoManager import: {e}")
#     raise


class GimbalServo:
    """
    Represents and controls a single gimbal servo motor via a UartServoManager.

    Handles position calculation based on deviation from a middle point
    and enforces angle limits.
    """
    def __init__(self, servo_id: int, mid_pos: int, min_dev: int, max_dev: int, uservo_manager: UartServoManager):
        """
        Initializes a GimbalServo instance.

        Args:
            servo_id (int): The unique ID of this servo on the bus.
            mid_pos (int): The raw servo value corresponding to the center position (0 deviation).
            min_dev (int): The minimum allowed deviation from the mid_pos in raw servo units.
            max_dev (int): The maximum allowed deviation from the mid_pos in raw servo units.
            uservo_manager (UartServoManager): The initialized UartServoManager instance
                                                that handles communication for this servo.
        """
        if not isinstance(uservo_manager, UartServoManager):
            raise TypeError("uservo_manager must be an instance of UartServoManager")
        if not isinstance(servo_id, int) or servo_id <= 0:
             raise ValueError("servo_id must be a positive integer")
        if not all(isinstance(val, int) for val in [mid_pos, min_dev, max_dev]):
             raise ValueError("mid_pos, min_dev, and max_dev must be integers")
        if min_dev >= max_dev:
            raise ValueError("min_dev must be less than max_dev")

        self.id = servo_id
        self.mid_pos = mid_pos
        self.min_dev = min_dev
        self.max_dev = max_dev
        self.uservo_manager = uservo_manager # Shared manager instance

        # Calculate absolute position limits based on deviation
        self.min_abs_pos = self.mid_pos + self.min_dev
        self.max_abs_pos = self.mid_pos + self.max_dev

        self.last_set_deviation = 0.0 # Keep track of the last set deviation

        print(f"GimbalServo initialized: ID={self.id}, Mid={self.mid_pos}, DevRange=[{self.min_dev}, {self.max_dev}], AbsRange=[{self.min_abs_pos}, {self.max_abs_pos}]")

    def set_deviation(self, angle_deviation: float):
        """
        Sets the servo's position based on a deviation from its middle position.

        Args:
            angle_deviation (float): The desired deviation from the middle position
                                     in raw servo units. Positive or negative.
        """
        if self.uservo_manager is None:
            print(f"Warning: Servo {self.id} cannot be set, UartServoManager not available.")
            return

        # Calculate target absolute position
        target_pos = self.mid_pos + angle_deviation

        # Clamp the calculated absolute position within the allowed absolute range
        clamped_pos = int(np.clip(target_pos, self.min_abs_pos, self.max_abs_pos))

        # Optional: Clamp the deviation itself to ensure it's within limits
        # clamped_deviation = np.clip(angle_deviation, self.min_dev, self.max_dev)
        # if angle_deviation != clamped_deviation:
        #      print(f"Warning: Servo {self.id} deviation {angle_deviation:.1f} clamped to {clamped_deviation:.1f}")
        # self.last_set_deviation = clamped_deviation # Store the actual clamped deviation

        try:
            # print(f"Setting Servo {self.id} -> Dev: {angle_deviation:.1f} -> TargetAbs: {target_pos:.1f} -> ClampedAbs: {clamped_pos}") # Debug
            self.uservo_manager.set_position(self.id, clamped_pos)
            self.last_set_deviation = angle_deviation # Store requested deviation if successful
        except Exception as e:
            print(f"Error setting servo {self.id} position: {e}")

    def center(self):
        """Moves the servo to its defined middle position (0 deviation)."""
        print(f"Centering Servo {self.id}...")
        self.set_deviation(0.0)

    def get_last_deviation(self) -> float:
        """Returns the last successfully requested deviation value."""
        return self.last_set_deviation

    # --- Potential Future Methods (require UartServoManager support) ---
    # def get_current_position(self) -> int | None:
    #     """Reads the current absolute position from the servo."""
    #     if self.uservo_manager and hasattr(self.uservo_manager, 'get_position'):
    #         try:
    #             return self.uservo_manager.get_position(self.id)
    #         except Exception as e:
    #             print(f"Error reading servo {self.id} position: {e}")
    #             return None
    #     else:
    #         print(f"Warning: Cannot read servo {self.id} position, UartServoManager missing or lacks get_position method.")
    #         return None

    # def get_current_deviation(self) -> float | None:
    #     """Reads the current position and calculates the deviation from mid_pos."""
    #     current_pos = self.get_current_position()
    #     if current_pos is not None:
    #         return float(current_pos - self.mid_pos)
    #     return None