from typing import List
import time
from i2c_io.i2c_device import I2CDevice
from enums.rotor import CalibrationStatus, MovementStatus, RotationDirection


class RotorDriver(I2CDevice):
    def rotor_reset(self):
        command_byte = 0x01
        self._bus.write_byte(self._device_address, command_byte)

    def wake_up(self):
        command_byte = 0x1C
        self._bus.write_byte(self._device_address, command_byte)

    def get_calibration_state(self) -> int:
        command_byte = 0x02
        return self.read_block_data_from(command_byte, 1)[0]

    def get_movement_state(self) -> int:
        command_byte = 0x03
        return self.read_block_data_from(command_byte, 1)[0]

    def wait_for_stop(self) -> None:
        status = MovementStatus.MovingClockwise
        while status != MovementStatus.Stopped:
            status = self.get_movement_state()
            time.sleep(0.05)

    def get_current_location(self) -> List[int]:
        command_byte = 0x04

        self.wait_for_stop()

        return self.read_block_data_from(command_byte, 2)

    def go_to_absolute_location(self, pos_bytes: List[int]):
        command_byte = 0x05
        #self.wait_for_stop()
        self.write_block_data_to(command_byte, pos_bytes)

    def go_to_relative_location(self, direction: int, pos_bytes: List[int]):
        command_byte = 0x040

        data_to_send = [direction] + pos_bytes

        self.wait_for_stop()

        self.write_block_data_to(command_byte, data_to_send)
