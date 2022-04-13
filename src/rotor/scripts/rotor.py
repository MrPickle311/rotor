from rotor_internal.rotor_driver import RotorDriver

from typing import Optional, List

from commons.msg import RotorProcessAction, RotorProcessResult, RotorProcessFeedback, RotorProcessGoal
from commons.msg import RotorReset, RotorRelativePosition, RotorAbsolutePosition

from module_io.module_interface import AbstractModule
from module_utils.signal_handlers import SigIntHandler

import rospy

from math import pi

from multitimer import MultiTimer
import time

class RotorController:
    UINT16_MAX = 65535

    def __init__(self, bus_number: int, device_address: int):
        self._rotor_driver = RotorDriver(bus_number, device_address)

    def get_uint16_for_radians(self, angle: float) -> int:
        return int(self.UINT16_MAX * (angle / (2 * pi)))

    def convert_angle_to_bytes_list(self, angle: float) -> List[int]:
        print(f'Angle:  {angle} radians')
        return list(self.get_uint16_for_radians(angle).to_bytes(2, byteorder='big'))

    # return radians
    def convert_bytes_list_to_angle(self, bytes_list: List[int]) -> float:
        return (self.convert_bytes_list_to_integer(bytes_list) / self.UINT16_MAX) * 2 * pi

    # return uint16
    def convert_bytes_list_to_integer(self, bytes_list: List[int]) -> int:
        return int.from_bytes(bytes_list, byteorder='big')

    # radians
    def go_to_absolute_position(self, angle: float):
        self._rotor_driver.go_to_absolute_location(
            self.convert_angle_to_bytes_list(angle))

    # radians
    def go_to_relative_position(self, direction: int, angle: float):
        bytes_to_send = self.convert_angle_to_bytes_list(angle)
        self._rotor_driver.go_to_relative_location(direction, bytes_to_send)

    def reset(self):
        self._rotor_driver.rotor_reset()
        time.sleep(3)
        self._rotor_driver.wake_up()

    def get_calibration_state(self) -> int:
        return self._rotor_driver.get_calibration_state()

    # returns angle in radians
    def get_current_location(self) -> float:
        bytes_list = self._rotor_driver.get_current_location()
        return self.convert_bytes_list_to_angle(bytes_list)


class RotorModule(AbstractModule):
    def __init__(self, module_name: str):
        AbstractModule.__init__(self, module_name, RotorProcessAction)
        self._rotor_controller: Optional[RotorController] = None
        self.add_signal_receiver('/rotor_reset', RotorReset, self.reset_rotor)
        self.add_signal_receiver('/go_to_relative_position', RotorRelativePosition, self.go_to_relative_position)
        self.add_signal_receiver('/go_to_absolute_position', RotorAbsolutePosition, self.go_to_absolute_position)

        self._feedback_timer: Optional[MultiTimer] = None

    def _init_rotor(self, bus_number: int, feedback_interval: float, device_address: int) -> None:
        self._rotor_controller = RotorController(bus_number, device_address)
        self._feedback_timer = MultiTimer(feedback_interval, self.send_feedback, runonstart=False)

    def start_sending_rotor_feedback(self):
        self._feedback_timer.start()

    def stop_sending_rotor_feedback(self):
        self._feedback_timer.stop()

    def start_module(self, start_args: RotorProcessGoal) -> None:
        def wait_for_callibration_finish():
            while self._rotor_controller.get_calibration_state() != 1:
                pass
        
        self._init_rotor(start_args.bus_number, start_args.feedback_interval, start_args.device_address)
        
        self._rotor_controller.reset()
        wait_for_callibration_finish()
        self.start_sending_rotor_feedback()
        
        print('Started')
        
        self.wait_for_module_finish()
        self.stop_sending_rotor_feedback()

        self.finish_module(0)

    def send_feedback(self) -> None:
        feedback = RotorProcessFeedback()
        feedback.rotor_calibration_state = self._rotor_controller.get_calibration_state()
        time.sleep(1)
        feedback.rotor_pos = self._rotor_controller.get_current_location()
        self._server.publish_feedback(feedback)

    def finish_module(self, exit_code: int) -> None:
        result = RotorProcessResult()
        result.exit_code = exit_code
        self._server.set_succeeded(result)

    def reset_rotor(self, msg: RotorReset):
        self._rotor_controller.reset()

    def go_to_relative_position(self, rel_pos: RotorRelativePosition):
        self.stop_sending_rotor_feedback()
        self._rotor_controller.go_to_relative_position(rel_pos.direction, rel_pos.position)
        self.start_sending_rotor_feedback()

    def go_to_absolute_position(self, abs_pos: RotorAbsolutePosition):
        self.stop_sending_rotor_feedback()
        self._rotor_controller.go_to_absolute_position(abs_pos.position)
        self.start_sending_rotor_feedback()


if __name__ == '__main__':
    rospy.init_node('rotor', anonymous=True)
    sigint_handler = SigIntHandler()
    module = RotorModule('rotor')
    rospy.spin()
