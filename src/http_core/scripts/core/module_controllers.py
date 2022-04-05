import time
from typing import List, Callable

from .collectors import DroneStateCollector, ComStateCollector, RotorStateCollector
from module_io.module_interface import AbstractModuleController

from commons.msg import DroneCommunicationProcessAction, DroneCommunicationProcessFeedback, \
    DroneCommunicationProcessResult, DroneCommunicationProcessGoal
from commons.msg import HttpClientProcessAction, HttpClientProcessGoal, HttpClientProcessResult, \
    HttpClientProcessFeedback
from commons.srv import TryDroneStart, TryLand, TryUploadMission, DroneReturn, DroneReturnRequest, DroneReturnResponse
from commons.srv import TryDroneStartRequest, TryLandRequest, TryUploadMissionRequest
from commons.msg import RotorProcessAction, RotorProcessResult, RotorProcessFeedback, RotorProcessGoal
from commons.msg import RotorReset, RotorRelativePosition, RotorAbsolutePosition
from commons.msg import DroneState, DroneEvent

from enums.rotor import CalibrationStatus


class DroneCommunicationModuleController(AbstractModuleController):

    def __init__(self, drone_state: DroneStateCollector):
        AbstractModuleController.__init__(
            self, 'drone_communication', DroneCommunicationProcessAction)
        self._drone_state = drone_state
        self.add_task_publisher('/start_drone', TryDroneStart)
        self.add_task_publisher('/send_mission', TryUploadMission)
        self.add_task_publisher('/land_drone', TryLand)

    def start_module(self):
        start_arguments = DroneCommunicationProcessGoal()
        AbstractModuleController.start_module(self, start_arguments)

    def process_feedback(self, feedback: DroneCommunicationProcessFeedback) -> None:
        state = self._drone_state
        state.drone_longitude = feedback.long
        state.drone_lattitude = feedback.latt
        state.drone_altitude = feedback.alt
        state.drone_battery_voltage = feedback.voltage
        state.drone_battery_temperature = feedback.temperature

    # TODO: change voltage -> temp

    def on_process_result_received(self, state: any, result: DroneCommunicationProcessResult) -> None:
        print(result.exit_code)

    def try_return_drone(self) -> bool:
        task = TryLandRequest()
        resp = self.send_task(DroneReturn, task)
        return resp.is_ok

    def try_land_normally(self) -> bool:
        task = TryLandRequest()
        task.mode = 'normally'
        resp = self.send_task(TryLand, task)
        return resp.is_drone_landed

    def try_land_emergency(self) -> bool:
        task = TryLandRequest()
        task.mode = 'emergency'
        resp = self.send_task(TryLand, task)
        return resp.is_drone_landed

    def try_drone_start(self) -> bool:
        task = TryDroneStartRequest()
        resp = self.send_task(TryDroneStart, task)
        return resp.drone_started

    def try_upload_mission(self, waypoints_path: str) -> bool:
        task = TryUploadMissionRequest(waypoints_path=waypoints_path)
        resp = self.send_task(TryUploadMission, task)
        return resp.is_mission_sent


class HttpClientController(AbstractModuleController):

    def __init__(self):
        AbstractModuleController.__init__(
            self, 'http_client', HttpClientProcessAction)
        self.add_signal_sender('/send_drone_state', DroneState)
        self.add_signal_sender('/drone_event', DroneEvent)

    def start_module(self):
        start_arguments = HttpClientProcessGoal()
        start_arguments.host_address = '192.168.1.10'
        start_arguments.port = 8080
        start_arguments.station_id = 0
        start_arguments.images_directory = ''
        AbstractModuleController.start_module(self, start_arguments)

    def process_feedback(self, feedback: HttpClientProcessFeedback) -> None:
        print('Feedback received')

    def on_process_result_received(self, state: any, result: HttpClientProcessResult) -> None:
        print(result.exit_code)

    def send_drone_state(self, lattitude: float, longitude: float, altitude: float, voltage: float,
                         temperature: float):
        status = DroneState()
        status.latt = lattitude
        status.long = longitude
        status.alt = altitude
        status.temperature = temperature
        status.voltage = voltage
        
        self.send_signal(DroneState, status)

    def send_drone_event(self, event: str):
        msg = DroneEvent()
        msg.event = event
        self.send_signal(DroneEvent, msg)


class RotorController(AbstractModuleController):
    def __init__(self, rotor_state: RotorStateCollector):
        AbstractModuleController.__init__(self, 'rotor', RotorProcessAction)
        self.add_signal_sender('/rotor_reset', RotorReset)
        self.add_signal_sender(
            '/go_to_relative_position', RotorRelativePosition)
        self.add_signal_sender(
            '/go_to_absolute_position', RotorAbsolutePosition)
        self._rotor_state = rotor_state

    def start_module(self):
        start_arguments = RotorProcessGoal()
        start_arguments.feedback_interval = 0.8
        start_arguments.bus_number = 1
        start_arguments.device_address = 0x28
        AbstractModuleController.start_module(self, start_arguments)

    def process_feedback(self, feedback: RotorProcessFeedback) -> None:
        self._rotor_state.state = feedback.rotor_calibration_state
        self._rotor_state.position = feedback.rotor_pos

    def on_process_result_received(self, state: any, result: RotorProcessResult) -> None:
        print(result.exit_code)

    def reset_rotor(self):
        msg = RotorReset()
        self.send_signal(RotorReset, msg)

    # radian
    def go_to_relative_pos(self, direction: int, angle: float):
        msg = RotorRelativePosition()
        msg.position = angle
        msg.direction = direction
        self.send_signal(RotorRelativePosition, msg)

    def go_to_absolute_pos(self, angle: float):
        msg = RotorAbsolutePosition()
        msg.position = angle
        self.send_signal(RotorAbsolutePosition, msg)


class ComModules:
    def __init__(self, state_collector: ComStateCollector):
        self.drone_controller = DroneCommunicationModuleController(
            state_collector.drone_state)
        self.http_client_controller = HttpClientController()
        self.rotor_controller = RotorController(state_collector.rotor_state)
        self._state_collector = state_collector

    def init_modules(self):
        self.drone_controller.start_module()
        time.sleep(2)
        self.http_client_controller.start_module()
        time.sleep(2)
        self.rotor_controller.start_module()
        self.wait_for_callibration_complete()
        print('Modules initialization completed')

    def upload_mission_to_drone(self, waypoints: List[str]):
        self.drone_controller.try_upload_mission(waypoints[0])
    
    def wait_for_callibration_complete(self):
        while True: 
            if self._state_collector.rotor_state.state == CalibrationStatus.CalibrationCompletedSuccessfully:
                break
        
