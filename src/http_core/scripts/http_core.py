import time

from core.upload_mission_event import MissionExtractor, MissionReceiver
from core.com_receiver import COMServerAPI
from http_io.simple_server_entities import SimpleResourceGetter, SimpleHttpEventProcessor
from multitimer import MultiTimer

from core.kss_event import KSSTaskReceiver

from module_io.module_interface import AbstractModule
from module_utils.signal_handlers import SigIntHandler
from threading import Thread

import rospy
from typing import Dict, Optional, List, Callable
from core.module_controllers import ComModules
from core.collectors import ComStateCollector
import math
from math import pi
from enums.kss_core import ComEvent

from dependency_injector.wiring import Provide, inject
from core.containers import GlobalContainer
from parameters.parameters import ParametersConfigurator
from enums.parameters import Params
from mavros_msgs.msg import StatusText


class DronePositionIndicator:
    DRONE_STARTED = 'drone_started'
    DRONE_IS_FAR = 'drone_is_far'
    DRONE_IS_VERY_FAR = 'drone_is_very_far'
    DRONE_APPROACHING = 'drone_approaching'
    DRONE_IS_READY_TO_LAND = 'drone_is_ready_to_land'


class DroneDistanceFollower:
    EARTH_RADIUS = 6378.137  # kilometers
    LANDING_RADIUS = 30  # meters

    def __init__(self, modules: ComModules, com_collector: ComStateCollector):
        self._modules = modules
        self._com_collector = com_collector
        self._drone_follow_timer = MultiTimer(self.get_follow_interval, self.check_drone_position,
                                              runonstart=False)
        self._drone_position_indicator = 'none'

    def start_following(self):
        self._drone_follow_timer.start()

    def stop_following(self):
        self._drone_follow_timer.stop()

    def set_drone_started(self) -> None:
        self._drone_position_indicator = DronePositionIndicator.DRONE_STARTED

    def drone_started(self) -> bool:
        return self._drone_position_indicator == DronePositionIndicator.DRONE_STARTED

    def set_drone_is_far(self) -> None:
        self._drone_position_indicator = DronePositionIndicator.DRONE_IS_FAR

    def drone_is_far(self) -> bool:
        return self._drone_position_indicator == DronePositionIndicator.DRONE_IS_FAR

    def drone_is_approaching(self) -> bool:
        return self._drone_position_indicator == DronePositionIndicator.DRONE_APPROACHING

    def set_drone_is_approaching(self):
        self._drone_position_indicator = DronePositionIndicator.DRONE_APPROACHING

    def drone_is_very_far(self):
        return self._drone_position_indicator == DronePositionIndicator.DRONE_IS_VERY_FAR

    def set_drone_is_very_far(self):
        self._drone_position_indicator = DronePositionIndicator.DRONE_IS_VERY_FAR

    def drone_is_ready_to_land(self):
        return self._drone_position_indicator == DronePositionIndicator.DRONE_IS_READY_TO_LAND

    def set_drone_is_ready_to_land(self):
        self._drone_position_indicator = DronePositionIndicator.DRONE_IS_READY_TO_LAND

    @staticmethod
    def meters(kilometers):
        return kilometers * 1000

    def get_miranda_latt(self):
        return rospy.get_param(Params.MIRANDA_LATT)

    def get_miranda_long(self):
        return rospy.get_param(Params.MIRANDA_LONG)

    def get_close_landing_radius(self):
        return rospy.get_param(Params.CLOSE_LANDING_RADIUS)

    def get_landing_radius(self):
        return rospy.get_param(Params.LANDING_RADIUS)

    def get_far_landing_radius(self):
        return rospy.get_param(Params.FAR_LANDING_RADIUS)

    def get_follow_interval(self):
        return rospy.get_param(Params.FOLLOW_INTERVAL)

    # https://en.wikipedia.org/wiki/Haversine_formula
    def get_drone_distance_from_miranda(self, drone_latt: float, drone_long: float) -> float:
        d_latt = math.radians(self.get_miranda_latt() - drone_latt)
        d_long = math.radians(self.get_miranda_long() - drone_long)
        a = math.sin(d_latt / 2) ** 2 + \
            math.cos(math.radians(self.get_miranda_latt())) * math.cos(math.radians(drone_latt)) * \
            math.sin(d_long / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = self.EARTH_RADIUS * c
        return self.meters(d)

    def check_drone_position(self):
        current_distance = self.get_drone_distance_from_miranda(
            self._com_collector.drone_state.drone_lattitude,
            self._com_collector.drone_state.drone_longitude)

        print(f'Curent distance: {current_distance}')

        if current_distance > self.get_far_landing_radius() and self.drone_is_far():
            self.set_drone_is_very_far()
            print('Drone is very far!')
            return

        if current_distance > self.get_landing_radius() and self.drone_started():
            self.set_drone_is_far()
            print('Drone is far!')
            return

        if current_distance < self.get_landing_radius() and self.drone_is_very_far():
            self._modules.http_client_controller.send_drone_event(
                ComEvent.DRONE_IS_APPROACHING)
            self.set_drone_is_approaching()
            print('Drone is approaching!')
            return

        if current_distance < self.get_close_landing_radius() and self.drone_is_approaching():
            time.sleep(10)
            self._modules.http_client_controller.send_drone_event(
                ComEvent.DRONE_IS_READY_TO_LAND)
            self.set_drone_is_ready_to_land()
            print('Drone is ready to land!')
            return


class DroneRotationFollower:
    def __init__(self, modules: ComModules, com_collector: ComStateCollector):
        self._modules = modules
        self._com_collector = com_collector
        self._drone_follow_timer = MultiTimer(self.get_follow_interval, self.correct_position,
                                              runonstart=False)

    def get_miranda_latt(self):
        return rospy.get_param(Params.MIRANDA_LATT)

    def get_miranda_long(self):
        return rospy.get_param(Params.MIRANDA_LONG)

    def get_follow_interval(self):
        return rospy.get_param(Params.FOLLOW_INTERVAL)

    def start_following(self):
        self._drone_follow_timer.start()

    def stop_following(self):
        self._drone_follow_timer.stop()

    def get_drone_distance_from_miranda(self, latt: float, long: float) -> float:
        return math.sqrt((latt - self.get_miranda_latt()) ** 2 + (long - self.get_miranda_long()) ** 2)

    # reverses anticlockwise rotor movement to clockwise movement
    @staticmethod
    def alternate(angle: float):
        return 2 * pi - angle

    # converts result of atan2 from (-180:180) to (0:360)
    @staticmethod
    def get_absolute_atan(x: float, y: float) -> float:
        res = math.atan2(x, y)

        if res < 0:
            res += 2 * math.pi

        return res

    def correct_position(self):
        x = self._com_collector.drone_state.drone_longitude - self.get_miranda_long()
        y = self._com_collector.drone_state.drone_lattitude - self.get_miranda_latt()

        angle = self.alternate(self.get_absolute_atan(x, y))
        self._modules.rotor_controller.go_to_absolute_pos(angle)


class KSSEventNotifiers:
    def __init__(self, com_server_api: COMServerAPI, upload_mission_task: Callable[[str], bool]):
        self._kss_server_api = com_server_api

        self.add_mission_upload_task_event(upload_mission_task)

    def add_mission_upload_task_event(self, task: Callable[[str], bool]):
        self._kss_server_api.add_api_path_handler(
            '/com', '/kss_event/upload_mission', MissionReceiver(task))

    def add_task_event(self, task_path: str, task: Callable[[], bool]):
        self._kss_server_api.add_api_path_handler(
            '/com', f'/kss_event/{task_path}', KSSTaskReceiver(task))


class HttpCoreModule:
    DRONE_STATE_UPDATE_INTERVAL = 1
    DRONE_LAND_TIMEOUT = 60

    def __init__(self):
        self._kss_server = COMServerAPI('com_server', 4000, '192.168.1.16')
        self._state_collector = ComStateCollector()
        self._modules = ComModules(self._state_collector)
        drone_com_module = self._modules.drone_controller
        self._drone_rotation_follower = DroneRotationFollower(
            self._modules, self._state_collector)
        self._drone_position_follower = DroneDistanceFollower(
            self._modules, self._state_collector)

        self._drone_state_timer = MultiTimer(self.DRONE_STATE_UPDATE_INTERVAL, self.send_drone_state_do_kss,
                                             runonstart=False)

        self._drone_landing_timer = MultiTimer(
            self.DRONE_LAND_TIMEOUT, self.land_emergency, runonstart=False)

        self._modules.drone_controller.add_signal_receiver(
            '/mavros/statustext/recv', StatusText)

        def try_start_and_upload_mission(mission_name) -> bool:
            is_ok = False
            is_ok = drone_com_module.try_upload_mission(mission_name)
            time.sleep(1)
            is_ok = drone_com_module.try_drone_start()

            if is_ok:
                self._drone_position_follower.set_drone_started()

            return is_ok

        self._event_notifiers = KSSEventNotifiers(
            self._kss_server, try_start_and_upload_mission)
        self._event_notifiers.add_task_event(
            'land_drone_normally', self.land_normally)
        self._event_notifiers.add_task_event(
            'land_drone_emergency', self.land_emergency)
        self._event_notifiers.add_task_event(
            'abort_drone', drone_com_module.try_return_drone)

    def notify_about_drone_landed(self, msg: StatusText):
        if msg.text == 'DRONE_LANDED':
            self._modules.http_client_controller.send_drone_event(
                ComEvent.DRONE_LANDED)

    def land_emergency(self):
        self._drone_landing_timer.stop()
        result = False
        while result == False:
            result = self._modules.drone_controller.try_land_emergency()
        return True

    def land_normally(self):
        self._drone_landing_timer.start()
        self._modules.drone_controller.try_land_normally()
        return True

    def send_drone_state_do_kss(self):
        state = self._state_collector.drone_state
        self._modules.http_client_controller.send_drone_state(
            state.drone_lattitude,
            state.drone_longitude,
            state.drone_altitude,
            state.drone_battery_voltage,
            state.drone_battery_temperature
        )

    def notify_about_com_is_initialized(self):
        self._modules.http_client_controller.send_drone_event(
            ComEvent.COM_INITIALIZED)

    def start_sending_drone_state_to_kss(self):
        self._drone_state_timer.start()

    def run(self) -> None:
        self._modules.init_modules()
        print('Modules initilized')
        self._drone_rotation_follower.start_following()
        self._drone_position_follower.start_following()
        print('The drone following started')
        self.start_sending_drone_state_to_kss()
        print('Enabled drone state sending to kss')
        self.notify_about_com_is_initialized()
        print('Notified com about initialization complete')

        self._kss_server.run_server()


@inject
def init_container(parametes_updater: ParametersConfigurator = Provide[GlobalContainer.parametes_updater]):
    ...


if __name__ == "__main__":
    rospy.init_node('core')
    container = GlobalContainer()
    container.reset_singletons()
    container.wire(modules=[__name__])
    init_container()

    th = Thread(target=lambda: rospy.init_node(
        'http_core', anonymous=True, disable_signals=True))
    th.run()
    http_core = HttpCoreModule()
    sigint_handler = SigIntHandler()
    http_core.run()
