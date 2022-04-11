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
from enums.com_client import DroneEventEnum


class DronePositionIndicator:
    DRONE_STARTED = 'drone_started'
    DRONE_IS_FAR = 'drone_is_far'
    DRONE_APPROACHING = 'drone_approaching'


class DroneDistanceFollower:
    MIRANDA_LATT = 52.2205449
    MIRANDA_LONG = 21.0060927
    LANDING_RADIUS = 50  # meters
    FOLLOW_INTERVAL = 20 # seconds
    EARTH_RADIUS = 6378.137  # kilometers

    def __init__(self, modules: ComModules, com_collector: ComStateCollector):
        self._modules = modules
        self._com_collector = com_collector
        self._drone_follow_timer = MultiTimer(self.FOLLOW_INTERVAL, self.check_drone_position,
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

    def set_drone_is_approaching(self):
        self._drone_position_indicator = DronePositionIndicator.DRONE_APPROACHING

    @staticmethod
    def meters(kilometers):
        return kilometers * 1000

    # https://en.wikipedia.org/wiki/Haversine_formula
    def get_drone_distance_from_miranda(self, drone_latt: float, drone_long: float) -> float:
        d_latt = (self.MIRANDA_LATT - drone_latt) * math.pi / 180
        d_long = (self.MIRANDA_LONG - drone_long) * math.pi / 180
        a = math.sin(d_latt / 2) * math.sin(d_latt) + \
            math.cos(self.MIRANDA_LATT * math.pi / 180) * math.cos(drone_latt * math.pi / 180) * \
            math.sin(d_long / 2) * math.sin(d_long / 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = self.EARTH_RADIUS * c
        return self.meters(d)

    def check_drone_position(self):
        current_distance = self.get_drone_distance_from_miranda(
            self._com_collector.drone_state.drone_lattitude,
            self._com_collector.drone_state.drone_longitude)

        if current_distance > self.LANDING_RADIUS and self.drone_started():
            self.set_drone_is_far()
            return

        if current_distance < self.LANDING_RADIUS and self.drone_is_far():
            self._modules.http_client_controller.send_drone_event(
                DroneEventEnum.DRONE_IS_READY_TO_LAND)
            self.set_drone_is_approaching()
            return


class DroneRotationFollower:
    MIRANDA_LATT = 52.2205449
    MIRANDA_LONG = 21.0060927
    FOLLOW_INTERVAL = 1

    def __init__(self, modules: ComModules, com_collector: ComStateCollector):
        self._modules = modules
        self._com_collector = com_collector
        self._drone_follow_timer = MultiTimer(self.FOLLOW_INTERVAL, self.correct_position,
                                              runonstart=False)

    def start_following(self):
        self._drone_follow_timer.start()

    def stop_following(self):
        self._drone_follow_timer.stop()

    def get_drone_distance_from_miranda(self, latt: float, long: float) -> float:
        return math.sqrt((latt - self.MIRANDA_LATT) ** 2 + (long - self.MIRANDA_LONG) ** 2)

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
        x = self._com_collector.drone_state.drone_longitude - self.MIRANDA_LONG
        y = self._com_collector.drone_state.drone_lattitude - self.MIRANDA_LATT

        # self.get_absolute_atan(x,y)
        angle = self.alternate(math.atan2(x, y))
        self._modules.rotor_controller.go_to_absolute_pos(angle)


class KSSEventNotifiers:
    def __init__(self, com_server_api: COMServerAPI, upload_mission_task: Callable[[str], bool]):
        self._kss_server_api = com_server_api

        self.add_mission_upload_task_event(upload_mission_task)

    def add_mission_upload_task_event(self, task: Callable[[str], bool]):
        self._kss_server_api.add_api_path_handler(
            '/com', '/kss_event/upload_mission', MissionReceiver(task))

    def add_task_event(self, task_path: str, task: Callable[[], bool]):
        # TODO: add abort mission
        # simple_kss_task_events = ['abort_drone']
        self._kss_server_api.add_api_path_handler(
            '/com', f'/kss_event/{task_path}', KSSTaskReceiver(task))


class HttpCoreModule:
    DRONE_STATE_UPDATE_INTERVAL = 1

    def __init__(self):
        self._kss_server = COMServerAPI('com_server', 4000, '192.168.1.16')
        self._state_collector = ComStateCollector()
        self._modules = ComModules(self._state_collector)
        drone_com_module = self._modules.drone_controller
        self._drone_follower = DroneRotationFollower(
            self._modules, self._state_collector)

        self._drone_state_timer = MultiTimer(self.DRONE_STATE_UPDATE_INTERVAL, self.send_drone_state_do_kss,
                                             runonstart=False)

        def try_start_and_upload_mission(mission_name) -> bool:
            is_ok = False
            is_ok = drone_com_module.try_upload_mission(mission_name)
            time.sleep(1)
            is_ok = drone_com_module.try_drone_start()

            if is_ok:
                print('Everything is sent ok')

            return is_ok

        self._event_notifiers = KSSEventNotifiers(
            self._kss_server, try_start_and_upload_mission)
        self._event_notifiers.add_task_event(
            'land_drone_normally', drone_com_module.try_land_normally)
        self._event_notifiers.add_task_event(
            'land_drone_emergency', drone_com_module.try_land_emergency)
        self._event_notifiers.add_task_event(
            'abort_drone', drone_com_module.try_return_drone)

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
        self._modules.http_client_controller.send_drone_event(DroneEventEnum.COM_INITIALIZED)

    def start_sending_drone_state_to_kss(self):
        self._drone_state_timer.start()

    def run(self) -> None:
        self._modules.init_modules()
        print('Modules initilized')
        self._drone_follower.start_following()
        print('The drone following started')
        self.start_sending_drone_state_to_kss()
        print('Enabled drone state sending to kss')
        self.notify_about_com_is_initialized()
        print('Notified com about initialization complete')

        self._kss_server.run_server()


if __name__ == "__main__":
    th = Thread(target=lambda: rospy.init_node(
        'http_core', anonymous=True, disable_signals=True))
    th.run()
    http_core = HttpCoreModule()
    sigint_handler = SigIntHandler()
    http_core.run()
