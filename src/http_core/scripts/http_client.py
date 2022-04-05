#!/usr/bin/env python3
from client.com_client_senders import DroneStateSender, DroneEventSender ,DroneEventEnum
from typing import List, Dict, Optional
from module_io.module_interface import AbstractModule
from module_utils.signal_handlers import SigIntHandler
from commons.msg import DroneState, DroneEvent
from commons.msg import HttpClientProcessAction, HttpClientProcessGoal, HttpClientProcessResult, \
    HttpClientProcessFeedback
import rospy


class HttpClientModule(AbstractModule):
    def __init__(self, module_name: str):
        AbstractModule.__init__(self, module_name, HttpClientProcessAction)
        self._err_sender: Optional[DroneStateSender] = None
        self.add_signal_receiver('/send_drone_state', DroneState, self.send_station_status)
        self.add_signal_receiver('/drone_event', DroneEvent, self.send_drone_event)

    def _init_senders(self, ip_address: str, port: int, station_id: int):
        self._state_sender = DroneStateSender(ip_address, port, station_id)
        self._event_sender = DroneEventSender(ip_address, port, station_id)

    def start_module(self, start_args: HttpClientProcessGoal) -> None:
        self._init_senders(start_args.host_address, start_args.port, start_args.station_id)

        self.wait_for_module_finish()

        self.finish_module(0)

    def send_feedback(self, feedback: HttpClientProcessFeedback) -> None:
        self._server.publish_feedback(feedback)

    def finish_module(self, exit_code: int) -> None:
        result = HttpClientProcessResult()
        result.exit_code = exit_code
        self._server.set_succeeded(result)

    def send_station_status(self, status: DroneState) -> None:
        self._state_sender.send_simple_message(status.latt, status.long, status.alt, status.voltage,
                                               status.temperature)

    def send_drone_event(self, event: str):
        self._event_sender.send_simple_message(event)


def main():
    rospy.init_node('http_client', anonymous=True)
    sigint_handler = SigIntHandler()
    http_client_process = HttpClientModule('http_client')
    rospy.spin()


if __name__ == '__main__':
    main()
