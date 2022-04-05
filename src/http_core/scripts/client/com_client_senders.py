import datetime
from http_io.simple_http_sender import SimpleMessage, SimpleHttpSender
from enums.com_client import DroneEventEnum
from data_models.com_client import DroneStateMessage, DroneEventModel


class DroneStateSender(SimpleHttpSender):
    MESSAGE_TYPE = 'drone_state'

    def _create_message(self, lattitude: float, longitude: float, altitude: float, battery_voltage: float,
                        battery_temp: float) -> DroneStateMessage:
        msg = DroneStateMessage(
        lattitude = lattitude,
        longitude = longitude,
        altitude = altitude,
        battery_volt = battery_voltage,
        battery_temp = battery_temp)
        return msg

    def __init__(self, host_address: str, port, station_id: int):
        SimpleHttpSender.__init__(self, host_address, port, station_id, self.MESSAGE_TYPE, 'com' )


class DroneEventSender(SimpleHttpSender):
    MESSAGE_TYPE = 'drone_event'

    def _create_message(self, incoming_event: str) -> DroneEventModel:
        msg = DroneEventModel(
                event=incoming_event.event)
        return msg

    def __init__(self, host_address: str, port, station_id: int):
        SimpleHttpSender.__init__(self, host_address, port, station_id, self.MESSAGE_TYPE, 'com' )
