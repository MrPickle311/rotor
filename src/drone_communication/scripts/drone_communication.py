import rospy
from std_msgs.msg import String
from sensor_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import PoseStamped
from module_io.module_interface import AbstractModule
from multitimer import MultiTimer
from threading import Lock, Condition
import os
from module_utils.signal_handlers import SigIntHandler
from commons.msg import DroneCommunicationProcessGoal, DroneCommunicationProcessAction, DroneCommunicationProcessResult, \
    DroneCommunicationProcessFeedback
from commons.srv import TryDroneStart, TryDroneStartRequest, TryDroneStartResponse
from commons.srv import TryUploadMission, TryUploadMissionRequest, TryUploadMissionResponse
from commons.srv import DroneReturn, DroneReturnRequest, DroneReturnResponse
from commons.srv import TryLand, TryLandRequest, TryLandResponse
from time import sleep

class BatteryData:
    def __init__(self):
        self._voltage = 0.0
        self._current = 0.0
        self._temperature = 0.0


class GPSData:
    def __init__(self):
        self._latitude = 0.0
        self._longitude = 0.0
        self._altitude = 0.0


class DroneCommunicationModule(AbstractModule):
    LEVEL_OVER_SEA = 145.36

    def __init__(self, module_name: str):
        AbstractModule.__init__(self, module_name, DroneCommunicationProcessAction)

        def gps_callback_closure(gps):
            self.gps_callback(gps)

        rospy.Subscriber("mavros/global_position/raw/fix", NavSatFix, gps_callback_closure)

        def battery_callback_closure(battery):
            self.battery_callback(battery)

        rospy.Subscriber("mavros/battery", sensor_msgs.msg.BatteryState, battery_callback_closure)

        self._currentFeedback = DroneCommunicationProcessFeedback()

        def feedback_callback_closure(feedback):
            self.send_feedback(feedback)

        self._timer = MultiTimer(0.5, feedback_callback_closure, kwargs={'feedback': self._currentFeedback})

        self.add_task_processor('/start_drone', TryDroneStart, self.service_drone_start)
        self.add_task_processor('/send_mission', TryUploadMission, self.service_send_mission)
        self.add_task_processor('/land_drone', TryLand, self.service_land)
        self.add_task_processor('/drone_return', DroneReturn, self.service_return)

    def service_return(self,start_arg: DroneReturn):
        try:
            # self.setMode("GUIDED")
            self.setMode("RTL")
            return DroneReturnResponse(True)
        except:
            print("Sending return command failed")
            return DroneReturnResponse(False)

    def service_land(self, start_args: TryLandRequest):
        if start_args.mode == 'normally':
            try:
                # self.setMode("GUIDED")
                self.setMode("LAND")
                return TryLandResponse(True)
            except:
                print("Start failed")
                return TryLandResponse(False)
        elif start_args.mode == 'emergency':
            try:
                self.setMode("ALT_HOLD")
                sleep(1)
                self.setMode("GUIDED")
                return TryLandResponse(True)
            except:
                return TryLandResponse(False)


    def service_send_mission(self, start_args: TryUploadMissionRequest):
        try:
            self.upload_trajectory(start_args.waypoints_path)
            print('Mission uploaded successfully')
            return TryUploadMissionResponse(True)
        except:
            print("Start failed")
            return TryUploadMissionResponse(False)

    def service_drone_start(self, start_args: TryDroneStartRequest):
        try:
            self.setMode("AUTO")
            return TryDroneStartResponse(True)
        except:
            print("Drone started successfully")
            return TryDroneStartResponse(False)

    def _start_timer(self):
        self._timer.start()

    def _stop_timer(self):
        self._timer.stop()

    def battery_callback(self, battery):
        self._currentFeedback.voltage = battery.voltage  # battery.percentage
        self._currentFeedback.temperature = 20.0
        #self._currentFeedback.temperature = battery.cell_voltage  # battery.percentage

    def gps_callback(self, gps_data):
        self._currentFeedback.latt = gps_data.latitude
        self._currentFeedback.long = gps_data.longitude
        self._currentFeedback.alt = gps_data.altitude - DroneCommunicationModule.LEVEL_OVER_SEA

    def start_module(self, start_args: DroneCommunicationProcessGoal) -> None:
        self._start_timer()

        self.wait_for_module_finish()

        self._stop_timer()
        self.finish_module(0)

    def send_feedback(self, feedback: DroneCommunicationProcessFeedback) -> None:
        self._server.publish_feedback(feedback)

    def finish_module(self, exit_code: int) -> None:  # jak normalnie to zwroc 0
        result = DroneCommunicationProcessResult()
        result.exit_code = exit_code
        self._server.set_succeeded(result)

    def setMode(self, MODE: str = "GUIDED"):
        rospy.wait_for_service('mavros/set_mode')
        FlightMode = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = FlightMode(custom_mode=MODE)
        # print(isModeChanged)
        # if isModeChanged == True:
        #     print("Succesfully changed TMotor mode to: " + MODE)
        # else:
        #     print("Encountered problem during changing mode")
        #     raise Exception

    def waypoint_clear_client(self):
        try:
            response = rospy.ServiceProxy('mavros/mission/clear', mavros_msgs.srv.WaypointClear)
            print('Successfully deleted previous trajectory: ' + str(response.call().success))
        except rospy.ServiceException as exc:
            print("Clearing trajectory service call failed: %s" % exc)

    def upload_trajectory(self, path):
        self.setMode("GUIDED")
        try:
            self.waypoint_clear_client()
            print("Previous trajectory deleted")
        except:
            print("Deleting previous trajectory failed")

        ROS_Waypoints = []
        with open(path, mode="r", encoding="utf-8") as Trajectory_Mission_Planner:
            next(Trajectory_Mission_Planner)
            for Point in Trajectory_Mission_Planner:
                Point = Point.split()
                try:
                    print(Point)
                    single_waypoint = mavros_msgs.msg.Waypoint(
                        is_current=bool(int(Point[1])),
                        frame=int(Point[2]),
                        command=int(Point[3]),
                        param1=float(Point[4]),
                        param2=float(Point[5]),
                        param3=float(Point[6]),
                        param4=float(Point[7]),
                        x_lat=float(Point[8]),
                        y_long=float(Point[9]), # TODO: dodaj autocontinue na koniec
                        z_alt=float(Point[10]), #TODO: ZMIENIC KOLEJNOSC WCZYTYWANYCH RZECZY
                        autocontinue=bool(int(Point[11]))
                    )
                    ROS_Waypoints.append(single_waypoint)
                except:
                    print("Encountered problem during creating Waypoints List")
        try:
            service = rospy.ServiceProxy('mavros/mission/push', mavros_msgs.srv.WaypointPush)
            # if service.call(start_index=0, waypoints=ROS_Waypoints).success:
            #     print('Mission uploaded succesfully.')
            # else:
            #     print('Uploading failed try again.')
        except rospy.ServiceException as exc:
            print("Service call failed: %s" + str(exc))


if __name__ == '__main__':
    rospy.init_node('drone_communication', anonymous=True)
    sigint_handler = SigIntHandler()
    module = DroneCommunicationModule('drone_communication')
    # listener()

    rospy.spin()
