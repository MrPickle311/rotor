#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import PoseStamped
import os
from nav_msgs.msg import Odometry
Position_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
def waypoint_clear_client():
    try:
        setMode('RTL')
        response = rospy.ServiceProxy('mavros/mission/clear', mavros_msgs.srv.WaypointClear)
        print('Usunieto poprzednia trajektorie: ' +str(response.call().success))
    except rospy.ServiceException as exc:
        print("Service call failed: %s" % exc)

def create_waypoint():
    # http://wiki.ros.org/mavros/CustomModes for custom modes
    #ROS_Waypoints = mavros_msgs.msg.WaypointList()
    setMode('GUIDED')
    ROS_Waypoints = []
    #with open("trajektoria.waypoints", mode="r", encoding="utf-8") as Trajectory_Mission_Planner:
    with open("/home/oze/trajektoria.waypoints", mode="r", encoding="utf-8") as Trajectory_Mission_Planner:
        next(Trajectory_Mission_Planner)
        for Point in Trajectory_Mission_Planner:
            Point = Point.split()
            waypoint = mavros_msgs.msg.Waypoint()
            try:
                waypoint = mavros_msgs.msg.Waypoint(
                    is_current=bool(int(Point[1])),
                    frame=int(Point[2]),
                    command=int(Point[3]),
                    param1=float(Point[4]),
                    param2=float(Point[5]),
                    param3=float(Point[6]),
                    param4=float(Point[7]),
                    x_lat=float(Point[8]),
                    y_long=float(Point[9]),
                    z_alt=float(Point[10]),
                    autocontinue=bool(int(Point[11]))
                )
                ROS_Waypoints.append(waypoint)
                print(waypoint)
            except:
                print("Padlo przy zapisywaniu punktow")

    try:
        service = rospy.ServiceProxy('mavros/mission/push', mavros_msgs.srv.WaypointPush)
        if(service.call(start_index=0,waypoints=ROS_Waypoints).success):
            print('Misja zostala wgrana')
        else:
            print("Misja nie zostala wgrana")
    except rospy.ServiceException as exc:
        print("Service call failed: %s" + str(exc))
        print("Misia nie zostala wgrana")

def sprawdzenieWaypoints():
    setMode('GUIDED')
    try:
        service = rospy.ServiceProxy('mavros/mission/pull', mavros_msgs.srv.WaypointPull)
        if service.call().success:
            print('Misja zostala wgrana wgrano '+str(service.call().wp_received)+" waypointow")
        else:
            print('Misja nie zostala odczytana')
    except rospy.ServiceException:
        print("Service call failed: %s" )

def setMode(Tryb : str="GUIDED"):
    rospy.wait_for_service('mavros/set_mode')
    try:
        FlightMode = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = FlightMode(custom_mode=Tryb)
        print("Udalo sie zmienic tryb na "+Tryb)
    except rospy.ServiceException as e:
        print("Changing flight mode failed" % e)

def DroneLand():
    rospy.wait_for_service('mavros/cmd/land')
    print("Wchodze do ladowania")
    try:
        Landing = rospy.ServiceProxy('mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        Result = Landing(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException as e:
        print("Landing failed " % e)

def setArm(ARM_OR_NOT:bool=False):
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        Arming = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        Arming(ARM_OR_NOT)
    except rospy.ServiceException as e:
        print("Arming/disarming failed" % e)

def TakeOff():
    rospy.wait_for_service('mavros/cmd/takeoff')
    try:
        TakingOff = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
        TakingOff(altitude=3, latitude=0, longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException as e:
        print("Service takeoff call failed: %s" % e)


def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    global altitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
    altitude = globalPositionCallback.altitude-145.36
    # print ("longitude: %.7f" %longitude)
    # print ("latitude: %.7f" %latitude)

def PoseCallback(PoseCallback):
    global current_x
    global current_y
    global current_z
    global current_rx
    global current_ry
    global current_rz
    global current_rw
    current_x = PoseCallback.pose.position.x
    current_y = PoseCallback.pose.position.y
    current_z = PoseCallback.pose.position.z
    current_rx = PoseCallback.pose.orientation.x
    current_ry = PoseCallback.pose.orientation.y
    current_rz = PoseCallback.pose.orientation.z
    current_rw = PoseCallback.pose.orientation.w

def BatteryCallback(Battery):
    global voltage
    global current
    global temperature
    voltage = Battery.voltage
    current = Battery.current
    temperature = Battery.temperature
    # print ("longitude: %.7f" %longitude)
    # print ("latitude: %.7f" %latitude)
def WeSieRusz(x,y,z):
    rospy.wait_for_service('mavros/navigate_global')
    navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)

def menu():
    print("Press")
    print("1 - Ustaw mode na guided wgraj waypointsy")
    print("2 - Sprawdz waypointsy")
    print("3 - Wybor trybu")
    print("4 - Arm")
    print("5 - GPS")
    print("6 - Bateria")
    print("7 - Take off")
    print("8 - Land")
    print("9 - start")

def myLoop():

    while (not rospy.is_shutdown()) :
        menu()
        x = input("Enter your input: ");
        if (x == '1'):
            try:
                create_waypoint()
            except:
                print("Nie udalo sie wczytac punktow")
        elif (x == '2'):
            sprawdzenieWaypoints()
        elif (x == '3'):
            WantedMode=input("Wpisz tryb pracy z duzych literek")
            setMode(WantedMode)
        elif (x == '4'):
            Uzbrojenie=input("Czy uzbroic drona")
            setArm(bool(Uzbrojenie))
        elif (x == '5'):
            print("longitude: %.7f" % longitude)
            print("latitude: %.7f" % latitude)
            print("altitude %.7f" %altitude)
        elif (x=='6'):
            print("Pradzik: %.7f" % current)
            print("Napiecie: %.7f" % voltage)
            print("Temperaturka: %.7f" % temperature)
        elif (x=='7'):
            TakeOff()
        elif (x=='8'):
            DroneLand()
        elif (x == '9'):
            setMode("GUIDED")
            setArm(True)
            TakeOff()
        elif(x=='10'):
            value_x = input("Wpisz o ile x")
            value_y = input("Wpisz o ile y")
            value_z = input("Wpisz o ile z")
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "8"
            goal_pose.header.seq = 1
            goal_pose.pose.position.x = current_x + float(value_x)
            goal_pose.pose.position.y = current_y + float(value_y)
            goal_pose.pose.position.z = current_z + float(value_z)
            for i in range(100):
                Position_pub.publish(goal_pose)
            rospy.loginfo(goal_pose)
        elif(x=='11'):
            waypoint_clear_client()



if __name__ == '__main__':
    rospy.init_node('dronemap_node', anonymous=True)
    rospy.Subscriber("mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    rospy.Subscriber("mavros/battery", sensor_msgs.msg.BatteryState, BatteryCallback)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped,PoseCallback)
    # spin() simply keeps python from exiting until this node is stopped

    # listener()
    myLoop()
    # rospy.spin()