import rospy

from mavros_msgs.srv import WaypointPush
from mavros_msgs.srv import WaypointPushRequest
from mavros_msgs.srv import WaypointSetCurrent
from mavros_msgs.srv import WaypointSetCurrentRequest
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool

#
#+---+------+------+---------+---------------+----------------+-------------------+------+-----+---------------+---------------+-------+
#| # | Curr | Auto |  Frame  |    Command    |       P1       |         P2        |  P3  |  P4 |     X Lat     |     Y Long    | Z Alt |
#+---+------+------+---------+---------------+----------------+-------------------+------+-----+---------------+---------------+-------+
#| 0 | Yes  | Yes  | GRA (3) |  TAKEOFF (22) | 0.261799395084 | 2.30242081791e-38 | -0.0 | 0.0 | 47.3978614807 | 8.54560279846 |  25.0 |
#| 1 |  No  | Yes  | GRA (3) | WAYPOINT (16) |      0.0       |        3.0        | -0.0 | 0.0 | 47.3978538513 | 8.54586410522 |  11.0 |
#| 2 |  No  | Yes  | GRA (3) | WAYPOINT (16) |      0.0       |        3.0        | -0.0 | 0.0 | 47.3977279663 | 8.54585456848 |  11.0 |
#| 3 |  No  | Yes  | GRA (3) | WAYPOINT (16) |      0.0       |        3.0        | -0.0 | 0.0 | 47.3977394104 | 8.54559516907 |  11.0 |
#| 4 |  No  | Yes  | GRA (3) |   LAND (21)   |      25.0      |        3.0        | -0.0 | 0.0 | 47.3977432251 | 8.54559516907 |  0.0  |
#+---+------+------+---------+---------------+----------------+-------------------+------+-----+---------------+---------------+-------+
#


def createTakeoffCurr(_lat, _lon, _alt):
    # 22	MAV_CMD_NAV_TAKEOFF	Takeoff from ground / hand
    # Mission Param #1	Minimum pitch (if airspeed sensor present), desired pitch without sensor
    # Mission Param #2	Empty
    # Mission Param #3	Empty
    # Mission Param #4	Yaw angle (if magnetometer present), ignored without magnetometer
    # Mission Param #5	Latitude
    # Mission Param #6	Longitude
    # Mission Param #7	Altitude
    wp = Waypoint()
    wp.frame = 3
    wp.command = 22
    wp.autocontinue = True
    wp.param1 = 0.261799395084
    wp.param2 = 0.0
    wp.param3 = 0.0
    wp.param4 = 0.0
    wp.x_lat = _lat
    wp.y_long = _lon
    wp.z_alt = _alt
    return wp

def createWaypoint(_visitationRadius, _lat, _lon, _alt):
    wp = Waypoint()
    wp.frame = 3
    wp.command = 16
    wp.autocontinue = True
    wp.param1 = 0
    wp.param2 = _visitationRadius
    wp.param3 = 0
    wp.param4 = 0
    wp.x_lat = _lat
    wp.y_long = _lon
    wp.z_alt = _alt
    return wp

def createLand(_lat, _lon,_alt):
    wp = Waypoint()
    wp.frame = 3
    wp.command = 21
    wp.autocontinue = True
    wp.param1 = 25.0
    wp.param2 = 3.0
    wp.param3 = -0.0
    wp.param4 = 0.0
    wp.x_lat = _lat
    wp.y_long = _lon
    wp.z_alt = _alt
    return wp



if __name__ == "__main__":
    rospy.wait_for_service('/mavros/mission/push')
    rospy.wait_for_service('/mavros/set_mode')

    waypointPushService = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
    wpPushRequest = WaypointPushRequest()
    wpPushRequest.waypoints.append(createTakeoffCurr(47.3978614807, 8.54560279846,11.0))
    wpPushRequest.waypoints.append(createWaypoint(5.0,47.3978538513,8.54586410522,11.0))
    wpPushRequest.waypoints.append(createWaypoint(5.0,47.3977279663,8.54585456848,11.0)) 
    wpPushRequest.waypoints.append(createWaypoint(5.0,47.3977394104,8.54559516907,11.0))
    wpPushRequest.waypoints.append(createLand(47.3977432251,8.54559516907,0.0))
    print(waypointPushService.call(wpPushRequest))

    rospy.sleep(1)
    arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    print(arm(True))
    
    rospy.sleep(1)
    setmode = rospy.ServiceProxy('mavros/set_mode', SetMode)
    print(setmode(0, 'AUTO.MISSION'))

