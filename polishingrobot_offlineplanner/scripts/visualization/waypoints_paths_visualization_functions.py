from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
import rospy

def path1_visualization(waypoints,frame,num1):
    marker1 = Marker()
    marker1.header.frame_id = frame
    marker1.type = Marker.LINE_STRIP
    marker1.action = Marker.ADD
    marker1.pose.orientation.w = 1.0
    marker1.scale.x = 0.02
    marker1.ns = 'paths and waypoints'
    marker1.id = num1
    marker1.color.r = 0.1
    marker1.color.g = 0.1
    marker1.color.b = 0.1
    marker1.color.a = 1.0
    marker1.lifetime = rospy.Duration()
    for i in range(len(waypoints)):
        pathpoints=Point()
        pathpoints.x = waypoints[i][0]
        pathpoints.y = waypoints[i][1]
        pathpoints.z = waypoints[i][2]
        marker1.points.append(pathpoints)
        # marker2.points.append(pathpoints)
    return marker1,num1

def path2_visualization(waypoints,frame,num1):
    marker1 = Marker()
    marker1.header.frame_id = frame
    marker1.type = Marker.LINE_STRIP
    marker1.action = Marker.ADD
    marker1.pose.orientation.w = 1.0
    marker1.scale.x = 0.02
    marker1.ns = 'paths'
    marker1.id = num1
    marker1.color.r = 0.8
    marker1.color.g = 0.1
    marker1.color.b = 0.1
    marker1.color.a = 1.0
    marker1.lifetime = rospy.Duration()

    pathpoints1=Point()
    pathpoints1.x = waypoints[0]
    pathpoints1.y = waypoints[1]
    pathpoints1.z = waypoints[2]
    marker1.points.append(pathpoints1)

    pathpoints2 = Point()
    pathpoints2.x = waypoints[3]
    pathpoints2.y = waypoints[4]
    pathpoints2.z = waypoints[5]
    marker1.points.append(pathpoints2)

    return marker1, num1


def targetpositions_visualization(waypoint, frame, num, scale, color):
    marker1 = Marker()
    marker1.header.frame_id = frame
    marker1.type = Marker.CUBE
    marker1.action = Marker.ADD

    marker1.scale.x = 2*scale[0]
    marker1.scale.y = scale[1]
    marker1.scale.z = scale[2]
    marker1.ns = 'targeposition'
    marker1.id = num
    marker1.color.r = color[0]
    marker1.color.g = color[1]
    marker1.color.b = color[2]
    marker1.color.a = 1.0
    marker1.lifetime = rospy.Duration()

    marker1.pose.position.x = waypoint[0]
    marker1.pose.position.y = waypoint[1]
    marker1.pose.position.z = waypoint[2]
    marker1.pose.orientation.x = waypoint[3]
    marker1.pose.orientation.y = waypoint[4]
    marker1.pose.orientation.z = waypoint[5]
    marker1.pose.orientation.w = waypoint[6]


    return marker1, num
