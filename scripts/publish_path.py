#!/usr/bin/env python

import rospy
import pickle
import sys
import copy

from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl

from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarkerFeedback
from interactive_markers.menu_handler import MenuHandler

def publish_path(pathpoints):
    path = Path()
    path.header.frame_id = 'path'

    for x, y in pathpoints:
        pose = PoseStamped()
        pose.header.frame_id = 'path'
        pose.pose.position = Point(x, y, 0)
        path.poses.append(pose)

    path_pub.publish(path)

def get_waypoint_id(marker_name):
    return int(marker_name[9:])

def remove_marker(feedback):
    waypoint_id = get_waypoint_id(feedback.marker_name)
    pathpoints.pop(waypoint_id)

    publish_markers(pathpoints)
    publish_path(pathpoints)

def insert_before(feedback):
    waypoint_id = get_waypoint_id(feedback.marker_name)

    x = (pathpoints[waypoint_id][0] + pathpoints[waypoint_id - 1][0]) * 0.5
    y = (pathpoints[waypoint_id][1] + pathpoints[waypoint_id - 1][1]) * 0.5

    pathpoints.insert(waypoint_id, (x, y))
    publish_markers(pathpoints)
    publish_path(pathpoints)

def insert_after(feedback):
    waypoint_id = get_waypoint_id(feedback.marker_name)

    x = (pathpoints[waypoint_id][0] + pathpoints[waypoint_id + 1][0]) * 0.5
    y = (pathpoints[waypoint_id][1] + pathpoints[waypoint_id + 1][1]) * 0.5

    pathpoints.insert(waypoint_id + 1, (x, y))
    publish_markers(pathpoints)
    publish_path(pathpoints)

def move_waypoint(feedback):
    waypoint_id = get_waypoint_id(feedback.marker_name)
    pathpoints[waypoint_id] = (feedback.pose.position.x, feedback.pose.position.y)

    publish_path(pathpoints)

def make_sphere(msg):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale
    marker.scale.y = msg.scale
    marker.scale.z = msg.scale
    marker.color.r = 1.0
    marker.color.g = 0.3
    marker.color.b = 0.0
    marker.color.a = 1.0

    return marker

def make_waypoint_marker(id, x, y):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'path'
    int_marker.pose.position = Point(x, y, 0)
    int_marker.scale = 0.1

    int_marker.name = 'waypoint_' + str(id)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker.controls.append(copy.deepcopy(control))

    control.markers.append( make_sphere(int_marker) )
    control.always_visible = True
    int_marker.controls.append(control)

    return int_marker

def publish_markers(pathpoints):
    marker_server.clear()

    for i in range(len(pathpoints)):
        marker = make_waypoint_marker(i, pathpoints[i][0], pathpoints[i][1])
        marker_server.insert(marker, move_waypoint)
        menu_handler.apply(marker_server, marker.name)

    marker_server.applyChanges()

if __name__ == '__main__':
    rospy.init_node('path_server')

    global marker_server, menu_handler
    marker_server = InteractiveMarkerServer('path_server')
    menu_handler = MenuHandler()

    menu_handler.insert('Remove waypoint', callback=remove_marker)

    insert_submenu = menu_handler.insert('Insert waypoint')
    menu_handler.insert('Before current', parent=insert_submenu, callback=insert_before)
    menu_handler.insert('After current',  parent=insert_submenu, callback=insert_after)

    # Read input file
    filename = sys.argv[1]
    with open(filename, 'rb') as f:
        map_data = pickle.load(f)
    pathpoints = map_data['pathpoints']

    global path_pub
    path_pub = rospy.Publisher('path', Path, latch=True, queue_size=1)

    publish_path(pathpoints)
    publish_markers(pathpoints)

    rospy.spin()