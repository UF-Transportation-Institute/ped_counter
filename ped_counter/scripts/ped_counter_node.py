#!/usr/bin/env python

## Pedestrian counter that listens to geometry_msgs::PoseArray published
## to the 'poses' topic
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geodesy.utm import *

from collections import namedtuple
import time


class Stat:

    def __init__(self, id, polygon):
        self.id = id
        self.polygon = polygon
        self.count = 0


class PedCounter:

    def callback(self, data):
        for pos in data.poses:
            self.update_stats(self.get_offset(pos.position))

        self.process_stats()

    def get_offset(self, point):
        return Geom.Point(self.detector_origin.x + point.x, self.detector_origin.y + point.y)

    def process_stats(self):
        ts = time.time()

        # 8:2:7.100581^^^^1,{1:0,2:1,3:0,4:0,5:3,6:0}     Format of a message
        stats = ['%s:%s' % (s.id,s.count) for s in self.stats]
        mssg = '%s^^^^1,{%s}' % (ts, ', '.join(stats))

        print(mssg)
        # publish for out of band processing
        self.pub.publish(mssg)

        # reset counters
        for stat_zone in self.stats:
            stat_zone.count = 0

    def update_stats(self, point):
        for stat_zone in self.stats:
            if Geom.point_in_polygon(point, stat_zone.polygon):
                stat_zone.count = stat_zone.count + 1

    def __init__(self):
        self.pub = rospy.Publisher('ped_counter/counts', String, queue_size=30)
        rospy.Subscriber('ped_detector/poses', PoseArray, self.callback)

        # init lidar origin
        x, y = rospy.get_param("ped_counter_node/detector_origin")
        det_utm = fromLatLong(x, y)
        self.detector_origin = Geom.Point(det_utm.easting, det_utm.northing)

        # init detection zones
        detection_zone = rospy.get_param("ped_counter_node/detection_zone")
        self.stats = []
        for zone in detection_zone:
            polygon = [Geom.Point(u.easting, u.northing) for u in [fromLatLong(x, y) for x, y in zone['coords']]]
            self.stats.append(Stat(zone['id'], polygon))


class Geom:
    Point = namedtuple('Point', ['x', 'y'])

    @staticmethod
    def on_segment(col_point_a, col_point_b, col_point_c):
        """
        Given three, 2-dimensional collinear points col_point_a,col_point_b,col_point_c, checks if
        point col_point_b lines on line segment 'ac'
        """
        if col_point_b.x <= max(col_point_a.x, col_point_c.x) and \
                col_point_b.x >= min(col_point_a.x, col_point_c.x) and \
                col_point_b.y <= max(col_point_a.y, col_point_c.y) and \
                col_point_b.y >= min(col_point_a.y, col_point_c.y):
            return True
        else:
            return False

    @staticmethod
    def orientation(point_a, point_b, point_c):
        """
        Finds orientation of ordered triplet of 2-dimensional points (point_a, point_b, point_c)
        0 --> (a, b, c) are colinear
        1 --> Clockwise
        2 --> Counterclockwise
        """
        val = (point_b.y - point_a.y) * (point_c.x - point_b.x) - \
              (point_b.x - point_a.x) * (point_c.y - point_b.y)
        if val == 0:
            return 0
        else:
            if val > 0:
                return 1
            else:
                return 2

    @staticmethod
    def do_intersect(point_a_l1, point_b_l1, point_a_l2, point_b_l2):
        """
        Checks if two given lines intersect.

        https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

        :return: True if line segment p1q1 and p2q2 intersect.
        """
        ori_1 = Geom.orientation(point_a_l1, point_b_l1, point_a_l2)
        ori_2 = Geom.orientation(point_a_l1, point_b_l1, point_b_l2)
        ori_3 = Geom.orientation(point_a_l2, point_b_l2, point_a_l1)
        ori_4 = Geom.orientation(point_a_l2, point_b_l2, point_b_l1)

        # General case
        if ori_1 != ori_2 and ori_3 != ori_4:
            return True

        # Special Cases
        # p1, q1, and p2 are colinear and p2 lies on segment p1q1
        if ori_1 == 0 and Geom.on_segment(point_a_l1, point_a_l2, point_b_l1):
            return True

        # p1, q1, and p2 are colinear and q2 lies on segment p1q1
        if ori_2 == 0 and Geom.on_segment(point_a_l1, point_b_l2, point_b_l1):
            return True

        # p2, q2, and p1 are colinear and p1 lies on segment p2q2
        if ori_3 == 0 and Geom.on_segment(point_a_l2, point_a_l1, point_b_l2):
            return True

        # p2, q2, and q1 are colinear and q1 lies on segment p2q2
        if ori_4 == 0 and Geom.on_segment(point_a_l2, point_b_l1, point_b_l2):
            return True

        return False

    @staticmethod
    # evaluate if a point is inside a polygon using the previous functions
    def point_in_polygon(query, polygon):
        """
        Fast algorithm for checking whether a query point
        is contained inside a polygon
        """
        # Must be at least 3 points in polygon
        if len(polygon) < 3:
            return False
        polygon_length = len(polygon)

        # Create a point for line segment from p to infinity
        extreme = Geom.Point(1000000, query.y)

        count = 0
        for i in range(1, polygon_length):
            # Check if the line segment from query to extreme
            # intersects with the line segment from polygon[i-1] to
            # polygon[i]
            if Geom.do_intersect(polygon[i - 1], polygon[i], query, extreme):
                # If the point query is colinear with line segment
                # polygon[i-1]-polygon[i]
                if Geom.orientation(polygon[i - 1], query, polygon[i]) == 0:
                    return Geom.on_segment(polygon[i - 1], query, polygon[i])
                count += 1
        return count % 2 == 1


if __name__ == '__main__':
    rospy.init_node('ped_counter_node', anonymous=True)

    counter = PedCounter()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
