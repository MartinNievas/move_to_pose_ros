from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def generate_markerarray_message(self, path_x, path_y, type="global"):
    marker_array = MarkerArray()
    self.get_logger().info('marker size: "%s"' % len(path_x))
    for i in range(len(path_x[:-1])):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = i
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.pose.position.x = path_x[i]
        marker.pose.position.y = path_y[i]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        if type == "local":
            marker.scale.x = 0.03
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif type == "global":
            marker.scale.x = 0.01
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        marker.color.a = 1.0

        # marker line points
        marker.points = []

        # first point
        first_line_point = Point()
        first_line_point.x = path_x[i]
        first_line_point.y = path_y[i]
        first_line_point.z = 0.0
        marker.points.append(first_line_point)
        # second point
        second_line_point = Point()
        second_line_point.x = path_x[i+1]
        second_line_point.y = path_y[i+1]
        second_line_point.z = 0.0
        marker.points.append(second_line_point)

        marker_array.markers.append(marker)
    return marker_array