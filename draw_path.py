#!/usr/bin/env python
import rospy
import tf
import math
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class draw_path():
    def __init__(self):
        self.listener = tf.TransformListener()
        self.root_frame = "map"
        self.drawed_frame = "base_link_ekf"
        self.tf_sampling_freq = 20.0  # Hz
        self.listener = tf.TransformListener()
        self.publisher = rospy.Publisher("robot_path", MarkerArray)
        self.count = 0

        #rospy.Timer(rospy.Duration.from_sec(1 / self.tf_sampling_freq), self.draw_path())

    def draw_path(self):
        try:
            self.listener.waitForTransform(self.root_frame,
                                           self.drawed_frame,
                                           rospy.Time(0),
                                           rospy.Duration.from_sec(1 / (2*self.tf_sampling_freq)))
            (trans, rot) = self.listener.lookupTransform(self.root_frame, self.drawed_frame, rospy.Time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logwarn(e)
            pass
        else:
            self.pub_marker_array(trans)
            self.count += 1

    def pub_marker_array(self, trans):
        markerArray = MarkerArray()

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = trans[0]
        marker.pose.position.y = trans[1]
        marker.pose.position.z = 0
        marker.id = self.count

        # We add the new marker to the MarkerArray
        markerArray.markers.append(marker)

        # Publish the MarkerArray
        self.publisher.publish(markerArray)

    def run(self):
        while not rospy.is_shutdown():
            rate = rospy.Rate(self.tf_sampling_freq)
            while not rospy.is_shutdown():
                # signal.signal(signal.SIGINT, self.signal_handler)
                self.draw_path()
                rate.sleep()

if __name__ == '__main__':
    rospy.init_node("draw_hannesrobo_path")
    dp = draw_path()
    dp.run()
