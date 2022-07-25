from __future__ import print_function

import time
from threading import Thread

import rospy
import tf2_ros

from geometry_msgs.msg import Pose, PoseStamped

from interactive_markers.interactive_marker_server import (
    InteractiveMarkerServer,
    InteractiveMarkerFeedback,
)
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


class InteractiveMarkerInterface(Thread):
    """ """

    def __init__(self, marker_info):
        """Initialize the InteractiveMarkerInterface.

        Args:
            marker_info: dict
        """
        super(InteractiveMarkerInterface, self).__init__()
        Thread.__init__(self)

        self._marker_name = marker_info["name"]
        self._target_frame_id = marker_info["target_frame_id"]
        self._reference_frame_id = marker_info["reference_frame_id"]

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._marker_server = None
        self._marker_pose = PoseStamped()
        self.reset_marker_pose()

        self._create_markers(marker_info)

    def _create_markers(self, info):
        assert isinstance(info, dict)
        self._marker_server = InteractiveMarkerServer(info["name"])
        marker = InteractiveMarker()
        marker.header.frame_id = info["reference_frame_id"]
        marker.scale = (
            0.3 if "scale" not in info or info["scale"] <= 0 else info["scale"]
        )
        marker.name = info["name"]
        marker.description = ""
        marker.pose = self._marker_pose.pose

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 1
        control.orientation.z = 1
        control.name = "move_3D"
        control.always_visible = True
        control.markers.append(self._make_sphere(marker.scale))
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        marker.controls.append(control)

        self._marker_server.insert(marker, self._process_feedback)
        self._marker_server.applyChanges()
        self._pose_publisher = rospy.Publisher(
            info["publish_topic"], Pose, queue_size=1
        )

    def _process_feedback(self, feedback):
        """
        :param feedback: feedback data of interactive marker
        :return: None
        """
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self._marker_pose.pose = feedback.pose
        self._marker_server.applyChanges()

    @staticmethod
    def _make_sphere(scale=0.3):
        """This function returns sphere marker for 3D translational movements.

        :param scale: scales the size of the sphere
        :return: sphere marker
        """
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = scale * 0.4
        marker.scale.y = scale * 0.4
        marker.scale.z = scale * 0.4
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        return marker

    def run(self):
        while not rospy.is_shutdown():
            self._publish_target_poses()
            rospy.sleep(0.01)

    def _publish_target_poses(self):
        """This function publishes desired target poses which the controller will subscribe to.

        :return: None
        """
        self._pose_publisher.publish(self._marker_pose.pose)

    def reset_marker_pose(self):
        rate = rospy.Rate(10.0)
        start = time.time()
        while not rospy.is_shutdown():
            try:
                trans = self._tf_buffer.lookup_transform(
                    self._reference_frame_id, self._target_frame_id, rospy.Time()
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                rate.sleep()
                rospy.logwarn_throttle(
                    3,
                    "Getting transform from {} to {}. "
                    "(If you constantly see this, check whether the TF is published)".format(
                        self._reference_frame_id, self._target_frame_id
                    ),
                )
                continue

            if time.time() - start > 5.0:
                return False

            self._marker_pose.pose.position.x = trans.transform.translation.x
            self._marker_pose.pose.position.y = trans.transform.translation.y
            self._marker_pose.pose.position.z = trans.transform.translation.z
            self._marker_pose.pose.orientation = trans.transform.rotation

            if self._marker_server is not None:
                self._marker_server.setPose(self._marker_name, self._marker_pose.pose)
                self._marker_server.applyChanges()
            return True
