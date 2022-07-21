from __future__ import print_function

import os
import cv2
import csv
import time
import rospy

from cv_bridge import CvBridge

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState, Image, CompressedImage
from nav_msgs.msg import Odometry

from rotools.utility import common, transform


class SnapshotInterface(object):
    def __init__(self, js_topics, odom_topics, pose_topics, save_dir, **kwargs):
        super(SnapshotInterface, self).__init__()

        self._entities = {}

        if not os.path.isdir(save_dir):
            self.save_dir = "/tmp"
        else:
            os.makedirs(save_dir, exist_ok=True)
            self.save_dir = save_dir
        rospy.loginfo("Saving snapshots to {}".format(self.save_dir))

        self._make_csv_entity(js_topics, "JS_")
        self._make_csv_entity(odom_topics, "ODOM_")
        self._make_csv_entity(pose_topics, "POSE_")

        self._bridge = CvBridge()

    def _make_csv_entity(self, topics, prefix):
        if topics is None:
            return
        for topic in topics:
            assert isinstance(topic, str), print(
                "Topic type is not str ({})".format(type(topic))
            )
            file_name = (
                prefix + time.strftime("%H%M%S") + topic.replace("/", "_") + ".csv"
            )
            file_path = os.path.join(self.save_dir, file_name)
            entity = [file_path, False]
            self._entities[topic] = entity

    def save_joint_state_msg(self, topic, msg, position_only, tag=""):
        assert isinstance(msg, JointState), print(type(msg))
        if topic not in self._entities:
            rospy.logerr("The interface does not hold the topic {}".format(topic))
            return False

        file_path, has_header = self._entities[topic]
        if not os.path.exists(file_path):
            has_header = (
                False  # In case the file is removed while the program is still running
            )

        with open(file_path, "a", newline="") as f:
            writer = csv.writer(f)
            if not has_header:
                writer.writerow(["tag"] + msg.name)
                self._entities[topic] = [file_path, True]
            writer.writerow(["q_" + str(tag)] + self.to_str_list(msg.position))
            if not position_only:
                writer.writerow(["dq_" + str(tag)] + self.to_str_list(msg.velocity))
                writer.writerow(["tau_" + str(tag)] + self.to_str_list(msg.effort))
        return True

    def save_odom_msg(self, topic, msg, tag=""):
        assert isinstance(msg, Odometry), print(type(msg))
        if topic not in self._entities:
            rospy.logerr("The interface does not hold the topic {}".format(topic))
            return False

        file_path, has_header = self._entities[topic]
        if not os.path.exists(file_path):
            has_header = False

        with open(file_path, "a", newline="") as f:
            writer = csv.writer(f)
            if not has_header:
                writer.writerow(
                    ["tag", "p_x", "p_y", "p_z", "o_x", "o_y", "o_z", "o_w"]
                )
                self._entities[topic] = [file_path, True]
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            writer.writerow(
                ["pose_" + str(tag)]
                + self.to_str_list([p.x, p.y, p.z, o.x, o.y, o.z, o.w])
            )
        return True

    def save_pose_msg(self, topic, msg, tag=""):
        if isinstance(msg, PoseStamped):
            return save_pose_msg(topic, msg.pose, tag)

        assert isinstance(msg, Pose), print(type(msg))
        if topic not in self._entities:
            rospy.logerr("The interface does not hold the topic {}".format(topic))
            return False

        file_path, has_header = self._entities[topic]
        if not os.path.exists(file_path):
            has_header = False

        with open(file_path, "a", newline="") as f:
            writer = csv.writer(f)
            if not has_header:
                writer.writerow(
                    ["tag", "p_x", "p_y", "p_z", "o_x", "o_y", "o_z", "o_w"]
                )
                self._entities[topic] = [file_path, True]
            p = msg.position
            o = msg.orientation
            writer.writerow(
                ["pose_" + str(tag)]
                + self.to_str_list([p.x, p.y, p.z, o.x, o.y, o.z, o.w])
            )
        return True

    def save_image_msgs(self, rgb_topic, depth_topic, rgb_msg, depth_msg, tag=""):
        prefix = time.strftime("%H%M%S") + tag + "_"
        rgb_ok = self._save_image_msg(rgb_topic, rgb_msg, prefix, ".jpg")
        depth_ok = self._save_image_msg(depth_topic, depth_msg, prefix, ".png")
        return rgb_ok | depth_ok

    def _save_image_msg(self, topic, msg, prefix="", suffix=""):
        file_name = prefix + topic.replace("/", "_") + suffix
        if isinstance(msg, Image):
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        elif isinstance(msg, CompressedImage):
            cv_image = self._bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
        else:
            raise NotImplementedError(
                "Image type {} is not implemented".format(type(msg))
            )
        cv2.imwrite(os.path.join(self.save_dir, file_name), cv_image)
        return True

    @staticmethod
    def to_str_list(values, precision=5):
        output = []
        for value in values:
            value_str = "{:.{}f}".format(value, precision)
            output.append(value_str)
        return output
