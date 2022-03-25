from __future__ import print_function

import os
import csv
import time
import rospy

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

try:
    import geometry_msgs.msg as GeometryMsg
    import control_msgs.msg as ControlMsg
    import trajectory_msgs.msg as TrajectoryMsg
    import std_msgs.msg as StdMsg
except ImportError:
    pass

from rotools.utility import common, transform


class SnapshotInterface(object):

    def __init__(
            self,
            js_topics,
            odom_topics,
            save_dir,
            **kwargs
    ):
        super(SnapshotInterface, self).__init__()

        self.entities = {}

        if not os.path.isdir(save_dir):
            self.save_dir = '/tmp'
        else:
            os.makedirs(save_dir, exist_ok=True)
            self.save_dir = save_dir
        rospy.loginfo("Saving snapshots to {}".format(self.save_dir))

        self._make_entity(js_topics, 'JS_')
        self._make_entity(odom_topics, 'ODOM_')

    def _make_entity(self, topics, prefix):
        for topic in topics:
            assert isinstance(topic, str), print("Topic type is not str ({})".format(type(topic)))
            file_name = prefix + time.strftime('%H%M%S') + topic.replace('/', '_') + '.csv'
            file_path = os.path.join(self.save_dir, file_name)
            entity = [file_path, False]
            self.entities[topic] = entity

    def save_joint_state_msg(self, topic, msg, position_only, tag=''):
        assert isinstance(msg, JointState), print(type(msg))
        if topic not in self.entities:
            rospy.logerr("The interface does not hold the topic {}".format(topic))
            return False

        file_path, has_header = self.entities[topic]
        if not os.path.exists(file_path):
            has_header = False  # In case the file is removed while the program is still running

        with open(file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if not has_header:
                writer.writerow(['tag'] + msg.name)
                self.entities[topic] = [file_path, True]
            writer.writerow(['q_' + str(tag)] + self.to_str_list(msg.position))
            if not position_only:
                writer.writerow(['dq_' + str(tag)] + self.to_str_list(msg.velocity))
                writer.writerow(['tau_' + str(tag)] + self.to_str_list(msg.effort))
        return True

    def save_odom_msg(self, topic, msg, tag=''):
        assert isinstance(msg, Odometry), print(type(msg))
        if topic not in self.entities:
            rospy.logerr("The interface does not hold the topic {}".format(topic))
            return False

        file_path, has_header = self.entities[topic]
        if not os.path.exists(file_path):
            has_header = False

        with open(file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if not has_header:
                writer.writerow(['tag', 'p_x', 'p_y', 'p_z', 'o_x', 'o_y', 'o_z', 'o_w'])
                self.entities[topic] = [file_path, True]
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            writer.writerow(['pose_' + str(tag)] + self.to_str_list([p.x, p.y, p.z, o.x, o.y, o.z, o.w]))
        return True

    @staticmethod
    def to_str_list(values, precision=5):
        output = []
        for value in values:
            value_str = '{:.{}f}'.format(value, precision)
            output.append(value_str)
        return output
