#!/usr/bin/env python

import json
from uuid import uuid4
# Note that this needs:
# sudo pip install websocket-client
# not the library called 'websocket'
import websocket
import yaml
import rospy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

from rotools.utility import message_converter


class WebsocketROSClient(object):
    def __init__(self, kwargs):
        """
        Class to manage publishing to ROS through a ros-bridge websocket.
        """
        print("Connecting to websocket: {}:{}".format(kwargs['ip'], kwargs['port']))
        self.ws = websocket.create_connection('ws://' + kwargs['ip'] + ':' + str(kwargs['port']))
        self._advertise_dict = {}

        # Down streams: the client subscribe to local topics and publish it to the server
        if kwargs['downstream_list'] is not None:
            self.local_subscribers = []
            self.downstream_list = kwargs['downstream_list']
            for i, entity in enumerate(self.downstream_list):
                local_topic_id, remote_topic_id, msg_type = entity
                subscriber = self.create_subscriber(local_topic_id, msg_type, remote_topic_id)
                self.local_subscribers.append(subscriber)

        # Up stream: the client receive topics from the server and publish them locally
        if kwargs['upstream_list'] is not None:
            self.local_timers = []
            self.local_publishers = []
            self.upstream_list = kwargs['upstream_list']
            for i, entity in enumerate(self.upstream_list):
                local_topic_id, remote_topic_id, msg_type = entity
                timer, publisher = self.create_timer(local_topic_id, msg_type)
                self.local_timers.append(timer)
                self.local_publishers.append(publisher)

    def create_subscriber(self, local_topic_id, msg_type, remote_topic_id):
        if 'PoseStamped' in msg_type:
            return rospy.Subscriber(local_topic_id, PoseStamped, self._pose_stamped_cb, remote_topic_id)
        elif 'Twist' in msg_type:
            return rospy.Subscriber(local_topic_id, Twist, self._twist_cb, remote_topic_id)
        else:
            raise NotImplementedError

    def create_timer(self, local_topic_id, msg_type):
        if 'Odometry' in msg_type:
            timer = rospy.Timer(rospy.Duration.from_sec(0.001), self._odometry_cb)
            publisher = rospy.Publisher(local_topic_id, Odometry, queue_size=1)
            return timer, publisher
        else:
            raise NotImplementedError

    def _pose_stamped_cb(self, msg, remote_topic_id):
        pose = PoseStamped()
        pose.header = msg.header
        self.publish(remote_topic_id, pose)

    def _twist_cb(self, msg, remote_topic_id):
        twist = Twist()
        twist.linear = msg.linear
        twist.angular = msg.angular
        self.publish(remote_topic_id, twist)

    def _odometry_cb(self, event):
        for i, entity in enumerate(self.upstream_list):
            local_topic_id, remote_topic_id, msg_type = entity
            if 'Odometry' in msg_type:
                result = self.subscribe(remote_topic_id, 'nav_msgs/Odometry')
                self.local_publishers[i].publish(result)

    def _advertise(self, topic_name, topic_type):
        """
        Advertise a topic with its type in 'package/Message' format.

        :param str topic_name: ROS topic name.
        :param str topic_type: ROS topic type, e.g. std_msgs/String.
        :returns str: ID to de-advertise later on.
        """
        new_uuid = str(uuid4())
        self._advertise_dict[new_uuid] = {'topic_name': topic_name,
                                          'topic_type': topic_type}
        advertise_msg = {"op": "advertise",
                         "id": new_uuid,
                         "topic": topic_name,
                         "type": topic_type
                         }
        self.ws.send(json.dumps(advertise_msg))
        return new_uuid

    def _un_advertise(self, uuid):
        unad_msg = {"op": "unadvertise",
                    "id": uuid,
                    # "topic": topic_name
                    }
        self.ws.send(json.dumps(unad_msg))

    def __del__(self):
        """Cleanup all advertising"""
        d = self._advertise_dict
        for k in d:
            self._un_advertise(k)

    def _publish(self, topic_name, message):
        """Publish onto the already advertised topic the msg in the shape of a Python dict.

        :param str topic_name: ROS topic name.
        :param dict msg: Dictionary containing the definition of the message.
        """
        msg = {
            'op': 'publish',
            'topic': topic_name,
            'msg': message
        }
        json_msg = json.dumps(msg)
        self.ws.send(json_msg)

    def publish(self, topic_name, ros_message):
        """
        Publish on a topic given ROS messages through ros-bridge.
        :param str topic_name: ROS topic name.
        :param * ros_message: Any ROS message instance, e.g. LaserScan()
            from sensor_msgs/LaserScan.
        """
        # First check if we already advertised the topic
        d = self._advertise_dict
        for k in d:
            if d[k]['topic_name'] == topic_name:
                # Already advertised, do nothing
                break
        else:
            # Not advertised, so we advertise
            topic_type = ros_message._type
            self._advertise(topic_name, topic_type)
        # Converting ROS message to a dictionary through YAML
        ros_message_as_dict = yaml.load(ros_message.__str__())
        # Publishing
        self._publish(topic_name, ros_message_as_dict)

    def subscribe(self, topic_name, msg_type):
        """

        Args:
            topic_name: str ROS topic name.
            msg_type: str Type string of the message

        Returns:
            result: ROS message
        """
        msg = {
            'op': 'subscribe',
            'topic': topic_name,
            'type': msg_type
        }
        json_msg = json.dumps(msg)
        self.ws.send(json_msg)
        json_message = self.ws.recv()

        dictionary = json.loads(json_message)['msg']
        result = message_converter.convert_dictionary_to_ros_message(msg_type, dictionary)
        # print("Type: '%s' \n Received: '%s'" % (type, result))
        return result