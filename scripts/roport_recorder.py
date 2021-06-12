#!/usr/bin/env python
from __future__ import print_function
import os
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String

from rotools.utility.common import get_param


class Recorder(object):
    def __init__(self):
        super(Recorder, self).__init__()

        # The topic publishing joint states (should be only one topic)
        js_topic = get_param('~js_topic')
        # The joint names needed to be recorded (optional, if not provided, record all joints in js_topic)
        self.joint_names = get_param('~joint_names')

        # The topic publishing image TODO record multiple images
        img_topic = get_param('~img_topic')

        self.write_position = False
        self.write_image = False
        self.positions = []
        self.max_positions = None
        self.min_positions = None
        self.images = []
        self.save_dir = '/home/dzp/'
        self.js_sub = rospy.Subscriber(js_topic, JointState, self.js_cb, queue_size=1)
        self.img_sub = rospy.Subscriber(img_topic, Image, self.img_cb, queue_size=1)
        self.cmd_sub = rospy.Subscriber('/recorder/cmd', String, self.cmd_cb, queue_size=1)

    def js_cb(self, msg):
        if not self.write_position:
            return
        positions = []
        for i, n in enumerate(msg.name):
            if n in self.joint_names:
                positions.append(msg.position[i])
        positions = np.asarray(positions)
        if self.max_positions is None:
            self.max_positions = np.ones_like(positions) * -100.
        self.max_positions = np.amax((self.max_positions, positions), axis=0)
        if self.min_positions is None:
            self.min_positions = np.ones_like(positions) * 100.
        self.min_positions = np.amin((self.min_positions, positions), axis=0)
        print('Added new position (deg): \n', positions * 180 / np.pi)
        print('Max position (rad): \n', self.max_positions)
        print('Min position (rad): \n', self.min_positions)
        self.positions.append(positions)
        self.write_position = False

    def img_cb(self, msg):
        if not self.write_image:
            return
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.images.append(cv_image)
        cv2.imwrite(os.path.join(self.save_dir, '{}.jpg'.format(len(self.images))), cv_image)
        print('Added new image: \n', cv_image.shape)
        self.write_image = False

    def cmd_cb(self, msg):
        if 'j' in msg.data:
            rospy.loginfo('recording joint states...')
            self.record_js()
        if 'i' in msg.data:
            rospy.loginfo('recording image...')
            self.record_img()
        if msg.data == 's':
            rospy.loginfo('saving...')
            self.save_js()

    def record_js(self):
        self.write_position = True

    def record_img(self):
        self.write_image = True

    def save_js(self):
        p = np.asarray(self.positions)
        np.save(os.path.join(self.save_dir, "samples_js.npy"), p)
        rospy.loginfo('Saved {} joint states'.format(len(p)))
        i = np.asarray(self.images)
        np.save(os.path.join(self.save_dir, "samples_img.npy"), i)
        rospy.loginfo('Saved {} images'.format(len(i)))


if __name__ == "__main__":
    try:
        rospy.init_node('roport_recorder')
        recorder = Recorder()
        rospy.loginfo("RoPort: Recorder ready.")
        rospy.loginfo('Send String msg to /recorder/cmd (j: record joint states, i: record image, s: save)')
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
