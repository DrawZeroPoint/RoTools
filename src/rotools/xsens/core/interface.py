from __future__ import print_function

import socket
import struct
import numpy as np

try:
    import rospy
    import tf2_ros
    import tf2_geometry_msgs  # import this is mandatory to use PoseStamped msg

    import moveit_commander

    import geometry_msgs.msg as GeometryMsg
    import moveit_msgs.msg as MoveItMsg
    import control_msgs.msg as ControlMsg
    import trajectory_msgs.msg as TrajectoryMsg
    import std_msgs.msg as StdMsg
    import sensor_msgs.msg as SensorMsg
except ImportError:
    pass

from rotools.utility import common, transform


class Header:
    def __init__(self, header):
        assert isinstance(header, list) and len(header) == 10
        self.ID_str = header[0]
        self.sample_counter = header[1]
        self.datagram_counter = header[2]
        self.item_counter = header[3]  # Amount of items (point/segments)
        self.time_code = header[4]  # Time since start measurement
        self.character_ID = header[5]  # Amount of people recorded at the same time
        self.body_segments_num = header[6]  # number of body segments measured
        self.props_num = header[7]  # Amount of property sensors
        self.finger_segments_num = header[8]  # Number of finger data segments
        self.payload_size = header[9]  # Size of the measurement excluding the header

    @property
    def is_valid(self):
        if self.ID_str != 'MXTP02':
            rospy.logwarn('Current only support MXTP02, but got {}'.format(self.ID_str))
            return False
        if self.item_counter != self.body_segments_num + self.props_num + self.finger_segments_num:
            rospy.logwarn('Segments number in total does not match item counter')
            return False
        if self.payload_size % self.item_counter != 0:
            rospy.logwarn('Payload of size {} is incomplete'.format(self.payload_size))
            return False
        return True

    @property
    def item_num(self):
        return self.item_counter

    @property
    def item_size(self):
        """Define how many bytes in a item"""
        return self.payload_size // self.item_num


class Payload:
    def __init__(self, payload, header):
        self.payload = payload
        self.header = header
        self.item_num = header.item_num
        self._payload_len = len(self.payload)
        self._item_size = self._payload_len // self.item_num

    def decode_to_pose_array_msg(self, ref_frame, ref_frame_id=None):
        """Decode the bytes in the streaming data to pose array message.

        :param ref_frame: str Reference frame name.
        :param ref_frame_id: If not None, all poses will be shifted subject to
                             the reference frame with this ID.
        """
        pose_array_msg = GeometryMsg.PoseArray()
        pose_array_msg.header.frame_id = ref_frame
        for i in range(self.item_num):
            item = self.payload[i * self._item_size:(i + 1) * self._item_size]
            pose_msg = self._type_02_decode_to_pose_msg(item)
            pose_array_msg.poses.append(pose_msg)

        if ref_frame_id is not None and ref_frame_id < len(pose_array_msg.poses):
            shifted_pose_array_msg = GeometryMsg.PoseArray()
            shifted_pose_array_msg.header.frame_id = ref_frame
            ref_pose = pose_array_msg.poses[ref_frame_id]
            for p in pose_array_msg.poses:
                shifted_pose = GeometryMsg.Pose()
                shifted_pose.position.x = p.position.x - ref_pose.position.x
                shifted_pose.position.y = p.position.y - ref_pose.position.y
                shifted_pose.position.z = p.position.z - ref_pose.position.z
                shifted_pose.orientation = p.orientation
                shifted_pose_array_msg.poses.append(shifted_pose)
            return shifted_pose_array_msg
        return pose_array_msg

    @staticmethod
    def _type_02_decode_to_pose_msg(item):
        # segment_id = common.byte_to_uint32(item[:4])
        x = common.byte_to_float(item[4:8])
        y = common.byte_to_float(item[8:12])
        z = common.byte_to_float(item[12:16])
        qw = common.byte_to_float(item[16:20])
        qx = common.byte_to_float(item[20:24])
        qy = common.byte_to_float(item[24:28])
        qz = common.byte_to_float(item[28:32])
        # Convert the pose from MVN frame (x forward, y up, z right) to ROS frame
        return common.to_ros_pose(np.array([x, y, z, qx, qy, qz, qw]))


class XsensInterface(object):

    def __init__(
            self,
            udp_ip,
            udp_port,
            ref_frame,
            buffer_size=2048,
    ):
        super(XsensInterface, self).__init__()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self.sock.bind((udp_ip, udp_port))
        self.buffer_size = buffer_size

        self._ref_frames = {'Pelvis': 0, 'T8': 12}
        if ref_frame in self._ref_frames:
            self.ref_frame = ref_frame
            self.ref_frame_id = self._ref_frames[ref_frame]
        elif ref_frame == '':
            rospy.logwarn('Reference frame is empty, using default (world)')
            self.ref_frame = 'world'
            self.ref_frame_id = None
        else:
            rospy.logwarn('Given reference frame {} not in known frames, using default (world)'.format(ref_frame))
            self.ref_frame = 'world'
            self.ref_frame_id = None

        # The streamer should be launched before this program
        try:
            self.header = self._get_header()
        except IndexError:
            pass

        # TF related handles
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def get_all_poses(self):
        data, _ = self.sock.recvfrom(self.buffer_size)
        if self.header.is_valid:
            payload = Payload(data[24:], self.header)
            pose_array_msg = payload.decode_to_pose_array_msg(self.ref_frame, self.ref_frame_id)
            return True, pose_array_msg
        else:
            return False, None

    def get_body_pose_array_msg(self, all_poses):
        assert isinstance(all_poses, GeometryMsg.PoseArray)
        body_pose_array_msg = GeometryMsg.PoseArray()
        body_pose_array_msg.header = all_poses.header
        cnt = self.header.body_segments_num
        left_tcp_msg = GeometryMsg.PoseStamped()
        right_tcp_msg = GeometryMsg.PoseStamped()
        left_tcp_msg.header.stamp = rospy.Time.now()
        left_tcp_msg.header.frame_id = self.ref_frame
        right_tcp_msg.header = left_tcp_msg.header
        for p in all_poses.poses:
            body_pose_array_msg.poses.append(p)
            if cnt == 9:
                left_tcp_msg.pose = p
            if cnt == 13:
                right_tcp_msg.pose = p
            cnt -= 1
            if cnt == 0:
                break
        assert len(body_pose_array_msg.poses) == self.header.body_segments_num
        return body_pose_array_msg, left_tcp_msg, right_tcp_msg

    def _get_header(self):
        data, _ = self.sock.recvfrom(self.buffer_size)
        ID_str = common.byte_to_str(data[0:6], 6)  # ID
        sample_counter = common.byte_to_uint32(data[6:10])
        datagram_counter = struct.unpack('!B', data[10])
        item_number = common.byte_to_uint8(data[11])
        rospy.loginfo('Number of item: {}'.format(item_number))
        time_code = common.byte_to_uint32(data[12:16])
        character_ID = common.byte_to_uint8(data[16])
        body_segments_num = common.byte_to_uint8(data[17])
        props_num = common.byte_to_uint8(data[18])
        finger_segments_num = common.byte_to_uint8(data[19])
        # 20 21 are reserved for future use
        payload_size = common.byte_to_uint16(data[22:24])
        return Header([ID_str, sample_counter, datagram_counter, item_number, time_code, character_ID,
                       body_segments_num, props_num, finger_segments_num, payload_size])
