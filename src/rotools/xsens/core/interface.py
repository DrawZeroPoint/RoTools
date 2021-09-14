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

from rotools.utility import common


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

    def __repr__(self):
        s = 'Header {}: \nsample_counter {}, datagram_counter {},\n' \
            'item #{}, body segment #{}, prop #{}, finger segment #{}\n'.format(
                self.ID_str, self.sample_counter, self.datagram_counter, self.item_counter,
                self.body_segments_num, self.props_num, self.finger_segments_num)
        return s

    @property
    def is_valid(self):
        if self.ID_str != 'MXTP02':
            rospy.logwarn('Current only support MXTP02, but got {}'.format(self.ID_str))
            return False
        if self.item_counter != self.body_segments_num + self.props_num + self.finger_segments_num:
            rospy.logwarn('Segments number in total does not match item counter')
            return False
        if self.payload_size % self.item_counter != 0:
            rospy.logwarn('Payload size {} is not dividable by item number {}'.format(self.payload_size, self.item_num))
            return False
        return True

    @property
    def is_object(self):
        """External tracked object's datagram have less segments than body datagram."""
        return self.item_counter < 23

    @property
    def item_num(self):
        return self.item_counter

    @property
    def item_size(self):
        """Define how many bytes in a item"""
        return self.payload_size // self.item_num


class Datagram(object):
    def __init__(
            self,
            header,
            payload
    ):
        self.header = header
        self.payload = payload

    @property
    def is_object(self):
        return self.header.is_object

    def decode_to_pose_array_msg(self, ref_frame, ref_frame_id=None, scaling_factor=1.0):
        """Decode the bytes in the streaming data to pose array message.

        :param ref_frame: str Reference frame name of the generated pose array message.
        :param ref_frame_id: None/int If not None, all poses will be shifted subject to
                             the frame with this ID. This frame should belong to the human.
        :param scaling_factor: float Scale the position of the pose if src_frame_id is not None.
                               Its value equals to the robot/human body dimension ratio
        """
        pose_array_msg = GeometryMsg.PoseArray()
        pose_array_msg.header.stamp = rospy.Time.now()
        pose_array_msg.header.frame_id = ref_frame

        for i in range(self.header.item_num):
            item = self.payload[i * self.header.item_size:(i + 1) * self.header.item_size]
            pose_msg = self._decode_to_pose_msg(item)
            if pose_msg is None:
                return None
            pose_array_msg.poses.append(pose_msg)

        if ref_frame_id is not None and ref_frame_id < len(pose_array_msg.poses):
            relative_pose_array_msg = GeometryMsg.PoseArray()
            relative_pose_array_msg.header.frame_id = ref_frame
            reference_pose = pose_array_msg.poses[ref_frame_id]
            for p in pose_array_msg.poses:
                std_relative_pose = common.get_transform_same_origin(reference_pose, p)
                relative_pose = common.to_ros_pose(std_relative_pose)
                relative_pose_array_msg.poses.append(relative_pose)
            return relative_pose_array_msg
        return pose_array_msg

    @staticmethod
    def _decode_to_pose_msg(item):
        """Decode a type 02 stream to ROS pose message.

        :param item: str String of bytes
        """
        if len(item) != 32:
            rospy.logerr('Payload pose data size is not 32: {}'.format(len(item)))
            return None
        # segment_id = common.byte_to_uint32(item[:4])
        x = common.byte_to_float(item[4:8])
        y = common.byte_to_float(item[8:12])
        z = common.byte_to_float(item[12:16])
        qw = common.byte_to_float(item[16:20])
        qx = common.byte_to_float(item[20:24])
        qy = common.byte_to_float(item[24:28])
        qz = common.byte_to_float(item[28:32])
        # We do not need to convert the pose from MVN frame (x forward, y up, z right) to ROS frame,
        # since the type 02 data is Z-up, see:
        # https://www.xsens.com/hubfs/Downloads/Manuals/MVN_real-time_network_streaming_protocol_specification.pdf
        return common.to_ros_pose(np.array([x, y, z, qx, qy, qz, qw]))


class XsensInterface(object):
    def __init__(
            self,
            udp_ip,
            udp_port,
            ref_frame,
            scaling=1.0,
            buffer_size=4096,
            **kwargs  # DO NOT REMOVE
    ):
        super(XsensInterface, self).__init__()

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self._sock.bind((udp_ip, udp_port))
        self._buffer_size = buffer_size

        self._ref_frames = {'Pelvis': 0, 'T8': 4}

        if ref_frame in self._ref_frames:
            self.ref_frame = ref_frame
            self.ref_frame_id = self._ref_frames[ref_frame]
        elif ref_frame == '' or ref_frame == 'world':
            rospy.logwarn('Reference frame is the world frame')
            self.ref_frame = 'world'
            self.ref_frame_id = None
        else:
            rospy.logwarn('Using customized reference frame {}'.format(ref_frame))
            self.ref_frame = ref_frame
            self.ref_frame_id = None

        self.scaling_factor = scaling
        self.header = None
        self.object_poses = None
        self.all_body_poses = None

    @property
    def first_object_pose(self):
        if self.object_poses is None:
            return None
        else:
            pose = GeometryMsg.PoseStamped()
            pose.header = self.object_poses.header
            pose.pose = self.object_poses.poses[0]
            return pose

    def get_datagram(self):
        """[Main entrance function] Get poses from the datagram.

        """
        data, _ = self._sock.recvfrom(self._buffer_size)
        datagram = self._get_datagram(data)
        if datagram is not None:
            pose_array_msg = datagram.decode_to_pose_array_msg(self.ref_frame, self.ref_frame_id)
            if pose_array_msg is None:
                return False
            if datagram.is_object:
                self.object_poses = pose_array_msg  # Only body poses are returned
            else:
                self.all_body_poses = pose_array_msg
            return True
        else:
            return False

    def get_body_poses(self, pub_details=False):
        """Given all segment poses, extract the body segment poses and TCP poses from that.
        The body poses are simply all poses without hand segment poses and property poses.

        :return: PoseArray PoseStamped PoseStamped PoseStamped PoseStamped
                 Body segment poses, left hand pose, right hand pose, left sole pose, right sole pose.
        """
        main_body_msg = GeometryMsg.PoseArray()
        main_body_msg.header = self.all_body_poses.header
        left_tcp_msg = GeometryMsg.PoseStamped()
        right_tcp_msg = GeometryMsg.PoseStamped()
        left_sole_msg = GeometryMsg.PoseStamped()
        right_sole_msg = GeometryMsg.PoseStamped()
        # left_shoulder_msg = GeometryMsg.PoseStamped()
        # right_shoulder_msg = GeometryMsg.PoseStamped()
        # left_upper_arm_msg = GeometryMsg.PoseStamped()
        # right_upper_arm_msg = GeometryMsg.PoseStamped()
        # left_forearm_msg = GeometryMsg.PoseStamped()
        # right_forearm_msg = GeometryMsg.PoseStamped()

        # Initialize message headers
        left_tcp_msg.header = self.all_body_poses.header
        right_tcp_msg.header = left_tcp_msg.header
        left_sole_msg.header = left_tcp_msg.header
        right_sole_msg.header = left_tcp_msg.header
        # left_shoulder_msg.header = left_tcp_msg.header
        # right_shoulder_msg.header = left_tcp_msg.header
        # left_upper_arm_msg.header = left_tcp_msg.header
        # right_upper_arm_msg.header = left_tcp_msg.header
        # left_forearm_msg.header = left_tcp_msg.header
        # right_forearm_msg.header = left_tcp_msg.header

        # all_poses should at least contain body segment poses
        segment_id = 0
        for p in self.all_body_poses.poses:
            main_body_msg.poses.append(p)
            # if segment_id == 7:
            #     right_shoulder_msg.pose = p
            # if segment_id == 8:
            #     right_upper_arm_msg.pose = p
            # if segment_id == 9:
            #     right_forearm_msg.pose = p
            if segment_id == 10:
                right_tcp_msg.pose = p
            # if segment_id == 11:
            #     left_shoulder_msg.pose = p
            # if segment_id == 12:
            #     left_upper_arm_msg.pose = p
            # if segment_id == 13:
            #     left_forearm_msg.pose = p
            if segment_id == 14:
                left_tcp_msg.pose = p
            if segment_id == 18:
                right_sole_msg.pose = p
            if segment_id == 22:
                left_sole_msg.pose = p
            segment_id += 1
            if segment_id == self.header.body_segments_num:
                break
        assert len(main_body_msg.poses) == self.header.body_segments_num
        return [self.all_body_poses, main_body_msg, left_tcp_msg, right_tcp_msg, left_sole_msg, right_sole_msg]

    def get_prop_msgs(self):
        if self.header.props_num == 1:
            try:
                prop_1_msg = GeometryMsg.PoseStamped()
                prop_1_msg.header = self.all_body_poses.header
                prop_1_msg.pose = self.all_body_poses.poses[24]
                return prop_1_msg
            except IndexError:
                rospy.logwarn_throttle(3, 'Prop number is not 0 but failed to get its pose')
                return None
        else:
            return None

    def get_hand_joint_states(self):
        """Get joint states for both hand.

        :return: JointState/None JointState/None.
                 Left hand joint states (10), right hand joint states (10)
        """
        if self.header is None or not self.header.is_valid:
            return None, None
        if self.header.finger_segments_num != 40:
            rospy.logwarn_throttle(3, "Finger segment number is not 40: {}".format(self.header.finger_segments_num))
            return None, None

        left_hand_js = SensorMsg.JointState()
        right_hand_js = SensorMsg.JointState()
        left_hand_js.header = self.all_body_poses.header
        right_hand_js.header = self.all_body_poses.header
        # Here the hand joint states are defined as j1 and j2 for thumb to pinky
        # j1 is the angle between metacarpals (parent) and proximal phalanges (child)
        # j2 is the angle between proximal phalanges and intermediate/distal phalanges (distal is only for thumb)
        base_num = self.header.body_segments_num + self.header.props_num
        for i in [1, 4, 8, 12, 16]:
            meta_id = base_num + i
            left_hand_js.position.extend(self._get_finger_j1_j2(self.all_body_poses.poses, meta_id))
        for i in [1, 4, 8, 12, 16]:
            meta_id = base_num + 20 + i
            right_hand_js.position.extend(self._get_finger_j1_j2(self.all_body_poses.poses, meta_id))
        return left_hand_js, right_hand_js

    @staticmethod
    def _get_finger_j1_j2(poses, meta_id, upper=1.57):
        """Get joint values between metacarpals and proximal phalanges (j1),
        and j2 between proximal phalanges and intermediate phalanges.
        j1 and j2 will always be non-negative and be normalized to the range [0, 1].

        :param poses: Pose[] Finger segment poses
        :param meta_id: int Id of the metacarpals in the poses
        :return: float float. J1 and J2
        """
        m_pose = poses[meta_id]
        pp_pose = poses[meta_id + 1]
        dp_pose = poses[meta_id + 2]
        _, euler_angles = common.get_relative_rotation(m_pose.orientation, pp_pose.orientation)
        # * -1 is to convert to walker conversion
        j1 = np.fabs(euler_angles[np.argmax(np.abs(euler_angles))])
        _, euler_angles = common.get_relative_rotation(pp_pose.orientation, dp_pose.orientation)
        j2 = np.fabs(euler_angles[np.argmax(np.abs(euler_angles))])
        return [np.minimum(j1, upper) / upper, np.minimum(j2, upper) / upper]

    @staticmethod
    def _get_header(data):
        """Get the header data from the received MVN Awinda datagram.

        :param data: Tuple From self._sock.recvfrom(self._buffer_size)
        :return: Header
        """
        if len(data) < 24:
            rospy.logwarn('Data length {} is less than 24'.format(len(data)))
            return None
        id_str = common.byte_to_str(data[0:6], 6)
        sample_counter = common.byte_to_uint32(data[6:10])
        datagram_counter = struct.unpack('!B', data[10])
        item_number = common.byte_to_uint8(data[11])
        time_code = common.byte_to_uint32(data[12:16])
        character_id = common.byte_to_uint8(data[16])
        body_segments_num = common.byte_to_uint8(data[17])
        props_num = common.byte_to_uint8(data[18])
        finger_segments_num = common.byte_to_uint8(data[19])
        # 20 21 are reserved for future use
        payload_size = common.byte_to_uint16(data[22:24])
        header = Header([id_str, sample_counter, datagram_counter, item_number, time_code, character_id,
                         body_segments_num, props_num, finger_segments_num, payload_size])
        rospy.logdebug(header.__repr__())
        return header

    def _get_datagram(self, data):
        header = self._get_header(data)
        if header is not None and header.is_valid:
            self.header = header
            return Datagram(header, data[24:])
        else:
            return None
