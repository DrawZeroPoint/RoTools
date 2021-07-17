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

    def decode_to_pose_array_msg(self, dst_frame, src_frame_id=None, scaling_factor=1.0):
        """Decode the bytes in the streaming data to pose array message.

        :param dst_frame: str Reference frame name of the generated pose array message.
        :param src_frame_id: None/int If not None, all poses will be shifted subject to
                             the frame with this ID. This frame should belong to the human.
        :param scaling_factor: float Scale the position of the pose if src_frame_id is not None.
                               Its value equals to the robot/human body dimension ratio
        """
        pose_array_msg = GeometryMsg.PoseArray()
        pose_array_msg.header.stamp = rospy.Time.now()
        pose_array_msg.header.frame_id = dst_frame
        for i in range(self.item_num):
            item = self.payload[i * self._item_size:(i + 1) * self._item_size]
            pose_msg = self._type_02_decode_to_pose_msg(item)
            if pose_msg is None:
                return None
            pose_array_msg.poses.append(pose_msg)

        if src_frame_id is not None and src_frame_id < len(pose_array_msg.poses):
            shifted_pose_array_msg = GeometryMsg.PoseArray()
            shifted_pose_array_msg.header.frame_id = dst_frame
            ref_pose = pose_array_msg.poses[src_frame_id]
            for p in pose_array_msg.poses:
                shifted_pose = GeometryMsg.Pose()
                # The reference frame's position will be 0 0 0 after shifting
                shifted_pose.position.x = (p.position.x - ref_pose.position.x) * scaling_factor
                shifted_pose.position.y = (p.position.y - ref_pose.position.y) * scaling_factor
                shifted_pose.position.z = (p.position.z - ref_pose.position.z) * scaling_factor
                # The reference frame's orientation will be 0 0 0 1 after shifting
                r_ref_p, _ = common.get_relative_rotation(ref_pose.orientation, p.orientation)
                shifted_pose.orientation = common.to_ros_orientation(transform.quaternion_from_matrix(r_ref_p))
                shifted_pose_array_msg.poses.append(shifted_pose)
            return shifted_pose_array_msg
        return pose_array_msg

    @staticmethod
    def _type_02_decode_to_pose_msg(item):
        """Decode a type 02 stream to ROS pose message.

        :param item: str String of bytes
        """
        # segment_id = common.byte_to_uint32(item[:4])
        if len(item) != 32:
            # FIXME some of the received data only has length=2
            return None
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
    ):
        super(XsensInterface, self).__init__()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self.sock.bind((udp_ip, udp_port))
        self.buffer_size = buffer_size
        self.scaling_factor = scaling

        self._ref_frames = {'Pelvis': 0, 'T8': 4}
        if ref_frame in self._ref_frames:
            self.ref_frame = ref_frame
            self.ref_frame_id = self._ref_frames[ref_frame]
        elif ref_frame == '':
            rospy.logwarn('Reference frame is empty, using default (world)')
            self.ref_frame = 'world'
            self.ref_frame_id = None
        else:
            self.ref_frame = ref_frame
            self.ref_frame_id = None

        # The streamer should be launched before this program
        try:
            self.header = self._get_header()
        except IndexError:
            self.header = None
            rospy.logerr('Get header failed')

    def get_all_poses(self):
        data, _ = self.sock.recvfrom(self.buffer_size)
        if self.header is not None and self.header.is_valid:
            payload = Payload(data[24:], self.header)
            pose_array_msg = payload.decode_to_pose_array_msg(self.ref_frame, self.ref_frame_id)
            if pose_array_msg is None:
                return False, None
            else:
                return True, pose_array_msg
        else:
            rospy.logerr('Header is not valid')
            return False, None

    def get_body_pose_array_msg(self, all_poses):
        """Given all segment poses, extract the body segment poses and TCP poses from that.
        The body poses are simply all poses without hand segment poses and property poses.

        :param all_poses: PoseArray A pose array composed of all segment poses.
        :return: PoseArray PoseStamped PoseStamped PoseStamped PoseStamped
                 Body segment poses, left hand pose, right hand pose, left sole pose, right sole pose.
        """
        assert isinstance(all_poses, GeometryMsg.PoseArray)
        body_pose_array_msg = GeometryMsg.PoseArray()
        body_pose_array_msg.header = all_poses.header
        left_tcp_msg = GeometryMsg.PoseStamped()
        right_tcp_msg = GeometryMsg.PoseStamped()
        left_sole_msg = GeometryMsg.PoseStamped()
        right_sole_msg = GeometryMsg.PoseStamped()
        left_shoulder_msg = GeometryMsg.PoseStamped()
        right_shoulder_msg = GeometryMsg.PoseStamped()
        left_upper_arm_msg = GeometryMsg.PoseStamped()
        right_upper_arm_msg = GeometryMsg.PoseStamped()
        left_forearm_msg = GeometryMsg.PoseStamped()
        right_forearm_msg = GeometryMsg.PoseStamped()

        # Initialize header
        left_tcp_msg.header = all_poses.header
        right_tcp_msg.header = left_tcp_msg.header
        left_sole_msg.header = left_tcp_msg.header
        right_sole_msg.header = left_tcp_msg.header
        left_shoulder_msg.header = left_tcp_msg.header
        right_shoulder_msg.header = left_tcp_msg.header
        left_upper_arm_msg.header = left_tcp_msg.header
        right_upper_arm_msg.header = left_tcp_msg.header
        left_forearm_msg.header = left_tcp_msg.header
        right_forearm_msg.header = left_tcp_msg.header

        # all_poses should at least contain body segment poses
        segment_id = 0
        for p in all_poses.poses:
            body_pose_array_msg.poses.append(p)
            if segment_id == 7:
                right_shoulder_msg.pose = p
            if segment_id == 8:
                right_upper_arm_msg.pose = p
            if segment_id == 9:
                right_forearm_msg.pose = p
            if segment_id == 10:
                right_tcp_msg.pose = p
            if segment_id == 11:
                left_shoulder_msg.pose = p
            if segment_id == 12:
                left_upper_arm_msg.pose = p
            if segment_id == 13:
                left_forearm_msg.pose = p
            if segment_id == 14:
                left_tcp_msg.pose = p
            if segment_id == 18:
                right_sole_msg.pose = p
            if segment_id == 22:
                left_sole_msg.pose = p
            segment_id += 1
            if segment_id == self.header.body_segments_num:
                break
        assert len(body_pose_array_msg.poses) == self.header.body_segments_num
        return [body_pose_array_msg, left_tcp_msg, right_tcp_msg, left_sole_msg, right_sole_msg,
                left_shoulder_msg, left_upper_arm_msg, left_forearm_msg,
                right_shoulder_msg, right_upper_arm_msg, right_forearm_msg]

    def get_prop_msgs(self, all_poses):
        if self.header.props_num == 1:
            try:
                prop_1_msg = GeometryMsg.PoseStamped()
                prop_1_msg.header = all_poses.header
                prop_1_msg.pose = all_poses.poses[24]
                return prop_1_msg
            except IndexError:
                rospy.logwarn_throttle(3, 'Prop number is not 0 but failed to get its pose')
                return None
        else:
            return None

    def get_hand_joint_states(self, all_poses):
        """Get joint states for both hand.

        :param all_poses: PoseArray Poses of all segment. Derived from @get_all_poses
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
        left_hand_js.header = all_poses.header
        right_hand_js.header = all_poses.header
        # Here the hand joint states are defined as j1 and j2 for thumb to pinky
        # j1 is the angle between metacarpals (parent) and proximal phalanges (child)
        # j2 is the angle between proximal phalanges and intermediate/distal phalanges (distal is only for thumb)
        base_num = self.header.body_segments_num + self.header.props_num
        for i in [1, 4, 8, 12, 16]:
            meta_id = base_num + i
            left_hand_js.position.extend(self._get_finger_j1_j2(all_poses.poses, meta_id))
        for i in [1, 4, 8, 12, 16]:
            meta_id = base_num + 20 + i
            right_hand_js.position.extend(self._get_finger_j1_j2(all_poses.poses, meta_id))
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

    def _get_header(self):
        """Get the header data from the received MVN Awinda datagram.

        :return: Header
        """
        data, _ = self.sock.recvfrom(self.buffer_size)
        id_str = common.byte_to_str(data[0:6], 6)  # ID
        sample_counter = common.byte_to_uint32(data[6:10])
        datagram_counter = struct.unpack('!B', data[10])
        item_number = common.byte_to_uint8(data[11])
        time_code = common.byte_to_uint32(data[12:16])
        character_id = common.byte_to_uint8(data[16])
        body_segments_num = common.byte_to_uint8(data[17])
        props_num = common.byte_to_uint8(data[18])
        finger_segments_num = common.byte_to_uint8(data[19])
        rospy.loginfo('Stream received: body segment #{}, property #{}, finger segment #{}'.format(
            body_segments_num, props_num, finger_segments_num))
        # 20 21 are reserved for future use
        payload_size = common.byte_to_uint16(data[22:24])
        return Header([id_str, sample_counter, datagram_counter, item_number, time_code, character_id,
                       body_segments_num, props_num, finger_segments_num, payload_size])
