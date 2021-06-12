import cv2
import base64
import struct
import numpy as np

try:
    import rospy
    import geometry_msgs.msg as GeometryMsg
    import moveit_msgs.msg as MoveItMsg
    import trajectory_msgs.msg as TrajectoryMsg

    from moveit_commander.conversions import pose_to_list
except ImportError:
    pass

from rotools.utility import transform


def all_close(goal, actual, tolerance):
    """Test if a list of values are within a tolerance of their counterparts in another list.

    :param goal: list/ndarray/Pose/PoseStamped
    :param actual: list/ndarray/Pose/PoseStamped
    :param tolerance: float
    :returns: bool
    """
    goal = to_list(goal)
    actual = to_list(actual)
    return np.allclose(goal, actual, atol=tolerance)


def to_list(values):
    if isinstance(values, GeometryMsg.PoseStamped):
        output = pose_to_list(values.pose)
    elif isinstance(values, GeometryMsg.Pose):
        output = pose_to_list(values)
    elif isinstance(values, list) or isinstance(values, tuple):
        output = list(values)
    elif isinstance(values, np.ndarray):
        output = list(values)
    else:
        raise NotImplementedError('Type {} cannot be converted to list'.format(type(values)))
    return output


def offset_ros_pose(pose, offset):
    output = GeometryMsg.Pose()
    output.position.x = pose.position.x + offset[0]
    output.position.y = pose.position.y + offset[1]
    output.position.z = pose.position.z + offset[2]
    output.orientation = pose.orientation
    return output


def regularize_pose(pose):
    """It is odd if we do not regularize the pose

    :param pose: geometry_msgs/Pose
    :return:
    """
    pose_mat = sd_pose(pose)
    return to_ros_pose(pose_mat)


def sd_pose(pose):
    """Standardize the input pose to the 4x4 homogeneous transformation
    matrix in special Euclidean group SE(3).

    :param pose:
    :return: transformation matrix
    """
    if isinstance(pose, np.ndarray):
        if pose.ndim == 1 and pose.size == 7:
            t = pose[:3]
            q = pose[3:]
            tm = transform.translation_matrix(t)
            rm = transform.quaternion_matrix(q)
            # make sure to let tm left product rm
            return np.dot(tm, rm)
        elif pose.ndim == 1 and pose.size == 6:
            t = pose[:3]
            rpy = pose[3:]
            tm = transform.translation_matrix(t)
            rm = transform.euler_matrix(rpy[0], rpy[1], rpy[2])
            return np.dot(tm, rm)
        elif pose.shape == (4, 4):
            return pose
        else:
            raise NotImplementedError
    elif isinstance(pose, list):
        return sd_pose(np.array(pose))
    elif isinstance(pose, GeometryMsg.Pose):
        p = pose.position
        o = pose.orientation
        return sd_pose(np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w]))
    elif isinstance(pose, GeometryMsg.PoseStamped):
        p = pose.pose.position
        o = pose.pose.orientation
        return sd_pose(np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w]))
    else:
        raise NotImplementedError


def to_ros_pose(pose):
    """Convert standard pose as 4x4 matrix to ROS geometry msg pose

    :param pose: ndarray, standard pose matrix representing a single pose
    :return: geometry_msgs.Pose
    """
    if isinstance(pose, np.ndarray):
        msg = GeometryMsg.Pose()
        if pose.shape == (4, 4):
            t = transform.translation_from_matrix(pose)
            q = transform.quaternion_from_matrix(pose)
            msg.position.x = t[0]
            msg.position.y = t[1]
            msg.position.z = t[2]
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            return msg
        elif pose.size == 7:
            msg.position.x = pose[0]
            msg.position.y = pose[1]
            msg.position.z = pose[2]
            msg.orientation.x = pose[3]
            msg.orientation.y = pose[4]
            msg.orientation.z = pose[5]
            msg.orientation.w = pose[6]
            return msg
        elif pose.size == 12:
            rotation_matrix = pose[:9].reshape(3, 3)
            translation = pose[9:]
            m = transform.identity_matrix()
            m[:3, :3] = rotation_matrix
            m[:3, 3] = translation
            return to_ros_pose(m)
        else:
            raise NotImplementedError
    else:
        raise NotImplementedError


def to_ros_poses(poses):
    msg = GeometryMsg.PoseArray()
    if isinstance(poses, np.ndarray):
        for pose in poses:
            ros_pose = to_ros_pose(pose)
            msg.poses.append(ros_pose)
        return msg
    else:
        raise NotImplementedError


def sd_position(position):
    if isinstance(position, np.ndarray):
        if position.shape == (3,):
            return position
        else:
            raise NotImplementedError
    elif isinstance(position, list):
        return sd_position(np.array(position))
    elif isinstance(position, GeometryMsg.Point):
        return sd_position(np.array([position.x, position.y, position.z]))
    else:
        raise NotImplementedError


def to_ros_plan(t, p, v=None, a=None):
    """Convert a series of time stamps and positions to ros MoveItMsg.RobotTrajectory msg.
    Note that the msg contains no joint name, which need to be added explicitly.

    :param t: timestamp of shape N
    :param p: way point positions of shape [dim, N]
    :param v: way point velocities of shape [dim, N], could be all 0
    :param a: way point accelerations of shape [dim, N], could be all 0
    :return: MoveItMsg.RobotTrajectory with joint names be empty
    """
    msg = MoveItMsg.RobotTrajectory()
    way_point_num = t.size
    dim = p.shape[0]
    zero_list = np.zeros(dim).tolist()

    for w in range(way_point_num):
        if w == 0:
            continue  # omit the starting point identical to the current pose
        wpt = TrajectoryMsg.JointTrajectoryPoint()

        wpt.positions = list(p[:, w])
        wpt.velocities = zero_list if v is None else list(v[:, w])
        wpt.accelerations = zero_list if a is None else list(a[:, w])
        wpt.time_from_start = rospy.Duration.from_sec(t[w])
        msg.joint_trajectory.points.append(wpt)

    return msg


def get_param(name, value=None):
    """Get ros param from param server

    :param name: String Param name
    :param value: Return value if param is not set
    :return:
    """
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


def encode_image_to_b64(image):
    """Use Base64 to encode the OpenCV image to a string

    :param image: ndarray OpenCV image to be encoded, could
                  be 8UC3 (bgr) or 16UC1 (depth)
    :return: b64_image_str
    """
    _, img_buffer = cv2.imencode('.png', image)
    encoded_buffer = base64.b64encode(img_buffer)
    # cf. https://code-examples.net/en/q/238024b
    return encoded_buffer.decode('utf-8')


def decode_b64_to_image(b64_str, is_bgr=True):
    """Decode the base64 string as OpenCV image, the b64_str is produced
    by encode_image_to_b64 with BGR or depth image.

    :param b64_str: str base64 string produced by encode_image_to_b64
    :param is_bgr: bool If true, the string is treated as BGR (8UC3), otherwise depth (16UC1)
    :return: ok, cv2_image
    """
    if "," in b64_str:
        b64_str = b64_str.partition(",")[-1]
    else:
        b64_str = b64_str

    try:
        img = base64.b64decode(b64_str)
        # imdecode use the same flag as imread. cf. https://docs.opencv.org/3.4/d8/d6a/group__imgcodecs__flags.html
        if is_bgr:
            return True, cv2.imdecode(np.frombuffer(img, dtype=np.uint8), cv2.IMREAD_COLOR)
        else:
            return True, cv2.imdecode(np.frombuffer(img, dtype=np.uint8), cv2.IMREAD_ANYDEPTH)
    except cv2.error:
        return False, None


def byte_to_str(data, n):
    fmt = '!{}c'.format(n)
    char_array = struct.unpack(fmt, data)
    str_out = ''
    for c in char_array:
        s = c.decode('utf-8')
        str_out += s
    return str_out


def byte_to_float(data):
    return struct.unpack('!f', data)[0]


def byte_to_uint32(data):
    return struct.unpack('!I', data)[0]


def byte_to_uint16(data):
    return struct.unpack('!H', data)[0]


def byte_to_uint8(data):
    return struct.unpack('!B', data)[0]


if __name__ == "__main__":
    bgr = cv2.imread("/media/dzp/datasets/graspnet/scenes/scene_0000/realsense/rgb/0000.png")
    bgr_encoded = encode_image_to_b64(bgr)
    print(bgr_encoded)

    ok, image = decode_b64_to_image(bgr_encoded)
    if ok:
        cv2.imshow("Encoded BGR", image)
    else:
        cv2.imshow("Original BGR", bgr)
    cv2.waitKey()

    depth = cv2.imread("/media/dzp/datasets/graspnet/scenes/scene_0000/realsense/depth/0000.png", cv2.IMREAD_ANYDEPTH)
    depth_encoded = encode_image_to_b64(depth)
    print(depth_encoded)

    ok, image = decode_b64_to_image(depth_encoded, is_bgr=False)
    # print(np.allclose(depth, image))
    # Normalize to make the depth more obvious
    cv2.normalize(image, image, alpha=0, beta=65025, norm_type=cv2.NORM_MINMAX)
    if ok:
        cv2.imshow("Encoded Depth", image)
    else:
        cv2.imshow("Original Depth", depth)
    cv2.waitKey()

