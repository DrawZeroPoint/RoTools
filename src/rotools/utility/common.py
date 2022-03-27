import os
import cv2
import json
import base64
import struct
import requests
import numpy as np

try:
    import rospy
    import geometry_msgs.msg as geo_msg
    import moveit_msgs.msg as moveit_msg
    import trajectory_msgs.msg as traj_msg
    from moveit_commander.conversions import pose_to_list
except ImportError:
    rospy = None
    geo_msg = None
    moveit_msg = None
    traj_msg = None
    pose_to_list = None

from rotools.utility import transform


def print_debug(content):
    """Print information with green."""
    print(''.join(['\033[1m\033[92m', content, '\033[0m']))


def print_info(content):
    """Print information with sky blue."""
    print(''.join(['\033[1m\033[94m', content, '\033[0m']))


def print_warn(content):
    """Print warning with yellow."""
    print(''.join(['\033[1m\033[93m', content, '\033[0m']))


def print_error(content):
    """Print error with red."""
    print(''.join(['\033[1m\033[91m', content, '\033[0m']))


def all_close(values_a, values_b, tolerance=1.e-8):
    """Test if a series of values are all within a tolerance of their counterparts in another series.

    Args:
        values_a: list/ndarray/Pose/PoseStamped
        values_b: list/ndarray/Pose/PoseStamped
        tolerance: float

    Returns:
        True if two arrays are element-wise equal within a tolerance.
    """
    return np.allclose(to_list(values_a), to_list(values_b), atol=tolerance)


def to_list(values):
    """Convert a series of values in various structure to a plain python list.

    Args:
        values: PoseStamped/Pose/list/tuple/ndarray

    Returns:
        A list of plain python number types.
    """
    if isinstance(values, geo_msg.PoseStamped):
        output = pose_to_list(values.pose)
    elif isinstance(values, geo_msg.Pose):
        output = pose_to_list(values)
    elif isinstance(values, list) or isinstance(values, tuple):
        output = list(values)
    elif isinstance(values, np.ndarray):
        output = values.tolist()
    else:
        raise NotImplementedError('Type {} cannot be converted to list'.format(type(values)))
    return output


def offset_ros_pose(pose, offset):
    """Translate the position of the given pose by offset.
    The orientation will not be changed.

    Args:
        pose: Pose/PoseStamped
        offset: list/tuple/ndarray

    Returns:
        Translated pose of the same type as the input pose.
    """
    if isinstance(pose, geo_msg.Pose):
        output = geo_msg.Pose()
        output.position.x = pose.position.x + offset[0]
        output.position.y = pose.position.y + offset[1]
        output.position.z = pose.position.z + offset[2]
        output.orientation = pose.orientation
    elif isinstance(pose, geo_msg.PoseStamped):
        output = geo_msg.PoseStamped()
        output.header = pose.header
        output.pose.position.x = pose.pose.position.x + offset[0]
        output.pose.position.y = pose.pose.position.y + offset[0]
        output.pose.position.z = pose.pose.position.z + offset[0]
        output.pose.orientation = pose.pose.orientation
    else:
        raise NotImplementedError('Type {} is not supported'.format(type(pose)))
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
    elif isinstance(pose, geo_msg.Pose):
        p = pose.position
        o = pose.orientation
        return sd_pose(np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w]))
    elif isinstance(pose, geo_msg.PoseStamped):
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
        msg = geo_msg.Pose()
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


def to_ros_pose_stamped(pose, frame_id=''):
    """Convert a pose to PoseStamped.

    Args:
        pose: list/ndarray A 1-D array of a pose.
        frame_id: str The pose's reference frame.

    Returns:
        PoseStamped.
    """
    pose_stamped = geo_msg.PoseStamped()
    ros_pose = to_ros_pose(pose)
    pose_stamped.pose = ros_pose
    pose_stamped.header.frame_id = frame_id
    return pose_stamped


def to_ros_poses(poses):
    """Convert a series of poses into ROS PoseArray.

    Args:
        poses: ndarray A 2-D array containing poses.

    Returns:
        PoseArray.
    """
    msg = geo_msg.PoseArray()
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
    elif isinstance(position, geo_msg.Point):
        return sd_position(np.array([position.x, position.y, position.z]))
    else:
        raise NotImplementedError


def sd_orientation(orientation):
    """Standardize the input to a np array representation of orientation.
    The order should be qx, qy, qz, qw.
    """
    if isinstance(orientation, np.ndarray):
        if orientation.shape == (4,):
            return orientation
        else:
            raise NotImplementedError
    elif isinstance(orientation, list):
        return sd_orientation(np.array(orientation))
    elif isinstance(orientation, geo_msg.Quaternion):
        return sd_orientation(np.array([orientation.x, orientation.y, orientation.z, orientation.w]))
    else:
        raise NotImplementedError


def to_ros_orientation(orientation):
    """Convert standard orientation as 4x4 matrix to ROS geometry msg quaternion

    :param orientation: ndarray, standard pose matrix representing a single orientation
    :return: geometry_msgs.Quaternion
    """
    if isinstance(orientation, np.ndarray):
        msg = geo_msg.Quaternion()
        if orientation.shape == (4, 4):
            q = transform.quaternion_from_matrix(orientation)
            msg.x = q[0]
            msg.y = q[1]
            msg.z = q[2]
            msg.w = q[3]
            return msg
        elif orientation.shape == (3, 3):
            i = transform.identity_matrix()
            i[:3, :3] = orientation
            return to_ros_orientation(i)
        elif orientation.size == 4:
            msg.x = orientation[0]
            msg.y = orientation[1]
            msg.z = orientation[2]
            msg.w = orientation[3]
            return msg
        else:
            raise NotImplementedError
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
    msg = moveit_msg.RobotTrajectory()
    way_point_num = t.size
    dim = p.shape[0]
    zero_list = np.zeros(dim).tolist()

    for w in range(way_point_num):
        if w == 0:
            continue  # omit the starting point identical to the current pose
        wpt = traj_msg.JointTrajectoryPoint()

        wpt.positions = list(p[:, w])
        wpt.velocities = zero_list if v is None else list(v[:, w])
        wpt.accelerations = zero_list if a is None else list(a[:, w])
        wpt.time_from_start = rospy.Duration.from_sec(t[w])
        msg.joint_trajectory.points.append(wpt)

    return msg


def get_relative_rotation(rot_1, rot_2):
    """Given a rotation from base to 1: rot_1 and a rotation rot_2,
    get the rotation matrix from 1 to 2 with Euler angles (sxyz)

    """
    R_1 = transform.quaternion_matrix(sd_orientation(rot_1))
    R_2 = transform.quaternion_matrix(sd_orientation(rot_2))
    R = np.dot(np.linalg.inv(R_1), R_2)
    ax, ay, az = transform.euler_from_matrix(R)
    return R, np.array([ax, ay, az])


def get_transform_same_origin(start, end):
    """Given 4x4 transform matrices t_wa from frame w to frame a,
    and t_wb from frame w to frame b, get the transform t_ab from
    frame a to frame b.

    :param start: array (4, 4) Transform from the origin frame to the start frame.
    :param end: array (4, 4) Transform from the origin frame to the end frame.
    :return: array (4, 4) Transform from the start frame to the end frame.
    """
    sd_start = sd_pose(start)
    sd_end = sd_pose(end)
    return np.dot(np.linalg.inv(sd_start), sd_end)


def get_transform_same_target(start, end):
    """Given 4x4 transform matrices t_aw from frame a to frame w,
    and t_bw from frame b to frame w, get the transform t_ab from
    frame a to frame b.

    :param start: array (4, 4) Transform from the start frame to the target frame.
    :param end: array (4, 4) Transform from the end frame to the target frame.
    :return: array (4, 4) Transform from the start frame to the end frame.
    """
    sd_start = sd_pose(start)
    sd_end = sd_pose(end)
    return np.dot(sd_start, np.linalg.inv(sd_end))


def local_pose_to_global_pose(local_to_target, global_to_local):
    if type(local_to_target) != type(global_to_local):
        print_warn(
            "Local to target pose type {} is not the same as global to local pose {}".format(
                type(local_to_target), type(global_to_local))
        )
    sd_local_to_target = sd_pose(local_to_target)
    sd_global_to_local = sd_pose(global_to_local)
    sd_global_to_target = np.dot(sd_global_to_local, sd_local_to_target)
    if isinstance(local_to_target, geo_msg.Pose):
        return to_ros_pose(sd_global_to_target)
    elif isinstance(local_to_target, geo_msg.PoseStamped):
        return to_ros_pose_stamped(sd_global_to_target, local_to_target.header.frame_id)
    else:
        raise NotImplementedError


def local_aligned_pose_to_global_pose(local_aligned_to_target, global_to_local):
    if type(local_aligned_to_target) != type(global_to_local):
        print_warn(
            "Local aligned to target pose type {} is not the same as global to local pose {}".format(
                type(local_aligned_to_target), type(global_to_local))
        )
    sd_local_aligned_to_target = sd_pose(local_aligned_to_target)
    sd_global_to_local_aligned = sd_pose(global_to_local)
    sd_global_to_local_aligned[:3, :3] = np.eye(3)
    sd_global_to_target = np.dot(sd_global_to_local_aligned, sd_local_aligned_to_target)
    if isinstance(local_aligned_to_target, geo_msg.Pose):
        return to_ros_pose(sd_global_to_target)
    elif isinstance(local_aligned_to_target, geo_msg.PoseStamped):
        return to_ros_pose_stamped(sd_global_to_target, local_aligned_to_target.header.frame_id)
    else:
        raise NotImplementedError


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


def get_path(param_name, value=None):
    """Get an absolute path str from the given ROS param.
    If the param provides a relative path, will find it under ~

    Args:
        param_name: str ROS param name.
        value: str Default value if param does not exist.

    Returns:
        str Got absolute path.
    """
    path = get_param(param_name, value)
    if path is None:
        raise FileNotFoundError('Failed to get the path from {}'.format(param_name))
    if path.startswith('/'):
        abs_path = path
    else:
        abs_path = os.path.join(os.path.join(os.path.expanduser('~'), path))
    if os.path.exists(abs_path):
        return abs_path
    raise FileNotFoundError('Path {} does not exist'.format(abs_path))


def pretty_print_configs(configs):
    """Print a dict of configurations in a visual friendly and organized way.

    Args:
        configs: dict A dict of configures. The items could be string, number, or a list/tuple.

    Returns:
        None
    """
    max_key_len = 0
    max_value_len = 0
    for key, value in configs.items():
        key_str = '{}'.format(key)
        if len(key_str) > max_key_len:
            max_key_len = len(key_str)
        if isinstance(value, list) or isinstance(value, tuple):
            for i in value:
                i_str = '{}'.format(i)
                if len(i_str) > max_value_len:
                    max_value_len = len(i_str)
        else:
            value_str = '{}'.format(value)
            if len(value_str) > max_value_len:
                max_value_len = len(value_str)

    print_info("\n{}{}{}".format('=' * (max_key_len + 1), " ROPORT CONFIGS ", '=' * (max_value_len - 15)))
    for key, value in configs.items():
        key_msg = '{message: <{width}}'.format(message=key, width=max_key_len)
        empty_key_msg = '{message: <{width}}'.format(message='', width=max_key_len)
        if isinstance(value, list) or isinstance(value, tuple):
            for i, i_v in enumerate(value):
                if i == 0:
                    print_info('{}: {}'.format(key_msg, i_v))
                else:
                    print_info('{}: {}'.format(empty_key_msg, i_v))
        else:
            print_info('{}: {}'.format(key_msg, value))
    print_info("{}{}{}\n".format('=' * (max_key_len + 1), " END OF CONFIGS ", '=' * (max_value_len - 15)))


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


def post_http_requests(url, payload, headers=None, params=None):
    """Send HTTP request and get back the results as a dict of string.

    :param url: str URL for sending request
    :param payload: dict, corresponding to json.loads(Flask.request.data)
    :param headers: dict, corresponding to Flask.request.headers
    :param params: dict, corresponding to Flask.request.args
    :return: feedback
    """
    json_data = json.dumps(payload)
    if params is None and headers is None:
        try:
            return requests.post(url, data=json_data)
        except requests.exceptions.RequestException as e:
            rospy.logerr(e)
            return None
    else:
        try:
            return requests.post(url, headers=headers, params=params, data=json_data)
        except requests.exceptions.RequestException as e:
            rospy.logerr(e)
            return None


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


def is_ip_valid(ip):
    if not isinstance(ip, str):
        print_error("IP is not a string")
        return False
    if len(ip.split('.')) != 4:
        print_error("IP {} is illegal".format(ip))
        return False
    if ':' in ip:
        print_error("IP {} should not contain port number".format(ip))
        return False
    return True


def is_port_valid(port):
    if not isinstance(port, int):
        print_error("Port is not a int")
        return False
    return True


def play_hint_sound(enable):
    """Play a hint sound according to if enabled.

    Args:
        enable: bool If true, play the sound 'Function activated'.
                Otherwise, play 'Function deactivated'.

    Returns:
        None
    """
    try:
        # TODO check playsound could work on Python 3
        from playsound import playsound
        import os.path as osp
        misc_dir = osp.dirname(osp.dirname(osp.dirname(osp.dirname(osp.realpath(__file__)))))
        if enable:
            misc_path = osp.join(misc_dir, 'misc/audio/Sophia_function_activated.mp3')
        else:
            misc_path = osp.join(misc_dir, 'misc/audio/Sophia_function_deactivated.mp3')
        playsound(misc_path)  # Not support block=True on Ubuntu
    except ImportError as e:
        rospy.logdebug('Sound not played due to missing dependence: {}'.format(e))


def wait_for_service(srv, time=0.2):
    try:
        rospy.wait_for_service(srv, time)
    except rospy.ROSException:
        rospy.logwarn("Waiting for service: {0}".format(srv))
        rospy.wait_for_service(srv)
        rospy.logwarn("Service {0} found.".format(srv))


def create_publishers(namespace, topic_ids, topic_types, queue_size=1):
    """Create Publishers to topics.

    Args:
        namespace: str Namespace of the topic_ids.
        topic_ids: list[str] List containing the topic_ids.
        topic_types: list[object] List containing the types of the topics.
        queue_size: int

    Returns:
        Publishers in a dict indexed by topic_ids.
    """
    assert isinstance(topic_ids, list) or isinstance(topic_ids, str)
    assert len(topic_ids) == len(topic_types)

    if isinstance(topic_ids, list):
        publishers = dict()
        for topic_id, topic_type in zip(topic_ids, topic_types):
            publishers[topic_id] = create_publishers(namespace, topic_id, topic_type, queue_size)
        return publishers
    else:
        return rospy.Publisher(namespace + '/' + topic_ids, topic_types, queue_size=queue_size)


def create_service_proxies(namespace, service_ids, service_types):
    """Create a dict of ServiceProxy.

    Args:
        namespace: str
        service_ids: list[str]
        service_types: list[object]

    Returns:
        ServiceProxy in a dict indexed by service_ids.
    """
    assert isinstance(service_ids, list) or isinstance(service_ids, str)
    assert len(service_ids) == len(service_types)

    if isinstance(service_ids, list):
        service_proxies = dict()
        for service_id, service_type in zip(service_ids, service_types):
            service_proxies[service_id] = create_service_proxies(namespace, service_id, service_type)
        return service_proxies
    else:
        service_id = namespace + '/' + service_ids
        wait_for_service(service_id)
        return rospy.ServiceProxy(service_id, service_types)


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
