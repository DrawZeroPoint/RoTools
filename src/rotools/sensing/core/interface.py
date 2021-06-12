from __future__ import print_function

import json
import requests
import numpy as np

try:
    import rospy
    import tf2_ros
    from cv_bridge import CvBridge

    import geometry_msgs.msg as GeometryMsg
    import std_msgs.msg as StdMsg
    import sensor_msgs.msg as SensorMsg
except ImportError:
    pass

from rotools.utility import common
from roport.srv import GetImageData, GetImageDataRequest


class SensingInterface(object):

    def __init__(
            self,
            device_names,
            algorithm_ports,
            algorithm_names=None,
    ):
        super(SensingInterface, self).__init__()

        self._bridge = CvBridge()
        self._pose_publisher = rospy.Publisher('roport_visualization/pose', GeometryMsg.PoseStamped, queue_size=1)

        if not isinstance(device_names, list) and not isinstance(device_names, tuple):
            raise TypeError('device_names should be list or tuple, but got {}'.format(type(device_names)))

        assert len(device_names) > 0, rospy.logerr('No sensing device given')
        self.device_names = device_names
        self.device_ids = np.arange(len(self.device_names))
        for i, name in enumerate(self.device_names):
            rospy.loginfo('Assigning id {} to the device {}'.format(i, name))

        # TODO check device status

        assert len(algorithm_ports) > 0, rospy.logerr('No sensing algorithm given')
        self.algorithm_ports = algorithm_ports
        self.algorithm_ids = np.arange(len(self.algorithm_ports))

        if algorithm_names:
            assert len(algorithm_names) == len(self.algorithm_ports)
            self.algorithm_names = algorithm_names
            for name, port in zip(self.algorithm_names, self.algorithm_ports):
                rospy.loginfo('Algorithm \'{}\' available at {}'.format(name, port))
        else:
            self.algorithm_names = None

        # TODO check algorithm service status

    def sense_manipulation_poses(self, device_names, algorithm_id, data_types=None):
        """Sensing the manipulation poses (i.e., the poses that the robot's end-effector
        should move to to perform manipulation.) By default, these poses are wrt the
        sensor's frame.
        
        :param device_names: str Names of the device.
        :param algorithm_id: int ID of the algorithm. 
        :param data_types: int Type ID of the data. If not given, the method will guess
                           it based on the device name.
        """
        for name in device_names:
            assert name in self.device_names, rospy.logerr('Device name {} is not registered'.format(name))
        assert algorithm_id in self.algorithm_ids

        if data_types is None:
            temp_types = []
            for name in device_names:
                if 'rgb' in name or 'RGB' in name or 'color' in name:
                    temp_types.append(0)
                elif 'depth' in name:
                    temp_types.append(1)
                else:
                    raise NotImplementedError
            data_types = temp_types

        # Get sensory data from the devices via the service provided by the robot
        # TODO add more modalities of data other than image
        ok, data = self._get_image_data_client(device_names)
        if not ok:
            return False, None, None
        # Send sensory data to the algorithm and get the poses
        ok, sd_poses = self._post_data(self.algorithm_ports[algorithm_id], data, data_types)
        if ok:
            return True, common.to_ros_poses(sd_poses), common.to_ros_pose(sd_poses[0])
        else:
            return False, None, None

    def _get_image_data_client(self, device_names, service_name=None):
        """This client call the get image data service provided by
        the robot API to obtain required data from given devices.

        :param device_names: list[str] A list of names for the devices providing the data,
                             we assume one device only produce one kind of data
        :param service_name: str
        :return: ok, list[ndarray]
        """
        if service_name is None:
            service_name = 'get_image_data'
        rospy.wait_for_service(service_name, 1000)  # wait for 1s
        try:
            get_img = rospy.ServiceProxy(service_name, GetImageData)
            req = GetImageDataRequest()
            req.device_names = device_names
            resp = get_img(req)
            if resp.result_status == resp.SUCCEEDED:
                image_list = []
                for img_msg in resp.images:
                    cv_img = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
                    image_list.append(cv_img)
                return True, image_list
            else:
                rospy.logwarn("Get image data failed")
                return False, None
        except rospy.ServiceException as e:
            # Retry 5 times
            cnt = 5
            while cnt:
                ok, res = self._get_image_data_client(device_names, service_name)
                if ok:
                    return ok, res
                else:
                    cnt -= 1
            rospy.logerr("Service call failed: %s" % e)
            return False, None

    def _post_data(self, port, data, data_types):
        """Post the sensory data to the algorithm server.
        Run the Flask server first to make this work.

        :param port: str The port of the algorithm server. It takes the form:
                     <ip>:<port>/<process_name>, i.e., localhost:6060/process
        :param data: ndarray Sensory data
        :param data_types: Types of the data, could be 0:8UC3 (bgr image), 1:16UC1 (depth image)
        :return: ok bool If success
                 results ndarray Results representing poses
        """
        header = {}
        type_list = []
        data_list = []
        for d, dt in zip(data, data_types):
            if dt == 0:
                encoded = common.encode_image_to_b64(d)
            elif dt == 1:
                encoded = common.encode_image_to_b64(d)
            else:
                raise NotImplementedError
            type_list.append(dt)
            data_list.append(str(encoded))

        body = {'data': json.dumps(data_list), 'data_types': json.dumps(data_types)}
        payload = {'header': header, 'body': body}
        feedback = self._post_http_requests('http://{}'.format(port), payload=payload)
        results = feedback.json()
        # The keys 'status' and 'results' should be coincide with the definition in the Flask server
        ok = results['response']['status']
        if ok:
            return True, np.array(json.loads(results['response']['results']))
        else:
            rospy.logerr('Post data failed to load results')
            return False, None

    @staticmethod
    def _post_http_requests(url, payload, headers=None, params=None):
        """Send HTTP request and get back the results as a dict of string

        :param url: str URL for sending request
        :param payload: dict, corresponding to json.loads(Flask.request.data)
        :param headers: dict, corresponding to Flask.request.headers
        :param params: dict, corresponding to Flask.request.args
        :return: feedback
        """
        json_data = json.dumps(payload)
        if params is None and headers is None:
            return requests.post(url, data=json_data)
        else:
            return requests.post(url, headers=headers, params=params, data=json_data)

    def visualize_pose(self, pose, frame):
        pose_msg = GeometryMsg.PoseStamped()
        pose_msg.pose = pose
        pose_msg.header.frame_id = frame
        self._pose_publisher.publish(pose_msg)
