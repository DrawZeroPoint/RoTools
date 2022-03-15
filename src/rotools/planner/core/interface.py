import rospy
import json
import numpy as np

from rotools.utility import common


class PlannerInterface(object):
    """

    """

    def __init__(
            self,
            algorithm_port,
            robot_initial_poses,
            planning_initial_poses,
            **kwargs
    ):
        """
        :param algorithm_port: str The port of the algorithm server. It takes the form:
               <ip>:<port>/<process_name>, i.e., localhost:6060/process
        :param robot_initial_poses: list[list] (N, 7) Initial poses of the robot end-effectors
        :param planning_initial_poses: list[list] (N, 7) Initial poses for starting planning,
                                       may be different with robot_initial_poses
        """
        super(PlannerInterface, self).__init__()

        if not isinstance(algorithm_port, str) or algorithm_port == '':
            rospy.logerr('algorithm_port is not defined: {}'.format(algorithm_port))

        self._algorithm_port = algorithm_port

        robot_initial_poses = np.array(robot_initial_poses)
        planning_initial_poses = np.array(planning_initial_poses)
        assert robot_initial_poses.shape[-1] == 7 and planning_initial_poses.shape == robot_initial_poses.shape, \
            rospy.logerr('poses shape should be (N, 7)')

        self._robot_initial_poses = robot_initial_poses
        self._planning_initial_poses = planning_initial_poses
        self._translations = self.set_initial_translations()

    def get_robot_initial_poses(self):
        """The initial poses number is equal to control_topics number,
        where one pose corresponds to a control topic.

        :return: list[PoseStamped] (N) N stands for the topics number.
        """
        out_list = []
        for pose in self._robot_initial_poses:
            pose_stamped_msg = common.to_ros_pose_stamped(pose)
            out_list.append(pose_stamped_msg)
        return out_list

    def set_initial_translations(self):
        """Given the initial poses in the robot and planning frames, get the translation from
        the robot frame to the planning frame.

        """
        translations = []
        for r_pose, p_pose in zip(self._robot_initial_poses, self._planning_initial_poses):
            translations.append(common.get_transform_same_target(r_pose, p_pose))
        return translations

    def get_plan(self):
        ok, trajectories = self._post_data()
        if not ok or not trajectories:
            return False, None
        out_list = []
        for poses in trajectories:
            assert len(poses) == len(self._translations), rospy.logerr('Poses number is not equal to translations')
            pose_msg_list = []
            for i, pose in enumerate(poses):
                std_pose = common.sd_pose(pose)  # pose in the planning frame
                std_robot_pose = np.dot(self._translations[i], std_pose)  # pose in the robot frame
                pose_stamped_msg = common.to_ros_pose_stamped(std_robot_pose)
                pose_msg_list.append(pose_stamped_msg)
            out_list.append(pose_msg_list)
        return True, out_list

    def _post_data(self):
        """Post the initial pose data to the algorithm server.
        Run the Flask server first to make this work.

        :return: ok bool If success
                 results list The results represent N trajectories for defined end-effectors (or body parts)
                 Its shape is (T, N, 7), where T is the rollout number, N is the ee number, 7 stands for the
                 pose [x, y, z, qx, qy, qz, qw] for each way point in the trajectory. If only the next state
                 is queried, then T=1. Note that the positions in the trajectory is defined with delta value
                 relative to the planning start point (set at the planner side, not here)
        """
        header = {}
        body = {'data': json.dumps(self._planning_initial_poses.tolist())}
        payload = {'header': header, 'body': body}

        feedback = common.post_http_requests('http://{}'.format(self._algorithm_port), payload=payload)
        if feedback is None:
            return False, None
        # print(feedback)
        results = feedback.json()
        # The keys 'status' and 'results' should be coincide with the definition in the Flask server
        ok = results['response']['status']
        if ok:
            return True, json.loads(results['response']['results'])
        else:
            print('Post data failed to load results')
            return False, None
