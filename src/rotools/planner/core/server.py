import rospy

from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped

import rotools.planner.core.interface as interface

from rotools.utility.emergency_stop import EStop
from rotools.utility.common import play_hint_sound


class PlannerServer(EStop):
    """The planner server exports the planning interface to the user.
    The interface as a planner is defined as a function that given the current state s_t,
    it computes s_t+1 ... s_t+T, where T >= 1. Based on the estimated states,
    the server will publish corresponding topics to control the robot to traverse these states.
    Note that the control is done in a best-effort way, that there is no guarantee that the
    robot will physically visit every state.
    """

    def __init__(self, kwargs):
        super(PlannerServer, self).__init__(function_name='Planner execution')
        modes = {0: 'step once', 1: 'loop once', 2: 'infinite loop'}

        mode = kwargs['mode']
        assert mode in [0, 1, 2], rospy.logerr('Mode should be 0 (step once), 1 (loop once), or 2 (infinite loop),'
                                               'but {} was given'.format(mode))
        self._mode = mode
        rospy.loginfo('Planner server running in mode: {}'.format(modes[self._mode]))

        # Publisher switch
        self.srv_pub_switch = rospy.Service('/planner/enable', SetBool, self.pub_switch_handle)

        self.srv_initialize = rospy.Service('/planner/initialize', Trigger, self.initialize_handle)

        self.interface = interface.PlannerInterface(**kwargs)

        # Cartesian pose publishers
        control_topics = kwargs['control_topics']
        assert isinstance(control_topics, list), rospy.logerr('control_topics is not a list')
        self._control_publishers = []
        for topic in control_topics:
            publisher = rospy.Publisher(topic, PoseStamped, queue_size=1)
            self._control_publishers.append(publisher)
        self._n_topics = len(self._control_publishers)

        self._pose_initialized = False
        self.rate = kwargs['rate']
        # Planner query rate is set as 100 Hz, note that this is not equal to the actual execution rate
        self.plan_execution_timer = rospy.Timer(rospy.Duration.from_sec(0.01), self.execute_plan_handle)

    def pub_switch_handle(self, req):
        if req.data:
            self.set_status(True)
            msg = 'Plan execution enabled'
        else:
            self.set_status(False)
            msg = 'Plan execution disabled'
        play_hint_sound(req.data)
        return SetBoolResponse(True, msg)

    def initialize_handle(self, req):
        self._initialize()
        rospy.loginfo('Initial poses of the robot have been published.')
        return TriggerResponse(True, 'Initialized')

    def _initialize(self):
        initial_poses = self.interface.get_robot_initial_poses()
        self._publish_poses(initial_poses)
        # TODO check if initialization is succeeded
        self._pose_initialized = True

    def execute_plan_handle(self, event):
        if not self.enabled:
            return
        if not self._pose_initialized:
            rospy.logwarn_throttle(3, 'Poses not initialized, call the service /planner/initialize first')
            return
        ok, trajectories = self.interface.get_plan()
        if ok:
            if self._mode == 0:
                target_poses = trajectories[0]
                self._publish_poses(target_poses)
                self.set_status(False)
                self._pose_initialized = False
            elif self._mode == 1:
                for target_poses in trajectories:
                    if self.enabled:
                        self._publish_poses(target_poses)
                        rospy.sleep(rospy.Duration.from_sec(1. / self.rate))
                self.set_status(False)
                self._pose_initialized = False
            else:
                cnt = 0
                while self.enabled:
                    target_poses = trajectories[cnt]
                    self._publish_poses(target_poses)
                    rospy.sleep(rospy.Duration.from_sec(1. / self.rate))
                    cnt += 1
                    if cnt >= len(trajectories):
                        self._initialize()
                        rospy.sleep(rospy.Duration.from_sec(1. / self.rate))
                        cnt = 0
                self._pose_initialized = False
        else:
            rospy.logwarn_throttle(3, 'Cannot get a plan')

    def _publish_poses(self, poses):
        """The poses order should be semantically the same as the publishers.

        """
        assert len(poses) == self._n_topics, \
            rospy.logerr('Target poses {} and publisher number {} mismatch'.format(len(poses), self._n_topics))
        for i, pose in enumerate(poses):
            assert isinstance(pose, PoseStamped), rospy.logerr('Target pose type is not PoseStamped')
            self._control_publishers[i].publish(pose)
