import rospy
from geometry_msgs.msg import PoseArray, PoseStamped

import os
import click
import numpy as np

# import sys
# script_dir = os.path.abspath(os.path.dirname(__file__))
# sys.path.insert(0, script_dir + os.sep + "..")

from rotools.utility import common

root_dir = '/home/smart/data'
print('Project root dir {}'.format(root_dir))


poses_pub = rospy.Publisher('/rollout', PoseArray, queue_size=1)
left_pub = rospy.Publisher('/cartesian/left_hand/reference', PoseStamped, queue_size=1)
right_pub = rospy.Publisher('/cartesian/right_hand/reference', PoseStamped, queue_size=1)


def publish_rollout(rollout, rate):
    n_rollout = len(rollout)
    for i, unit in enumerate(rollout):
        pose_array = PoseArray()
        left_pose_stamped = PoseStamped()
        right_pose_stamped = PoseStamped()
        pose_array.header.frame_id = 'map'
        left_pose_stamped.header.frame_id = 'ci/torso'
        right_pose_stamped.header.frame_id = 'ci/torso'
        left_p_quat = unit[0, :]
        right_p_quat = unit[1, :]
        left_pose = common.to_ros_pose(left_p_quat)
        right_pose = common.to_ros_pose(right_p_quat)
        pose_array.poses.extend([left_pose, right_pose])
        poses_pub.publish(pose_array)

        left_pose_stamped.pose = left_pose
        right_pose_stamped.pose = right_pose
        left_pub.publish(left_pose_stamped)
        right_pub.publish(right_pose_stamped)

        rospy.loginfo('Step: {}/{}'.format(i, n_rollout))
        rospy.sleep(rospy.Duration.from_sec(1.0 / rate))


@click.command()
@click.option('-n', '--name', default='ctrl_rollout')
@click.option('-r', '--rate', default=10.0)
def run(name, rate):
    rollout_path = os.path.join(root_dir, name + '.npy')
    if not os.path.exists(rollout_path):
        rospy.logerr('Rollout path {} is empty'.format(rollout_path))
        return
    rollout = np.load(rollout_path)
    if rollout.shape[-1] == 0:
        rospy.logwarn('Rollout {} is empty'.format(rollout_path))
    else:
        publish_rollout(rollout, rate)


if __name__ == "__main__":
    try:
        rospy.init_node('rollout_visualization')
        run()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
