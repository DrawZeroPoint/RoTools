import socket
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose,Point,Quaternion
import re

def data_process(data):
    Position = re.findall(r'Position\s*:\s*(.*)', data)
    Orientation = re.findall(r'Orientation\s*:\s*(.*)', data)
    Position = Position[0]
    Orientation = Orientation[0]
    return eval(Position),eval(Orientation)

def talker():
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #connect to windows IP
    client.connect(('192.168.13.118', 6688))
    print('connected')
    pub = rospy.Publisher('rigid_body_pose', Pose, queue_size=10)
    rospy.init_node('rigid_body_pose_publisher')
    rate = rospy.Rate(100)  # 100hz
    pose = Pose()

    while not rospy.is_shutdown():
        rec_data = client.recv(1024)
        rec_data = rec_data.decode('utf-8')
        position,orientation = data_process(rec_data)
        pose.position = Point(position[0],position[1],position[2])
        pose.orientation = Quaternion(orientation[0],orientation[1],orientation[2],orientation[3])
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        print(pose)
        pub.publish(pose)
        rate.sleep()
        str = 'ok'
        client.send(str.encode('utf-8'))


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


