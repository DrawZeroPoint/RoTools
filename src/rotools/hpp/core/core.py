class Object(object):
    rootJointType = 'freeflyer'
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__(self, object_name, pkg_name):
        self.__setattr__('urdfName', object_name)
        self.__setattr__('packageName', pkg_name)
        self.__setattr__('meshPackageName', pkg_name)


class Environment(object):
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__(self, env_name, pkg_name):
        self.__setattr__('urdfName', env_name)
        self.__setattr__('packageName', pkg_name)
        self.__setattr__('meshPackageName', pkg_name)


class Model(object):

    def __init__(self, name, pkg_name, urdf_name='', srdf_name='', surface='', handle=''):
        self._name = name
        self._pkg_name = pkg_name
        if urdf_name == '':
            self._urdf_name = name
        else:
            self._urdf_name = urdf_name
        if srdf_name == '':
            self._srdf_name = name
        else:
            self._srdf_name = srdf_name
        self._surface = surface
        self._handle = handle

    @property
    def name(self):
        return self._name

    @property
    def pkg_name(self):
        return self._pkg_name

    @property
    def urdf_name(self):
        return self._urdf_name

    @property
    def srdf_name(self):
        return self._srdf_name

    @property
    def surface(self):
        return self._surface

    @property
    def handle(self):
        return self._handle


class Gripper(object):

    def __init__(self, robot_model, name, fingers, finger_joints, joint_values):
        """

        Args:
            robot_model:
            name:
            fingers:
            finger_joints:
            joint_values:
        """
        super(Gripper, self).__init__()

        self._dof = 2
        assert len(fingers) == len(finger_joints) == len(joint_values) == self._dof

        self._fingers = fingers
        self._finger_joints = finger_joints
        self._joint_values = joint_values

        self._name = name
        self._robot_model = robot_model

    @property
    def dof(self):
        return self._dof

    @property
    def name(self):
        return self._name

    @property
    def fingers(self):
        return self._fingers

    @property
    def joints(self):
        return self._finger_joints

    @property
    def values(self):
        return self._joint_values

    def get_urdf(self):
        pass
