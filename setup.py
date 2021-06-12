## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

# setuptools is not recommended since it generates files into your src folder
# https://wiki.ros.org/rospy_tutorials/Tutorials/Makefile
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['rotools',
                 'rotools.cnn', 'rotools.cnn.vae',
                 'rotools.moveit', 'rotools.moveit.core',
                 'rotools.policy', 'rotools.policy.rmp_flow',
                 'rotools.robot', 'rotools.robot.serial',
                 'rotools.simulation',
                 'rotools.utility',
                 ]
d['package_dir'] = {'': 'src'}
# d['scripts'] = ['scripts/test_moveit.py']

setup(**d)
