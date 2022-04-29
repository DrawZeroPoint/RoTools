import numpy as np  # type: ignore


def ur10e_mdh_model():
    """Get UR10e MDH model.

    References:
        https://github.com/ros-industrial/universal_robot/blob/melodic-devel/ur_e_description/urdf/ur10e.urdf.xacro

    Returns:
        ndarray, ndarray MDH params and joint limits.
    """

    mdh = np.array([
        [0, 0, 0, 0.1807],
        [np.pi / 2, 0, np.pi, 0],
        [0, 0.6127, 0, 0],
        [0, 0.57155, 0, 0.17415],
        [-np.pi / 2, 0, 0, 0.11985],
        [np.pi / 2, 0, np.pi, 0.11655],
    ])

    q_limits = np.array([
        [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi],
    ])

    return mdh, q_limits


def ur10e_poe_model():
    """Get UR10e product of exponential model.

    Notes:
        This model use the same base frame as the MDH model, and this base frame is not the same as the one
        in the "Modern Robotics" pg. 146. If that one is preferred, use this:

        def ur10e_poe_model():

            M = np.array([
                [-1, 0, 0, 1.18425],  # L1+L2
                [0, 0, 1, 0.2907],  # W1+W2
                [0, 1, 0, 0.06085],  # H1-H2
                [0, 0, 0, 1],
            ])

            screw_axes = np.array([
                [0, 0, 1, 0, 0, 0],
                [0, 1, 0, -0.1807, 0, 0],  # -H1
                [0, 1, 0, -0.1807, 0, 0.6127],  # -H1 L1
                [0, 1, 0, -0.1807, 0, 1.18425],  # -H1 L1+L2
                [0, 0, -1, -0.17415, 1.18425, 0],  # -W1 L1+L2
                [0, 1, 0, -0.06085, 0, 1.18425],  # H2-H1 L1+L2
            ]).T

            q_limits = np.array([
                [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi],
            ])

            return [M, screw_axes], q_limits

    Returns:
        list[ndarray], ndarray. POE params and joint limits.
    """

    M = np.array([
        [1, 0, 0, -1.18425],  # L1+L2
        [0, 0, -1, -0.2907],  # W1+W2
        [0, 1, 0, 0.06085],  # H1-H2
        [0, 0, 0, 1],
    ])

    screw_axes = np.array([
        [0, 0, 1, 0, 0, 0],
        [0, -1, 0, 0.1807, 0, 0],  # -H1
        [0, -1, 0, 0.1807, 0, 0.6127],  # -H1 L1
        [0, -1, 0, 0.1807, 0, 1.18425],  # -H1 L1+L2
        [0, 0, -1, 0.17415, -1.18425, 0],  # -W1 L1+L2
        [0, -1, 0, 0.06085, 0, 1.18425],  # H2-H1 L1+L2
    ]).T

    q_limits = np.array([
        [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi], [-np.pi, np.pi],
    ])

    return [M, screw_axes], q_limits


def panda_mdh_model():
    """MDH parameters in 'alpha, a, theta, d' order, with units are meter or radian.

    Notes:
        'alpha' is z-axis' rotation around x-axis
        'a' is the translation distance along +x-axis
        'theta' is x-axis' rotation around z-axis
        'd' is the translation distance along +z-axis

    References:
        https://raw.githubusercontent.com/petercorke/robotics-toolbox-matlab/master/models/mdl_panda.m

    Returns:
        ndarray of size (7, 4), the MDH parameters.
    """
    mdh = np.array(
        [
            [0, 0, 0, 0.333],  # theta in [-2.8973 2.8973] 0 -> 1
            [-np.pi / 2, 0, 0, 0],  # theta in [-1.7628 1.7628] 1 -> 2
            [np.pi / 2, 0, 0, 0.316],  # theta in [-2.8973 2.8973] 2 -> 3
            [np.pi / 2, 0.088, 0, 0],  # theta in [-3.0718 -0.0698], note the ref value of theta is 0, 3 -> 4
            [-np.pi / 2, -0.088, 0, 0.384],  # theta in [-2.8973 2.8973] 4 -> 5
            [np.pi / 2, 0, 0, 0],  # theta in [-0.0175 3.7525] 5 -> 6
            [np.pi / 2, 0.088, 0, 0.107],  # theta in [-2.8973 2.8973], 0.107 is distance between link 6 and 8
        ],
        dtype=float
    )

    q_limits = np.array([
        [-2.8973, 2.8973], [-1.7628, 1.7628], [-2.8973, 2.8973], [-3.0718, -0.0698], [-2.8973, 2.8973],
        [-0.0175, 3.7525], [-2.8973, 2.8973]
    ])

    return mdh, q_limits


def panda_poe_model():
    """Get panda product of exponential model.

    Returns:
        list[ndarray], ndarray POE params and joint limits.
    """

    M = np.array([
        [1, 0, 0, 0.088],
        [0, -1, 0, 0],
        [0, 0, -1, 0.926],
        [0, 0, 0, 1],
    ])

    screw_axes = np.array([
        [0, 0, 1, 0, 0, 0],
        [0, 1, 0, -0.333, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, -1, 0, 0.649, 0, -0.088],
        [0, 0, 1, 0, 0, 0],
        [0, -1, 0, 1.033, 0, 0],
        [0, 0, -1, 0, 0.088, 0],
    ]).T

    q_limits = np.array([
        [-2.8973, 2.8973], [-1.7628, 1.7628], [-2.8973, 2.8973], [-3.0718, -0.0698], [-2.8973, 2.8973],
        [-0.0175, 3.7525], [-2.8973, 2.8973]
    ])

    return [M, screw_axes], q_limits


def walker_left_arm_poe():
    """[Archived] Get poe model of the UBTech Walker's left arm.
    See test/test_create_model for details.

    verified date: 2020/6/11
    """
    # home matrix (homogeneous matrix transform base frame to eef frame)
    M = np.array([
        [9.98925860e-01, 4.63370949e-02, -1.70247991e-07, 4.51820844e-01],
        [3.40368002e-07, -1.10117044e-05, -1.00000000e+00, -8.40714493e-08],
        [-4.63370949e-02, 9.98925860e-01, -1.10156479e-05, -1.02911897e-02],
        [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
    ])
    # screw axes all relative to the base_frame (fixed relative to the robot base)
    # to calculate rotation axis, we first align the origin of the frame in query
    # to that of the base frame
    screw_axes = np.array([
        [0, 0, -1, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [1, 0, 0, 0, 0, 0],
        [0, -1, 0, -2.47791593e-02, 0, -2.28499909e-01],
        [1, 0, 0, 0, 0, 0],
        [0, 0, -1, 0, 4.51820844e-01, 0],
        [0, 1, 0, 0, 0, 4.51820844e-01]
    ]).T
    # transform from robot base link to arm base link (i.e., base_to_s)
    base_to_static = np.array([
        [-9.56714879e-07, -1., -3.73952529e-06, 0],
        [2.60457789e-01, -3.85964030e-06, 9.65485236e-01, 2.41420000e-01],
        [-9.65485236e-01, -5.02943990e-08, 2.60457789e-01, 1.83860000e-02],
        [0., 0., 0., 1.]
    ])
    return M, screw_axes, base_to_static
