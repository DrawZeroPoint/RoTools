#!/usr/bin/env python

import rospy
import subprocess

from std_msgs.msg import UInt8, Int8


def print_info(content):
    """Print information with sky blue."""
    print("".join(["\033[1m\033[94m", content, "\033[0m"]))


def pretty_print_configs(cfgs):
    """Print a dict of configurations in a visual friendly and organized way.

    Args:
        cfgs: dict A dict of configures. The items could be string, number, or a list/tuple.

    Returns:
        None
    """
    max_key_len = 0
    max_value_len = 0
    for key, value in cfgs.items():
        key_str = "{}".format(key)
        if len(key_str) > max_key_len:
            max_key_len = len(key_str)
        if isinstance(value, list) or isinstance(value, tuple):
            for i in value:
                i_str = "{}".format(i)
                if len(i_str) > max_value_len:
                    max_value_len = len(i_str)
        else:
            value_str = "{}".format(value)
            if len(value_str) > max_value_len:
                max_value_len = len(value_str)

    print_info(
        "\n{}{}{}".format(
            "=" * (max_key_len + 1), " ROPORT CONFIGS ", "=" * (max_value_len - 15)
        )
    )
    for key, value in cfgs.items():
        key_msg = "{message: <{width}}".format(message=key, width=max_key_len)
        empty_key_msg = "{message: <{width}}".format(message="", width=max_key_len)
        if isinstance(value, list) or isinstance(value, tuple):
            for i, i_v in enumerate(value):
                if i == 0:
                    print_info("{}: {}".format(key_msg, i_v))
                else:
                    print_info("{}: {}".format(empty_key_msg, i_v))
        else:
            print_info("{}: {}".format(key_msg, value))
    print_info(
        "{}{}{}\n".format(
            "=" * (max_key_len + 1), " END OF CONFIGS ", "=" * (max_value_len - 15)
        )
    )


def get_param(name, value=None):
    """Get private/public param from param server.
    If the param's name does not have a leading ~, it will first be searched in private params,
    and then in public params. If with a leading ~, it will only be searched in private params.

    Args:
        name: str Param name.
        value: Any Return value if param is not set.

    Returns:
        Any Param value.
    """
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


class TriggerClient(object):
    def __init__(self, kwargs):
        super(TriggerClient, self).__init__()
        self._code_cmd_dict = {}
        for k, v in zip(kwargs["codes"], kwargs["cmds"]):
            self._code_cmd_dict[k] = v

        self._trigger_code_subscriber = rospy.Subscriber(
            "/trigger_code", UInt8, self._trigger_cb
        )
        self._return_code_publisher = rospy.Publisher(
            "/return_code", Int8, queue_size=1
        )
        self.processes = []

    def _trigger_cb(self, msg):
        if msg.data in self._code_cmd_dict:
            cmd = self._code_cmd_dict[msg.data]
            rospy.loginfo("Running cmd #{}: {}".format(msg.data, cmd))

            process = subprocess.Popen(
                cmd.split(" "),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            try:
                # TODO set individual timeout for each cmd
                while process.wait() != 0:
                    rospy.sleep(0.01)
            except subprocess.TimeoutExpired:
                self._return_code_publisher.publish(Int8(255))

            rospy.loginfo(
                "Cmd #{} finished with return code {}".format(
                    msg.data, process.returncode
                )
            )
            self._return_code_publisher.publish(Int8(process.returncode))
            self.processes.append(process)


if __name__ == "__main__":
    try:
        rospy.init_node("trigger_client")

        configs = {
            "codes": get_param("codes"),
            "cmds": get_param("cmds"),
        }
        assert len(configs["codes"]) == len(configs["cmds"]), print(
            "Code len {} does not match cmd len {}".format(
                len(configs["codes"]), len(configs["cmds"])
            )
        )

        pretty_print_configs(configs)
        client = TriggerClient(configs)
        rospy.loginfo("Trigger Client ready.")
        rospy.spin()
        for p in client.processes:
            p.kill()
    except rospy.ROSInterruptException as e:
        print(e)
