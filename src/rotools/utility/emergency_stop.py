from rotools.utility.common import play_hint_sound

try:
    from pynput.keyboard import Key, Listener
except ImportError as err:
    Key = None
    Listener = None

from rotools.utility.common import print_warn, print_debug


class EStop(object):
    def __init__(self, function_name=None):
        super(EStop, self).__init__()
        self._enable = False

        if Listener is not None:
            hot_key = "Alt"
            if isinstance(function_name, str):
                print_warn(
                    "{} is deactivated, press {} button to activate/deactivate".format(
                        function_name, hot_key
                    )
                )
            else:
                print_warn(
                    "Function is deactivated, press {} button to activate/deactivate".format(
                        hot_key
                    )
                )

            self.listener = Listener(on_press=self._on_press)
            self.listener.start()  # start the thread and run subsequent codes
        else:
            print_warn("{}".format(err))
            print_warn(
                "To use keyboard interactions: sudo pip install pynput playsound"
            )

    def _on_press(self, key):
        if key == Key.alt or key == Key.alt_r:
            status = ~self._enable
            self.set_status(status)

    def set_status(self, status):
        self._enable = status
        if self._enable:
            print_warn("\nActivated")
        else:
            print_debug("\nStopped")
        play_hint_sound(status)

    @property
    def enabled(self):
        return self._enable
