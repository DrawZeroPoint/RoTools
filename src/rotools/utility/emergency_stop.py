try:
    from pynput.keyboard import Key, Listener
except ImportError:
    Key = None
    Listener = None

from rotools.utility.common import print_warn, print_debug


class EStop(object):
    def __init__(self, function_name=None):
        super(EStop, self).__init__()
        self._enable = False

        hot_key = 'CapsLock'
        if function_name:
            print_warn('{} is deactivated, press {} to activate/deactivate'.format(function_name, hot_key))
        else:
            print_warn('Function is deactivated, press {} to activate/deactivate'.format(hot_key))
        if Listener is not None:
            self.listener = Listener(on_press=self._on_press)
            self.listener.start()  # start the thread and run subsequent codes
        else:
            print_warn('Keyboard emergency stop control is not supported on the current platform')

    def _on_press(self, key):
        if key == Key.caps_lock:
            status = ~self._enable
            self.set_status(status)

    def set_status(self, status):
        self._enable = status
        if self._enable:
            print_warn('\nActivated')
        else:
            print_debug('\nStopped')

    @property
    def enabled(self):
        return self._enable
