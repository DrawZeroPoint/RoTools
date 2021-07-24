from __future__ import print_function

from pynput.keyboard import Key, Listener

from rotools.utility.common import print_warn, print_debug


class EStop(object):
    def __init__(self, function_name=None):
        super(EStop, self).__init__()
        self.enable = False

        hot_key = 'CapsLock'
        if function_name:
            print('{} is deactivated, press {} to activate/deactivate'.format(function_name, hot_key))
        else:
            print('Function is deactivated, press {} to activate/deactivate'.format(hot_key))
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()  # start the thread and run subsequent codes

    def _on_press(self, key):
        if key == Key.caps_lock:
            self.enable = ~self.enable
            if self.enable:
                print_warn('\nActivated')
            else:
                print_debug('\nStopped')
