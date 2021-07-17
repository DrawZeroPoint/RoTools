from __future__ import print_function

from pynput.keyboard import Key, Listener

from rotools.utility.common import print_warn, print_debug


class EStop(object):
    def __init__(self):
        super(EStop, self).__init__()
        self.enable = False

        print('Use Esc key to activate/deactivate, default is deactivate')
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()  # start the thread and run subsequent codes

    def _on_press(self, key):
        if key == Key.esc:
            self.enable = ~self.enable
            if self.enable:
                print_warn('\nActivated')
            else:
                print_debug('\nStopped')
