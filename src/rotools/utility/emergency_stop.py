from __future__ import print_function

from pynput.keyboard import Key, Listener

from rotools.utility.common import print_warn, print_debug


class EStop(object):
    def __init__(self):
        super(EStop, self).__init__()
        self.enable = False

        # Collect events until released
        with Listener(on_press=self._on_press) as listener:

            listener.join()

    def _on_press(self, key):
        if key == Key.space:
            self.enable = ~self.enable
            if self.enable:
                print_debug('\nActivated')
            else:
                print_warn('\nEmergency stopped')
        else:
            if self.enable:
                print('\nCurrent state: Activated')
            else:
                print('\nCurrent state: Emergency stopped')
