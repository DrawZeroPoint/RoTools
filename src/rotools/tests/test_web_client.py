#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from __future__ import print_function

import unittest

import rotools.websocket.core.client as cli


class Test(unittest.TestCase):
    def test_server_not_found(self):
        """0.0.0.0 is a valid ip indicating the local host,
        whereas 9092 is not a valid port under the default config (9090)
        """
        configs = {
            "ip": "0.0.0.0",
            "port": 9092,
        }
        with self.assertRaises(ConnectionRefusedError):
            cli.WebsocketClient(configs)


if __name__ == "__main__":
    unittest.main()
