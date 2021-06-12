#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from __future__ import print_function

import unittest

import rotools.utility.mesh as mesh


class Test(unittest.TestCase):

    def test_webots_converter(self):
        """
        """
        converter = mesh.WebotsConverter()
        converter.read_proto('/home/dzp/walker_blender/test1.proto', '/home/dzp/walker_blender/test1.idx')
        converter.write_to_stl('/home/dzp/walker_blender/out1.stl')


if __name__ == '__main__':
    unittest.main()
