#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ## ##################################################################
# pyGUI.py
#
# CLIPS control GUI in python
#
# @author: Mauricio Matamoros
#
# ## ##################################################################

import rospy
from controlgui import ControlGUI
from controlnode import ControlNode




def main():
    gui = ControlGUI()
    node = ControlNode()
    node.init()
    node.gui = gui
    gui.setPublisherFunc(node.publish)

    node.start()
    gui.loop()
    print('GUI terminated. Awaiting for ROS...')
    rospy.signal_shutdown('GUI Closed')
    node.join()
#end def

if __name__ == '__main__':
    main()
