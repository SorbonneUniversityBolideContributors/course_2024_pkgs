#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Raphael KHORASSANI"

import numpy as np
import os
from PySide6.QtWidgets import QApplication
import sys
import rospy

from scripts.widget import MainWindow
import sys
sys.settrace

rospy.init_node('GUI_node')

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window=MainWindow()
    window.show()
    window.connect()
    app.exec()
