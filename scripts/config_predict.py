#!/usr/bin/env python
'''
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

from joint_configurator import JointConfigurator
import os
import numpy as np

class ConfigPredict(QWidget):
    def __init__(self, main):
        super(QWidget, self).__init__()
        self.main = main

        self.main.robot_viz.urdf_loaded.connect(self.on_urdf_path_load)

        self.leg_name = "LEFT FRONT"

        self.hint_row = QFormLayout()
        self.hint_column = QHBoxLayout()
        self.joint_column = QHBoxLayout()
        self.row = QVBoxLayout()

        self.tab_label = QLabel("AUTO LEG CONFIGURATOR")
        self.tab_label.setFont(QFont("Default", pointSize=10, weight=QFont.Bold))
        self.tab_label.setAlignment(Qt.AlignCenter)

        self.left_front_hint_l = QLabel("  LEFT FRONT")
        self.left_front_hint_l.setFont(QFont("Default", pointSize=10))
        self.left_front_hint_l.setAlignment(Qt.AlignCenter)
        self.left_front_hint = QComboBox()
        self.left_front_hint.setFixedWidth(100)
        self.hint_row.addRow(self.left_front_hint_l, self.left_front_hint)

        self.right_front_hint_l = QLabel("  RIGHT FRONT")
        self.right_front_hint_l.setFont(QFont("Default", pointSize=10))
        self.right_front_hint_l.setAlignment(Qt.AlignCenter)
        self.right_front_hint = QComboBox()
        self.right_front_hint.setFixedWidth(100)
        self.hint_row.addRow(self.right_front_hint_l, self.right_front_hint)

        self.left_hind_hint_l = QLabel("  LEFT HIND")
        self.left_hind_hint_l.setFont(QFont("Default", pointSize=10))
        self.left_hind_hint_l.setAlignment(Qt.AlignCenter)
        self.left_hind_hint = QComboBox()
        self.left_hind_hint.setFixedWidth(100)
        self.hint_row.addRow(self.left_hind_hint_l, self.left_hind_hint)

        self.right_hind_hint_l = QLabel("  RIGHT HIND")
        self.right_hind_hint_l.setFont(QFont("Default", pointSize=10))
        self.right_hind_hint_l.setAlignment(Qt.AlignCenter)
        self.right_hind_hint = QComboBox()
        self.right_hind_hint.setFixedWidth(100)
        self.hint_row.addRow(self.right_hind_hint_l, self.right_hind_hint)

        self.hint_column.addLayout(self.hint_row)

        self.hip_joint = JointConfigurator(self.main, self.leg_name, "HIP")
        self.joint_column.addWidget(self.hip_joint)
        self.upper_leg_joint = JointConfigurator(self.main, self.leg_name, "UPPER LEG")
        self.joint_column.addWidget(self.upper_leg_joint)
        self.lower_leg_joint = JointConfigurator(self.main, self.leg_name, "LOWER LEG")
        self.joint_column.addWidget(self.lower_leg_joint)
        self.foot_joint = JointConfigurator(self.main, self.leg_name, "FOOT")
        self.joint_column.addWidget(self.foot_joint)

        self.row.addWidget(self.tab_label)
        self.row.addLayout(self.hint_column)
        self.row.addLayout(self.joint_column)

        self.setLayout(self.row)

    def on_urdf_path_load(self):
        foot_links = self.main.robot.foot_links
        ns = self.main.robot.foot_links_ns
        for i in range(4):            
            self.left_front_hint.addItem(ns[i])
            self.right_front_hint.addItem(ns[i])
            self.left_hind_hint.addItem(ns[i])
            self.right_hind_hint.addItem(ns[i])        
