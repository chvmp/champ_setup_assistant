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

        self.left_front_hint_l = QLabel("  LEFT FRONT NS")
        self.left_front_hint_l.setFont(QFont("Default", pointSize=10))
        self.left_front_hint_l.setAlignment(Qt.AlignCenter)
        self.left_front_hint = QComboBox()
        self.left_front_hint.addItem("")
        self.left_front_hint.setFixedWidth(100)
        self.left_front_hint.currentIndexChanged.connect(self.lf_clicked)
        self.hint_row.addRow(self.left_front_hint_l, self.left_front_hint)

        self.right_front_hint_l = QLabel("  RIGHT FRONT NS")
        self.right_front_hint_l.setFont(QFont("Default", pointSize=10))
        self.right_front_hint_l.setAlignment(Qt.AlignCenter)
        self.right_front_hint = QComboBox()
        self.right_front_hint.addItem("")
        self.right_front_hint.setFixedWidth(100)
        self.right_front_hint.currentIndexChanged.connect(self.rf_clicked)
        self.hint_row.addRow(self.right_front_hint_l, self.right_front_hint)

        self.left_hind_hint_l = QLabel("  LEFT HIND NS")
        self.left_hind_hint_l.setFont(QFont("Default", pointSize=10))
        self.left_hind_hint_l.setAlignment(Qt.AlignCenter)
        self.left_hind_hint = QComboBox()
        self.left_hind_hint.addItem("")
        self.left_hind_hint.setFixedWidth(100)
        self.left_hind_hint.currentIndexChanged.connect(self.lh_clicked)
        self.hint_row.addRow(self.left_hind_hint_l, self.left_hind_hint)

        self.right_hind_hint_l = QLabel("  RIGHT HIND NS")
        self.right_hind_hint_l.setFont(QFont("Default", pointSize=10))
        self.right_hind_hint_l.setAlignment(Qt.AlignCenter)
        self.right_hind_hint = QComboBox()
        self.right_hind_hint.addItem("")
        self.right_hind_hint.setFixedWidth(100)
        self.right_hind_hint.currentIndexChanged.connect(self.rh_clicked)
        self.hint_row.addRow(self.right_hind_hint_l, self.right_hind_hint)

        self.hint_column.addLayout(self.hint_row)

        self.hints = []
        self.hints.append(self.left_front_hint)
        self.hints.append(self.right_front_hint)
        self.hints.append(self.left_hind_hint)
        self.hints.append(self.right_hind_hint)

        self.hip_joint = JointConfigurator(self.main, self.leg_name, "HIP")
        self.hip_joint.link_added.connect(self.hip_link_added)
        self.hip_joint.cleared.connect(self.hip_cleared)
        self.hip_joint.setEnabled(False)
        self.joint_column.addWidget(self.hip_joint)

        self.upper_leg_joint = JointConfigurator(self.main, self.leg_name, "UPPER LEG")
        self.upper_leg_joint.link_added.connect(self.upper_leg_link_added)
        self.upper_leg_joint.cleared.connect(self.upper_leg_cleared)
        self.upper_leg_joint.setEnabled(False)
        self.joint_column.addWidget(self.upper_leg_joint)

        self.lower_leg_joint = JointConfigurator(self.main, self.leg_name, "LOWER LEG")
        self.lower_leg_joint.link_added.connect(self.lower_leg_link_added)
        self.lower_leg_joint.cleared.connect(self.lower_leg_cleared)
        self.lower_leg_joint.setEnabled(False)
        self.joint_column.addWidget(self.lower_leg_joint)

        self.foot_joint = JointConfigurator(self.main, self.leg_name, "FOOT")
        self.foot_joint.link_added.connect(self.foot_link_added)
        self.foot_joint.cleared.connect(self.foot_link_cleared)
        self.foot_joint.setEnabled(False)
        self.joint_column.addWidget(self.foot_joint)

        self.joint_configurators = []
        self.joint_configurators.append(self.hip_joint)
        self.joint_configurators.append(self.upper_leg_joint)
        self.joint_configurators.append(self.lower_leg_joint)
        self.joint_configurators.append(self.foot_joint)

        self.row.addWidget(self.tab_label)
        self.row.addLayout(self.hint_column)
        self.row.addLayout(self.joint_column)

        self.setLayout(self.row)

        self.namespaces_complete = False

        self.namespaces = ["","","",""]
        self.link_substrings = ["","","",""]

    def on_urdf_path_load(self):
        foot_links = self.main.robot.foot_links
        ns = self.main.robot.foot_links_ns
        for i in range(4):            
            self.left_front_hint.addItem(ns[i])
            self.right_front_hint.addItem(ns[i])
            self.left_hind_hint.addItem(ns[i])
            self.right_hind_hint.addItem(ns[i])

    def lf_clicked(self):
        self.namespaces[0] = self.left_front_hint.currentText()

        for link in self.main.robot.foot_links:
            if link.count(self.namespaces[0]):
                self.link_substrings[3] = link.replace(self.namespaces[0], "")
                self.foot_joint.add_link(link)
                self.enable_joint_configurator()  
                self.disable_namespaces(self.hints[0].currentText(), 0)

    def rf_clicked(self):
        self.namespaces[1] = self.right_front_hint.currentText()
        self.enable_joint_configurator()  
        self.disable_namespaces(self.hints[1].currentText(), 1)

    def lh_clicked(self):
        self.namespaces[2] = self.left_hind_hint.currentText()
        self.enable_joint_configurator()  
        self.disable_namespaces(self.hints[2].currentText(), 2)

    def rh_clicked(self):
        self.namespaces[3] = self.right_hind_hint.currentText()
        self.enable_joint_configurator()  
        self.disable_namespaces(self.hints[3].currentText(), 3)

    def disable_namespaces(self, ns, leg_id):
        ns_index = self.namespaces.index(ns)
        for i in range(4):
            if i != leg_id and self.hints[i].currentText() == ns:
                self.hints[i].setCurrentIndex(0)
                if self.namespaces_complete:
                    self.clear_joint_configurators(i)

    def update_joint_configurators(self, link_name, part_id):
        #check if the current link being added matches the ns defined
        ret = link_name.count(self.left_front_hint.currentText())
        if not ret:
            QMessageBox.information(self, "LINK NAMESPACE MISMATCH", "The link you're trying to add doesn't match the leg namespace defined.")
            self.joint_configurators[part_id].clear()
            self.clear_joint_configurators(part_id)
            return 0

        #check if this link is valid
        ret = self.link_exists(link_name)
        if not ret:
            return 0

        #add a link to front leg[part id] 0-hip 1-upperleg 2-lowerleg 3-foot
        self.main.leg_configurator.leg_configurators[0].joint_configurators[part_id].add_link(link_name)
        #every time a user adds a link, we get more semantics of the robot
        #we can get the name of the part by omitting the ns from the link name
        link_substring = link_name.replace(self.namespaces[0], "")

        #confirm that the substring works by checking if this exists when ns is added back
        ret = self.link_exists(self.namespaces[0] + link_substring)
        if not ret:
            return 0

        #once confirmed we can update the list of link substrings
        self.link_substrings[part_id] = link_substring

        #now we can update the rest of the legs
        for leg_id in range(1,4):
            #using the updated list of namespaces and substrings we can update the rest of the legs
            #ie. lf_hip_link ns = lf_ ; substring = hip_link 
            link_name = self.namespaces[leg_id] + self.link_substrings[part_id]
            self.main.leg_configurator.leg_configurators[leg_id].joint_configurators[part_id].add_link(link_name)

    def clear_joint_configurators(self, part_id):
        for leg_id in range(4):
            self.main.leg_configurator.leg_configurators[leg_id].joint_configurators[part_id].clear()

    def hip_link_added(self, link_name):
        self.update_joint_configurators(link_name, 0)

    def hip_cleared(self):
        self.clear_joint_configurators(0)

    def upper_leg_link_added(self, link_name):
        self.update_joint_configurators(link_name, 1)

    def upper_leg_cleared(self):
        self.clear_joint_configurators(1)

    def lower_leg_link_added(self, link_name):
        self.update_joint_configurators(link_name, 2)

    def lower_leg_cleared(self):
        # pass
        self.clear_joint_configurators(2)

    def foot_link_added(self, link_name):
        if self.namespaces_complete:
            self.update_joint_configurators(link_name, 3)

    def foot_link_cleared(self):
        self.clear_joint_configurators(3)

    def link_exists(self, link_name):
        for link in self.main.robot.link_names:
            if link.count(link_name):
                return True
                
        return False

    def enable_joint_configurator(self):
        if self.left_front_hint.currentText() != "" and\
           self.right_front_hint.currentText() != "" and\
           self.left_hind_hint.currentText() != "" and\
           self.right_hind_hint.currentText() != "":

            self.hip_joint.setEnabled(True)
            self.upper_leg_joint.setEnabled(True)
            self.lower_leg_joint.setEnabled(True)
            self.foot_joint.setEnabled(True)

            self.namespaces_complete = True

            for i in range(4):
                #dont clear the foot area
                if i < 3:
                    self.joint_configurators[i].clear()
                #clear all configurations
                #this is useful if the user change the namespace
                self.clear_joint_configurators(i)

            #update all foot links for all legs
            self.foot_link_added(self.namespaces[0] + self.link_substrings[3])
     
            return True

        else:
            self.hip_joint.setEnabled(False)
            self.upper_leg_joint.setEnabled(False)
            self.lower_leg_joint.setEnabled(False)
            self.foot_joint.setEnabled(False)

            self.namespaces_complete = False
            return False