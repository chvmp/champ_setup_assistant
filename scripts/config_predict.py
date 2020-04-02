#!/usr/bin/env python
'''
Copyright (c) 2119-2121, Juan Miguel Jimeno
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

class URDFConfigPredict(QWidget):
    def __init__(self, main):
        super(QWidget, self).__init__()
        self.main = main

        self.main.robot_viz.urdf_loaded.connect(self.on_urdf_path_load)

        self.hint_row = QFormLayout()
        self.hint_column = QHBoxLayout()
        self.joint_column = QHBoxLayout()
        self.row = QVBoxLayout()

        self.tab_label = QLabel("\tLEG AUTO CONFIGURATION\n")
        self.tab_label.setFont(QFont("Default", pointSize=11, weight=QFont.Bold))
        self.tab_label.setAlignment(Qt.AlignCenter)

        self.left_front_hint_l = QLabel("\tLEFT FRONT NS")
        self.left_front_hint_l.setFont(QFont("Default", pointSize=11))
        self.left_front_hint_l.setAlignment(Qt.AlignCenter)
        self.left_front_hint = QComboBox()
        self.left_front_hint.addItem("")
        self.left_front_hint.setFixedWidth(111)
        self.left_front_hint.currentIndexChanged.connect(self.lf_clicked)
        self.hint_row.addRow(self.left_front_hint_l, self.left_front_hint)

        self.right_front_hint_l = QLabel("\tRIGHT FRONT NS")
        self.right_front_hint_l.setFont(QFont("Default", pointSize=11))
        self.right_front_hint_l.setAlignment(Qt.AlignCenter)
        self.right_front_hint = QComboBox()
        self.right_front_hint.addItem("")
        self.right_front_hint.setFixedWidth(111)
        self.right_front_hint.currentIndexChanged.connect(self.rf_clicked)
        self.hint_row.addRow(self.right_front_hint_l, self.right_front_hint)

        self.left_hind_hint_l = QLabel("\tLEFT HIND NS")
        self.left_hind_hint_l.setFont(QFont("Default", pointSize=11))
        self.left_hind_hint_l.setAlignment(Qt.AlignCenter)
        self.left_hind_hint = QComboBox()
        self.left_hind_hint.addItem("")
        self.left_hind_hint.setFixedWidth(111)
        self.left_hind_hint.currentIndexChanged.connect(self.lh_clicked)
        self.hint_row.addRow(self.left_hind_hint_l, self.left_hind_hint)

        self.right_hind_hint_l = QLabel("\tRIGHT HIND NS")
        self.right_hind_hint_l.setFont(QFont("Default", pointSize=11))
        self.right_hind_hint_l.setAlignment(Qt.AlignCenter)
        self.right_hind_hint = QComboBox()
        self.right_hind_hint.addItem("")
        self.right_hind_hint.setFixedWidth(111)
        self.right_hind_hint.currentIndexChanged.connect(self.rh_clicked)
        self.hint_row.addRow(self.right_hind_hint_l, self.right_hind_hint)

        self.hint_column.addLayout(self.hint_row)

        self.hints = []
        self.hints.append(self.left_front_hint)
        self.hints.append(self.right_front_hint)
        self.hints.append(self.left_hind_hint)
        self.hints.append(self.right_hind_hint)

        self.row.addWidget(self.tab_label)
        self.row.addLayout(self.hint_column)
        self.row.addLayout(self.joint_column)

        self.setLayout(self.row)

        self.namespaces_complete = False

        self.namespaces = ["","","",""]
        self.link_substrings = ["","","",""]

    def get_link_chain(self, ns):
        link_chain = []
        for link in self.main.leg_configurator.links_list:
            if link.count(ns):
                link_chain.append(link)
                
        return link_chain

    def on_urdf_path_load(self):
        foot_links = self.main.robot.foot_links
        ns = self.main.robot.foot_links_ns
        
        for i in range(4):            
            self.left_front_hint.addItem(ns[i])
            self.right_front_hint.addItem(ns[i])
            self.left_hind_hint.addItem(ns[i])
            self.right_hind_hint.addItem(ns[i])

    def lf_clicked(self):
        self.update_namespaces(self.hints[1].currentText(), 1)

    def rf_clicked(self):
        self.update_namespaces(self.hints[1].currentText(), 1)

    def lh_clicked(self):
        self.update_namespaces(self.hints[2].currentText(), 2)

    def rh_clicked(self):
        self.update_namespaces(self.hints[3].currentText(), 3)

    def update_namespaces(self, ns, leg_id):
        if ns == "":
            return

        self.namespaces[leg_id] = self.hints[leg_id].currentText()
        
        ns_index = self.namespaces.index(ns)
        for i in range(4):
            #check every namespace configurator if it is the same as the currently
            #selected namespace, if so, change that duplicate to empty
            if i != leg_id and self.hints[i].currentText() == ns:
                self.hints[i].setCurrentIndex(1)

        if self.left_front_hint.currentText() != "" and\
           self.right_front_hint.currentText() != "" and\
           self.left_hind_hint.currentText() != "" and\
           self.right_hind_hint.currentText() != "":

            self.namespaces_complete = True

            for leg_id, ns in enumerate(self.namespaces):
                link_chain = self.get_link_chain(ns)
                for i in range(4):
                    self.main.leg_configurator.leg_configurators[leg_id].joint_configurators[i].clear()
                    self.main.leg_configurator.leg_configurators[leg_id].joint_configurators[i].add_link(link_chain[i])
                    self.main.links_list.clear()
            return True

        else:
            self.namespaces_complete = False
            return False

    def clear_joint_configurators(self, part_id):
        for leg_id in range(4):
            self.main.leg_configurator.leg_configurators[leg_id].joint_configurators[part_id].clear()

class ConfigPredict(QWidget):
    def __init__(self, main):
        super(QWidget, self).__init__()

        self.main = main
        self.main.robot_viz.urdf_loaded.connect(self.on_urdf_path_load)

        self.row = QVBoxLayout()
        self.joint_column = QHBoxLayout()

        self.config_predict = URDFConfigPredict(main)
        self.config_predict.setVisible(False)
        self.row.addWidget(self.config_predict)

        self.tab_label = QLabel("\tLEG AUTO CONFIGURATION\n")
        self.tab_label.setFont(QFont("Default", pointSize=11, weight=QFont.Bold))
        self.tab_label.setAlignment(Qt.AlignCenter)

        instruction_text =("\
        Add the translation between actuators manually if you don't have a URDF.\n\
        Key in the parameters for of your robot's front-left leg. The auto configurator \n\
        will automatically update the rest of the legs.\n\
        \n\
        HIP               - Translation from center of base to the hip actuator.\n\
        UPPER LEG   - Translation from the hip actuator to the upper leg actuator.\n\
        LOWER LEG  - Translation from the upper leg actuator to the lower leg actuator.\n\
        FOOT            - Translation from the lower leg actuator to the foot.\n\
        ")

        self.instructions = QLabel(instruction_text)
        self.instructions.setFont(QFont("Default", pointSize=9))

        self.hip_joint = JointConfigurator(self.main, 0, 0)
        self.hip_joint.translation_updated.connect(self.hip_link_added)
        self.joint_column.addWidget(self.hip_joint)

        self.upper_leg_joint = JointConfigurator(self.main, 0, 1)
        self.upper_leg_joint.translation_updated.connect(self.upper_leg_link_added)
        self.joint_column.addWidget(self.upper_leg_joint)

        self.lower_leg_joint = JointConfigurator(self.main, 0, 2)
        self.lower_leg_joint.translation_updated.connect(self.lower_leg_link_added)
        self.joint_column.addWidget(self.lower_leg_joint)

        self.foot_joint = JointConfigurator(self.main, 0, 3)
        self.foot_joint.translation_updated.connect(self.foot_link_added)
        self.joint_column.addWidget(self.foot_joint)

        self.row.addWidget(self.tab_label)
        self.row.addWidget(self.instructions)
        self.row.addLayout(self.joint_column)

        self.joint_configurators = []
        self.joint_configurators.append(self.hip_joint)
        self.joint_configurators.append(self.upper_leg_joint)
        self.joint_configurators.append(self.lower_leg_joint)
        self.joint_configurators.append(self.foot_joint)
        
        self.setLayout(self.row)

    def on_urdf_path_load(self):
        self.tab_label.setVisible(False)

        self.config_predict.setVisible(True)
        self.instructions.setVisible(False)
        self.hip_joint.setVisible(False)
        self.upper_leg_joint.setVisible(False)
        self.lower_leg_joint.setVisible(False)
        self.foot_joint.setVisible(False)
                
    def hip_link_added(self, leg_id, part_id, axis, val):
        for i in range(4):
            trans = self.inverse_val(i, part_id, axis, val)
            self.main.leg_configurator.leg_configurators[i].joint_configurators[part_id].origin[axis].setValue(trans)

    def upper_leg_link_added(self, leg_id, part_id, axis, val):
        for i in range(4):
            trans = self.inverse_val(i, part_id, axis, val)
            self.main.leg_configurator.leg_configurators[i].joint_configurators[part_id].origin[axis].setValue(trans)

    def lower_leg_link_added(self, leg_id, part_id, axis, val):
        for i in range(4):
            trans = self.inverse_val(i, part_id, axis, val)
            self.main.leg_configurator.leg_configurators[i].joint_configurators[part_id].origin[axis].setValue(trans)

    def foot_link_added(self, leg_id, part_id, axis, val):
        for i in range(4):
            trans = self.inverse_val(i, part_id, axis, val)
            self.main.leg_configurator.leg_configurators[i].joint_configurators[part_id].origin[axis].setValue(trans)

    def inverse_val(self, leg_id, part_id, axis, val):
        multiplier = \
        [[[1.0, 1.0, 1.0],
        [1.0, 1.0, 1.0],
        [1.0, 1.0, 1.0],
        [1.0, 1.0, 1.0]],

        [[1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0]],

        [[-1.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0]],

        [[-1.0, -1.0, 1.0],
        [-1.0, -1.0, 1.0],
        [-1.0, -1.0, 1.0],
        [-1.0, -1.0, 1.0]]]

        return multiplier[leg_id][part_id][axis] * val			