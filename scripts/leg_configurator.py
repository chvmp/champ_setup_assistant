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

class LegConfigurator(QWidget):
    link_added = Signal(str, int, int)
    
    def __init__(self, main, leg_id):
        super(QWidget, self).__init__()
        self.main = main
        self.leg_id = leg_id

        self.main.robot_viz.urdf_loaded.connect(self.on_urdf_path_load)

        self.leg_names = ["LEFT FRONT", "RIGHT FRONT", "LEFT HIND", "RIGHT HIND"]

        self.column = QHBoxLayout()
        self.row = QVBoxLayout()

        self.tab_label = QLabel("\t%s LEG CONFIGURATION\n" % self.leg_names[leg_id])
        self.tab_label.setFont(QFont("Default", pointSize=10, weight=QFont.Bold))
        self.tab_label.setAlignment(Qt.AlignCenter)
        instruction_text =("\
        Add the translation between actuators. As you key in the configurations for the left-front leg, the\n\
        rest of the legs automatically configures.\n\
        \n\
        HIP               - Translation from center of base to the hip actuator.\n\
        UPPER LEG   - Translation from the hip actuator to the upper leg actuator.\n\
        LOWER LEG  - Translation from the upper leg actuator to the lower leg actuator.\n\
        FOOT            - Translation from the lower leg actuator to the foot.\n\
        ")

        self.instructions = QLabel(instruction_text)
        self.instructions.setFont(QFont("Default", pointSize=9))

        self.hip_joint = JointConfigurator(self.main, self.leg_id, 0)
        self.hip_joint.link_added.connect(self.hip_link_added)
        self.column.addWidget(self.hip_joint)

        self.upper_leg_joint = JointConfigurator(self.main, self.leg_id, 1)
        self.upper_leg_joint.link_added.connect(self.upper_leg_link_added)
        self.column.addWidget(self.upper_leg_joint)

        self.lower_leg_joint = JointConfigurator(self.main, self.leg_id, 2)
        self.lower_leg_joint.link_added.connect(self.lower_leg_link_added)
        self.column.addWidget(self.lower_leg_joint)

        self.foot_joint = JointConfigurator(self.main, self.leg_id, 3)
        self.foot_joint.link_added.connect(self.foot_link_added)
        self.column.addWidget(self.foot_joint)

        self.joint_configurators = []
        self.joint_configurators.append(self.hip_joint)
        self.joint_configurators.append(self.upper_leg_joint)
        self.joint_configurators.append(self.lower_leg_joint)
        self.joint_configurators.append(self.foot_joint)

        self.row.addWidget(self.tab_label)
        self.row.addWidget(self.instructions)
        self.row.addLayout(self.column)

        self.setLayout(self.row)

    def hip_link_added(self, link_name, leg_id, part_id):
        self.link_added.emit(link_name, leg_id, part_id)
        
    def upper_leg_link_added(self, link_name, leg_id, part_id):   
        self.link_added.emit(link_name, leg_id, part_id)

    def lower_leg_link_added(self, link_name, leg_id, part_id):
        self.link_added.emit(link_name, leg_id, part_id)

    def foot_link_added(self, link_name, leg_id, part_id):
        self.link_added.emit(link_name, leg_id, part_id)

    def get_prefix(self):
        leg_prefixes = ["lf", "rf", "lh", "rh"]
        return leg_prefixes[self.leg_id]

    def get_leg_links(self):
        leg_prefix = self.get_prefix()

        hip = leg_prefix + "_hip_link"
        upper_leg = leg_prefix + "_upper_leg_link"
        lower_leg = leg_prefix + "_lower_leg_link"
        foot = leg_prefix + "_foot_link"

        if self.main.robot_viz.using_urdf:
            hip = str(self.hip_joint.leg_links_list.item(0).text())
            upper_leg = str(self.upper_leg_joint.leg_links_list.item(0).text())
            lower_leg = str(self.lower_leg_joint.leg_links_list.item(0).text())
            foot = str(self.foot_joint.leg_links_list.item(0).text())

        return [hip, upper_leg, lower_leg, foot]

    def get_leg_joints(self):
        leg_prefix = self.get_prefix()
        links = self.get_leg_links()

        hip = leg_prefix + "_hip_joint"
        upper_leg = leg_prefix + "_upper_leg_joint"
        lower_leg = leg_prefix + "_lower_leg_joint"
        foot = leg_prefix + "_foot_joint"

        if self.main.robot_viz.using_urdf:
            hip = self.main.robot.get_attached_joint(links[0])
            upper_leg = self.main.robot.get_attached_joint(links[1])
            lower_leg = self.main.robot.get_attached_joint(links[2])
            foot = self.main.robot.get_attached_joint(links[3])

        return [hip, upper_leg, lower_leg, foot]

    def get_transform(self):
        if self.main.robot_viz.using_urdf:
            return self.get_transform_from_urdf()
        else:
            return self.get_transform_from_box()

    def get_transform_from_box(self):
        transform = [0,0,0,0]
        transform[0] = [self.hip_joint.x_edit.value(), self.hip_joint.y_edit.value(), self.hip_joint.z_edit.value(), 0.0, 0.0, 0.0] 
        transform[1] = [self.upper_leg_joint.x_edit.value(), self.upper_leg_joint.y_edit.value(), self.upper_leg_joint.z_edit.value(), 0.0, 0.0, 0.0] 
        transform[2] = [self.lower_leg_joint.x_edit.value(), self.lower_leg_joint.y_edit.value(), self.lower_leg_joint.z_edit.value(), 0.0, 0.0, 0.0]
        transform[3] = [self.foot_joint.x_edit.value(), self.foot_joint.y_edit.value(), self.foot_joint.z_edit.value(), 0.0, 0.0, 0.0]
        
        return transform

    def get_transform_from_urdf(self):
        links = self.get_leg_links()
        joint_chain = self.main.robot.get_joint_chain(links[3])
        transform = []
        #get from transform from base to hip
        transform.append(self.main.robot.get_transform(joint_chain, self.main.robot.base, links[0]))

        #from hip all the way to the foot
        for i in range(3):
            transform.append(self.main.robot.get_transform(joint_chain, links[i], links[i+1]))

        return transform

    def on_urdf_path_load(self):
        self.instructions.setText("        Add all links related to the leg's part.\n\
        \n\
        ADDING A LINK\n\
        Select a link from the left pane.\n\
        Click > to add into the leg part.\n\
        *The link added to a part must be where the axis of rotation is. \n\
        *This is normally the link that represents the actuator. \n\
        *Foot links are not visible as there's no mesh attached. \n\
        \n\
        DELETING A LINK\n\
        Click < to clear the leg part.\n\
            ")