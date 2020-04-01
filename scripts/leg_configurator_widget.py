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

from difflib import SequenceMatcher
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

from leg_configurator import LegConfigurator
from config_predict import ConfigPredict

class LegConfiguratorWidget(QWidget):
    def __init__(self, main):
        super(QWidget, self).__init__()
        self.main = main

        self.main.robot_viz.urdf_loaded.connect(self.on_urdf_path_load)

        self.layout = QGridLayout()

        self.column = QHBoxLayout()
    
        self.lf_links_column = QHBoxLayout()
        self.rf_links_column = QHBoxLayout()
        self.lh_links_column = QHBoxLayout()
        self.rh_links_column = QHBoxLayout()

        self.list_stack_row = QVBoxLayout()

        self.links_label = QLabel("LINKS")
        self.links_label.setAlignment(Qt.AlignCenter)
        self.list_stack_row.addWidget(self.links_label)

        self.list_stack_row.addWidget(self.main.links_list)
        self.main.links_list.setFixedWidth(140)
        self.main.links_list.setFont(QFont("Default", pointSize=10))
        # self.main.links_list.setFixedHeight(400)

        self.config_predict = ConfigPredict(main)
        self.lf_configurator = LegConfigurator(main, "LEFT FRONT")
        self.rf_configurator = LegConfigurator(main, "RIGHT FRONT")
        self.lh_configurator = LegConfigurator(main, "LEFT HIND")
        self.rh_configurator = LegConfigurator(main, "RIGHT HIND")

        self.leg_configurators = []

        self.leg_configurators.append(self.lf_configurator)
        self.leg_configurators.append(self.rf_configurator)
        self.leg_configurators.append(self.lh_configurator)
        self.leg_configurators.append(self.rh_configurator)

        self.leg_tabs = QTabWidget()
        self.leg_tabs.addTab(self.config_predict,  "Configuration")
        self.leg_tabs.addTab(self.lf_configurator, "Left Front Leg")
        self.leg_tabs.addTab(self.lf_configurator, "Left Front Leg")
        self.leg_tabs.addTab(self.rf_configurator, "Right Front Leg")
        self.leg_tabs.addTab(self.lh_configurator, "Left Hind Leg")
        self.leg_tabs.addTab(self.rh_configurator, "Right Hind Leg")

        self.column.addLayout(self.list_stack_row)
        self.column.addWidget(self.leg_tabs)

        self.setLayout(self.column)

    def on_urdf_path_load(self):
        links = self.main.robot.link_names
        link_no = 0
        foot_no = 0
        for link in links:
            joint_name = self.main.robot.get_attached_joint(link)
            if self.main.robot.joint_is_revolute(joint_name):
                self.main.links_list.add_link(link)
                link_no += 1
                if link_no % 3 == 0 or link_no == 0:
                    foot_link = self.main.robot.foot_links[foot_no]
                    self.main.links_list.add_link(foot_link)
                    foot_no += 1


    def get_configuration(self):
        
        leg_configuration = {
            "links": {
                "base": "",
                "left_front": ["", "", "", ""],
                "right_front": ["", "", "", ""],
                "left_hind": ["", "", "", ""],
                "right_hind": ["", "", "", ""]
            },
            "joints": {
                "left_front": ["", "", "", ""],
                "right_front": ["", "", "", ""],
                "left_hind": ["", "", "", ""],
                "right_hind": ["", "", "", ""]
            },
            "firmware": {
                "transforms": {
                    "left_front": {
                        "hip": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "upper_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "lower_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "foot": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    },
                    "right_front": {
                        "hip": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "upper_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "lower_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "foot": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    },
                    "left_hind": {
                        "hip": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "upper_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "lower_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "foot": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    },
                    "right_hind": {
                        "hip": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "upper_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "lower_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "foot": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    }
                }
            }
        }

        leg_names = ["left_front", "right_front", "left_hind", "right_hind"]
        leg_configuration["links"]["base"] = self.main.robot.base
        leg_configurated = 0

        for i in range(4):
            try:            
                #populate links
                leg_configuration["links"][leg_names[i]] = self.leg_configurators[i].get_leg_links()
                
                #populate joints
                leg_configuration["joints"][leg_names[i]] = self.leg_configurators[i].get_leg_joints()

                #populate transforms 
                leg_transform = self.leg_configurators[i].get_transform()
                '''
                make sure x and y of hip are using the correct sign
                translation from reference
                leg_transform[part][axis]
                where part:
                0 - hip joint
                1 - upper leg joint
                2 - lower leg joint 
                3 - foot
                where axis is:
                0 - x
                1 - y
                2 - z
                '''
                if i == 0:
                    #lf hip -x
                    leg_transform[0][0] = abs(leg_transform[0][0])
                    #lf hip -y
                    leg_transform[0][1] = abs(leg_transform[0][1])

                elif i == 1:
                    #rf hip -x
                    leg_transform[0][0] = abs(leg_transform[0][0])
                    #rf hip -y
                    leg_transform[0][1] = -abs(leg_transform[0][1])

                elif i == 2:
                    #lh hip -x
                    leg_transform[0][0] = -abs(leg_transform[0][0])
                    #lh hip -y
                    leg_transform[0][1] = abs(leg_transform[0][1])

                elif i == 3:
                    #rh hip -x
                    leg_transform[0][0] = -abs(leg_transform[0][0])
                    #rh hip -y
                    leg_transform[0][1] = -abs(leg_transform[0][1])

                #z of the rest
                leg_transform[1][2] = -abs(leg_transform[1][2])
                leg_transform[2][2] = -abs(leg_transform[2][2])
                leg_transform[3][2] = -abs(leg_transform[3][2])

                leg_configuration["firmware"]["transforms"][leg_names[i]]["hip"] = leg_transform[0]
                leg_configuration["firmware"]["transforms"][leg_names[i]]["upper_leg"] = leg_transform[1]
                leg_configuration["firmware"]["transforms"][leg_names[i]]["lower_leg"] = leg_transform[2]
                leg_configuration["firmware"]["transforms"][leg_names[i]]["foot"] = leg_transform[3]

                leg_configurated += 1
            except:
                QMessageBox.information(self, "WARN", "Missing or incorrect link in %s leg" % leg_names[i])

        #check if the configuration is complete
        if leg_configurated == 4:
            return leg_configuration
    
        else:
            return None