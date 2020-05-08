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

import os, sys
import shutil
import json

from package_creator import PackageCreator
from jinja2 import Environment, FileSystemLoader
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

import rospkg

class CodeGenWidget(QWidget):
    def __init__(self, main):
        super(QWidget, self).__init__()
        self.main = main
        self.proj_path = rospkg.RosPack().get_path('champ_setup_assistant')

        self.column = QHBoxLayout()
        self.row = QVBoxLayout()

        self.tab_label = QLabel("\tGENERATE CONFIGURATION\n")
        self.tab_label.setFont(QFont("Default", pointSize=10 ,weight=QFont.Bold))
        self.tab_label.setAlignment(Qt.AlignCenter)
        self.row.addWidget(self.tab_label)

        #add gif https://gist.github.com/Svenito/4000025
        img_path = os.path.dirname(sys.modules['__main__'].__file__) + "/../docs/images/champ_running.gif"
        self.movie = QMovie(img_path, QByteArray(), self)
        size = self.movie.scaledSize()
        self.movie_screen = QLabel()
        self.movie_screen.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.movie_screen.setAlignment(Qt.AlignCenter)
        self.row.addWidget(self.movie_screen)

        instruction_text =("\n\
        Enter your robot's name and choose where to save the configuration package(\" Click Browse\"). \n\
        Click \"Generate\" to create the configuration package.\n\
        ")

        self.instructions = QLabel(instruction_text)
        self.instructions.setFont(QFont("Default", pointSize=9))
        self.row.addWidget(self.instructions)

        self.robot_name_label =  QLabel("\tRobot Name")
        self.column.addWidget(self.robot_name_label)
        self.robot_name_edit = QLineEdit("")
        # self.robot_name_edit.setFixedWidth(300)
        self.robot_name_edit.textChanged.connect(self.robot_name_edited)
        self.column.addWidget(self.robot_name_edit)

        self.folder_text = QLineEdit("")
        self.folder_text.setEnabled(False)
        self.column.addWidget(self.folder_text)

        self.browse_button = QPushButton("Browse")
        self.browse_button.clicked.connect(self.browse_button_clicked)
        self.column.addWidget(self.browse_button)

        self.load_button = QPushButton("Generate")
        self.load_button.clicked.connect(self.gen_button_clicked)
        self.column.addWidget(self.load_button)

        self.row.addLayout(self.column)
        self.setLayout(self.row)

        #Add the QMovie object to the label
        self.movie.setCacheMode(QMovie.CacheAll)
        self.movie.setSpeed(100)
        self.movie_screen.setMovie(self.movie)
        self.movie.start()

        self.workspace_path = ""
        self.package_name = ""
        self.robot_name = ""
        self.package_path = ""

        self.config = {
            "robot_name" : "",
            "default_urdf" : "",
            "urdf_path": "",
            "links": {
                "base": "",
                "left_front": ["", "", "", ""],
                "right_front": ["", "", "", ""],
                "left_hind": ["", "", "", ""],
                "right_hind": ["", "", "", ""]
            },
            "joints": {
                "left_front": ["", "", ""],
                "right_front": ["", "", ""],
                "left_hind": ["", "", ""],
                "right_hind": ["", "", ""]
            },
            "firmware": {
                "transforms": {
                    "left_front": {
                        "hip": [0,0,0,0,0,0],
                        "upper_leg": [0,0,0,0,0,0],
                        "lower_leg": [0,0,0,0,0,0],
                        "foot": [0,0,0,0,0,0]
                    },
                    "right_front": {
                        "hip": [0,0,0,0,0,0],
                        "upper_leg": [0,0,0,0,0,0],
                        "lower_leg": [0,0,0,0,0,0],
                        "foot": [0,0,0,0,0,0]
                    },
                    "left_hind": {
                        "hip": [0,0,0,0,0,0],
                        "upper_leg": [0,0,0,0,0,0],
                        "lower_leg": [0,0,0,0,0,0],
                        "foot": [0,0,0,0,0,0]
                    },
                    "right_hind": {
                        "hip": [0,0,0,0,0,0],
                        "upper_leg": [0,0,0,0,0,0],
                        "lower_leg": [0,0,0,0,0,0],
                        "foot": [0,0,0,0,0,0]
                    }
                },
                "gait": {
                    "knee_orientation": "",
                    "is_pantograph": "",
                    "max_linear_vel_x": 0,
                    "max_linear_vel_y": 0,
                    "max_angular_vel_z": 0,
                    "stance_duration": 0,
                    "swing_height": 0,
                    "stance_depth": 0,
                    "nominal_height": 0
                }
            }
        }

        self.packager = PackageCreator()
        self.template_env = Environment(loader = FileSystemLoader(self.proj_path + "/templates"),   trim_blocks=True, lstrip_blocks=True)

    def robot_name_edited(self):
        self.robot_name = self.robot_name_edit.text()
        self.robot_name = self.robot_name.strip()

    def browse_button_clicked(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        options |= QFileDialog.ShowDirsOnly

        self.workspace_path = QFileDialog.getExistingDirectory(self, 'Select your catkin workspace src folder:', '', options=options)
        self.folder_text.setText(self.workspace_path)

    def gen_button_clicked(self):
        self.package_name = self.robot_name_edit.text() + "_config"
        if not self.robot_name:
            QMessageBox.information(self, "ERROR", "Robot Name is empty")

        if not self.workspace_path:
            QMessageBox.information(self, "ERROR", "Export folder is empty")

        if self.robot_name and self.workspace_path:
            self.generate_package()

    def save_config(self, config, path):
        path = path + "/" + "config.json"
        f = open(path, "w")
        content = json.dumps(config, indent=4)
        f.write(content)
        f.close()

    def generate_package(self):
        leg_configuration = self.main.leg_configurator.get_configuration()
        gait_configuration = self.main.gait_configurator.get_configuration()

        if leg_configuration != None:
            self.config["robot_name"] = self.robot_name
            self.config["links"] = leg_configuration["links"]
            self.config["joints"] = leg_configuration["joints"]
            self.config["firmware"]["transforms"] = leg_configuration["firmware"]["transforms"]
            self.config["firmware"]["gait"] = gait_configuration

            if self.main.robot_viz.using_urdf:
                self.config["urdf_path"] = self.main.file_browser.description_path
                self.config["default_urdf"] = "False"
            else:
                self.config["links"]["base"] = "base_link"
                self.config["urdf_path"] = "$(find " + self.robot_name + "_config)/urdf/quadruped.urdf"
                self.config["default_urdf"] = "True"
                
            self.package_path = self.workspace_path + "/" + self.package_name
            self.packager.generate_configuration_package(self.config, self.package_path)

            self.save_config(self.config, self.package_path)
            QMessageBox.information(self, "SUCCESS", "Configuration Package Generated: %s" % self.workspace_path)
