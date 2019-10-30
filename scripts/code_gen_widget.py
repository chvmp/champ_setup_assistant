#!/usr/bin/env python
import os
import shutil

from jinja2 import Environment, FileSystemLoader
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

import rospkg

class CodeGenWidget(QWidget):
    def __init__(self, robot_description, leg_configurator, gait_configurator):
        super(QWidget, self).__init__()
        self.proj_path = rospkg.RosPack().get_path('champ_setup_assistant')

        self.robot = robot_description
        self.leg_configurator = leg_configurator
        self.gait_configurator = gait_configurator

        self.column = QHBoxLayout()
        self.row = QVBoxLayout()

        self.tab_label = QLabel("\tGENERATE CONFIGURATION\n")
        self.tab_label.setFont(QFont("Default", pointSize=12 ,weight=QFont.Bold))
        self.tab_label.setAlignment(Qt.AlignCenter)
        self.row.addWidget(self.tab_label)

        self.robot_name_label =  QLabel("\tRobot Name")
        self.column.addWidget(self.robot_name_label)
        self.robot_name_edit = QLineEdit("")
        self.robot_name_edit.setFixedWidth(300)
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

        self.workspace_path = ""
        self.package_name = ""
        self.robot_name = ""

        self.package_path = ""
        self.config_path = ""
        self.firmware_include_path = ""
        self.launch_path = ""
        self.scripts_path = ""

        self.config = {
            "robot_name" : "",
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
                "max_linear_vel_x": 0,
                "max_linear_vel_y": 0,
                "max_angular_vel_z": 0,
                "max_step_length": 0,
                "max_theta": 0,
                "swing_height": 0,
                "stance_depth": 0,
                "nominal_height": 0
                }
            }
        }

        self.template_env = Environment(loader = FileSystemLoader(self.proj_path + "/templates"),   trim_blocks=True, lstrip_blocks=True)

    def robot_name_edited(self):
        self.robot_name = self.robot_name_edit.text()
        self.robot_name = self.robot_name.strip()

    def browse_button_clicked(self):
        self.workspace_path = QFileDialog.getExistingDirectory(None, 'Select your ROS src folder:', '', QFileDialog.ShowDirsOnly)
        self.folder_text.setText(self.workspace_path)

    def generate_package_folder(self, ws_src):
        self.package_path = ws_src + "/" + self.package_name
        self.config_path = self.package_path + "/config"
        self.firmware_include_path = self.package_path + "/include/firmware"
        self.launch_path = self.package_path + "/launch"
        self.scripts_path = self.package_path + "/scripts"

        try:
            os.makedirs(self.package_path)
            os.makedirs(self.config_path)
            os.makedirs(self.firmware_include_path)
            os.makedirs(self.launch_path)
            os.makedirs(self.scripts_path)

        except OSError:
            QMessageBox.information(self, "CONFIG GENERATION FAILED", "Package %s already exists in %s folder" % (self.package_name, self.workspace_path))
            return False
        else:
            return True

    def gen_button_clicked(self):
        self.package_name = self.robot_name_edit.text() + "_champ_config"
        if not self.robot_name:
            QMessageBox.information(self, "ERROR", "Robot Name is empty")

        if not self.workspace_path:
            QMessageBox.information(self, "ERROR", "Export folder is empty")

        if self.robot_name and self.workspace_path:
            self.generate_configuration_package()

    def copy_from_template(self, config):
        shutil.copy(self.proj_path + '/templates/firmware_utils.py', self.scripts_path)
        os.chmod(self.scripts_path + '/firmware_utils.py', 509)

    def generate_from_template(self, config, template_file, dest):
        template = self.template_env.get_template(template_file)
        content = template.render(config)
        path = dest + "/" + template_file
        f = open(path, "w")
        f.write(content)
        f.close()

    def generate_configuration_package(self):
        leg_configuration = self.leg_configurator.get_configuration()
        gait_configuration = self.gait_configurator.get_configuration()

        if leg_configuration != None:
            if self.leg_configurator.using_urdf:
                self.config["urdf_path"] = self.robot.path
            else:
                leg_configuration["links"]["base"] = "base_link"
                #TODO add correct generated urdf path
                self.config["urdf_path"] = "test"
                
            self.config["robot_name"] = self.robot_name
            self.config["links"] = leg_configuration["links"]
            self.config["joints"] = leg_configuration["joints"]
            self.config["firmware"]["transforms"] = leg_configuration["firmware"]["transforms"]
            self.config["firmware"]["gait"] = gait_configuration

            self.generate_package_folder(self.workspace_path)
            self.copy_from_template(self.config)
            self.generate_from_template(self.config, "CMakeLists.txt", self.package_path)
            self.generate_from_template(self.config, "bringup.launch", self.launch_path)
            self.generate_from_template(self.config["firmware"]["transforms"], "quadruped_description.h", self.firmware_include_path)
            self.generate_from_template(self.config["firmware"]["gait"], "gait_config.h", self.firmware_include_path)
            self.generate_from_template(self.config["joints"], "joints.yaml", self.config_path)
            self.generate_from_template(self.config["links"], "links.yaml", self.config_path)
            self.generate_from_template(self.config, "package.xml", self.package_path)

            QMessageBox.information(self, "SUCCESS", "Configuration Package Generated: %s" % self.workspace_path)
