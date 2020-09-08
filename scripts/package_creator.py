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
from distutils.util import strtobool

from jinja2 import Environment, FileSystemLoader
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

import rospkg

class PackageCreator():
    def __init__(self, update=False):
        self.proj_path = rospkg.RosPack().get_path('champ_setup_assistant')

        self.package_path = ""
        self.package_joints_map_path = ""
        self.package_links_map_path = ""
        self.package_gait_config_path = ""
        self.package_include_path = ""
        self.package_launch_path = ""
        self.package_launch_include_path = ""
        self.package_scripts_path = ""
        self.package_urdf_path = ""

        self.template_env = Environment(loader = FileSystemLoader(self.proj_path + "/templates"),   trim_blocks=True, lstrip_blocks=True)

    def update_package_path(self, package_path):
        self.package_path = package_path
        self.package_joints_map_path = self.package_path + "/config/joints"
        self.package_links_map_path = self.package_path + "/config/links"
        self.package_gait_config_path = self.package_path + "/config/gait"
        self.package_ros_control_path = self.package_path + "/config/ros_control"
        self.package_include_path = self.package_path + "/include"
        self.package_launch_path = self.package_path + "/launch"
        self.package_launch_include_path = self.package_path + "/launch/include"
        self.package_navigation_config_path = self.package_path + "/config/move_base"
        self.package_maps_path = self.package_path + "/maps"
        self.package_urdf_path = self.package_path + "/urdf"

    def create_dir(self, path):
        try:
            os.makedirs(path)
        except:
            pass

    def generate_package_folder(self, using_urdf):
        self.create_dir(self.package_path)
        self.create_dir(self.package_joints_map_path)
        self.create_dir(self.package_links_map_path)
        self.create_dir(self.package_gait_config_path)
        self.create_dir(self.package_ros_control_path)
        self.create_dir(self.package_include_path)
        self.create_dir(self.package_launch_path)
        self.create_dir(self.package_launch_include_path)
        self.create_dir(self.package_maps_path)
        self.create_dir(self.package_navigation_config_path)

        if using_urdf:
            self.create_dir(self.package_urdf_path)

    def copy_from_template(self, update=False):
        shutil.copy(self.proj_path + '/templates/hardware_config.h', self.package_include_path)
        shutil.copy(self.proj_path + '/templates/setup.bash', self.package_path)
        shutil.copy(self.proj_path + '/templates/base_local_planner_holonomic_params.yaml', self.package_navigation_config_path)
        shutil.copy(self.proj_path + '/templates/costmap_common_params.yaml', self.package_navigation_config_path)
        shutil.copy(self.proj_path + '/templates/global_costmap_params.yaml', self.package_navigation_config_path)
        shutil.copy(self.proj_path + '/templates/local_costmap_params.yaml', self.package_navigation_config_path)
        shutil.copy(self.proj_path + '/templates/move_base_params.yaml', self.package_navigation_config_path)
        shutil.copy(self.proj_path + '/templates/gmapping.launch', self.package_launch_include_path)
        shutil.copy(self.proj_path + '/templates/amcl.launch', self.package_launch_include_path)
        shutil.copy(self.proj_path + '/templates/map.pgm', self.package_maps_path)
        shutil.copy(self.proj_path + '/templates/map.yaml', self.package_maps_path)

    def generate_from_template(self, config, template_file, dest):
        template = self.template_env.get_template(template_file)
        content = template.render(config)
        path = dest + "/" + template_file
        f = open(path, "w")
        f.write(content)
        f.close()

    def save_config(self, config, path):
        path = path + "/" + "config.json"
        f = open(path, "w")
        content = json.dumps(config, indent=4)
        f.write(content)
        f.close()

    def generate_configuration_package(self, config, package_path, update=False):
        self.update_package_path(package_path)
        self.generate_package_folder(strtobool(config["default_urdf"]))

        if strtobool(config["default_urdf"]):
            self.generate_from_template(config["firmware"]["transforms"], "quadruped.urdf", self.package_urdf_path)
        
        if not update:
            self.generate_from_template(config["joints"], "ros_control.yaml", self.package_ros_control_path)
            self.generate_from_template(config["firmware"]["gait"], "gait_config.h", self.package_include_path)
            self.generate_from_template(config["firmware"]["gait"], "gait.yaml", self.package_gait_config_path)

        self.generate_from_template(config, "CMakeLists.txt", self.package_path)
        self.generate_from_template(config, "bringup.launch", self.package_launch_path)
        self.generate_from_template(config, "slam.launch", self.package_launch_path)
        self.generate_from_template(config, "navigate.launch", self.package_launch_path)
        self.generate_from_template(config, "gazebo.launch", self.package_launch_path)
        self.generate_from_template(config, "move_base.launch", self.package_launch_include_path)
        self.generate_from_template(config["firmware"]["transforms"], "quadruped_description.h", self.package_include_path)
        self.generate_from_template(config["joints"], "joints.yaml", self.package_joints_map_path)
        self.generate_from_template(config["links"], "links.yaml", self.package_links_map_path)
        self.generate_from_template(config, "package.xml", self.package_path)
        

        self.copy_from_template(update=update)
            
        print("Configuration Generated")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("No config package provided")
        sys.exit()

    package_path = sys.argv[1]
    if not os.path.exists(package_path):
        print("Package path doesn't exist.")
        sys.exit()

    if package_path[-1] != '/':
        package_path += '/'

    packager = PackageCreator()
    packager.update_package_path(package_path)

    try:
        config = None
        with open(package_path + 'config.json') as json_file:
            config = json.load(json_file)

        packager.generate_configuration_package(config, package_path, update=True)
    except:
        print("No config file found. Exiting.")