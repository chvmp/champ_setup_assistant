#!/usr/bin/env python
import sys, os
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

import rviz
import roslaunch

class RvizWidget(QWidget):
    urdf_loaded = Signal()

    def __init__(self, robot_description, file_browser):
        super(QWidget, self).__init__()
        self.setFixedHeight(400)

        rviz_config_path = str(os.path.dirname(sys.path[0]) + "/config/setup_assistant.rviz")
        description_launch_path = str(os.path.dirname(sys.path[0]) + "/launch/description.launch")

        self.file_browser = file_browser
        self.robot = robot_description

        self.file_browser.new_urdf.connect(self.on_urdf_path_load)

        self.column = QHBoxLayout()
        self.row = QVBoxLayout()

        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()

        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, rviz_config_path)

        self.frame.load(config)
        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)

        self.manager = self.frame.getManager()

        self.robot_model_display = self.manager.getRootDisplayGroup().getDisplayAt(2) 
        self.robot_model_display.setBool(False)

        self.link_highlighter = self.robot_model_display.subProp("Highlight Link")
        self.link_unhighlighter = self.robot_model_display.subProp("Unhighlight Link")

        self.column.addWidget(self.frame)

        self.row.addLayout(self.column)
        self.setLayout(self.row)

        self.highlighted_link = None

        self.description_launch_args = [description_launch_path, "description_file:foo"]
        
        description_loader_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(description_loader_uuid)
        self.description_launcher = roslaunch.parent.ROSLaunchParent(description_loader_uuid, [self.description_launch_args[0]])

    def highlight_link(self, link_name):
        self.link_highlighter.setValue(link_name)

    def unhighlight_link(self, link_name):
        self.link_unhighlighter.setValue(link_name)

    def update_urdf_file(self, urdf_path):
        path_arg = "description_file:" + str(urdf_path)
        self.description_launch_args[1] = path_arg

    def launch_file(self, urdf_path):
        #this is a hack to pass urdf file
        os.environ["CHAMP_SETUP_ASSISTANT_URDF"] = urdf_path

        self.description_launcher.shutdown()
        self.description_launcher.start()

    def load_robot_description(self, fixed_frame, urdf_path):
        self.update_urdf_file(urdf_path)
        self.launch_file(urdf_path)
        self.robot_model_display.setBool(True)
        self.frame.getManager().setFixedFrame(fixed_frame)

    def on_urdf_path_load(self):
        urdf_path = self.file_browser.urdf_path
        self.robot.load_urdf(urdf_path)
        self.load_robot_description(self.robot.base, urdf_path)
        self.urdf_loaded.emit()