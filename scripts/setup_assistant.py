#!/usr/bin/env python
import sys
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

from file_browser_widget import FileBrowserWidget
from rviz_widget import RvizWidget
from leg_configurator_widget import LinkListWidget, LegConfiguratorWidget
from gait_configurator_widget import GaitConfiguratorWidget
from code_gen_widget import CodeGenWidget
from urdf_parser import URDFParser

class SetupAssistant(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

        self.robot = URDFParser()
        self.row = QVBoxLayout()

        self.file_browser_widget = FileBrowserWidget()
        self.rviz_widget = RvizWidget(self.robot, self.file_browser_widget)

        self.links_list = LinkListWidget(self.rviz_widget)

        self.tabs = QTabWidget()

        self.leg_configurator_widget = LegConfiguratorWidget(self.links_list, self.rviz_widget)
        self.gait_configurator_widget = GaitConfiguratorWidget()
        self.code_gen_widget = CodeGenWidget(self.robot, self.leg_configurator_widget, self.gait_configurator_widget)

        self.tabs.addTab(self.leg_configurator_widget, "Leg Configuration")
        self.tabs.addTab(self.gait_configurator_widget, "Gait Configuration")
        self.tabs.addTab(self.code_gen_widget, "Generate Config")

        self.row.addWidget(self.file_browser_widget)
        self.row.addWidget(self.rviz_widget)
        self.row.addWidget(self.tabs)

        self.setLayout(self.row)

if __name__ == '__main__':
    app = QApplication( sys.argv )

    sa = SetupAssistant()
    sa.show()

    sys.exit(app.exec_())