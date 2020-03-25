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

        self.leg_configurator_widget = LegConfiguratorWidget(self.links_list, self.file_browser_widget, self.rviz_widget)
        self.gait_configurator_widget = GaitConfiguratorWidget(self.file_browser_widget)
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