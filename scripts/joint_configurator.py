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

from link_list_widget import LinkListWidget

class AddDeleteButtonWidget(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

        self.buttons = QVBoxLayout()

        self.add_button = QPushButton(">")
        self.add_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        self.add_button.setFixedWidth(20)
        self.add_button.setFixedHeight(20)

        self.buttons.addWidget(self.add_button)

        self.delete_button = QPushButton("<")
        self.delete_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        self.delete_button.setFixedWidth(20)
        self.delete_button.setFixedHeight(20)
        self.buttons.addWidget(self.delete_button)

        self.setLayout(self.buttons)

class JointConfigurator(QWidget):
    link_added = Signal(str, int, int)
    translation_updated = Signal(int, int, int, float)

    def __init__(self, main, leg_id, part_id):
        super(QWidget, self).__init__()
        self.main = main
        self.leg_id = leg_id
        self.part_id = part_id
        self.main.robot_viz.urdf_loaded.connect(self.on_urdf_path_load)
        self.setFont(QFont("Default", pointSize=9))
        self.setFixedWidth(150)

        link_names = ["HIP", "UPPER_LEG", "LOWER_LEG", "FOOT"]
        leg_names = ["LEFT FRONT", "RIGHT FRONT", "LEFT HIND", "HIP"]

        self.column = QHBoxLayout()
        self.row = QVBoxLayout()
        self.layout = QGridLayout()

        self.link_label = QLabel(link_names[part_id])
        self.link_label.setFont(QFont("Default", pointSize=9))
        self.link_label.setAlignment(Qt.AlignCenter)

        self.leg_links_list = LinkListWidget(self.main)
        self.leg_links_list.setVisible(False)
        self.buttons_widget = AddDeleteButtonWidget()
        self.buttons_widget.add_button.clicked.connect(self.add_button_clicked)
        self.buttons_widget.delete_button.clicked.connect(self.clear)
        self.buttons_widget.setVisible(False)

        x_min = -2
        x_max = 2
        y_min = -2
        y_max = 2
        if leg_id == 0 and part_id == 0:
            x_min = 0
            x_max = 2
            y_min = 0
            y_max = 2
        elif leg_id == 1 and part_id == 0:
            x_min = 0
            x_max = 2
            y_min = -2
            y_max = 0
        elif leg_id == 2 and part_id == 0:
            x_min = -2
            x_max = 0
            y_min = 0
            y_max = 2
        elif leg_id == 3 and part_id == 0:
            x_min = -2
            x_max = 0
            y_min = -2
            y_max = 0

        self.x_label = QLabel("x :")
        self.x_label.setFont(QFont("Default", pointSize=9))
        self.x_label.setAlignment(Qt.AlignCenter)
        self.x_edit = QDoubleSpinBox()
        self.x_edit.setSingleStep(0.001)
        self.x_edit.setRange(x_min,x_max)
        self.x_edit.setDecimals(4)
        self.x_edit.setFixedWidth(90)
        self.x_edit.setFixedHeight(30)
        self.x_edit.valueChanged.connect(self.trans_x_changed)

        self.y_label = QLabel("y :")
        self.y_label.setFont(QFont("Default", pointSize=9))
        self.y_label.setAlignment(Qt.AlignCenter)
        self.y_edit = QDoubleSpinBox()
        self.y_edit.setSingleStep(0.001)
        self.y_edit.setRange(y_min,y_max)
        self.y_edit.setDecimals(4)
        self.y_edit.setFixedWidth(90)
        self.y_edit.setFixedHeight(30)
        self.y_edit.valueChanged.connect(self.trans_y_changed)

        z_max = 2
        if part_id == 2:
            z_max = 0
        elif part_id == 3:
            z_max = 0

        self.z_label = QLabel("z :")
        self.z_label.setAlignment(Qt.AlignCenter)
        self.z_label.setFont(QFont("Default", pointSize=9))
        self.z_edit = QDoubleSpinBox()
        self.z_edit.setSingleStep(0.001)
        self.z_edit.setRange(-2, z_max)
        self.z_edit.setDecimals(4)
        self.z_edit.setFixedWidth(90)
        self.z_edit.setFixedHeight(30)
        self.z_edit.valueChanged.connect(self.trans_z_changed)

        self.origin = []
        self.origin.append(self.x_edit)
        self.origin.append(self.y_edit)
        self.origin.append(self.z_edit)

        self.layout.addWidget(self.link_label, 0, 1)
        self.layout.addWidget(self.buttons_widget, 1, 0)
        self.layout.addWidget(self.leg_links_list, 1, 1)

        self.layout.addWidget(self.x_label, 2, 0)
        self.layout.addWidget(self.x_edit, 2, 1)

        self.layout.addWidget(self.y_label, 3, 0)
        self.layout.addWidget(self.y_edit, 3, 1)

        self.layout.addWidget(self.z_label, 4, 0)
        self.layout.addWidget(self.z_edit, 4, 1)

        self.setLayout(self.layout)

    def trans_x_changed(self):
        val = self.x_edit.value()
        self.translation_updated.emit(self.leg_id, self.part_id, 0, val)

    def trans_y_changed(self):
        val = self.y_edit.value()
        self.translation_updated.emit(self.leg_id, self.part_id, 1, val)  

    def trans_z_changed(self):
        val = self.z_edit.value()
        self.translation_updated.emit(self.leg_id, self.part_id, 2, val)  

    def on_urdf_path_load(self):
        self.x_label.hide()
        self.y_label.hide()
        self.z_label.hide()
        self.x_edit.hide()
        self.y_edit.hide()
        self.z_edit.hide()
        self.leg_links_list.show()
        self.buttons_widget.show()
        self.leg_links_list.setFixedWidth(120)
        self.leg_links_list.setFixedHeight(50)
        self.setFixedHeight(90)

    def add_button_clicked(self):
        link_name = self.main.links_list.highlighted_link()
        self.link_added.emit(link_name, self.leg_id, self.part_id)
        ret = self.add_link(link_name)
        if ret:
            if self.main.links_list.count() > 0:
                current_selected_link = self.main.links_list.currentItem().text()
                self.main.robot_viz.highlight_link(current_selected_link)
        else:
            QMessageBox.information(self, "WARN", "No link selected")

    def add_link(self, link_name):
        if len(link_name):
            self.clear()
            self.leg_links_list.add_link(link_name)
            self.main.links_list.delete_link(link_name)
            self.buttons_widget.add_button.setEnabled(False)
            return True
        else:
            return False
        
    def clear(self):
        if len(self.leg_links_list):
            i = self.leg_links_list.currentRow()
            link_name = self.leg_links_list.item(0).text()
            self.leg_links_list.delete_link(link_name)
            self.main.links_list.add_link(link_name)
            self.buttons_widget.add_button.setEnabled(True)

    