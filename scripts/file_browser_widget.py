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

import os
import json
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

class FileBrowserWidget(QWidget):
    new_config = Signal()
    new_urdf = Signal()
    def __init__(self):
        super(QWidget, self).__init__()

        self.column = QHBoxLayout()
        self.row = QVBoxLayout()

        self.file_text = QLineEdit("")
        self.file_text.textChanged.connect(self.file_path_changed)

        self.column.addWidget(self.file_text)

        self.browse_button = QPushButton("BROWSE URDF")
        self.browse_button.clicked.connect(self.browse_button_clicked)
        self.column.addWidget(self.browse_button)

        self.load_button = QPushButton("LOAD")
        self.load_button.clicked.connect(self.load_button_clicked)
        self.load_button.setEnabled(False)
        self.column.addWidget(self.load_button)

        self.row.addLayout(self.column)
        self.setLayout(self.row)

        self.urdf_path = None
        self.config_path = None
        self.loaded_config = None
        self.file_format = None

    def file_path_changed(self):
        file_path = str(self.file_text.text())
        file_path = file_path.strip()

        if self.urdf_is_valid(file_path):
            self.urdf_path = file_path
            self.load_button.setEnabled(True)
            self.file_text.setText(self.urdf_path)
            self.file_format = "urdf"
        elif self.config_is_valid(file_path):
            self.config_path = file_path
            self.load_button.setEnabled(True)
            self.file_text.setText(self.urdf_path)
            self.file_format = "json"
        else:
            self.load_button.setEnabled(False)

    def urdf_is_valid(self, urdf_path):
        filename, file_extension = os.path.splitext(urdf_path)

        if file_extension == ".URDF" or file_extension == ".urdf":
            return os.path.isfile(urdf_path)
        else:
            return False
            
    def config_is_valid(self, config_path):
        filename, file_extension = os.path.splitext(config_path)

        if file_extension == ".JSON" or file_extension == ".json":
            return os.path.isfile(config_path)
        else:
            return False
    
    def browse_button_clicked(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_path, _ = QFileDialog.getOpenFileName(self,"CHAMP SETUP ASSISTANT", "","URDF(*.urdf);;Champ Config File (*.json)", options=options)

        if self.urdf_is_valid(file_path):
            self.urdf_path = file_path
            self.file_text.setText(self.urdf_path)
            self.load_button.setEnabled(True)
            self.file_format = "urdf"
        elif self.config_is_valid(file_path):
            self.config_path = file_path
            self.file_text.setText(self.config_path)
            self.load_button.setEnabled(True)
            self.file_format = "json"
        elif file_path == "":
            pass
        else:
            QMessageBox.information(self, "ERROR", "Invalid URDF: " + "\r\n" + file_path)

    def load_config_file(self):
        f = open(self.config_path, "r")
        self.loaded_config = json.load(f)

    def load_button_clicked(self):
        self.browse_button.setEnabled(False)
        self.load_button.setEnabled(False)

        if self.file_format == "json":
            self.load_config_file()
            self.new_config.emit()

        else:    
            self.new_urdf.emit()