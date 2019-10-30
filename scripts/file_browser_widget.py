#!/usr/bin/env python
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

        self.browse_button = QPushButton("Browse")
        self.browse_button.clicked.connect(self.browse_button_clicked)
        self.column.addWidget(self.browse_button)

        self.load_button = QPushButton("Load")
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