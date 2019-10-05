#!/usr/bin/env python
import os
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

class FileBrowserWidget(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

        self.column = QHBoxLayout()
        self.row = QVBoxLayout()

        self.file_text = QLineEdit("")
        self.file_text.textChanged.connect(self.urdf_path_changed)

        self.column.addWidget(self.file_text)

        self.browse_button = QPushButton("Browse")
        self.browse_button.clicked.connect(self.browse_button_clicked)
        self.column.addWidget(self.browse_button)

        self.load_button = QPushButton("Load URDF")
        self.load_button.clicked.connect(self.load_button_clicked)
        self.load_button.setEnabled(False)
        self.column.addWidget(self.load_button)

        self.row.addLayout(self.column)
        self.setLayout(self.row)

        self.urdf_path = None

    def urdf_path_changed(self):
        urdf_path = str(self.file_text.text())
        urdf_path = urdf_path.strip()
        if self.urdf_is_valid(urdf_path):
            self.load_button.setEnabled(True)
            self.file_text.setText(self.urdf_path)
            self.urdf_path = urdf_path
            self.browse_button.setEnabled(False)
        else:
            self.load_button.setEnabled(False)

    def urdf_is_valid(self, urdf_path):
        filename, file_extension = os.path.splitext(urdf_path)
        if file_extension == ".URDF" or file_extension == ".urdf":
            return os.path.isfile(urdf_path)
        else:
            return False
            
    def browse_button_clicked(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        urdf_path, _ = QFileDialog.getOpenFileName(self,"CHAMP SETUP ASSISTANT", "","URDF File (*);;URDF File (*.urdf)", options=options)
        if self.urdf_is_valid(urdf_path):
            self.urdf_path = urdf_path
            self.file_text.setText(self.urdf_path)
            self.browse_button.setEnabled(False)
            self.load_button.setEnabled(True)
        else:
            QMessageBox.information(self, "ERROR", "Invalid URDF: " + "\r\n" + urdf_path)

    def load_button_clicked(self):
        self.load_button.setEnabled(False)