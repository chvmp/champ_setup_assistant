#!/usr/bin/env python
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

class GaitConfiguratorWidget(QWidget):
    def __init__(self, file_browser_widget):
        super(GaitConfiguratorWidget, self).__init__()
        self.row = QFormLayout()
        
        self.tab_label = QLabel("\tGAIT CONFIGURATION\n")
        self.tab_label.setFont(QFont("Default", pointSize=12 ,weight=QFont.Bold))
        self.tab_label.setAlignment(Qt.AlignCenter)
        self.row.addRow(self.tab_label)

        self.knee_orientation_label = QLabel("\tKnee Orientation")
        self.knee_orientation_edit = QComboBox()
        self.knee_orientation_edit.addItem(">>")
        self.knee_orientation_edit.addItem("<<")
        self.knee_orientation_edit.addItem("><")
        self.knee_orientation_edit.addItem("<>")
        self.knee_orientation_edit.setFixedWidth(200)
        self.row.addRow(self.knee_orientation_label, self.knee_orientation_edit)

        self.pantograph_leg_label = QLabel("\tPantograph Leg")
        self.pantograph_leg_edit = QComboBox()
        self.pantograph_leg_edit.addItem("false")
        self.pantograph_leg_edit.addItem("true")
        self.pantograph_leg_edit.setFixedWidth(200)
        self.row.addRow(self.pantograph_leg_label, self.pantograph_leg_edit)

        self.linear_linear_vel_x_label =  QLabel("\tMax Linear Velocity, X Axis (m/s)")
        self.linear_linear_vel_x_edit = QDoubleSpinBox()
        self.linear_linear_vel_x_edit.setValue(0.5)
        self.linear_linear_vel_x_edit.setFixedWidth(200)
        self.linear_linear_vel_x_edit.setSingleStep(0.001)
        self.linear_linear_vel_x_edit.setDecimals(3)
        self.row.addRow(self.linear_linear_vel_x_label, self.linear_linear_vel_x_edit)

        self.linear_linear_vel_y_label =  QLabel("\tMax Linear Velocity, Y Axis (m/s)")
        self.linear_linear_vel_y_edit = QDoubleSpinBox()
        self.linear_linear_vel_y_edit.setValue(0.25)
        self.linear_linear_vel_y_edit.setFixedWidth(200)
        self.linear_linear_vel_y_edit.setSingleStep(0.001)
        self.linear_linear_vel_y_edit.setDecimals(3)
        self.row.addRow(self.linear_linear_vel_y_label, self.linear_linear_vel_y_edit)

        self.linear_angular_vel_z_label =  QLabel("\tMax Angular Velocity, Z Axis (rad/s)")
        self.linear_angular_vel_z_edit = QDoubleSpinBox()
        self.linear_angular_vel_z_edit.setValue(1.0)
        self.linear_angular_vel_z_edit.setFixedWidth(200)
        self.linear_angular_vel_z_edit.setSingleStep(0.001)
        self.linear_angular_vel_z_edit.setDecimals(3)
        self.row.addRow(self.linear_angular_vel_z_label, self.linear_angular_vel_z_edit)

        self.stance_duration_label =  QLabel("\tMax Step Length (m)")
        self.stance_duration_edit = QDoubleSpinBox()
        self.stance_duration_edit.setValue(0.250)
        self.stance_duration_edit.setFixedWidth(200)
        self.stance_duration_edit.setSingleStep(0.001)
        self.stance_duration_edit.setDecimals(3)
        self.row.addRow(self.stance_duration_label, self.stance_duration_edit)

        self.swing_height_label =  QLabel("\tGait Swing Height (m)")
        self.swing_height_edit = QDoubleSpinBox()
        self.swing_height_edit.setValue(0.04)
        self.swing_height_edit.setFixedWidth(200)
        self.swing_height_edit.setSingleStep(0.001)
        self.swing_height_edit.setDecimals(3)
        self.row.addRow(self.swing_height_label, self.swing_height_edit)

        self.stance_height_label =  QLabel("\tGait Stance Height (m)")
        self.stance_height_edit = QDoubleSpinBox()
        self.stance_height_edit.setValue(0.0)
        self.stance_height_edit.setFixedWidth(200)
        self.stance_height_edit.setSingleStep(0.001)
        self.stance_height_edit.setDecimals(3)
        self.row.addRow(self.stance_height_label, self.stance_height_edit)

        self.nominal_height_label = QLabel("\tRobot Walking Height (m)")
        self.nominal_height_edit = QDoubleSpinBox()
        self.nominal_height_edit.setValue(0.2)
        self.nominal_height_edit.setFixedWidth(200)
        self.nominal_height_edit.setSingleStep(0.001)
        self.nominal_height_edit.setDecimals(3)
        self.row.addRow(self.nominal_height_label, self.nominal_height_edit)

        self.setLayout(self.row)

    def get_configuration(self):
        gait_config = {
            "knee_orientation": ">>",
            "max_linear_vel_x": "false",
            "max_linear_vel_x": 0,
            "max_linear_vel_y": 0,
            "max_angular_vel_z": 0,
            "stance_duration": 0,
            "swing_height": 0,
            "stance_depth": 0,
            "nominal_height": 0
        }

        gait_config["knee_orientation"] = str(self.knee_orientation_edit.currentText())
        gait_config["pantograph_leg"] = str(self.pantograph_leg_edit.currentText())
        gait_config["max_linear_vel_x"] = self.linear_linear_vel_x_edit.value()
        gait_config["max_linear_vel_y"] = self.linear_linear_vel_y_edit.value()
        gait_config["max_angular_vel_z"] = self.linear_angular_vel_z_edit.value()
        gait_config["stance_duration"] = self.stance_duration_edit.value()
        gait_config["swing_height"] = self.swing_height_edit.value()
        gait_config["stance_depth"] =  self.stance_height_edit.value()
        gait_config["nominal_height"] = self.nominal_height_edit.value()

        return gait_config