#!/usr/bin/env python
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

class GaitConfiguratorWidget(QWidget):
    def __init__(self,):
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

        self.linear_linear_vel_x_label =  QLabel("\tMax Linear Velocity, X Axis (m/s)")
        self.linear_linear_vel_x_edit = QDoubleSpinBox()
        self.linear_linear_vel_x_edit.setValue(0.5)
        self.linear_linear_vel_x_edit.setFixedWidth(200)
        self.linear_linear_vel_x_edit.setSingleStep(0.01)
        self.row.addRow(self.linear_linear_vel_x_label, self.linear_linear_vel_x_edit)

        self.linear_linear_vel_y_label =  QLabel("\tMax Linear Velocity, Y Axis (m/s)")
        self.linear_linear_vel_y_edit = QDoubleSpinBox()
        self.linear_linear_vel_y_edit.setValue(0.25)
        self.linear_linear_vel_y_edit.setFixedWidth(200)
        self.linear_linear_vel_y_edit.setSingleStep(0.01)
        self.row.addRow(self.linear_linear_vel_y_label, self.linear_linear_vel_y_edit)

        self.linear_angular_vel_z_label =  QLabel("\tMax Angular Velocity, Z Axis (rad/s)")
        self.linear_angular_vel_z_edit = QDoubleSpinBox()
        self.linear_angular_vel_z_edit.setValue(1.0)
        self.linear_angular_vel_z_edit.setFixedWidth(200)
        self.linear_angular_vel_z_edit.setSingleStep(0.01)
        self.row.addRow(self.linear_angular_vel_z_label, self.linear_angular_vel_z_edit)

        self.max_theta_label =  QLabel("\tMax Whole Body Rotational Angle (rad)")
        self.max_theta_edit = QDoubleSpinBox()
        self.max_theta_edit.setValue(0.35)
        self.max_theta_edit.setFixedWidth(200)
        self.max_theta_edit.setSingleStep(0.01)
        self.row.addRow(self.max_theta_label, self.max_theta_edit)

        self.max_step_length_label =  QLabel("\tMax Step Length (m)")
        self.max_step_length_edit = QDoubleSpinBox()
        self.max_step_length_edit.setValue(0.11)
        self.max_step_length_edit.setFixedWidth(200)
        self.max_step_length_edit.setSingleStep(0.01)
        self.row.addRow(self.max_step_length_label, self.max_step_length_edit)

        self.swing_height_label =  QLabel("\tGait Swing Height (m)")
        self.swing_height_edit = QDoubleSpinBox()
        self.swing_height_edit.setValue(0.04)
        self.swing_height_edit.setFixedWidth(200)
        self.swing_height_edit.setSingleStep(0.01)
        self.row.addRow(self.swing_height_label, self.swing_height_edit)

        self.stance_height_label =  QLabel("\tGait Stance Height (m)")
        self.stance_height_edit = QDoubleSpinBox()
        self.stance_height_edit.setValue(0.0)
        self.stance_height_edit.setFixedWidth(200)
        self.stance_height_edit.setSingleStep(0.01)
        self.row.addRow(self.stance_height_label, self.stance_height_edit)

        self.nominal_height_label = QLabel("\tRobot Walking Height (m)")
        self.nominal_height_edit = QDoubleSpinBox()
        self.nominal_height_edit.setValue(0.2)
        self.nominal_height_edit.setFixedWidth(200)
        self.nominal_height_edit.setSingleStep(0.01)
        self.row.addRow(self.nominal_height_label, self.nominal_height_edit)

        self.setLayout(self.row)

    def get_configuration(self):
        gait_config = {
            "knee_orientation": 0,
            "max_linear_vel_x": 0,
            "max_linear_vel_y": 0,
            "max_angular_vel_z": 0,
            "max_step_length": 0,
            "max_theta": 0,
            "swing_height": 0,
            "stance_depth": 0,
            "nominal_height": 0
        }

        gait_config["knee_orientation"] = str(self.knee_orientation_edit.currentText())
        gait_config["max_linear_vel_x"] = self.linear_linear_vel_x_edit.value()
        gait_config["max_linear_vel_y"] = self.linear_linear_vel_y_edit.value()
        gait_config["max_angular_vel_z"] = self.linear_angular_vel_z_edit.value()
        gait_config["max_step_length"] = self.max_step_length_edit.value()
        gait_config["max_theta"] = self.max_theta_edit.value()
        gait_config["swing_height"] = self.swing_height_edit.value()
        gait_config["stance_depth"] =  self.stance_height_edit.value()
        gait_config["nominal_height"] = self.nominal_height_edit.value()

        return gait_config