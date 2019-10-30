#!/usr/bin/env python
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

class LinkListWidget(QListWidget):
    def __init__(self, rviz_widget):
        super(LinkListWidget, self).__init__()
        self.rviz_widget = rviz_widget

        self.itemClicked.connect(self.on_click)

    def add_link(self, link):
        if link:
            items_list = self.findItems(link, Qt.MatchExactly)
            if not items_list:
                self.addItem(link)
                # self.clear_highlight()

    def delete_selected_link(self):
        link_name = self.highlighted_link()

        links_list = self.findItems(link_name, Qt.MatchExactly)
        for link in links_list:
            mathced_link = self.row(link)
            self.takeItem(mathced_link)
            # self.clear_highlight()

    def on_click(self, link_name):
        link_name = link_name.text()
        prev_highlighted_link = self.rviz_widget.highlighted_link
        if link_name != prev_highlighted_link:
            self.rviz_widget.unhighlight_link(prev_highlighted_link)
            self.rviz_widget.highlight_link(link_name)
            self.rviz_widget.highlighted_link = link_name
                
    def highlighted_link(self):
        link = ""
        try:
            link = self.currentItem().text()
        except:
            pass

        return link

    def delete_first_link(self):
        self.takeItem(0)
        # self.clear_highlight()

    def clear_highlight(self):
        for i in range(self.count()):
            item = self.item(i)
            item.setSelected(False)

    def is_selected(self):
        for i in range(self.count()):
            item = self.item(i)
            if item.isSelected():
                return True
        return False

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
    def __init__(self, parent, label):
        super(QWidget, self).__init__()
        self.parent = parent

        self.parent.rviz_widget.urdf_loaded.connect(self.on_urdf_path_load)

        self.setFixedHeight(150)
        self.setFixedWidth(270)

        self.column = QHBoxLayout()
        self.row = QVBoxLayout()
        self.layout = QGridLayout()

        self.link_label = QLabel(label)
        self.link_label.setAlignment(Qt.AlignCenter)

        self.leg_links_list = LinkListWidget(self.parent.rviz_widget)
        self.leg_links_list.setVisible(False)

        self.buttons_widget = AddDeleteButtonWidget()
        self.buttons_widget.add_button.clicked.connect(self.add_button_clicked)
        self.buttons_widget.delete_button.clicked.connect(self.clear_button_clicked)
        self.buttons_widget.setVisible(False)

        self.x_label = QLabel("x :")
        self.x_label.setAlignment(Qt.AlignCenter)
        self.x_edit = QDoubleSpinBox()
        self.x_edit.setSingleStep(0.001)
        self.x_edit.setRange(-100,100)
        self.x_edit.setDecimals(3)

        self.y_label = QLabel("y :")
        self.y_label.setAlignment(Qt.AlignCenter)
        self.y_edit = QDoubleSpinBox()
        self.y_edit.setSingleStep(0.001)
        self.y_edit.setRange(-100,100)
        self.y_edit.setDecimals(3)

        self.z_label = QLabel("z :")
        self.z_label.setAlignment(Qt.AlignCenter)
        self.z_edit = QDoubleSpinBox()
        self.z_edit.setSingleStep(0.001)
        self.z_edit.setRange(-100,100)
        self.z_edit.setDecimals(3)

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

    def on_urdf_path_load(self):
        self.leg_links_list.show()
        self.buttons_widget.show()
        self.x_label.hide()
        self.y_label.hide()
        self.z_label.hide()
        self.x_edit.hide()
        self.y_edit.hide()
        self.z_edit.hide()
        self.setFixedHeight(100)

    def add_button_clicked(self):
        link_name = self.parent.links_list.highlighted_link()
        if len(link_name) and self.parent.links_list.is_selected():
            self.leg_links_list.addItem(link_name)
            self.parent.links_list.delete_selected_link()
            self.buttons_widget.add_button.setEnabled(False)
        else:
            QMessageBox.information(self, "WARN", "No link selected")

    def clear_button_clicked(self):
        if self.leg_links_list.count():
            link_name = self.leg_links_list.item(0).text()
            self.parent.links_list.add_link(link_name)
            self.leg_links_list.delete_first_link()
            self.buttons_widget.add_button.setEnabled(True)
        else:
            QMessageBox.information(self, "WARN", "Leg part is empty")

    def delete_button_clicked(self):
        link_name = self.leg_links_list.highlighted_link()
        if len(link_name):
            self.parent.links_list.add_link(link_name)
            self.leg_links_list.delete_selected_link()
            self.buttons_widget.add_button.setEnabled(True)

class LegConfigurator(QWidget):
    def __init__(self, parent, leg_name):
        super(QWidget, self).__init__()
        self.parent = parent
        
        self.leg_name = leg_name

        self.column = QHBoxLayout()
        self.row = QVBoxLayout()

        self.tab_label = QLabel("\t%s LEG CONFIGURATION\n" % self.leg_name)

        instruction_text =("        Add all links related to the leg's part.\n\
        \n\
        ADDING A LINK         :   Click the link where the revolute joint is attached for each leg part. This is usually the child link of the joint.\n\
        DELETING A LINK     :   Click < to clear the leg part.\
        \n\
            ")

        self.instructions = QLabel(instruction_text)

        self.tab_label.setFont(QFont("Default", pointSize=12 ,weight=QFont.Bold))
        self.tab_label.setAlignment(Qt.AlignCenter)
        self.hip_joint = JointConfigurator(parent, "HIP")
        self.column.addWidget(self.hip_joint)
        self.upper_leg_joint = JointConfigurator(parent, "UPPER LEG")
        self.column.addWidget(self.upper_leg_joint)
        self.lower_leg_joint = JointConfigurator(parent, "LOWER LEG")
        self.column.addWidget(self.lower_leg_joint)
        self.foot_joint = JointConfigurator(parent, "FOOT")
        self.column.addWidget(self.foot_joint)

        self.row.addWidget(self.tab_label)
        self.row.addWidget(self.instructions)
        self.row.addLayout(self.column)

        self.setLayout(self.row)

    def get_prefix(self):
        leg_names = ["LEFT FRONT", "RIGHT FRONT", "LEFT HIND", "RIGHT HIND"]
        leg_prefixes = ["lf", "rf", "lh", "rh"]
        return leg_prefixes[leg_names.index(self.leg_name)]

    def get_leg_links(self):
        leg_prefix = self.get_prefix()

        hip = leg_prefix + "_hip_link"
        upper_leg = leg_prefix + "_upper_leg_link"
        lower_leg = leg_prefix + "_lower_leg_link"
        foot = leg_prefix + "_foot_link"

        if self.parent.using_urdf:
            hip = str(self.hip_joint.leg_links_list.item(0).text())
            upper_leg = str(self.upper_leg_joint.leg_links_list.item(0).text())
            lower_leg = str(self.lower_leg_joint.leg_links_list.item(0).text())
            foot = str(self.foot_joint.leg_links_list.item(0).text())

        
        return [hip, upper_leg, lower_leg, foot]

    def get_leg_joints(self):
        leg_prefix = self.get_prefix()
        links = self.get_leg_links()

        hip = leg_prefix + "_hip_joint"
        upper_leg = leg_prefix + "_upper_leg_joint"
        lower_leg = leg_prefix + "_lower_leg_joint"
      
        if self.parent.using_urdf:
            hip = self.parent.rviz_widget.robot.get_attached_joint(links[0])
            upper_leg = self.parent.rviz_widget.robot.get_attached_joint(links[1])
            lower_leg = self.parent.rviz_widget.robot.get_attached_joint(links[2])
           
        return [hip, upper_leg, lower_leg]

    def get_transform(self):
        if self.parent.using_urdf:
            return self.get_transform_from_urdf()
        else:
            return self.get_transform_from_box()

    def get_transform_from_box(self):
        transform = [0,0,0,0]
        transform[0] = [self.hip_joint.x_edit.value(), self.hip_joint.y_edit.value(), self.hip_joint.z_edit.value(), 0.0, 0.0, 0.0] 
        transform[1] = [self.upper_leg_joint.x_edit.value(), self.upper_leg_joint.y_edit.value(), self.upper_leg_joint.z_edit.value(), 0.0, 0.0, 0.0] 
        transform[2] = [self.lower_leg_joint.x_edit.value(), self.lower_leg_joint.y_edit.value(), self.lower_leg_joint.z_edit.value(), 0.0, 0.0, 0.0]
        transform[3] = [self.foot_joint.x_edit.value(), self.foot_joint.y_edit.value(), self.foot_joint.z_edit.value(), 0.0, 0.0, 0.0]
        
        return transform

    def get_transform_from_urdf(self):
        links = self.get_leg_links()
        joint_chain = self.parent.rviz_widget.robot.get_joint_chain(links[3])
        transform = []
        #get from transform from base to hip
        transform.append(self.parent.rviz_widget.robot.get_transform(joint_chain, self.parent.rviz_widget.robot.base, links[0]))

        #from hip all the way to the foot
        for i in range(3):
            transform.append(self.parent.rviz_widget.robot.get_transform(joint_chain, links[i], links[i+1]))

        return transform

class LegConfiguratorWidget(QWidget):
    def __init__(self, links_list, rviz_widget):
        super(QWidget, self).__init__()
        self.using_urdf = False
        self.rviz_widget = rviz_widget
        self.links_list = links_list
        self.leg_configurators = []
        self.rviz_widget.urdf_loaded.connect(self.on_urdf_path_load)

        self.layout = QGridLayout()

        self.column = QHBoxLayout()
    
        self.lf_links_column = QHBoxLayout()
        self.rf_links_column = QHBoxLayout()
        self.lh_links_column = QHBoxLayout()
        self.rh_links_column = QHBoxLayout()

        self.list_stack_row = QVBoxLayout()

        self.links_label = QLabel("LINKS")
        self.links_label.setAlignment(Qt.AlignCenter)
        self.list_stack_row.addWidget(self.links_label)

        self.links_list = links_list
        self.list_stack_row.addWidget(self.links_list)
        self.links_list.setFixedWidth(230)
        self.links_list.setFixedHeight(400)

        self.lf_configurator = LegConfigurator(self, "LEFT FRONT")
        self.rf_configurator = LegConfigurator(self, "RIGHT FRONT")
        self.lh_configurator = LegConfigurator(self, "LEFT HIND")
        self.rh_configurator = LegConfigurator(self, "RIGHT HIND")

        self.leg_configurators.append(self.lf_configurator)
        self.leg_configurators.append(self.rf_configurator)
        self.leg_configurators.append(self.lh_configurator)
        self.leg_configurators.append(self.rh_configurator)

        self.leg_tabs = QTabWidget()
        self.leg_tabs.addTab(self.lf_configurator, "Left Front")
        self.leg_tabs.addTab(self.rf_configurator, "Right Front")
        self.leg_tabs.addTab(self.lh_configurator, "Left Hind")
        self.leg_tabs.addTab(self.rh_configurator, "Right Hind")

        self.column.addLayout(self.list_stack_row)
        self.column.addWidget(self.leg_tabs)

        self.setLayout(self.column)

    def on_urdf_path_load(self):
        self.using_urdf = True
        links = self.rviz_widget.robot.link_names
        for link in links:
            self.links_list.addItem(link)

    def get_configuration(self):
        
        leg_configuration = {
            "links": {
                "base": "",
                "left_front": ["", "", "", ""],
                "right_front": ["", "", "", ""],
                "left_hind": ["", "", "", ""],
                "right_hind": ["", "", "", ""]
            },
            "joints": {
                "left_front": ["", "", ""],
                "right_front": ["", "", ""],
                "left_hind": ["", "", ""],
                "right_hind": ["", "", ""]
            },
            "firmware": {
                "transforms": {
                    "left_front": {
                        "hip": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "upper_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "lower_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "foot": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    },
                    "right_front": {
                        "hip": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "upper_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "lower_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "foot": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    },
                    "left_hind": {
                        "hip": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "upper_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "lower_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "foot": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    },
                    "right_hind": {
                        "hip": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "upper_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "lower_leg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "foot": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    }
                }
            }
        }

        leg_names = ["left_front", "right_front", "left_hind", "right_hind"]
        leg_configuration["links"]["base"] = self.rviz_widget.robot.base
        leg_configurated = 0

        for i in range(4):
            try:            
                #populate links
                leg_configuration["links"][leg_names[i]] = self.leg_configurators[i].get_leg_links()
                
                #populate joints
                leg_configuration["joints"][leg_names[i]] = self.leg_configurators[i].get_leg_joints()

                #populate transforms 
                leg_transform = self.leg_configurators[i].get_transform()
                leg_configuration["firmware"]["transforms"][leg_names[i]]["hip"] = leg_transform[0]
                leg_configuration["firmware"]["transforms"][leg_names[i]]["upper_leg"] = leg_transform[1]
                leg_configuration["firmware"]["transforms"][leg_names[i]]["lower_leg"] = leg_transform[2]
                leg_configuration["firmware"]["transforms"][leg_names[i]]["foot"] = leg_transform[3]

                leg_configurated += 1
            except:
                QMessageBox.information(self, "WARN", "Missing or incorrect link in %s leg" % leg_names[i])

        #check if the configuration is complete
        if leg_configurated == 4:
            return leg_configuration
    
        else:
            return None