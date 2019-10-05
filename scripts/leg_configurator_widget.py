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
                self.clear_highlight()

    def delete_selected_link(self):
        link_name = self.highlighted_link()

        links_list = self.findItems(link_name, Qt.MatchExactly)
        for link in links_list:
            mathced_link = self.row(link)
            self.takeItem(mathced_link)
            self.clear_highlight()

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
        self.clear_highlight()

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

class LegSubConfigurator(QWidget):
    def __init__(self, parent, label):

        super(QWidget, self).__init__()
        self.setFixedHeight(100)
        self.setFixedWidth(270)

        self.parent = parent

        self.column = QHBoxLayout()
        self.row = QVBoxLayout()
        self.layout = QGridLayout()

        self.link_label = QLabel(label)
        self.link_label.setAlignment(Qt.AlignCenter)

        self.leg_links_list = LinkListWidget(self.parent.rviz_widget)

        self.buttons_widget = AddDeleteButtonWidget()
        self.buttons_widget.add_button.clicked.connect(self.add_button_clicked)
        self.buttons_widget.delete_button.clicked.connect(self.clear_button_clicked)

        self.layout.addWidget(self.link_label,0,1)
        self.layout.addWidget(self.buttons_widget,1,0)
        self.layout.addWidget(self.leg_links_list,1,1)

        self.setLayout(self.layout)

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
    
        self.column = QHBoxLayout()
        self.row = QVBoxLayout()

        self.tab_label = QLabel("\t%s LEG CONFIGURATION\n" % leg_name)

        instruction_text =("        Add all links related to the leg's part.\n\
        \n\
        ADDING A LINK         :   Click the link where the revolute joint is attached for each leg part. This is usually the child link of the joint.\n\
        DELETING A LINK     :   Click < to clear the leg part.\
        \n\
            ")

        self.instructions = QLabel(instruction_text)

        self.tab_label.setFont(QFont("Default", pointSize=12 ,weight=QFont.Bold))
        self.tab_label.setAlignment(Qt.AlignCenter)
        self.hip_link = LegSubConfigurator(parent, "HIP")
        self.column.addWidget(self.hip_link)
        self.upper_leg_link = LegSubConfigurator(parent, "UPPER LEG")
        self.column.addWidget(self.upper_leg_link)
        self.lower_leg_link = LegSubConfigurator(parent, "LOWER LEG")
        self.column.addWidget(self.lower_leg_link)
        self.foot_link = LegSubConfigurator(parent, "FOOT")
        self.column.addWidget(self.foot_link)

        self.row.addWidget(self.tab_label)
        self.row.addWidget(self.instructions)
        self.row.addLayout(self.column)

        self.setLayout(self.row)

    def get_configuration(self):
        try:
            hip = self.hip_link.leg_links_list.item(0).text()
            upper_leg = self.upper_leg_link.leg_links_list.item(0).text()
            lower_leg = self.lower_leg_link.leg_links_list.item(0).text()
            foot = self.foot_link.leg_links_list.item(0).text()
            return [str(hip), str(upper_leg), str(lower_leg), str(foot)]

        except:
            return [None, None, None, None] 

class LegConfiguratorWidget(QWidget):
    def __init__(self, links_list, rviz_widget):
        super(QWidget, self).__init__()
        self.rviz_widget = rviz_widget
        self.parent_links_list = links_list
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

        self.leg_tabs = QTabWidget()
        self.leg_tabs.addTab(self.lf_configurator, "Left Front")
        self.leg_tabs.addTab(self.rf_configurator, "Right Front")
        self.leg_tabs.addTab(self.lh_configurator, "Left Hind")
        self.leg_tabs.addTab(self.rh_configurator, "Right Hind")

        self.column.addLayout(self.list_stack_row)
        self.column.addWidget(self.leg_tabs)

        self.setLayout(self.column)

    def on_urdf_path_load(self):
        self.links_list = self.parent_links_list
        links = self.rviz_widget.robot.link_names
        for link in links:
            self.links_list.addItem(link)

    def get_transforms(self, links):
        joint_chain = self.rviz_widget.robot.get_joint_chain(links[3])
        transforms = []
        #get from transform from base to hip
        transforms.append(self.rviz_widget.robot.get_transform(joint_chain, self.rviz_widget.robot.base, links[0]))

        #from hip all the way to the foot
        for i in range(3):
            transforms.append(self.rviz_widget.robot.get_transform(joint_chain, links[i], links[i+1]))

        return transforms

    def get_leg_joints(self, links):
        joints = []

        for i in range(3):
            joints.append(self.rviz_widget.robot.get_attached_joint(links[i]))

        return joints

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

        links = []
        transforms = []
        joints = []

        links.append(self.lf_configurator.get_configuration())
        links.append(self.rf_configurator.get_configuration())
        links.append(self.lh_configurator.get_configuration())
        links.append(self.rh_configurator.get_configuration())
        leg_configuration["links"]["base"] = self.rviz_widget.robot.base

        for i in range(4):
            try:
                transforms.append(self.get_transforms(links[i]))
                joints.append(self.get_leg_joints(links[i]))
                
                #populate links
                leg_configuration["links"][leg_names[i]] = links[i]
                
                #populate joints
                leg_configuration["joints"][leg_names[i]] = joints[i]

                #populate transforms 
                leg_configuration["firmware"]["transforms"][leg_names[i]]["hip"] = transforms[i][0]
                leg_configuration["firmware"]["transforms"][leg_names[i]]["upper_leg"] = transforms[i][1]
                leg_configuration["firmware"]["transforms"][leg_names[i]]["lower_leg"] = transforms[i][2]
                leg_configuration["firmware"]["transforms"][leg_names[i]]["foot"] = transforms[i][3]

            except:
                QMessageBox.information(self, "WARN", "Missing or incorrect link in %s leg" % leg_names[i])

        #check if the configuration is complete
        if len(transforms) == 4 and len(joints) == 4:
            return leg_configuration
    
        else:
            return None