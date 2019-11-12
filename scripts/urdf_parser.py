#!/usr/bin/env python
import roslib; roslib.load_manifest('urdfdom_py')
import rospy
from urdf_parser_py.urdf import URDF
from rosparam import upload_params
from yaml import load
import yaml

class URDFParser():
    def __init__(self):
        self.robot = None
        self.joints = None
        self.links = None
        self.link_names = None
        self.base = ""

    def load_urdf(self, path):
        self.path = path
        self.robot = URDF.from_xml_file(path)
        self.links = self.parse_links(self.robot)
        self.joints = self.parse_joints(self.robot)
        self.link_names = self.parse_link_names(self.links)
        self.joint_names = self.parse_joint_names(self.joints)
        self.base = self.link_names[0]

    def open_urdf(self, full_path):
        return URDF.from_xml_file(full_path)

    def open_config_file(self, full_path):

        configs = None
        with open(full_path, 'r') as stream:
            try:
                configs = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        return configs

    def parse_joints(self, robot_description):
        joints = robot_description.joints
        buffer = []

        for i in range(len(joints)):
            buffer.append(joints[i])

        return buffer

    def parse_links(self, robot_description):
        links = robot_description.links
        buffer = []

        for i in range(len(links)):
            buffer.append(links[i])

        return buffer
    
    def parse_link_names(self, links):
        buffer = []

        for i in range(len(links)):
            buffer.append(links[i].name)

        return buffer

    def parse_joint_names(self, joints):
        buffer = []

        for i in range(len(joints)):
            buffer.append(joints[i].name)

        return buffer

    def get_transform(self, chain, ref_link, end_link):
        if ref_link == self.base:
            start_index = 0
        else:
            start_joint = self.get_attached_joint(ref_link)
            start_index = chain.index(start_joint) + 1

        end_joint = self.get_attached_joint(end_link)
        end_index = chain.index(end_joint)
        trans_x = 0
        trans_y = 0
        trans_z = 0
        roll = 0
        pitch = 0
        yaw = 0
 
        for i in range(start_index, end_index + 1):
            pos, orientation = self.get_joint_origin(chain[i])
            if orientation == None:
                orientation = [0,0,0]
                
            trans_x += pos[0]
            trans_y += pos[1]
            trans_z += pos[2]
            roll += orientation[0]
            pitch += orientation[1]
            yaw += orientation[2]

        return [trans_x, trans_y, trans_z, roll, pitch, yaw]

    def get_chain(self, base_link, end_link):
        self.robot.get_chain(base_link, end_link)
    
    def get_link_chain(self, end_link):
        return self.robot.get_chain(self.base, end_link, joints=False )

    def get_joint_chain(self, end_link):
        return self.robot.get_chain(self.base, end_link, links=False)

    def get_attached_joint(self, link_name):
        attached_joint = ""
        for joint in self.joints:
            if joint.child == link_name:
                attached_joint = joint.name

        return attached_joint

    def get_joint_origin(self, joint_name):
        for joint in self.joints:
            if joint.name == joint_name:
                return joint.origin.xyz, joint.origin.rpy


