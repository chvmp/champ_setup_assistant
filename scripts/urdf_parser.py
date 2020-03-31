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

import roslib; roslib.load_manifest('urdfdom_py')
import rospy
from urdf_parser_py.urdf import URDF
from rosparam import upload_params
from yaml import load
import yaml
from difflib import SequenceMatcher 
import numpy as np

class URDFParser():
    def __init__(self):
        self.end_links = None
        self.foot_links = None
        self.foot_links_ns = None
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
        self.base = self.robot.get_root()
        self.end_links = self.get_end_links()
        self.foot_links, self.foot_links_ns = self.get_foot_links()

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
    
    def link_has_child(self, link_name):
        return self.robot.child_map.has_key(link_name)

    def link_attached_to_base(self, link_name):
        attached_joint, parent_link = self.robot.parent_map[link_name]
        #add more filter - if it's attached to base, it's not an end link
        if parent_link != self.base:
            return False
        else:
            return True

    def joint_is_revolute(self, joint_name):
        for joint in self.joints:
            if joint.name == joint_name:
                if joint.type == "revolute":
                    return True

        return False

    def has_actuator_in_chain(self, link_name):
        joints = self.get_joint_chain(link_name)
        for joint in joints:
            if self.joint_is_revolute(joint):
                return True
        
        return False

    def get_end_links(self):
        end_links = []
        for link in self.link_names:
            #check if this link has a child
            if not self.link_has_child(link) and not self.link_attached_to_base(link) and self.has_actuator_in_chain(link):
                end_links.append(link)

        return end_links
    
    def get_foot_links(self):
        def get_common_string(str1,str2):
            #returns common substring between two strings
            #https://www.geeksforgeeks.org/sequencematcher-in-python-for-longest-common-substring/
            # initialize SequenceMatcher object with  
            # input string 
            seqMatch = SequenceMatcher(None,str1,str2) 
        
            # find match of longest sub-string 
            # output will be like Match(a=0, b=0, size=5) 
            match = seqMatch.find_longest_match(0, len(str1), 0, len(str2)) 
        
            # print longest substring 
            if (match.size!=0): 

                return (str1[match.a: match.a + match.size])  
            else: 
                return None

        #get links that has no child
        end_links = self.get_end_links()
        if len(end_links) == 4:
            foot_name = get_common_string(end_links[0], end_links[3])
            ns =[]
            for link in end_links:
                ns.append(link.replace(foot_name, ""))

            return end_links, ns

        else:
            common_substrings = []
            #iterate all end links to each other
            for i in  end_links:
                for ii in end_links:
                    #get common substring between two link names
                    common = get_common_string(i, ii)

                    #make sure it's not some giberrish string
                    if len(common) > 3:
                        common_substrings.append(get_common_string(i, ii))
            #count the most no of occurance in the collected substrings
            strings, indices = np.unique(common_substrings, return_counts=True)
            #name of the link without ns / identifier
            foot_name = strings[0] 
        
            foot_links = []
            ns = []
            for link in end_links:
                #check if the footname matches the end_links
                if link.count(foot_name):
                    foot_links.append(link)
                    #remove footname on the link name to get namespace
                    ns.append(link.replace(foot_name, ""))

            return foot_links, ns
