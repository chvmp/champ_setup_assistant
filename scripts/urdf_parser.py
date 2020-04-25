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
            
            if pos == None:
                pos = [0,0,0]
                
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
                if joint.type == "revolute" or joint.type == "continuous":
                    return True

        return False

    def get_no_of_actuators(self, link_name):
        joints = self.get_joint_chain(link_name)
        no_of_actuators = 0
        for joint in joints:
            if self.joint_is_revolute(joint):
                no_of_actuators += 1
        
        return no_of_actuators

    def get_end_links(self):
        end_links = []
        for link in self.link_names:
            #check if this link has a child
            if not self.link_has_child(link) and not self.link_attached_to_base(link):
                end_links.append(link)

        return end_links
    
    def remove_manipulator(self, end_links):
        #this removes a possible manipulator in a list of end links
        #how this works is it counts the no of actuators of every end link's chain
        #all leg chains will have the same no of actuators
        #if an end link has a no of actuator in its chain
        #that's not equal to the common no of actuators for each leg, it will be removed

        no_of_actuators = []
        filtered_end_links = []
        #store no of actuators in each end links chain
        for link in end_links:
            no_of_actuators.append(self.get_no_of_actuators(link))
        
        #index of this array is the no of actuators
        #each element is the no of times that no of actuator occured
        bin_array = np.bincount(no_of_actuators)

        for i in range(len(end_links)):
            #get no of actuator for an end link's chain
            chain_actuator_count = no_of_actuators[i]
            #only add in the new list if it occured 4 times
            if bin_array[chain_actuator_count] == 4:
                filtered_end_links.append(end_links[i])
        
        return filtered_end_links

    def get_max(self, end_links):
        #this returns end links with more no of actuators

        new_end_links = []
        no_of_actuators = []

        #get no of actuators for each link
        for link in end_links:
            no_of_actuators.append(self.get_no_of_actuators(link))

        max_actuator_count = max(no_of_actuators)

        #add end_links that only has the max counted no of actuators
        for i in range(len(end_links)):
            if no_of_actuators[i] == max_actuator_count:
                new_end_links.append(end_links[i])

        return new_end_links

    def get_foot_links(self):
        def get_common_string(str1,str2):
            #returns common substring between two strings
            #https://www.geeksforgeeks.org/sequencematcher-in-python-for-longest-common-substring
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
        end_links = self.remove_manipulator(end_links)
        end_links = self.get_max(end_links)

        foot_name = get_common_string(end_links[0], end_links[3])
        ns =[]
        for link in end_links:
            ns.append(link.replace(foot_name, ""))

        return end_links, ns