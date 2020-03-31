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

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

class LinkListWidget(QListWidget):
    def __init__(self, main):
        super(LinkListWidget, self).__init__()
        self.main = main

        self.itemClicked.connect(self.on_click)
        self.prev_highlighted_link = None

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
        self.main.robot_viz.highlight_link(link_name)
                
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