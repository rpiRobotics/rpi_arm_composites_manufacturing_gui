#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QMutex, QMutexLocker,QSemaphore, QThread
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


class UserAuthenticationWindow(QWidget):
    def __init__(self):
        super(UserAuthenticationWindow,self).__init__()
        self.userlogins={'username': 'password'}
        self.returned=False
        self.success=False
        self.loginprompt = os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'resource', 'loginprompt.ui')
        loadUi(self.loginprompt,self)
        self.loginprompt.ok.pressed.connect(self.proceed)
        self.loginprompt.cancel.pressed.connect(self.cancel)



    def cancel(self):
        self.returned=True
        self.close()

    def proceed(self):
        if(str(self.loginprompt.username.text) in self.userlogins):
            if(str(self.loginprompt.password.text) == self.userlogins[str(self.loginprompt.username.text)]):
                self.success=True
                self.returned=True
                self.close()
            else:
                self.loginprompt.password.setText("Password Incorrect")
        else:
            self.loginprompt.username.setText("Username Incorrect")
            self.loginprompt.password.setText("")

