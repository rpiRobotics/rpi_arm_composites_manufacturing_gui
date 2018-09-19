import os
import rospy
import rospkg
import collections
import time
import sys
import subprocess
import numpy as np
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QMutex, QMutexLocker,QSemaphore, QThread
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


from arm_composites_manufacturing_process import ProcessController

from rpi_arm_composites_manufacturing_abb_egm_controller.msg import ControllerState as controllerstate
from rpi_arm_composites_manufacturing_process.msg import ProcessStepAction, ProcessStepGoal
import actionlib
from rqt_console import console_widget
from rqt_console import message_proxy_model
from rqt_plot import plot
import rosservice
import rviz
import arm_composites_manufacturing_controller_commander as controller_commander_pkg
from panel_selector_window import PanelSelectorWindow
import pyqtgraph as pg


'''
freeBytes=QSemaphore(100)
usedBytes=QSemaphore()
consoleData=collections.deque(maxlen=200)
'''
class LEDIndicator(QAbstractButton):
    scaledSize=1000.0
    def __init__(self):
        QAbstractButton.__init__(self)
        self.setMinimumSize(24, 24)
        self.setCheckable(True)
        # Green
        self.on_color_1 = QColor(0, 255, 0)
        self.on_color_2 = QColor(0, 192, 0)
        self.off_color_1 = QColor(255, 0, 0)
        self.off_color_2 = QColor(128, 0, 0)

    def resizeEvent(self, QResizeEvent):
        self.update()

    def paintEvent(self, QPaintEvent):
        realSize = min(self.width(), self.height())

        painter = QPainter(self)
        pen = QPen(Qt.black)
        pen.setWidth(1)

        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(self.width() / 2, self.height() / 2)
        painter.scale(realSize / self.scaledSize, realSize / self.scaledSize)

        gradient = QRadialGradient(QPointF(-500, -500), 1500, QPointF(-500, -500))
        gradient.setColorAt(0, QColor(224, 224, 224))
        gradient.setColorAt(1, QColor(28, 28, 28))
        painter.setPen(pen)
        painter.setBrush(QBrush(gradient))
        painter.drawEllipse(QPointF(0, 0), 500, 500)

        gradient = QRadialGradient(QPointF(500, 500), 1500, QPointF(500, 500))
        gradient.setColorAt(0, QColor(224, 224, 224))
        gradient.setColorAt(1, QColor(28, 28, 28))
        painter.setPen(pen)
        painter.setBrush(QBrush(gradient))
        painter.drawEllipse(QPointF(0, 0), 450, 450)

        painter.setPen(pen)
        if self.isChecked():
            gradient = QRadialGradient(QPointF(-500, -500), 1500, QPointF(-500, -500))
            gradient.setColorAt(0, self.on_color_1)
            gradient.setColorAt(1, self.on_color_2)
        else:
            gradient = QRadialGradient(QPointF(500, 500), 1500, QPointF(500, 500))
            gradient.setColorAt(0, self.off_color_1)
            gradient.setColorAt(1, self.off_color_2)

        painter.setBrush(gradient)
        painter.drawEllipse(QPointF(0, 0), 400, 400)

    @pyqtProperty(QColor)
    def onColor1(self):
        return self.on_color_1

    @onColor1.setter
    def onColor1(self, color):
        self.on_color_1 = color

    @pyqtProperty(QColor)
    def onColor2(self):
        return self.on_color_2

    @onColor2.setter
    def onColor2(self, color):
        self.on_color_2 = color

    @pyqtProperty(QColor)
    def offColor1(self):
        return self.off_color_1

    @offColor1.setter
    def offColor1(self, color):
        self.off_color_1 = color

    @pyqtProperty(QColor)
    def offColor2(self):
        return self.off_color_2

    @offColor2.setter
    def offColor2(self, color):
        self.off_color_2 = color


class VacuumConfirm(QWidget):
    def __init__(self):
        super(VacuumConfirm,self).__init__()







class ConsoleThread(QThread):
    def __init__(self,State_info):
		super(ConsoleThread,self).__init__()
		self.State_info=State_info

    def run(self):
		while(True):
			usedBytes.acquire()
			dataval=consoleData.pop()
			self.State_info.addItem(dataval)
			self.State_info.scrollToBottom()
			#print dataval
			freeBytes.release()

class RQTPlotWindow(QMainWindow):
    def __init__(self, parent=None):
        super(RQTPlotWindow, self).__init__(None)
        self.rqtgraph=plot.Plot(parent)



class ExperimentGUI(Plugin):

    def __init__(self, context):
        super(ExperimentGUI, self).__init__(context)
        # Give QObjects reasonable names
        self.plans=['Starting Position','Pickup Prepare','Pickup Lower','Pickup Grab','Pickup Raise','Transport Payload','Place Panel and Lift']
        self.setObjectName('MyPlugin')

        #self.controller_commander=controller_commander_pkg.arm_composites_manufacturing_controller_commander()
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self.in_process=None
        self.recover_from_pause=False
        self.force_torque_plot=False
        self.joint_angle_plot=False
        self._mainwidget = QWidget()
        self.layout = QGridLayout()
        self._mainwidget.setLayout(self.layout)
        self.disconnectreturnoption=False
        self.stackedWidget=QStackedWidget()#self._mainwidget)
        self.layout.addWidget(self.stackedWidget,0,0)
        self._welcomescreen=QWidget()
        self._runscreen=QWidget()
        self._errordiagnosticscreen=QWidget()
        self.stackedWidget.addWidget(self._welcomescreen)
        self.stackedWidget.addWidget(self._runscreen)
        self.stackedWidget.addWidget(self._errordiagnosticscreen)
        #self._data_array=collections.deque(maxlen=500)
        self._proxy_model=message_proxy_model.MessageProxyModel()
        self._rospack=rospkg.RosPack()
        #self.console=console_widget.ConsoleWidget(self._proxy_model,self._rospack)
        self.robotconnectionled=LEDIndicator()
        self.robotconnectionled.setDisabled(True)  # Make the led non clickable
        self.forcetorqueled=LEDIndicator()
        self.forcetorqueled.setDisabled(True)  # Make the led non clickable
        self.overheadcameraled=LEDIndicator()
        self.overheadcameraled.setDisabled(True)  # Make the led non clickable
        self.grippercameraled=LEDIndicator()
        self.grippercameraled.setDisabled(True)  # Make the led non clickable
        #self.client=actionlib.SimpleActionClient('process_step', ProcessStepAction)
        #self.client.wait_for_server()
        self.placement_target='panel_nest_leeward_mid_panel_target'
        self.panel_type='leeward_mid_panel'

        self.led_change(self.overheadcameraled,True)
        self.led_change(self.grippercameraled,True)
        self.mode=0
        self.count=0
        self.data_count=0
        self.force_torque_data=np.zeros((3,1))

        # Get path to UI file which should be in the "resource" folder of this package
        self.welcomescreenui = os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'resource', 'welcomeconnectionscreen.ui')
        self.runscreenui = os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'resource', 'Runscreen.ui')
        self.errorscreenui = os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'resource', 'errordiagnosticscreen.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(self.welcomescreenui, self._welcomescreen)
        loadUi(self.runscreenui, self._runscreen)
        loadUi(self.errorscreenui,self._errordiagnosticscreen)
        # Give QObjects reasonable names
        self._mainwidget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._mainwidget.setWindowTitle(self._mainwidget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._mainwidget)
        self.context=context

        self.plugin_settings=None
        self.instance_settings=None
        #self._errordiagnosticscreen.consoleWidget=console_widget.ConsoleWidget(self._proxy_model,self._rospack)
        #####consoleThread=ConsoleThread(self._widget.State_info)
       # self._welcomescreen.statusFormLayout.takeAt(0)
        self._welcomescreen.statusFormLayout.addWidget(self.robotconnectionled,0,0)
        self._welcomescreen.statusFormLayout.addWidget(self.forcetorqueled,2,0)
        self._welcomescreen.statusFormLayout.addWidget(self.overheadcameraled,4,0)
        self._welcomescreen.statusFormLayout.addWidget(self.grippercameraled,6,0)
        #self._welcomescreen.robotConnectionWidget.addWidget(self.led)
        #consoleThread.finished.connect(app.exit)

        #####consoleThread.start()
        self.rviz_starter=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'rviz_starter.py')
        self.reset_code=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'Reset_Start_pos_wason2.py')
        self.YC_place_code=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'Vision_MoveIt_new_Cam_YC.py')
        # Add widget to the user interface
        #context.add_widget(console)
            #context.add_widget(rqt_console)

        for entry in self.plans:
            listentry=QListWidgetItem(entry)
            listentry.setFlags(Qt.ItemIsSelectable)
            self._runscreen.planList.addItem(listentry)

        self._runscreen.planList.item(0).setSelected(True)
        self.planListIndex=0
        self._runscreen.vacuum.setText("OFF")
        self._runscreen.panel.setText("Detached")
        self._runscreen.panelTag.setText("Not Localized")
        self._runscreen.nestTag.setText("Not Localized")
        self._runscreen.overheadCamera.setText("OFF")
        self._runscreen.gripperCamera.setText("OFF")
        self._runscreen.forceSensor.setText("Biased to 0")
        self._runscreen.pressureSensor.setText("[0,0,0]")
        self._runscreen.panelType.setText("Leeward Mid Panel")
        self._runscreen.placementNestTarget.setText("Leeward Mid Panel Nest")
        self._runscreen.vacuum.setReadOnly(True)
        self._runscreen.panel.setReadOnly(True)
        self._runscreen.panelTag.setReadOnly(True)
        self._runscreen.nestTag.setReadOnly(True)
        self._runscreen.overheadCamera.setReadOnly(True)
        self._runscreen.gripperCamera.setReadOnly(True)
        self._runscreen.forceSensor.setReadOnly(True)
        self._runscreen.pressureSensor.setReadOnly(True)
        self._runscreen.panelType.setReadOnly(True)
        self._runscreen.placementNestTarget.setReadOnly(True)
        """
        self.above_panel=False
        self._widget.Speed_scalar.setInputMask("9.99")
        self._widget.Speed_scalar.setText("1.00")
        self._widget.Vacuum.toggled.connect(self._handle_vacuum_change)
        self._widget.Speed_scalar.textEdited.connect(self._change_values)
        """
        rospy.Subscriber("controller_state", controllerstate, self.callback)
        self._welcomescreen.openConfig.clicked.connect(lambda: self.led_change(self.robotconnectionled,False))
        #self._welcomescreen.openAdvancedOptions.pressed.connect(self._open_login_prompt)
        self._welcomescreen.toRunScreen.pressed.connect(self._to_run_screen)
        self._runscreen.backToWelcome.pressed.connect(self._to_welcome_screen)
        self._runscreen.toErrorScreen.pressed.connect(self._to_error_screen)
        self._runscreen.nextPlan.pressed.connect(self._nextPlan)
        self._runscreen.previousPlan.pressed.connect(self._previousPlan)
        self._runscreen.resetToHome.pressed.connect(self._reset_position)
        self._runscreen.stopPlan.pressed.connect(self._stopPlan)
        #self._errordiagnosticscreen.openOverheadCameraView.pressed.connect(self._open_overhead_camera_view)
        #self._errordiagnosticscreen.openGripperCameraViews.pressed.connect(self._open_gripper_camera_views)
        self._errordiagnosticscreen.openForceTorqueDataPlot.pressed.connect(self._open_force_torque_data_plot)
        self._errordiagnosticscreen.openJointAngleDataPlot.pressed.connect(self._open_joint_angle_data_plot)
        self._errordiagnosticscreen.backToRun.pressed.connect(self._to_run_screen)
        #self._runscreen.widget.frame=rviz.VisualizationFrame()
        #self._runscreen.widget.frame.setSplashPath( "" )

        ## VisualizationFrame.initialize() must be called before
        ## VisualizationFrame.load().  In fact it must be called
        ## before most interactions with RViz classes because it
        ## instantiates and initializes the VisualizationManager,
        ## which is the central class of RViz.
        #self._runscreen.widget.frame.initialize()
        #self.manager = self._runscreen.widget.frame.getManager()


#        self._welcomescreen.openAdvancedOptions.pressed.connect(self._open_advanced_options)

    def led_change(self,led,state):
        led.setChecked(state)

    def _to_welcome_screen(self):
        self.stackedWidget.setCurrentIndex(0)

    def _to_run_screen(self):
        self.stackedWidget.setCurrentIndex(1)
        self.messagewindow=PanelSelectorWindow()
        self.messagewindow.show()
        self.messagewindow.setFixedSize(self.messagewindow.size())

    def _to_error_screen(self):
        self.stackedWidget.setCurrentIndex(2)

    #def _open_overhead_camera_view(self):


    #def _open_gripper_camera_views(self):



    def _open_force_torque_data_plot(self):
        x = np.arange(1000)
        y = np.random.normal(size=(3, 1000))
        self.force_torque_plot_widget=QDockWidget()
        self.force_torque_plot_widget=pg.plot()
        #self.layout.addWidget(self.force_torque_plot_widget,0,1)




        self.force_torque_plot_widget.plot()

        #self.force_torque_plotter=PlotManager(title='Force Torque Data',nline=3,widget=self.force_torque_plot_widget)
        #self.force_torque_plot_widget.show()
        self.force_torque_plot=True
        #self.force_torque_plotter.add("Hello", np.arange(10))
        #self.force_torque_plotter.update()


        #self.rosGraph.show()
        #self.rosGraph.exec_()
    def _open_joint_angle_data_plot(self):
        self.rosGraph=RQTPlotWindow(self.context)

    def _open_rviz_prompt(self):
        subprocess.Popen(['python', self.rviz_starter])
#    def _open_advanced_options(self):
#        main = Main()
#        sys.exit(main.main(sys.argv, standalone='rqt_rviz/RViz', plugin_argument_provider=add_arguments))

    def _raise_rviz_window(self):
        subprocess.call(["xdotool", "search", "--name", "rviz", "windowraise"])

    def _execute_step(self,step, target=""):
        client=self.client
        g=ProcessStepGoal(step, target)
        while(self.in_process==True):
            pass
        client.send_goal(g)
        self.in_process=True

        print client.get_result()
        #TODO: using client.get_state can implemen action state recall to eliminate plan from moveit?
    #TODO: make it so that next plan throws it back into automatic mode every time and then teleop switches to teleop mode and plans the next move
    def _nextPlan(self):
        if(self.planListIndex+1==self._runscreen.planList.count()):
            self.planListIndex=0
        elif self.recover_from_pause:
            self.recover_from_pause=False
        else:
            self.planListIndex+=1
        #self._open_rviz_prompt()
        self._raise_rviz_window()

        self._runscreen.planList.item(self.planListIndex).setSelected(True)
        time.sleep(1)

        if(self.planListIndex==0):
            subprocess.Popen(['python', self.reset_code])
            self._runscreen.vacuum.setText("OFF")
            self._runscreen.panel.setText("Detached")
            self._runscreen.panelTag.setText("Not Localized")
            self._runscreen.nestTag.setText("Not Localized")
            self._runscreen.overheadCamera.setText("OFF")
            self._runscreen.gripperCamera.setText("OFF")
            self._runscreen.forceSensor.setText("Biased to 0")
            self._runscreen.pressureSensor.setText("[0,0,0]")
        elif(self.planListIndex==1):

            self._execute_step('plan_pickup_prepare',self.panel_type)
            self._execute_step('move_pickup_prepare')
            self._runscreen.vacuum.setText("OFF")
            self._runscreen.panel.setText("Detached")
            self._runscreen.panelTag.setText("Localized")
            self._runscreen.nestTag.setText("Not Localized")
            self._runscreen.overheadCamera.setText("ON")
            self._runscreen.gripperCamera.setText("OFF")
            self._runscreen.forceSensor.setText("ON")
            self._runscreen.pressureSensor.setText("[0,0,0]")
        elif(self.planListIndex==2):
            self._execute_step('plan_pickup_lower')
            self._execute_step('move_pickup_lower')
            self._runscreen.vacuum.setText("OFF")
            self._runscreen.panel.setText("Detached")
            self._runscreen.panelTag.setText("Localized")
            self._runscreen.nestTag.setText("Not Localized")
            self._runscreen.overheadCamera.setText("OFF")
            self._runscreen.gripperCamera.setText("OFF")
            self._runscreen.forceSensor.setText("ON")
            self._runscreen.pressureSensor.setText("[0,0,0]")
        elif(self.planListIndex==3):
            self._execute_step('plan_pickup_grab_first_step')
            self._execute_step('move_pickup_grab_first_step')
            self._execute_step('plan_pickup_grab_second_step')
            self._execute_step('move_pickup_grab_second_step')
            self._runscreen.vacuum.setText("ON")
            self._runscreen.panel.setText("Attached")
            self._runscreen.panelTag.setText("Localized")
            self._runscreen.nestTag.setText("Not Localized")
            self._runscreen.overheadCamera.setText("OFF")
            self._runscreen.gripperCamera.setText("OFF")
            self._runscreen.forceSensor.setText("ON")
            self._runscreen.pressureSensor.setText("[1,1,1]")
        elif(self.planListIndex==4):
            self._execute_step('plan_pickup_raise')
            self._execute_step('move_pickup_raise')
            self._runscreen.vacuum.setText("ON")
            self._runscreen.panel.setText("Attached")
            self._runscreen.panelTag.setText("Localized")
            self._runscreen.nestTag.setText("Not Localized")
            self._runscreen.overheadCamera.setText("OFF")
            self._runscreen.gripperCamera.setText("OFF")
            self._runscreen.forceSensor.setText("OFF")
            self._runscreen.pressureSensor.setText("[1,1,1]")
        elif(self.planListIndex==5):
            self._execute_step('plan_transport_payload',self.placement_target)
            self._execute_step('move_transport_payload')
            self._runscreen.vacuum.setText("ON")
            self._runscreen.panel.setText("Attached")
            self._runscreen.panelTag.setText("Localized")
            self._runscreen.nestTag.setText("Not Localized")
            self._runscreen.overheadCamera.setText("OFF")
            self._runscreen.gripperCamera.setText("OFF")
            self._runscreen.forceSensor.setText("OFF")
            self._runscreen.pressureSensor.setText("[1,1,1]")
        elif(self.planListIndex==6):
            subprocess.Popen(['python', self.YC_place_code])
            self._runscreen.vacuum.setText("OFF")
            self._runscreen.panel.setText("Detached")
            self._runscreen.panelTag.setText("Localized")
            self._runscreen.nestTag.setText("Localized")
            self._runscreen.overheadCamera.setText("OFF")
            self._runscreen.gripperCamera.setText("ON")
            self._runscreen.forceSensor.setText("ON")
            self._runscreen.pressureSensor.setText("[0,0,0]")




    def _previousPlan(self):
        if(self.planListIndex==0):
            self.planListIndex=self._runscreen.planList.count()-1
        else:
            self.planListIndex-=1
        self._runscreen.planList.item(self.planListIndex).setSelected(True)

    def _stopPlan(self):
        #client=self.client
        #
        self.controller_commander.set_controller_mode(self.controller_commander.MODE_HALT, 0, [])
        client=self.client
        client.cancel_all_goals()
        self.recover_from_pause=True


    def _reset_position(self):
        messagewindow=VacuumConfirm()
        reply = QMessageBox.question(messagewindow, 'Path Verification',
                     'Proceed to Reset Position', QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply==QMessageBox.Yes:
            self.planListIndex=0
            self._runscreen.planList.item(self.planListIndex).setSelected(True)
            subprocess.Popen(['python', self.reset_code])
            self._runscreen.vacuum.setText("OFF")
            self._runscreen.panel.setText("Detached")
            self._runscreen.panelTag.setText("Not Localized")
            self._runscreen.nestTag.setText("Not Localized")
            self._runscreen.overheadCamera.setText("OFF")
            self._runscreen.gripperCamera.setText("OFF")
            self._runscreen.forceSensor.setText("Biased to 0")
            self._runscreen.pressureSensor.setText("[0,0,0]")
        else:
            rospy.loginfo("Reset Rejected")   

    def start_shared_control(self):
        if(self.planListIndex+1==self._runscreen.planList.count()):
            self.planListIndex=0
        elif self.recover_from_pause:
            self.recover_from_pause=False
        else:
            self.planListIndex+=1

        if(self.planListIndex==1):
            self._execute_step('plan_pickup_prepare',self.panel_type)
        elif(self.planListIndex==2):
            self._execute_step('plan_pickup_lower')
        elif(self.planListIndex==3):
            self._execute_step('plan_pickup_grab_first_step')
            #TODO: How to handle what happens after threshold exceeded to generate next plan step
        elif(self.planListIndex==4):
            self._execute_step('plan_pickup_raise')
        elif(self.planListIndex==5):
            self._execute_step('plan_transport_payload',self.placement_target)
        self.controller_commander.set_controller_mode(self.controller_commander.MODE_SHARED_TRAJECTORY, 0, [])

    def callback(self,data):
        #self._widget.State_info.append(data.mode)
        if(self.stackedWidget.currentIndex()==2):
            if(self.count>10):
            #stringdata=str(data.mode)
            #freeBytes.acquire()
            #####consoleData.append(str(data.mode))
                self._errordiagnosticscreen.consoleWidget_2.addItem(str(data.joint_position))
                self.count=0

                #print data.joint_position

            self.count+=1
            #self._widget.State_info.scrollToBottom()
            #usedBytes.release()
            #self._data_array.append(stringdata)
            #print self._widget.State_info.count()
            if(self._errordiagnosticscreen.consoleWidget_2.count()>200):

                item=self._errordiagnosticscreen.consoleWidget_2.takeItem(0)
                #print "Hello Im maxed out"
                del item

        if self.in_process:
            if self.client.get_state() == actionlib.GoalStatus.PENDING or self.client.get_state() == actionlib.GoalStatus.ACTIVE:
                self._runscreen.nextPlan.setDisabled(True)
                self._runscreen.previousPlan.setDisabled(True)
                self._runscreen.resetToHome.setDisabled(True)
                rospy.loginfo("Pending")
            elif self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                self._runscreen.nextPlan.setDisabled(False)
                self._runscreen.previousPlan.setDisabled(False)
                self._runscreen.resetToHome.setDisabled(False)
                self.in_process=False
                rospy.loginfo("Succeeded")
            elif self.client.get_state() == actionlib.GoalStatus.ABORTED:
                self.in_process=False
                if(not self.recover_from_pause):
                    raise Exception("Process step failed and aborted")

            elif self.client.get_state() == actionlib.GoalStatus.REJECTED:
                self.in_process=False
                raise Exception("Process step failed and Rejected")
            elif self.client.get_state() == actionlib.GoalStatus.LOST:
                self.in_process=False
                raise Exception("Process step failed and lost")

        if(self.count>10):
            self.count=0
            if(data.error_msg=="No data received from robot"):
                #self.stackedWidget.setCurrentIndex(0)
                #messagewindow=VacuumConfirm()
                #reply = QMessageBox.question(messagewindow, 'Connection Lost',
                         #    'Robot Connection Lost, Return to Welcome Screen?', QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
                #if reply==QMessageBox.Yes:
                 #
                 #   self.disconnectreturnoption=False
                #else:
       #             self.disconnectreturnoption=True

                self.led_change(self.robotconnectionled,False)
            else:
                self.led_change(self.robotconnectionled,True)
            if(data.ft_wrench_valid=="False"):
                self.stackedWidget.setCurrentIndex(0)

                self.led_change(self.forcetorqueled,False)
            else:

                self.led_change(self.forcetorqueled,True)

            #if(self.disconnectreturnoption and data.error_msg==""):
             #   self.disconnectreturnoption=False
        self.count+=1

        if(self.force_torque_plot):
            incoming=np.array([data.ft_wrench]).reshape(3,1)
            self.force_torque_data.append(incoming,axis=1)
            if(self.data_count>500):
                self.force_torque_data=self.force_torque_data[...,1:]
            else:
                self.data_count+=1

            #if(len(self._data_array)>10):
        #	for x in self._data_array:
        #		self._widget.State_info.append(x)
    def change_controller_state(self,mode,speed_scalar,ft_threshold):
        self.mode=mode

        #self.armcontroller.set_controller(mode,speed_scalar,ft_threshold)
        #self._widget.Speed_scalar_display.setText(str(speed_scalar))
        if(len(ft_threshold)>0):
            self._widget.Ft_threshold_display.setText(str(ft_threshold[0]))
        else:
            self._widget.Ft_threshold_display.setText(str("None"))
        self._widget.Mode_display.setText(str(mode))
		

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
# Usually used to open a modal configuration dialog
