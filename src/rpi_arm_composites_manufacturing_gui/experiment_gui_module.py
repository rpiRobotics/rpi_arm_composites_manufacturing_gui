import os
import rospy
import rospkg
import collections
import time
import sys
import subprocess

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QMutex, QMutexLocker,QSemaphore, QThread
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


from arm_composites_manufacturing_process import ProcessController

from rpi_arm_composites_manufacturing_abb_egm_controller.msg import ControllerState as controllerstate

from rqt_console import console_widget
from rqt_console import message_proxy_model
from rqt_graph import ros_graph
import rosservice




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
        self.rqtgraph=ros_graph.RosGraph(parent)



class ExperimentGUI(Plugin):

    def __init__(self, context):
        #rospy.init_node('collision_checker','move_group_python_interface_tutorial', anonymous=True)
        super(ExperimentGUI, self).__init__(context)
        # Give QObjects reasonable names
        self.plans=['Starting Position','Pickup Prepare','Pickup Lower','Pickup Grab','Pickup Raise','Transport Payload','Place Panel and Lift']
        self.setObjectName('MyPlugin')

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
        self._mainwidget = QWidget()
        self.stackedWidget=QStackedWidget(self._mainwidget)
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
        self.process_controller=ProcessController()


        self.mode=0
        self.count=0
        change_controller_state=self.change_controller_state
        #self.armcontroller=ARMControllerCommander()
        #self.Vision_MoveIt=VisionMoveIt(self.armcontroller,change_controller_state)
        #self.Vision_MoveIt.moveit_init()
        #self.Vision_MoveIt.load_offsets_from_yaml()
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
        """
        self.above_panel=False
        self._widget.Speed_scalar.setInputMask("9.99")
        self._widget.Ft_threshold.setInputMask("999")
        self._widget.Speed_scalar.setText("1.00")
        self._widget.Ft_threshold.setText("300")
        rospy.Subscriber("controller_state", controllerstate, self.callback)
        self._widget.tabWidget.currentChanged.connect(self._handle_controller_state_change)
        self._widget.Vacuum.toggled.connect(self._handle_vacuum_change)

        self._widget.Speed_scalar.textEdited.connect(self._change_values)
        self._widget.Ft_threshold.textEdited.connect(self._change_values)
        self._widget.Panel_pickup.pressed.connect(self._pickup_panel)
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
        self._errordiagnosticscreen.openDataPlot.pressed.connect(self._open_data_plot)
        self._errordiagnosticscreen.backToRun.pressed.connect(self._to_run_screen)

#        self._welcomescreen.openAdvancedOptions.pressed.connect(self._open_advanced_options)

    def led_change(self,led,state):
        led.setChecked(state)

    def _to_welcome_screen(self):
        self.stackedWidget.setCurrentIndex(0)

    def _to_run_screen(self):
        self.stackedWidget.setCurrentIndex(1)

    def _to_error_screen(self):
        self.stackedWidget.setCurrentIndex(2)

    def _open_data_plot(self):
        self.rosGraph=RQTPlotWindow(self.context)
        #self.rosGraph.show()
        #self.rosGraph.exec_()

    def _open_rviz_prompt(self):
        subprocess.Popen(['python', self.rviz_starter])
#    def _open_advanced_options(self):
#        main = Main()
#        sys.exit(main.main(sys.argv, standalone='rqt_rviz/RViz', plugin_argument_provider=add_arguments))
    def _nextPlan(self):
        if(self.planListIndex+1==self._runscreen.planList.count()):
            self.planListIndex=0
        else:
            self.planListIndex+=1
        #self._open_rviz_prompt()
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

            self.process_controller.pickup_prepare('leeward_mid_panel')
            self._runscreen.vacuum.setText("OFF")
            self._runscreen.panel.setText("Detached")
            self._runscreen.panelTag.setText("Localized")
            self._runscreen.nestTag.setText("Not Localized")
            self._runscreen.overheadCamera.setText("ON")
            self._runscreen.gripperCamera.setText("OFF")
            self._runscreen.forceSensor.setText("ON")
            self._runscreen.pressureSensor.setText("[0,0,0]")
        elif(self.planListIndex==2):
            self.process_controller.pickup_lower()
            self._runscreen.vacuum.setText("OFF")
            self._runscreen.panel.setText("Detached")
            self._runscreen.panelTag.setText("Localized")
            self._runscreen.nestTag.setText("Not Localized")
            self._runscreen.overheadCamera.setText("OFF")
            self._runscreen.gripperCamera.setText("OFF")
            self._runscreen.forceSensor.setText("ON")
            self._runscreen.pressureSensor.setText("[0,0,0]")
        elif(self.planListIndex==3):
            self.process_controller.pickup_grab()
            self._runscreen.vacuum.setText("ON")
            self._runscreen.panel.setText("Attached")
            self._runscreen.panelTag.setText("Localized")
            self._runscreen.nestTag.setText("Not Localized")
            self._runscreen.overheadCamera.setText("OFF")
            self._runscreen.gripperCamera.setText("OFF")
            self._runscreen.forceSensor.setText("ON")
            self._runscreen.pressureSensor.setText("[1,1,1]")
        elif(self.planListIndex==4):
            self.process_controller.pickup_raise()
            self._runscreen.vacuum.setText("ON")
            self._runscreen.panel.setText("Attached")
            self._runscreen.panelTag.setText("Localized")
            self._runscreen.nestTag.setText("Not Localized")
            self._runscreen.overheadCamera.setText("OFF")
            self._runscreen.gripperCamera.setText("OFF")
            self._runscreen.forceSensor.setText("OFF")
            self._runscreen.pressureSensor.setText("[1,1,1]")
        elif(self.planListIndex==5):
            self.process_controller.transport_payload('panel_nest_leeward_mid_panel_target')
            self.process_controller.pickup_raise()
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
            self.process_controller.pickup_raise()
            self._runscreen.vacuum.setText("OFF")
            self._runscreen.panel.setText("Detached")
            self._runscreen.panelTag.setText("Localized")
            self._runscreen.nestTag.setText("Localized")
            self._runscreen.overheadCamera.setText("OFF")
            self._runscreen.gripperCamera.setText("ON")
            self._runscreen.forceSensor.setText("ON")
            self._runscreen.pressureSensor.setText("[0,0,0]")

        self._runscreen.planList.item(self.planListIndex).setSelected(True)


    def _previousPlan(self):
        print "Not Implemented yet"

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
	
    def _move_to_panel(self):
        rospy.loginfo("Move to Panel command given")
        #self.Vision_MoveIt.camera_read()
        #self.Vision_MoveIt.set_positions()
        rospy.loginfo("Camera read and positions set")
        #self.armcontroller.set_controller(4,self.speed_scalar,self.ft_threshold)

        #plan1=self.Vision_MoveIt.generate_plan(1)
        rospy.loginfo("Plans generated, asking for confirmation of plan")
        messagewindow=VacuumConfirm()
        reply = QMessageBox.question(messagewindow, 'Path Verification',
		             'Is MoveIt Motion Path Acceptable?', QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply==QMessageBox.Yes:
            rospy.loginfo("Path Accepted, Proceeding to Panel")
            self.Vision_MoveIt.execute_plans(plan1)
            self.above_panel=True
        else:
            rospy.logerr("Path Rejected, Press Move Above Panel again to retry")
        
        

    def _pickup_panel(self):
        if(self.above_panel):
            rospy.loginfo("Pickup Panel Command given")
            #self.armcontroller.set_controller(4,self.speed_scalar,self.ft_threshold)
            '''if(self.speed_scalar>0.4):
                self._widget.Error_msg.setText("Speed Scalar Value is too High for Pickup Operation")
                return
            '''
            plan2=self.Vision_MoveIt.generate_plan(2)
            rospy.loginfo("Generating Pickup plan")
            self.Vision_MoveIt.execute_plans(plan2)
            rospy.loginfo("Finished lowering robot")
            #self.armcontroller.set_controller(4,self.speed_scalar,[])

            self.armcontroller.set_vacuum(1)
            self._widget.Vacuum.setCheckState(True)
            rospy.sleep(1)
            time.sleep(1)
            plan3=self.Vision_MoveIt.generate_plan(3)
            rospy.loginfo("Generated Raise plan")
            self.Vision_MoveIt.execute_plans(plan3)
            rospy.loginfo("Finished raising panel")
        else:
            #self._widget.Error_msg.setText("Not above Panel")
            rospy.logerr("You must move above Panel before asking to pickup")

    def _change_values(self):
	
	
        if self._widget.Speed_scalar.isModified():
            if(self._widget.Speed_scalar.hasAcceptableInput()):

                self.speed_scalar=float(self._widget.Speed_scalar.text())
                rospy.loginfo("Speed scalar value modified to %f", self.speed_scalar)
                #add in setter function
                self.change_controller_state(self.mode,self.speed_scalar,self.ft_threshold)
                self._widget.Error_msg.setText("")
				    
            else:
                rospy.logerr("Speed scalar value not accepted, new value needed")
                #self._widget.Error_msg.setText("Speed Scalar Value not Acceptable")
                enter_error=True
                self._widget.Speed_scalar.setModified(False)

        if self._widget.Ft_threshold.isModified():
            if(self._widget.Ft_threshold.hasAcceptableInput()):
                '''if(not len(self.ft_threshold)>0):
                    self.ft_threshold.extend([None,None,None,None,None,None])
                    print self.ft_threshold
                    print "hello"
                print len(self.ft_threshold)
                '''
                for i in range(len(self.ft_threshold)):
                    self.ft_threshold[i]=int(self._widget.Ft_threshold.text())
                    #add in setter function
                rospy.loginfo("Force Threshold value modified to %i", self.ft_threshold[0])
                self.change_controller_state(self.mode,self.speed_scalar,self.ft_threshold)
                self._widget.Error_msg.setText("")
				
            else:
                rospy.logerr("Force Threshold value not accepted, new value needed")
                #self._widget.Error_msg.setText("Force Threshold Value not Acceptable")

                self._widget.Ft_threshold.setModified(False)
		    
			

    def _handle_vacuum_change(self):
        if self._widget.Vacuum.isChecked():
            rospy.loginfo("GUI activated Vacuum")
            #self.armcontroller.set_vacuum(1)
	    
        else:
            messagewindow=VacuumConfirm()
            reply = QMessageBox.question(messagewindow, 'Release Vacuum?',
                 'Shutdown Vacuum?', QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply==QMessageBox.Yes:
                rospy.loginfo("Vacuum shutdown from GUI")
                #self.armcontroller.set_vacuum(0)
            else:
                self._widget.Vacuum.setCheckState(True)
                rospy.loginfo("Vacuum shutdown cancelled from GUI")

    def _handle_controller_state_change(self, tabIndex):
        #print tabIndex
        if tabIndex==0:
			
            print "Manual Mode set"
        if tabIndex==1:
            self.change_controller_state(4,self.speed_scalar,self.ft_threshold)
        if tabIndex==2:
            self.change_controller_state(2,self.speed_scalar,self.ft_threshold)
            #print self.speed_scalar

        #if self._widget.Automatic_mode.isDown():
            #print "Fully autonomous motion selected"
        #if self._widget.Manual_mode.isDown():
            #print "Manual Mode selected use Teach Pendant to Control"


    def callback(self,data):
        #self._widget.State_info.append(data.mode)
        if(self.stackedWidget.currentIndex()==2):
            if(self.count>10):
            #stringdata=str(data.mode)
            #freeBytes.acquire()
            #####consoleData.append(str(data.mode))
                self._errordiagnosticscreen.State_info.addItem(str(data.joint_position))
                self.count=0

                #print data.joint_position

            self.count+=1
            #self._widget.State_info.scrollToBottom()
            #usedBytes.release()
            #self._data_array.append(stringdata)
            #print self._widget.State_info.count()
            if(self._widget.State_info.count()>200):

                item=self._widget.State_info.takeItem(0)
                #print "Hello Im maxed out"
                del item
        if(self.count>10):
            self.count=0
            if(data.error_msg=="No data received from robot"):
                self.stackedWidget.setCurrentIndex(0)
                self.led_change(self.robotconnectionled,False)
            else:
                self.led_change(self.robotconnectionled,True)
            if(data.ft_wrench_valid=="False"):
                self.stackedWidget.setCurrentIndex(0)

                self.led_change(self.forcetorqueled,False)
            else:

                self.led_change(self.forcetorqueled,True)
        self.count+=1

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
