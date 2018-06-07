import os
import rospy
import rospkg
import collections

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QMutex, QMutexLocker,QSemaphore, QThread
from PyQt5.QtWidgets import QMessageBox
from rpi_arm_composites_manufacturing_abb_egm_controller.msg import ControllerState as controllerstate
from rpi_arm_composites_manufacturing_abb_egm_controller.srv import \
    SetControllerMode, SetControllerModeRequest, SetControllerModeResponse
from rqt_console import console_widget
from rqt_console import message_proxy_model
		

freeBytes=QSemaphore(100)
usedBytes=QSemaphore()
consoleData=collections.deque(maxlen=200)

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
	   #print dataval
	   freeBytes.release()


class ExperimentGUI(Plugin):

    def __init__(self, context):
        super(ExperimentGUI, self).__init__(context)
        # Give QObjects reasonable names
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
        self._widget = QWidget()
	#self._data_array=collections.deque(maxlen=500)
	#self._proxy_model=message_proxy_model.MessageProxyModel()
	#self._rospack=rospkg.RosPack()
	#console=console_widget.ConsoleWidget(self._proxy_model,self._rospack)
	self.speed_scalar=1
	self.ft_threshold=[300,300,300,300,300,300]
	self._widget.Speed_scalar.setInputMask("9")
	self._widget.Ft_threshold.setInputMask("9")
	self._widget.Speed_scalar.setText("1")
	self._widget.Ft_threshold.setText("300")
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('experiment_gui'), 'resource', 'experiment_gui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
       
        context.add_widget(self._widget)
	consoleThread=ConsoleThread(self._widget.State_info)
	#consoleThread.finished.connect(app.exit)
	consoleThread.start()
	# Add widget to the user interface
	#context.add_widget(console)
        #context.add_widget(rqt_console)
        rospy.Subscriber("controller_state", controllerstate, self.callback)
	self._widget.Vacuum.toggled.connect(self._handle_vacuum_change)
        self._widget.Gamepad_control.pressed.connect(self._handle_controller_state_change)
        self._widget.Automatic_mode.pressed.connect(self._handle_controller_state_change)
        self._widget.Manual_mode.pressed.connect(self._handle_controller_state_change)
	self._widget.Speed_scalar.modified.connect(self._change_values)
	self._widget.Ft_threshold.modified.connect(self._change_values)

    def _change_values(self):
	if enter_error:
	    enter_error=False
	    self._widget.Error_msg.setText("")
	if self._widget.Speed_scalar.modified():
	    if(self._widget.Speed_scalar.acceptableInput()):
		self.speed_scalar=int(self._widget.Speed_scalar.text())
		#add in setter function
	    
	     else:
		self._widget.Error_msg.setText("Speed Scalar Value not Acceptable")
		enter_error=True
	     self._widget.Speed_scalar.setModified(False)

	if self._widget.Ft_threshold.modified():
	    if(self._widget.Ft_threshold.acceptableInput()):
		for i in self.ft_threshold:
		    self.ft_threshold[i]=int(self._widget.Ft_threshold.text())
	    	#add in setter function
	    else:
		self._widget.Error_msg.setText("Force Threshold Value not Acceptable")
		enter_error=True
	    self._widget.Ft_threshold.setModified(False)



    def _handle_vacuum_change(self):
        if self._widget.Vacuum.isChecked():
            print("Vacuum Activated")
	else:
            messagewindow=VacuumConfirm()
	    reply = QMessageBox.question(messagewindow, 'Continue?', 
                 'Shutdown Vacuum?', QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply==QMessageBox.Yes:
                print("Vacuum shutdown")
            else:
		self._widget.Vacuum.setCheckState(True)
                print("Vacuum shutdown cancelled")

    def _handle_controller_state_change(self):
        if self._widget.Gamepad_control.isDown():
	    self.set_controller_mode(2)

	if self._widget.Automatic_mode.isDown():
	    print "Fully autonomous motion selected"
	if self._widget.Manual_mode.isDown():
	    print "Manual Mode selected use Teach Pendant to Control"

    def set_controller_mode(self, mode):
	req=SetControllerModeRequest()
    	req.mode.mode=mode
    	req.speed_scalar=self.speed_scalar
    	req.force_torque_stop_threshold=self.ft_threshold

    def callback(self,data):
        #self._widget.State_info.append(data.mode)
	#stringdata=str(data.mode)
	freeBytes.acquire()
	consoleData.append(str(data.mode))
	usedBytes.release()
	#self._data_array.append(stringdata)
	
	
        #if(len(self._data_array)>10):
	#	for x in self._data_array:
	#		self._widget.State_info.append(x)

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
