import os
import rospy
import rospkg
import collections
import time
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QMutex, QMutexLocker,QSemaphore, QThread
from PyQt5.QtWidgets import QMessageBox

from arm_vision_moveit.arm_controller_commander import ARMControllerCommander
from arm_vision_moveit.Vision_MoveIt_Classpaired import VisionMoveIt
from rpi_arm_composites_manufacturing_abb_egm_controller.msg import ControllerState as controllerstate

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
			self.State_info.scrollToBottom()
			#print dataval
			freeBytes.release()


class ExperimentGUI(Plugin):

	def __init__(self, context):
	#rospy.init_node('collision_checker','move_group_python_interface_tutorial', anonymous=True)
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
		self.mode=0
		self.count=0
		change_controller_state=self.change_controller_state
		self.armcontroller=ARMControllerCommander()
		self.Vision_MoveIt=VisionMoveIt(self.armcontroller,change_controller_state)
		self.Vision_MoveIt.moveit_init()
		self.Vision_MoveIt.load_offsets_from_yaml()
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

		#####consoleThread=ConsoleThread(self._widget.State_info)



		#consoleThread.finished.connect(app.exit)

		#####consoleThread.start()

		# Add widget to the user interface
		#context.add_widget(console)
		    #context.add_widget(rqt_console)
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
		self._widget.Home_button.pressed.connect(self._reset_position)
		self._widget.Move_to_panel.pressed.connect(self._move_to_panel)
		self._widget.Panel_pickup.pressed.connect(self._pickup_panel)

	def _reset_position(self):
		self.Vision_MoveIt.reset_pos()
		self.above_panel=False
	
	def _move_to_panel(self):
		self.Vision_MoveIt.camera_read()
		self.Vision_MoveIt.set_positions()
		#self.armcontroller.set_controller(4,self.speed_scalar,self.ft_threshold)
		
		plan1=self.Vision_MoveIt.generate_plan(1)
		messagewindow=VacuumConfirm()
		reply = QMessageBox.question(messagewindow, 'Path Verification', 
		             'Is MoveIt Motion Path Acceptable?', QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
		if reply==QMessageBox.Yes:
			print("Path Accepted, Proceeding to Panel")
			self.Vision_MoveIt.execute_plans(plan1)
			self.above_panel=True
		else:
			print("Path Rejected, Press Move Above Panel again to retry")
        
        

	def _pickup_panel(self):
		if(self.above_panel):
			#self.armcontroller.set_controller(4,self.speed_scalar,self.ft_threshold)
			'''if(self.speed_scalar>0.4):
				self._widget.Error_msg.setText("Speed Scalar Value is too High for Pickup Operation")
				return
			'''
			plan2=self.Vision_MoveIt.generate_plan(2)
			self.Vision_MoveIt.execute_plans(plan2)
			#self.armcontroller.set_controller(4,self.speed_scalar,[])
			self.armcontroller.set_vacuum(1)
			self._widget.Vacuum.setCheckState(True)
			rospy.sleep(1)
			time.sleep(1) 
			plan3=self.Vision_MoveIt.generate_plan(3)
			self.Vision_MoveIt.execute_plans(plan3)
		else:
			self._widget.Error_msg.setText("Not above Panel")
			print("You must move above Panel before asking to pickup")

	def _change_values(self):
	
	
		if self._widget.Speed_scalar.isModified():
			if(self._widget.Speed_scalar.hasAcceptableInput()):
				self.speed_scalar=float(self._widget.Speed_scalar.text())
				#add in setter function
				self.change_controller_state(self.mode,self.speed_scalar,self.ft_threshold)
				self._widget.Error_msg.setText("")
				    
			else:
				self._widget.Error_msg.setText("Speed Scalar Value not Acceptable")
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
				self.change_controller_state(self.mode,self.speed_scalar,self.ft_threshold)
				self._widget.Error_msg.setText("")
				
			else:
				self._widget.Error_msg.setText("Force Threshold Value not Acceptable")
		
				self._widget.Ft_threshold.setModified(False)
		    
			

	def _handle_vacuum_change(self):
		if self._widget.Vacuum.isChecked():
			self.armcontroller.set_vacuum(1)
	    
		else:
			messagewindow=VacuumConfirm()
			reply = QMessageBox.question(messagewindow, 'Release Vacuum?', 
                 'Shutdown Vacuum?', QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
			if reply==QMessageBox.Yes:
				print("Vacuum shutdown")
				self.armcontroller.set_vacuum(0)
			else:
				self._widget.Vacuum.setCheckState(True)
				print("Vacuum shutdown cancelled")

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
	
		if(self.count>10):
		#stringdata=str(data.mode)
		#freeBytes.acquire()
		#####consoleData.append(str(data.mode))
			self._widget.State_info.addItem(str(data.joint_position))
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
	
		    #if(len(self._data_array)>10):
		#	for x in self._data_array:
		#		self._widget.State_info.append(x)
	def change_controller_state(self,mode,speed_scalar,ft_threshold):
		self.mode=mode
		
		self.armcontroller.set_controller(mode,speed_scalar,ft_threshold)
		self._widget.Speed_scalar_display.setText(str(speed_scalar))
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
