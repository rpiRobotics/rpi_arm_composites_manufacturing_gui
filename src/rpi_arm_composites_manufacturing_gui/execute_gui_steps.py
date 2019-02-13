import os
import rospy	
import rospkg
import subprocess
import time
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import actionlib
from rpi_arm_composites_manufacturing_process.msg import ProcessStepAction, ProcessStepGoal, ProcessState
import threading

class ErrorConfirm(QWidget):
    def __init__(self):
        super(ErrorConfirm,self).__init__()



class GUI_Step_Executor():
    def __init__(self):
        self.in_process=None
        self.recover_from_pause=False
        self.rewound=False

        self.execute_states=[['plan_to_reset_position','move_to_reset_position'],['plan_pickup_prepare','move_pickup_prepare'],['plan_pickup_lower','move_pickup_lower','plan_pickup_grab_first_step','move_pickup_grab_first_step','plan_pickup_grab_second_step','move_pickup_grab_second_step','plan_pickup_raise','move_pickup_raise'],
                            ['plan_transport_payload','move_transport_payload'],['plan_place_set_second_step']]
        self.reset_code=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'Reset_Start_pos_wason2.py')
        self.YC_place_code=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'Vision_MoveIt_new_Cam_WL_Jcam2_DJ_01172019_Panel1.py')
        self.YC_place_code2=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'Vision_MoveIt_new_Cam_WL_Jcam2_DJ_01172019_Panel2.py')
        self.YC_transport_code=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'test_moveit_commander_custom_trajectory_YC_TransportPath_Panels.py')
        self.client=actionlib.SimpleActionClient('process_step', ProcessStepAction)
        self.client.wait_for_server()
        self.client_handle=None
        self.start_step=0
        self.commands_sent=False
        self.current_state=0
        self.current_command=0
        self.target_index=-1
        self.target=None
        self.gui_state_pub = rospy.Publisher("GUI_state", ProcessState, queue_size=100, latch=True)
        
    def _feedback_receive(self,state,result):
        rospy.loginfo("Feedback_receive")
        messagewindow=ErrorConfirm()
        QMessageBox.information(messagewindow, 'Error', 'Operation failed',str(result))
        
    def _next_command(self,state,result):
    	rospy.loginfo("Next_command")
        if(self.recover_from_pause):
            return
        self.current_command+=1
        if(not(self.current_command==len(self.execute_states[self.current_state]))):
            rospy.loginfo("Next_command")
            
            if(self.current_command==self.target_index):
                g=ProcessStepGoal(self.execute_states[self.current_state][self.current_command], self.target)
            else:
                g=ProcessStepGoal(self.execute_states[self.current_state][self.current_command], "")
            self.client_handle=self.client.send_goal(g,feedback_cb=self._feedback_receive,done_cb=self._next_command)
            #self.client.wait_for_result()
        else:
            s=ProcessState()
            s.state=str(self.execute_states[self.current_state])
            s.payload=""
            s.target=""
            self.gui_state_pub.publish(s)

    def _execute_steps(self,steps_index, target="",target_index=-1):
        #TODO Create separate thread for each execution step that waits until in_process is true
        def send_action(goal):

            self.client_handle=self.client.send_goal(goal,feedback_cb=self._feedback_receive,done_cb=self._next_command)
            #self.client.wait_for_result()
            
        self.start_step=0
        self.current_state=steps_index
        self.target_index=target_index
        self.target=target
        #for step_num in range(resume_index,len(self.execute_states[steps_index])):
        if(self.recover_from_pause):
            if('plan' in self.execute_states[steps_index][self.current_command]):
                self.start_step=self.current_command
            else:
                self.start_step=self.current_command-1

            self.recover_from_pause=False
            
        if(self.start_step==target_index):
            g=ProcessStepGoal(self.execute_states[steps_index][self.start_step], target)
        else:
            g=ProcessStepGoal(self.execute_states[steps_index][self.start_step], "")
            	
        	
		
            #self._send_event.wait()
        
        
		
        send_action(g)
            #self.in_process=True

        



        self.commands_sent=True

        #if( not self.recover_from_pause):
        #self.last_step=0

        
        #TODO: using client.get_state can implemen action state recall to eliminate plan from moveit?
    #TODO: make it so that next plan throws it back into automatic mode every time and then teleop switches to teleop mode and plans the next move
    def _nextPlan(self,panel_type,planListIndex):

        rospy.loginfo("next plan")

        #if self.recover_from_pause and planListIndex !=0:
        #    planListIndex-=1

        #self._open_rviz_prompt()
        #self._raise_rviz_window()



        time.sleep(1)

        if(planListIndex==0):
            ret_code=-1
            try:
                reset_popen=subprocess.Popen(['python', self.reset_code])
                reset_popen.wait()
                ret_code=reset_popen.returncode
            finally:
                pass

            if ret_code != 0:
                messagewindow=ErrorConfirm()
                QMessageBox.information(messagewindow, 'Error', 'Reset Operation failed')

        elif(planListIndex==1):
            
            self._execute_steps(1,panel_type,0)
            
            

        elif(planListIndex==2):
            
            self._execute_steps(2)
            

        elif(planListIndex==3):
            retcode=-1
            try:
                if(panel_type=="leeward_mid_panel"):
                    p=subprocess.Popen(['python', self.YC_transport_code, 'leeward_mid_panel'])
                elif(panel_type=="leeward_tip_panel"):
                    p=subprocess.Popen(['python', self.YC_transport_code, 'leeward_tip_panel'])
                p.wait()
                ret_code=p.returncode
            finally:
                pass

            if ret_code != 0:
                messagewindow=ErrorConfirm()
                QMessageBox.information(messagewindow, 'Error', 'Operation failed')


        elif(planListIndex==4):
            retcode=-1
            try:
                if(panel_type=="leeward_mid_panel"):
                    p=subprocess.Popen(['python', self.YC_place_code])
                elif(panel_type=="leeward_tip_panel"):
                    p=subprocess.Popen(['python', self.YC_place_code2])
                p.wait()
                ret_code=p.returncode
            finally:
                pass


            if ret_code != 0:
                messagewindow=ErrorConfirm()
                QMessageBox.information(messagewindow, 'Error', 'Operation failed')

        if(self.rewound):
            self.rewound=False
            


    def _previousPlan(self):

        self.rewound=True


    def _stopPlan(self):
        client=self.client
        #
        #self.controller_commander.set_controller_mode(self.controller_commander.MODE_HALT, 0,[], [])
        client.cancel_all_goals()
        #client.cancel_all_goals()
        self.recover_from_pause=True
        



