import os
import rospy	
import rospkg
import subprocess
import time
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from python_qt_binding.QtWidgets import QWidget, QDialog
import actionlib
from rpi_arm_composites_manufacturing_process.msg import ProcessStepAction, ProcessStepGoal, ProcessState
import threading


class GUI_Step_Executor(QObject):
    
    error_signal= pyqtSignal()
    
    def __init__(self):
        super(GUI_Step_Executor, self).__init__()
        self.in_process=None
        self.recover_from_pause=False
        self.rewound=False

        self.execute_states=[['reset_position'],['plan_pickup_prepare','move_pickup_prepare'],['plan_pickup_lower','move_pickup_lower','plan_pickup_grab_first_step','move_pickup_grab_first_step','plan_pickup_grab_second_step','move_pickup_grab_second_step','plan_pickup_raise','move_pickup_raise'],
                            ['plan_transport_payload','move_transport_payload'],['place_panel'],['plan_place_set_second_step'],['stop_motion'],['rewind_motion']]
        self.reset_code=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'Reset_Start_pos_wason2.py')
        self.YC_place_code=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'Vision_MoveIt_new_Cam_WL_Jcam2_DJ_01172019_Panel1.py')
        self.YC_place_code2=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'Vision_MoveIt_new_Cam_WL_Jcam2_DJ_01172019_Panel2.py')
        self.YC_transport_code=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'test_moveit_commander_custom_trajectory_YC_TransportPath_Panels.py')
        self.client=actionlib.ActionClient('process_step', ProcessStepAction)
        self.client.wait_for_server()
        self.client_handle=None
        self.start_step=0
        self.commands_sent=False
        self.current_state=0
        self.current_command=0
        self.target_index=-1
        self.target=None
        self.error=None
        self.gui_state_pub = rospy.Publisher("GUI_state", ProcessState, queue_size=100, latch=True)
        #rospy.Subscriber("process_state",ProcessState,self._next_command)
        
        
    #Receives error messages from process controller action calls and signals gui to put up alert
    def _feedback_receive(self,status,result):
        """
        emits signal when error is received from process controller
        
        """
        rospy.loginfo("Feedback_receive_function")
        self.error=result.error_msg
        self.error_signal.emit()
        
        self._publish_state_message()
    
    #callback triggered by process_state message subscription that makes the next process action call 
    #def _next_command(self,data):
    def _next_command(self,status,result):
        """
        the process controller publishes to process_state when commands are completed successfully
        data is the published process state data it is not actually checked, the GUI instead manages its own state machine 
        self.execute_states is a list of all the process commands in the process_controller
        self.recover_from_pause is used as a flag to stop sending the rest of the commands in the sequence if pause is commanded
        self.current_state is the first index for self.execute_states and is only changed by the user pressing a button
        self.current_command is the second index for self.execute_states and is incremented in this function, this function is called until self.current_command>=len(self.execute_states) at which point the GUI step has been completed
        self.target_index is used to signal which step should contain the target being interacted with
        self.target is the target id of the operation
        ProcessStepGoal is the action goal sent to the process_controller action server
        self.client_handle is the variable used to store the current action_client_handle in case a pause is commanded
        self._feedback_receive is the error received callback function for the action client
        self._publish_state_message is a function called to alert the main GUI program that the GUI step has been completed, this is used to reenable the GUI command buttons
        """
        if(status==actionlib.GoalStatus.SUCCEEDED):
            
        	rospy.loginfo("Next_command")
            if(self.recover_from_pause):
                return
            self.current_command+=1
            rospy.loginfo("current state %i"%self.current_state)
            rospy.loginfo("current command %i"%self.current_command)
            if(not(self.current_command>=len(self.execute_states[self.current_state]))):
                
                
                if(self.current_command==self.target_index):
                    g=ProcessStepGoal(self.execute_states[self.current_state][self.current_command], self.target)
                else:
                    g=ProcessStepGoal(self.execute_states[self.current_state][self.current_command], "")
                #self.client_handle=self.client.send_goal(g,feedback_cb=self._feedback_receive)
                #self.client_handle=self.client.send_goal(g,feedback_cb=self._feedback_receive,done_cb=self._next_command)
                self.client_handle=self.client.send_goal(g,done_cb=self._next_command)
                #self.client.wait_for_result()
            else:
                self._publish_state_message()
        else:
            rospy.loginfo("Feedback_receive")
            self.error=result.error_msg
            self.error_signal.emit()
        
            self._publish_state_message()

    def _execute_steps(self,steps_index, target="",target_index=-1):
        #TODO Create separate thread for each execution step that waits until in_process is true
        #TODO try with done cb
        def send_action(goal):
            #self.client_handle=self.client.send_goal(goal,feedback_cb=self._feedback_receive)
            #self.client_handle=self.client.send_goal(goal,feedback_cb=self._feedback_receive,done_cb=self._next_command)
            self.client_handle=self.client.send_goal(g,done_cb=self._next_command)
            #self.client.wait_for_result()
            
        self.start_step=0
        self.current_state=steps_index
        self.target_index=target_index
        self.target=target
        #for step_num in range(resume_index,len(self.execute_states[steps_index])):
        if(self.recover_from_pause):
            if(steps_index!=0):
                if('plan' in self.execute_states[steps_index][self.current_command]):
                    self.start_step=self.current_command
                else:
                    self.start_step=self.current_command-1
            else:
                pass

            self.recover_from_pause=False

        self.current_command=self.start_step
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
    def _nextPlan(self,panel_type,planListIndex,panel_nest=None):

        rospy.loginfo("next plan planListIndex: "+str(planListIndex))

        #if self.recover_from_pause and planListIndex !=0:
        #    planListIndex-=1

        #self._open_rviz_prompt()
        #self._raise_rviz_window()



        time.sleep(1)

        if(planListIndex==0):
            
            self._execute_steps(0)
            '''try:
                g=ProcessStepGoal(self.execute_states[0][0], "")
                self.client_handle=self.client.send_goal(g,feedback_cb=self._feedback_receive)
                #self.client_handle=self.client.send_goal(g,feedback_cb=self._feedback_receive,done_cb=self._next_command)
            finally:
                pass
                #self._publish_state_message()
            '''
            

        elif(planListIndex==1):
            
            self._execute_steps(1,panel_type,0)
            
            

        elif(planListIndex==2):
            
            self._execute_steps(2)
            
        

        elif(planListIndex==3):
            
            self._execute_steps(3,panel_nest,0)

        elif(planListIndex==4):
            
            
            self._execute_steps(4)

            

        if(self.rewound):
            self.rewound=False
            


    def _previousPlan(self):
        try:
            g=ProcessStepGoal(self.execute_states[7][0], "")
            self.client_handle=self.client.send_goal(g,feedback_cb=self._feedback_receive)
            #self.client_handle=self.client.send_goal(g,feedback_cb=self._feedback_receive,done_cb=self._next_command)
        finally:
            pass
        self.rewound=True


    def _stopPlan(self):
        client=self.client
        #
        #self.controller_commander.set_controller_mode(self.controller_commander.MODE_HALT, 0,[], [])
        #client.cancel_all_goals()
        rospy.loginfo("cancelled all goals")
        #g=ProcessStepGoal(self.execute_states[6][0], "")
        #self.client_handle=self.client.send_goal(g,feedback_cb=self._feedback_receive)
        client.cancel_all_goals()
        #TODO make it so it checks previous goal handle has returned before reenabling
        self.recover_from_pause=True

    def _publish_state_message(self):
        s=ProcessState()
        s.state=str(self.execute_states[self.current_state])
        s.payload=""
        s.target=""
        self.gui_state_pub.publish(s)



