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
    def __init__(self,execute_states):
        self.in_process=None
        self.recover_from_pause=False
        self.rewound=False
        self.execute_states=execute_states

        self.reset_code=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'Reset_Start_pos_wason2.py')
        self.YC_place_code=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'Vision_MoveIt_new_Cam_WL_Jcam2_DJ_01172019_Panel1.py')
        self.YC_place_code2=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'Vision_MoveIt_new_Cam_WL_Jcam2_DJ_01172019_Panel2.py')
        self.YC_transport_code=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'test_moveit_commander_custom_trajectory_YC_TransportPath_Panels.py')
        self.client=actionlib.SimpleActionClient('process_step', ProcessStepAction)
        self.client.wait_for_server()
        self.client_handle=None
        self.last_step=0
        self.commands_sent=False

    def _feedback_receive(self,state,result):

        messagewindow=ErrorConfirm()
        QMessageBox.information(messagewindow, 'Error', 'Operation failed',str(result))

    def _execute_steps(self,steps_index,resume_index=0, target="",target_index=-1):
        #TODO Create separate thread for each execution step that waits until in_process is true
        def send_action(goal):

            self.client_handle=self.client.send_goal(goal,feedback_cb=self._feedback_receive)
            self.client.wait_for_result()

        for step_num in range(resume_index,len(self.execute_states[steps_index])):
            if(step_num==target_index):
                g=ProcessStepGoal(self.execute_states[steps_index][step_num], target)
            else:
                g=ProcessStepGoal(self.execute_states[steps_index][step_num], "")

            #self._send_event.wait()
            if(self.recover_from_pause):
                if('plan' in self.execute_states[steps_index][step_num]):
                    self.last_step=step_num
                else:
                    self.last_step=step_num-1

                self.recover_from_pause=False
                break

            send_action(g)
            #self.in_process=True

        



        self.commands_sent=True

        if( not self.recover_from_pause):
            self.last_step=0

        
        #TODO: using client.get_state can implemen action state recall to eliminate plan from moveit?
    #TODO: make it so that next plan throws it back into automatic mode every time and then teleop switches to teleop mode and plans the next move
    def _nextPlan(self,panel_type,planListIndex):

        rospy.loginfo("next plan")

        if self.recover_from_pause and planListIndex !=0:
            planListIndex-=1

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
            self.send_thread=threading.Thread(target=self._execute_steps,args=(1,self.last_step, panel_type,0))
            rospy.loginfo("thread_started")
            self.send_thread.setDaemon(True)
            self.send_thread.start()
            

        elif(planListIndex==2):
            self.send_thread=threading.Thread(target=self._execute_steps,args=(2,self.last_step))
            self.send_thread.setDaemon(True)
            self.send_thread.start()
            

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
            self._runscreen.previousPlan.setDisabled(False)


    def _previousPlan(self,planListIndex):

        self.rewound=True


    def _stopPlan(self):
        #client=self.client
        #
        #self.controller_commander.set_controller_mode(self.controller_commander.MODE_HALT, 0,[], [])
        self.client_handle.cancel()
        #client.cancel_all_goals()
        self.recover_from_pause=True



