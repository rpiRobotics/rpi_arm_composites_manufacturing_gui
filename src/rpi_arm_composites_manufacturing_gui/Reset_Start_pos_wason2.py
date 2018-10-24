import numpy as np
import rospy

import general_robotics_toolbox as rox
from general_robotics_toolbox import urdf

import abb_irc5_rapid_node_commander as rapid_node_pkg
from safe_kinematic_controller.ros.commander import ControllerCommander

#Old link6 tool position
#P = [[ 1.8288, -0.0447, 1.237]]
#Q = [0.718181636243,-0.0836401543762,0.687115714468,0.0713544453462]


Q=[0.02196692, -0.10959773,  0.99369529, -0.00868731]
P=[1.8475985 , -0.04983688,  0.82486047]

if __name__ == '__main__':
    
    controller_commander=ControllerCommander()
    
    P=np.reshape(P,(3,))    
        
    rospy.init_node('Reset_Start_pos_wason2', anonymous=True)
    
    controller_commander.set_controller_mode(controller_commander.MODE_HALT, 1, [],[])
    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 1, [],[])  
        
    print "============ Printing robot Pose"
    print controller_commander.get_current_pose_msg()  
    #print robot.get_current_state().joint_state.position
    print "============ Generating plan 1"
    
    pose_target=rox.Transform(rox.q2R(Q), np.copy(P))
        
    print 'Target:',pose_target
    
    print "============ Executing plan1"
    controller_commander.plan_and_move(pose_target)        
    print 'Execution Finished.'
  
       
