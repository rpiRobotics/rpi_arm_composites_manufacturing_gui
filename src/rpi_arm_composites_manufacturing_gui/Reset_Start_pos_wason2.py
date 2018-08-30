import numpy as np
import rospy

import general_robotics_toolbox as rox
from general_robotics_toolbox import urdf

import abb_irc5_rapid_node_commander as rapid_node_pkg
import arm_composites_manufacturing_controller_commander as controller_commander_pkg

#Old link6 tool position
#P = [[ 1.8288, -0.0447, 1.237]]
#Q = [0.718181636243,-0.0836401543762,0.687115714468,0.0713544453462]


Q=[0.02196692, -0.10959773,  0.99369529, -0.00868731]
P=[1.8475985 , -0.04983688,  0.82486047]

if __name__ == '__main__':
    print rox.q2R(Q)
    rapid_node = rapid_node_pkg.AbbIrc5RAPIDNodeCommander()
    controller_commander=controller_commander_pkg.arm_composites_manufacturing_controller_commander()
    
    P=np.reshape(P,(3,))    
        
    rospy.init_node('Reset_Start_pos_wason2', anonymous=True)
    
    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 1, [])  
        
    rospy.loginfo( "============ Printing robot Pose")
    rospy.loginfo( controller_commander.get_current_pose_msg())
    #print robot.get_current_state().joint_state.position
    rospy.loginfo( "============ Generating plan 1")
    
    pose_target=rox.Transform(rox.q2R(Q), np.copy(P))

        
    rospy.loginfo( 'Target:',pose_target)
    
    rospy.loginfo( "============ Executing plan1")
    controller_commander.plan_and_move(pose_target)        
    rospy.loginfo( 'Execution Finished.')
  
       
