import numpy as np
import copy
import rospy
import actionlib

import general_robotics_toolbox as rox
import general_robotics_toolbox.urdf as urdf
import general_robotics_toolbox.ros_msg as rox_msg

import abb_irc5_rapid_node_commander as rapid_node_pkg
import arm_composites_manufacturing_controller_commander as controller_commander_pkg

from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionGoal

import tf
import time
import sys
from CameraService_multicamera_YC import *
from arm_composites_manufacturing_process import PayloadTransformListener

dP_desired = np.array([0.0353,   -0.2100,    0.1073])
#dR_desired = np.array([[ 0.997509745766743,  -0.016334033297502,  -0.068611270623735],[ 0.016346203218063,   0.999866318123232,  -0.000384087289671],[0.068608372234889,  -0.000738402957993,   0.997643396219486]])
#dR_desired = np.array([[ 0.997614303173567,  -0.007628607141157,  -0.068611270623735],[0.007620211886001,   0.999970892000246,  -0.000384087289671],[  0.068612203537927,  -0.000139661446078,   0.997643396219486]]) 
dR_desired = np.array([[ 0.997599469296042,  -0.009369760063681,  -0.068611270623735],[0.009365477842638,   0.999956069185909,  -0.000384087289671],[ 0.068611855280508,  -0.000259412058443,   0.997643396219486 ]])      
     
   
Ttc1 = np.array([[ 0.419450883727994,   0.022584111376874, 0.907497060079603,  -0.159116077005362],
   [0.010831135302534,  -0.999743831260542,   0.019873559433375, 1.002313912576243],
   [0.907713414381358,  0.001493241377224,  -0.419588045089856, 0.199072133281554],
                  [ 0 ,                  0,                   0, 1.0]])

Ttc2 = np.array([[0.056340393207203,  -0.010254605007221, 0.998358955070471,  -0.220108541877344],
  [-0.040607055127084 , -0.999143397560287,  -0.007971084342250, 1.105507675472595],
   [0.997585498675258,  -0.040091323099035,  -0.056708541199762, 0.018798286199474],
                  [ 0,      0,  0 ,  1.0]])


ft_threshold=[250,250,250,250,250,250]
ft_threshold_place=[500,500,500,500,500,500]

class ObjectRecognitionCommander(object):
    def __init__(self):
        self.client=actionlib.SimpleActionClient("recognize_objects_overhead", ObjectRecognitionAction)
        self.listener=PayloadTransformListener()
    
    def get_object_pose(self, key):
        self.client.wait_for_server()
        
        goal=ObjectRecognitionGoal(False, [-1e10,1e10,-1e10,1e10])
        self.client.send_goal(goal)
        self.client.wait_for_result()
        ret=self.client.get_result()
        
        for r in ret.recognized_objects.objects:
            if r.type.key == key:
                return rox_msg.msg2pose(r.pose.pose.pose)
            
        raise Exception("Requested object not found")
    
    def get_object_gripper_target_pose(self, key):
        
        object_pose=self.get_object_pose(key)
                
        (trans1,rot1) = self.listener.lookupTransform(key, key + "_gripper_target", rospy.Time(0))
        tag_rel_pose=rox.Transform(rox.q2R([rot1[3], rot1[0], rot1[1], rot1[2]]), trans1)
        return object_pose * tag_rel_pose
        

def main():
            
    t1=time.time()
    
    do_place="place-panel" in sys.argv
    
    if not "disable-ft" in sys.argv:
        ft_threshold1=ft_threshold
    else:
        ft_threshold1=[]
    
    rospy.init_node('Vision_MoveIt_new_Cam_wason2', anonymous=True)
    
    print "============ Starting setup"   
    
    listener=PayloadTransformListener()
    
    rapid_node = rapid_node_pkg.AbbIrc5RAPIDNodeCommander()
    controller_commander=controller_commander_pkg.arm_composites_manufacturing_controller_commander()
    
    object_commander=ObjectRecognitionCommander()
    
    #object_target=object_commander.get_object_gripper_target_pose("leeward_mid_panel")
    
    #controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 1.2, ft_threshold1)  
    '''
    print "============ Printing robot Pose"
    print controller_commander.get_current_pose_msg()  
    #print robot.get_current_state().joint_state.position
    print "============ Generating plan 1"
    
    pose_target=copy.deepcopy(object_target)
    pose_target.p[2] += 0.8
  
    print 'Target:',pose_target
    
    print "============ Executing plan1"
    controller_commander.plan_and_move(pose_target)        
    print 'Execution Finished.'
    
    ########## Vertical Path 1 ############

    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.8, [])
    
    print "============ Printing robot Pose"
    print controller_commander.get_current_pose_msg()  
    print "============ Generating plan 2"

    pose_target2=copy.deepcopy(object_target)
    pose_target2.p[2] += 0.15
    
    print 'Target:',pose_target2
    
    print "============ Executing plan2"
    controller_commander.compute_cartesian_path_and_move(pose_target2, avoid_collisions=False)
    print 'Execution Finished.'
        
    ########## Vertical Path 2 ############

    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.4, ft_threshold1)
    
    print "============ Printing robot Pose"
    print controller_commander.get_current_pose_msg()  
    print "============ Generating plan 3"

    pose_target2=copy.deepcopy(object_target)
    pose_target2.p[2] -= 0.15
    
    print 'Target:',pose_target2
    
    print "============ Executing plan3"
    controller_commander.compute_cartesian_path_and_move(pose_target2, avoid_collisions=False)
    print 'Execution Finished.'
    
    ########## Lift Path ############

    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.7, [])

    print "============ Lift panel!"
            
    rapid_node.set_digital_io("Vacuum_enable", 1)
    time.sleep(0.5)
    pose_target3=copy.deepcopy(object_target)
    pose_target3.p[2] += 0.8
            
    
    print 'Target:',pose_target3
    
    print "============ Executing plan4"
    controller_commander.compute_cartesian_path_and_move(pose_target3, avoid_collisions=False)
    '''
        
    
    if (do_place):

        print "=========== Do place!"
        print ""
        
        controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.8, [])
        
        print "============ Generating plan 5"
        
        panel_target_pose = listener.lookupTransform("world", "panel_nest_leeward_mid_panel_target", rospy.Time(0))
        #panel_target_pose=rox.Pose(rox.q2R([rot1[3], rot1[0], rot1[1], rot1[2]]), trans1)
        panel_gripper_target_pose = listener.lookupTransform("leeward_mid_panel", "leeward_mid_panel_gripper_target", rospy.Time(0))
        #panel_gripper_target_pose=rox.Pose(rox.q2R([rot2[3], rot2[0], rot2[1], rot2[2]]), trans2)
        pose_target=panel_target_pose * panel_gripper_target_pose
        print pose_target.R
        print pose_target.p               
        
         
        pose_target2=copy.deepcopy(pose_target)
        pose_target2.p[2] += 0.5
        
        #print "============ Executing plan 5"        
        #controller_commander.plan_and_move(pose_target2)
        print "============ Executing plan 6"
        controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.4, [])
        


        pose_target2=copy.deepcopy(pose_target)
        pose_target2.p[2] += 0.10
        
        Cur_Pose = controller_commander.get_current_pose_msg()
        print "============ Printing robot Pose"
        print Cur_Pose  
        print "============ Generating plan 7"
        Tca1,Tca2 = CameraService()
       
        #(trans3,rot3) = listener.lookupTransform("world", "link_6", rospy.Time(0))
        Tot_pose = listener.lookupTransform("world", "link_6", rospy.Time(0))
        #Tot_pose=rox.Pose(rox.q2R([rot3[3], rot3[0], rot3[1], rot3[2]]), trans3)

        Tca1_pose = rox.Transform(Tca1[0:3,0:3], Tca1[0:3,3])
        Tca2_pose = rox.Transform(Tca2[0:3,0:3], Tca2[0:3,3])
        Ttc_pose = rox.Transform(Ttc2[0:3,0:3], Ttc2[0:3,3])

        Toa1 = Tot_pose * Ttc_pose * Tca1_pose
        Toa2 = Tot_pose * Ttc_pose * Tca2_pose

        dP = Toa1.p-Toa2.p 
        dR = np.matmul(Toa1.R,Toa2.R.T)
        print 'dP',dP
        print 'dR',dR

        pose_target2.R = np.matmul(pose_target2.R,np.matmul(dR_desired,dR.T).T)
        pose_target2.p = pose_target2.p - (dP-dP_desired)
        pose_target2.p[2] = 0.006
        #pose_target2.p[0] = 2.2
    
        pose_target3=copy.deepcopy(pose_target)
        pose_target3.p[2] -= 0.15
        
        print 'Target:',pose_target2.R,pose_target2.p
        print 'Compare:',pose_target3.R,pose_target3.p
        
        controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.4, ft_threshold_place)

        print "============ Executing plan3"
        try:
            controller_commander.compute_cartesian_path_and_move(pose_target2, avoid_collisions=False)
        except:
            pass
        print 'Execution Finished.'      
         
        Tca1,Tca2 = CameraService()
        Tot_pose = listener.lookupTransform("world", "link_6", rospy.Time(0))
        #Tot_pose=rox.Pose(rox.q2R([rot3[3], rot3[0], rot3[1], rot3[2]]), trans3)
        Tca1_pose = rox.Transform(Tca1[0:3,0:3], Tca1[0:3,3])
        Tca2_pose = rox.Transform(Tca2[0:3,0:3], Tca2[0:3,3])
        Ttc_pose = rox.Transform(Ttc2[0:3,0:3], Ttc2[0:3,3])

        Toa1 = Tot_pose * Ttc_pose * Tca1_pose
        Toa2 = Tot_pose * Ttc_pose * Tca2_pose

        dP = Toa1.p-Toa2.p 
        dR = np.matmul(Toa1.R,Toa2.R.T)
        print 'dP',dP.shape
        print 'dR',dR.shape
        tmp = np.hstack([dR,np.reshape(dP,[3,1])])

        filename = "dR_dP.txt"
        f_handle = file(filename, 'a')
        np.savetxt(f_handle, tmp)
        f_handle.close()
        
        print "============ Lift gripper!"
        controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.7, [])
        raw_input("confirm release vacuum")
        #rapid_node.set_digital_io("Vacuum_enable", 0)
        print "VACUUM OFF!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        time.sleep(0.5)
        pose_target3 = copy.deepcopy(pose_target)
        pose_target3.p[2] += 0.5
                
        
        print 'Target:',pose_target3
        
        print "============ Executing plan4"
        controller_commander.compute_cartesian_path_and_move(pose_target3, avoid_collisions=False)
        
        
        
    t2 = time.time()
    print 'Execution Finished.'
    print "Execution time: " + str(t2-t1) + " seconds"

if __name__ == '__main__':
    main()
