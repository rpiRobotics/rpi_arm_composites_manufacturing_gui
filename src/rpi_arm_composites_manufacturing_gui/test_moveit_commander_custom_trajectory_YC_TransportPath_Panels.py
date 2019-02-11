import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import safe_kinematic_controller.ros.commander as controller_commander_pkg
from rpi_arm_composites_manufacturing_process.msg import ProcessState
import numpy as np
import sys
import os
import rospkg

def main():
    rospy.init_node("test_moveit_commander_custom_trajectory", anonymous=True)
    final_pos=None
    controller_commander=controller_commander_pkg.ControllerCommander()
    process_state_pub = rospy.Publisher("process_state", ProcessState, queue_size=100, latch=True)
    transport1=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'Joint_all_Panel1.txt')
    transport2=os.path.join(rospkg.RosPack().get_path('rpi_arm_composites_manufacturing_gui'), 'src', 'rpi_arm_composites_manufacturing_gui', 'Joint_all_Panel2.txt')

    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.1, [], [])

    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.5, [], [])

    if('leeward_mid_panel' in sys.argv):
        waypoints = np.loadtxt(transport1, comments="#", delimiter=",", unpack=False)
        panel_type='leeward_mid_panel'
        final_pos=np.array([0.5201124869846093235, 1.076912628142645101, -0.6322520329564644825, -0.008162611460859465345, 1.131674733905407404, 0.4480798367216944356])
    elif('leeward_tip_panel' in sys.argv):
        waypoints = np.loadtxt(transport2, comments="#", delimiter=",", unpack=False)
        panel_type='leeward_tip_panel'
        final_pos=np.array([-0.4619627594947815, 1.0956398248672485, -0.7398647665977478, 0.01724303886294365, 1.1420093774795532, -0.49965518712997437])
    plan0=RobotTrajectory()    
    plan0.joint_trajectory.joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    plan0.joint_trajectory.header.frame_id='/world'
    p1=JointTrajectoryPoint()
    p1.positions = controller_commander.get_current_joint_values()
    p1.velocities = np.zeros((6,))
    p1.accelerations = np.zeros((6,))
    p1.time_from_start = rospy.Duration(0)    
    p2=JointTrajectoryPoint()
    p2.positions = np.array(waypoints[0,:])
    p2.velocities = np.zeros((6,))
    p2.accelerations = np.zeros((6,))
    t2 = 3
    p2.time_from_start = rospy.Duration(t2)
    
    plan0.joint_trajectory.points.append(p1)
    plan0.joint_trajectory.points.append(p2)
    
    controller_commander.execute(plan0)
    
    plan=RobotTrajectory()
    
    plan.joint_trajectory.joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    plan.joint_trajectory.header.frame_id='/world'
    p1=JointTrajectoryPoint()
    p1.positions = controller_commander.get_current_joint_values()
    p1.velocities = np.zeros((6,))
    p1.accelerations = np.zeros((6,))
    p1.time_from_start = rospy.Duration(0)
    
    p2=JointTrajectoryPoint()
    p2.positions = np.array(waypoints[0,:])
    p2.velocities = np.zeros((6,))
    p2.accelerations = np.zeros((6,))
    t2 = 0.5
    p2.time_from_start = rospy.Duration(t2)
    

    
    plan.joint_trajectory.points.append(p1)
    plan.joint_trajectory.points.append(p2)
    
    cnt =0.0
    for w in waypoints:
        cnt=cnt+1
        if cnt%10==0:

            p3=JointTrajectoryPoint()
            p3.positions = np.array(w)
            p3.velocities = np.zeros((6,))
            p3.accelerations = np.zeros((6,))
            p3.time_from_start = rospy.Duration(t2 + cnt*0.02)
            print p3.time_from_start.to_sec()
            plan.joint_trajectory.points.append(p3)

    
    p3=JointTrajectoryPoint()
    p3.positions = final_pos
    #p3.positions = np.array([-0.4619627594947815, 1.0956398248672485, -0.7398647665977478, 0.01724303886294365, 1.1420093774795532, -0.49965518712997437]) # panel 2
    p3.velocities = np.zeros((6,))
    p3.accelerations = np.zeros((6,))
    p3.time_from_start = rospy.Duration(t2 + cnt*0.02+3)  
    plan.joint_trajectory.points.append(p3)     
    
    controller_commander.execute(plan)
    #Process state publish to notify GUI to free user inputs and alert that process is finished"
    s=ProcessState()
    s.state="move_transport_payload"
    s.payload=panel_type
    s.target=""
    process_state_pub.publish(s)
    
    
    

if __name__ == '__main__':
    main()
