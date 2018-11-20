import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import safe_kinematic_controller.ros.commander as controller_commander_pkg
import numpy as np

def main():
    rospy.init_node("test_moveit_commander_custom_trajectory", anonymous=True)
    
    controller_commander=controller_commander_pkg.ControllerCommander()
    
    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 1.0, [], [])
    waypoints = np.loadtxt("/home/rpi-cats/catkin_ws/src/rpi_arm_composites_manufacturing_gui/src/rpi_arm_composites_manufacturing_gui/Path.txt", comments="#", delimiter=",", unpack=False)

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
    t2 = 5
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
    t2 = 1
    p2.time_from_start = rospy.Duration(t2)
    

    
    plan.joint_trajectory.points.append(p1)
    plan.joint_trajectory.points.append(p2)
    
    cnt =0
    for w in waypoints:
        cnt=cnt+1
        p3=JointTrajectoryPoint()
        p3.positions = np.array(w)
        p3.velocities = np.zeros((6,))
        p3.accelerations = np.zeros((6,))
        p3.time_from_start = rospy.Duration(t2 + cnt*0.006)
        plan.joint_trajectory.points.append(p3)

    p3=JointTrajectoryPoint()
    p3.positions = np.array([0.5158949494361877, 1.2091666460037231, -0.9566897749900818, -0.04086354747414589, 1.269188404083252, 0.5332122445106506])
    p3.velocities = np.zeros((6,))
    p3.accelerations = np.zeros((6,))
    p3.time_from_start = rospy.Duration(t2 + cnt*0.006+3)  
    plan.joint_trajectory.points.append(p3)     
    controller_commander.execute(plan)
    
    
    

if __name__ == '__main__':
    main()
