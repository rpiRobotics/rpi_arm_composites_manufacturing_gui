from execute_gui_steps import GUI_Step_Executor
from rpi_arm_composites_manufacturing_gui.msg import GUIStepAction, GUIStepGoal
class GUIExecutionServer(object):
    def __init__(self):
        self.executor=GUI_Step_Executor()
        self.server=actionlib.SimpleActionServer("gui_step", GUIStepAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()


    def execute_cb(self, goal):

        command = goal.command
        if command == "reset":
            self.executor._nextPlan(goal.target,0)
        elif command == "panel_pickup":
            self.executor._nextPlan(goal.target,1)
        elif command == "pickup_grab":
            self.executor._nextPlan(goal.target,2)
        elif command == "transport_panel":
            self.executor._nextPlan(goal.target,3)
        elif command == "place_panel":
            self.executor._nextPlan(goal.target,4)
        elif command == "previous_plan":
            self.executor._previousPlan()
        elif command == "stop_plan":
            self.executor._stopPlan()
        else:
            assert False, "Invalid command"

        res = ProcessStepResult()
        res.state=self.controller.state
        res.target=self.controller.current_target if self.controller.current_target is not None else ""
        res.payload=self.controller.current_payload if self.controller.current_payload is not None else ""

        self.server.set_succeeded(res)

def process_controller_server_main():
    rospy.init_node("process_controller_server")

    disable_ft=rospy.get_param('~disable_ft', False)

    s=ProcessControllerServer(disable_ft=disable_ft)

    rospy.spin()
