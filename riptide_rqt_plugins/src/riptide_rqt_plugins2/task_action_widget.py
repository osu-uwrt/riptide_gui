from riptide_msgs2.action import ExecuteTask

from .action_widget import ActionWidget

class TaskActionWidget(ActionWidget):

    def __init__(self, node, action_name, namespace, action_topic, behaviortree_file, actions_layout):
        super(TaskActionWidget, self).__init__(node, action_name, namespace, action_topic, ExecuteTask, actions_layout, True)

        self._behaviortree_file = behaviortree_file
        self.console_history = ""

        self._results_btn.setText("Last Console")

    ########################################
    # Function Overrides
    ########################################

    def _feedback_callback(self, feedback_msg):
        self.console_history += feedback_msg.feedback.stdout_data
        self.console_history += feedback_msg.feedback.stderr_data

    def _result_callback(self, future):
        super()._result_callback(future)
        if self.last_result.returncode != 0:
            self.count_string = f" Code: {self.last_result.returncode}"
        else:
            self.count_string = ""

    def _generate_goal(self):
        self.console_history = ""
        self.count_string = ""
        action_goal = ExecuteTask.Goal()
        action_goal.behaviortree_file = self._behaviortree_file
        return action_goal

    def _generate_results_text(self):
        return self.console_history