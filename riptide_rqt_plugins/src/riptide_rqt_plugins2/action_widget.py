from asyncio import Future
import rclpy
import os
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from ament_index_python.packages import get_package_share_directory

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QPushButton, QTextEdit
from python_qt_binding.QtCore import Slot

package_share_dir = get_package_share_directory('riptide_rqt_plugins2')

class ActionWidget(QWidget):
    # Negative numbers for states, since actionserver uses non-negative numbers for status
    STATE_UNINITIALIZED = -1
    STATE_LOADING = -2
    STATE_LOADING_BOOT = -3
    STATE_NOT_FOUND = -4
    STATE_IDLE = -5
    STATE_REJECTED = -6
    STATE_CANCEL_REJECTED = -7
    STATE_RETRIEVE_FROM_GOAL = -8
    STATE_ERROR = -9

    STYLE_NORMAL = "QLabel{ color: rgb(85, 87, 83) }"
    STYLE_RED = "QLabel{ color: rgb(239, 41, 41) }"
    STYLE_GREEN = "QLabel{ color: rgb(78, 154, 6) }"

    # Initialized instance variables
    state = STATE_UNINITIALIZED

    last_result = None
    count_string = ""
    results_window = None
    _future_state: int = None
    _client_goal_handle: 'ClientGoalHandle | None' = None
    _client_goal_future: 'Future | None' = None
    _client_cancel_future: 'Future | None' = None

    def __init__(self, node: 'rclpy.Node', action_name, namespace, action_topic, action_spec, actions_layout, has_results):
        super(ActionWidget, self).__init__()
        self._node = node

        # Initialize instance variables
        self.name = action_name
        self.action_spec = action_spec
        self.namespace = namespace
        self.topic = action_topic
        self.has_results = has_results

        # Load UI into class
        self.ui_file = os.path.join(package_share_dir, 'resource', 'ActionControl.ui')
        self.results_ui_file = os.path.join(package_share_dir, 'resource', 'TextWindow.ui')
        loadUi(self.ui_file, self)

        self.setObjectName('ActionWidget-' + action_topic)
        
        self._action_name = self.findChild(QLabel, "actionTitle")
        self._action_name.setText(action_name)

        self._action_status = self.findChild(QLabel, "statusLabel")
        self._start_btn = self.findChild(QPushButton, "startActionButton")
        self._cancel_btn = self.findChild(QPushButton, "cancelActionButton")
        self._results_btn = self.findChild(QPushButton, "resultsButton")
        self._results_btn.setVisible(has_results)

        self._start_btn.clicked.connect(self._start_callback)
        self._cancel_btn.clicked.connect(self._cancel_callback)
        self._results_btn.clicked.connect(self._results_callback)

        # Configure UI
        self._set_state(ActionWidget.STATE_LOADING)
        self._init_topics()

        actions_layout.addWidget(self)

    ########################################
    # Private Functions
    ########################################

    def _init_topics(self):
        self._action_client = ActionClient(self._node, self.action_spec, self.namespace + "/" + self.topic)
        self._set_state(ActionWidget.STATE_LOADING_BOOT)

        self._results_btn.setEnabled(False)
        self.last_result = None
        self.count_string = ""
        self._future_state = None
        self._client_goal_handle = None
        self._client_goal_future = None
        self._client_cancel_future = None

    def _drop_active_goal(self):
        if self._client_goal_future is not None:
            self._client_goal_future.cancel()
            self._client_goal_future = None

        if self._client_cancel_future is not None:
            self._client_cancel_future.cancel()
            self._client_cancel_future = None
        
        # Note: This will discard the handle to the goal, but won't stop it from running
        self._client_goal_handle = None

    def _generate_goal(self):
        return self.action_spec.Goal()

    def _generate_results_text(self):
        return str(self.last_result)

    ########################################
    # Future Callback Functions
    ########################################

    def _cancel_goal_callback(self, future: Future):
        assert self._client_cancel_future == future
        self._client_cancel_future = None

        if future.cancelled():
            self._node.get_logger().error("Cancel interrupted, this might lead to loss of goal handle")
            self._future_state = ActionWidget.STATE_ERROR
            return

        exc = future.exception()
        if exc is not None:
            self._node.get_logger().error("Exception during goal cancelling: %s", exc)
            self._future_state = ActionWidget.STATE_ERROR
            return

        goal_result: CancelGoal.Response = future.result()
        if goal_result is None:
            self._node.get_logger().error("Unexpected None goal result received from goal cancelling")
            self._future_state = ActionWidget.STATE_ERROR
            return

        if goal_result.return_code == CancelGoal.Response.ERROR_REJECTED:
            self._future_state = ActionWidget.STATE_CANCEL_REJECTED
            return

        if goal_result.return_code != CancelGoal.Response.ERROR_NONE:
            self._node.get_logger().error("Unexpected result when trying to cancel the goal: %d", goal_result.return_code)
            self._future_state = ActionWidget.STATE_ERROR
            return

        if len(goal_result.goals_canceling) > 1:
            self._node.get_logger().error("Multiple goals cancelled when only one expected")
            self._future_state = ActionWidget.STATE_ERROR
            return
        
        if goal_result.goals_canceling[0].goal_id != self._client_goal_handle.goal_id:
            self._node.get_logger().error("Mismatched goal id cancelled with cancel request")
            self._future_state = ActionWidget.STATE_ERROR
            return

        # If we get this far it means the action server is handling cancelling
        # We can just let it poll the action server to report all messages
        # Result callback should handle the cleanup
        self._future_state = ActionWidget.STATE_RETRIEVE_FROM_GOAL

    def _feedback_callback(self, feedback_msg):
        pass

    def _result_callback(self, future: Future):
        assert self._client_goal_future == future
        self._client_goal_future = None
        self._client_goal_handle = None

        if future.cancelled():
            self._node.get_logger().error("Action interrupted! The goal handle will be lost!")
            self._future_state = ActionWidget.STATE_ERROR
            return

        exc = future.exception()
        if exc is not None:
            self._node.get_logger().error("Exception during waiting for goal result: %s", exc)
            self._future_state = ActionWidget.STATE_ERROR
            return

        goal_result = future.result()
        if goal_result is None:
            self._node.get_logger().error("Unexpected None goal result received from action client")
            self._future_state = ActionWidget.STATE_ERROR
            return

        self._future_state = goal_result.status
        self.last_result = goal_result.result

    def _send_goal_callback(self, future: Future):
        assert self._client_goal_future == future
        self._client_goal_future = None

        if future.cancelled():
            self._node.get_logger().error("Action interrupted! The goal handle will be lost!")
            self._future_state = ActionWidget.STATE_ERROR

        exc = future.exception()
        if exc is not None:
            self._node.get_logger().error("Exception during sending goal: %s", exc)
            self._future_state = ActionWidget.STATE_ERROR
            return

        goal_handle: ClientGoalHandle = future.result()
        if goal_handle is None:
            self._node.get_logger().error("Unexpected None goal_handle received from action client")
            self._future_state = ActionWidget.STATE_ERROR
            return

        if not goal_handle.accepted:
            self._future_state = ActionWidget.STATE_REJECTED
            return

        self._client_goal_handle = goal_handle
        self._future_state = ActionWidget.STATE_RETRIEVE_FROM_GOAL

        self._client_goal_future = goal_handle.get_result_async()
        self._client_goal_future.add_done_callback(self._result_callback)

    ########################################
    # QT Callback Functions
    ########################################

    @Slot()
    def _start_callback(self):
        if self._action_client is not None:
            assert self._client_goal_handle == None
            assert self._client_goal_future == None
            assert self._client_cancel_future == None

            self._set_state(ActionWidget.STATE_LOADING)
            self._client_goal_future = self._action_client.send_goal_async(self._generate_goal(), self._feedback_callback)
            self._client_goal_future.add_done_callback(self._send_goal_callback)

    @Slot()
    def _cancel_callback(self):
        if self._client_goal_handle is not None:
            assert self._client_cancel_future == None
            self._set_state(ActionWidget.STATE_LOADING)
            self._client_cancel_future = self._client_goal_handle.cancel_goal_async()
            self._client_cancel_future.add_done_callback(self._cancel_goal_callback)

    @Slot()
    def _results_callback(self):
        if self.last_result is not None:
            if self.results_window is None:
                self.results_window = QWidget()
                loadUi(self.results_ui_file, self.results_window)

                self.results_window.setWindowTitle(self.name + " Results")
                title_label = self.results_window.findChild(QLabel, "titleLabel")
                title_label.setText(self.name + " Last Results:")

            text_area = self.results_window.findChild(QTextEdit, "textBox")
            text_area.setPlainText(self._generate_results_text())
            
            self.results_window.show()

    ########################################
    # Public Functions
    ########################################

    def cleanup_topics(self):
        self._drop_active_goal()

        action_client = self._action_client
        self._action_client = None
        #action_client.destroy()  # TODO: Find out why this crashes

        self._future_state = None
        self._set_state(ActionWidget.STATE_LOADING)

        if self.results_window is not None:
            self.results_window.destroy(destroyWindow = True)
            self.results_window = None

    def tick(self):
        if self._action_client is None:
            return
        
        self._results_btn.setEnabled(self.last_result is not None)

        server_available = self._action_client.server_is_ready()

        # Get server connection status immediately
        if server_available:
            if self._future_state is not None:
                self._set_state(self._future_state)
            
            if self.state == ActionWidget.STATE_LOADING_BOOT or self.state == ActionWidget.STATE_NOT_FOUND:
                self._set_state(ActionWidget.STATE_IDLE)
        else:
            if self._client_goal_handle is not None:
                self._drop_active_goal()
            self._future_state = None

            self.count_string = ""
            self._set_state(ActionWidget.STATE_NOT_FOUND)

    def set_namespace(self, namespace):
        self.cleanup_topics()
        self.namespace = namespace
        self._init_topics()

    ########################################
    # State Decoding/Processing
    ########################################

    def _set_state(self, state):
        if state == ActionWidget.STATE_RETRIEVE_FROM_GOAL:
            if self._client_goal_handle is not None:
                state = self._client_goal_handle.status
            else:
                state = GoalStatus.STATUS_UNKNOWN
        
        # Update the action state
        if self.state != state or self.prev_count_string != self.count_string:
            self.prev_count_string = self.count_string
            count_string = self.count_string
            self.state = state

            if state == ActionWidget.STATE_LOADING or state == ActionWidget.STATE_LOADING_BOOT:
                self._action_status.setText("(Loading)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_NORMAL)
                self._action_status.setToolTip("State is loading. The status of action is unknown")

                self._start_btn.setEnabled(False)
                self._cancel_btn.setEnabled(False)
            elif state == ActionWidget.STATE_NOT_FOUND:
                self._action_status.setText("(Not Found)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_RED)
                self._action_status.setToolTip("The action could not be found")

                self._start_btn.setEnabled(False)
                self._cancel_btn.setEnabled(False)
            elif state == ActionWidget.STATE_REJECTED:
                self._action_status.setText("(Rejected)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_RED)
                self._action_status.setToolTip("The goal was rejected by the action server without being processed")

                self._start_btn.setEnabled(True)
                self._cancel_btn.setEnabled(False)
            elif state == ActionWidget.STATE_CANCEL_REJECTED:
                self._action_status.setText("(Cancel Rejected; Running)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_RED)
                self._action_status.setToolTip("The cancel request was rejected by the action server and the action is still running")

                self._start_btn.setEnabled(False)
                self._cancel_btn.setEnabled(True)
            elif state == ActionWidget.STATE_IDLE:
                self._action_status.setText("(Idle)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_NORMAL)
                self._action_status.setToolTip("There are no goals pending on the actino client")

                self._start_btn.setEnabled(True)
                self._cancel_btn.setEnabled(False)
            elif state == ActionWidget.STATE_ERROR:
                self._action_status.setText("(Error)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_RED)
                self._action_status.setToolTip("The action widget has encountered an unexpected error")

                self._start_btn.setEnabled(False)
                self._cancel_btn.setEnabled(False)
            elif state == GoalStatus.STATUS_ACCEPTED:
                self._action_status.setText("(Starting)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_NORMAL)
                self._action_status.setToolTip("The goal has yet to be processed by the action server")

                self._start_btn.setEnabled(False)
                self._cancel_btn.setEnabled(True)
            elif state == GoalStatus.STATUS_EXECUTING:
                self._action_status.setText("(Running)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_GREEN)
                self._action_status.setToolTip("The goal is currently being processed by the action server")

                self._start_btn.setEnabled(False)
                self._cancel_btn.setEnabled(True)
            elif state == GoalStatus.STATUS_CANCELED:
                self._action_status.setText("(Cancelled)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_NORMAL)
                self._action_status.setToolTip("The goal received a cancel request after it started executing and has since completed its execution")

                self._start_btn.setEnabled(True)
                self._cancel_btn.setEnabled(False)
            elif state == GoalStatus.STATUS_SUCCEEDED:
                self._action_status.setText("(Finished)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_NORMAL)
                self._action_status.setToolTip("The goal was achieved successfully by the action server")

                self._start_btn.setEnabled(True)
                self._cancel_btn.setEnabled(False)
            elif state == GoalStatus.STATUS_ABORTED:
                self._action_status.setText("(Failure)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_RED)
                self._action_status.setToolTip("The goal was aborted during execution by the action server due to some failure")

                self._start_btn.setEnabled(True)
                self._cancel_btn.setEnabled(False)
            elif state == GoalStatus.STATUS_CANCELING:
                self._action_status.setText("(Cancelling)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_NORMAL)
                self._action_status.setToolTip("The goal received a cancel request after it started executing and has not yet completed execution")

                self._start_btn.setEnabled(False)
                self._cancel_btn.setEnabled(False)
            else:
                self._action_status.setText("(INVALID)" + count_string)
                self._action_status.setStyleSheet(ActionWidget.STYLE_RED)
                self._action_status.setToolTip("An invalid state was sent by the action server")

                self._start_btn.setEnabled(False)
                self._cancel_btn.setEnabled(False)