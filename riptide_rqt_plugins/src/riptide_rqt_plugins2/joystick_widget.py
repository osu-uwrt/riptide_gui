
import os
import rclpy
import signal
import subprocess
from ament_index_python.packages import get_package_share_directory
from ros2run.api import get_executable_path

from std_msgs.msg import Bool

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QPushButton, QComboBox, QTextEdit, QMessageBox, QAbstractButton
from python_qt_binding.QtCore import Slot, pyqtSlot

package_share_dir = get_package_share_directory('riptide_rqt_plugins2')

class PS3TeleopWidget(QWidget):
    confirm_close = None
    confirm_close_open = False
    instructions_window = None
    process: 'subprocess.Popen | None' = None

    STYLE_NORMAL = "QLabel{ color: rgb(85, 87, 83) }"
    STYLE_RED = "QLabel{ color: rgb(239, 41, 41) }"
    STYLE_GREEN = "QLabel{ color: rgb(78, 154, 6) }"

    TELEOP_NODE_NAME = "ps3_teleop"
    JOYSTICK_NODE = {'package_name': 'joy_linux', 'executable_name': 'joy_linux_node'}

    selected_joystick_device = ""
    teleop_enabled = False
    waiting_for_stop = False

    def __init__(self, namespace, node: 'rclpy.Node'):
        super(PS3TeleopWidget, self).__init__()
        self._node: 'rclpy.Node' = node

        self.namespace = namespace
        self.available_joysticks = []
        self.prev_input_list = []

        # Load UI into class
        self.ui_file = os.path.join(package_share_dir, 'resource', 'PS3TeleopControl.ui')
        self.instructions_ui_file = os.path.join(package_share_dir, 'resource', 'TextWindow.ui')
        self.instructions_html_file = os.path.join(package_share_dir, 'resource', 'TeleopInstructions.html')
        loadUi(self.ui_file, self)

        self.setObjectName('PS3TeleopWidget')

        # Load all of the needed controls
        self._status_label = self.findChild(QLabel, "joyNodeStatusLabel")
        self._start_button = self.findChild(QPushButton, "startButton")
        self._stop_button = self.findChild(QPushButton, "stopButton")
        self._joystick_selector = self.findChild(QComboBox, "joystickSelector")
        self._instructions_button = self.findChild(QPushButton, "instructionsButton")

        # Connect the slots
        self._start_button.clicked.connect(self._start_callback)
        self._stop_button.clicked.connect(self._stop_callback)
        self._instructions_button.clicked.connect(self._instructions_callback)

        # Set default state
        self._status_label.setText("(Loading)")
        self._status_label.setToolTip("")
        self._start_button.setEnabled(False)
        self._stop_button.setEnabled(False)
        self._joystick_selector.setEnabled(False)
        self._joystick_selector.clear()
        self._joystick_selector.addItem("Loading...")
        self._joystick_selector.setCurrentIndex(0)

        self._init_topics()

    def _joy_node_running(self):
        return self.process is not None and self.process.poll() is None

    def _init_topics(self):
        self._teleop_enabled_sub = self._node.create_subscription(Bool, self.namespace + '/teleop_enabled', self._teleop_enabled_callback, 1)
        self.teleop_enabled = False

    def _cleanup_topics(self):
        self._node.destroy_subscription(self._teleop_enabled_sub)

    ########################################
    # Callback Functions
    ########################################

    def _teleop_enabled_callback(self, msg):
        self.teleop_enabled = msg.data

    @Slot()
    def _start_callback(self):
        assert not self._joy_node_running()

        # Set UI State
        self._start_button.setEnabled(False)
        self.selected_joystick_device = self._joystick_selector.currentText()

        # Launch the node
        joystick_node_executable = get_executable_path(**self.JOYSTICK_NODE)

        if joystick_node_executable is not None:
            self.waiting_for_stop = False
            self.process = subprocess.Popen([joystick_node_executable, '--ros-args',
                                            '-p', 'dev:='+self.selected_joystick_device])
        else:
            self._node.get_logger().error("Cannot start joystick node, cannot find executable for %s", str(self.JOYSTICK_NODE))
        
    
    def _confim_stop_btn_callback(self, i):
        self.confirm_close_open = False
        if i.text() == "&Yes":
            if self._joy_node_running():
                self.process.send_signal(signal.SIGINT)
                self.waiting_for_stop = True

    def _confirm_stop_close_callback(self, event):
        self.confirm_close_open = False

    @Slot()
    def _stop_callback(self):
        # Check to make sure that the joystick controller isn't still enabled
        teleop_node_name = self.namespace + "/" + self.TELEOP_NODE_NAME
        self._stop_button.setEnabled(False)
        if self.teleop_enabled:
            if self.confirm_close is None:
                self.confirm_close = QMessageBox()
                self.confirm_close.setIcon(QMessageBox.Critical)
                self.confirm_close.setText("The PS3 teleop node still appears to be enabled.\nStopping the joystick node while teleop is still running does not disable the controller!\nOnly stop if you are sure you want to do this!\n\nDo you still want to stop the joystick node?")
                self.confirm_close.setWindowTitle("Are you sure?")
                self.confirm_close.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
                self.confirm_close.buttonClicked.connect(self._confim_stop_btn_callback)
                self.confirm_close.closeEvent = self._confirm_stop_close_callback
            self.confirm_close_open = True
            self.confirm_close.show()
        else:
            if self._joy_node_running():
                self.process.send_signal(signal.SIGINT)
                self.waiting_for_stop = True

    @Slot()
    def _instructions_callback(self):
        if self.instructions_window is None:
            self.instructions_window = QWidget()
            loadUi(self.instructions_ui_file, self.instructions_window)

            self.instructions_window.setWindowTitle("Teleop Instructions")
            title_label = self.instructions_window.findChild(QLabel, "titleLabel")
            title_label.setText("PS3 Teleop Instructions:")

            text_area = self.instructions_window.findChild(QTextEdit, "textBox")
            with open(self.instructions_html_file) as instructions_file:
                text_area.setHtml(instructions_file.read())
            
            self.instructions_window.show()
        elif not self.instructions_window.isVisible():
            self.instructions_window.show()

    ########################################
    # Public Functions
    ########################################

    def cleanup(self):
        if self._joy_node_running():
            # Since it needs to be cleaned up, there isn't time to wait for graceful close
            # Just kill it and hope for the best
            self.process.kill()
            self.process.communicate()

        self._cleanup_topics()

        if self.instructions_window is not None:
            self.instructions_window.destroy(destroyWindow = True)
            self.instructions_window = None

        if self.confirm_close is not None:
            self.confirm_close.destroy(destroyWindow=True)
            self.confirm_close = None

    def tick(self):
        teleop_node_running = (True in map(lambda node: node[0] == self.TELEOP_NODE_NAME and node[1] == self.namespace, 
                                           self._node.get_node_names_and_namespaces()))
        process_running = self._joy_node_running()

        # Update Buttons
        if not teleop_node_running or process_running:
            self._start_button.setEnabled(False)
        
        if not process_running and teleop_node_running and self._joystick_selector.isEnabled():
            self._start_button.setEnabled(True)
        
        if process_running and not self.confirm_close_open and not self.waiting_for_stop:
            self._stop_button.setEnabled(True)
        else:
            self._stop_button.setEnabled(False)

        # Update status label
        if not teleop_node_running:
            self._status_label.setText("(Teleop Not Found)")
            self._status_label.setToolTip("The PS3 Teleop Node not found, make sure it is in bringup to run")
            self._status_label.setStyleSheet(self.STYLE_RED)
        elif process_running:
            if self.teleop_enabled:
                self._status_label.setText("(Enabled)")
                self._status_label.setToolTip("")
                self._status_label.setStyleSheet(self.STYLE_GREEN)
            else:
                self._status_label.setText("(Ready)")
                self._status_label.setToolTip("")
                self._status_label.setStyleSheet(self.STYLE_GREEN)
        elif self.teleop_enabled:
            self._status_label.setText("(Enabled w/o Joy)")
            self._status_label.setToolTip("")
            self._status_label.setStyleSheet(self.STYLE_RED)
        else:
            self._status_label.setText("(Stopped)")
            self._status_label.setToolTip("")
            self._status_label.setStyleSheet(self.STYLE_NORMAL)
        
        # Update Joystick Selector
        if process_running:
            self._joystick_selector.setEnabled(False)
            self._joystick_selector.clear()
            self._joystick_selector.addItem(self.selected_joystick_device)
            self._joystick_selector.setCurrentIndex(0)
        else:
            # If the combo box should be refreshed
            needs_refresh = False
            current_selection = self._joystick_selector.currentText()
            target_index = 0

            # Refresh if combo box not enabled (Transitioning from previous state)
            if not self._joystick_selector.isEnabled():
                needs_refresh = True
                current_selection = self.selected_joystick_device

            # Refresh available_joysticks if the input list changes
            input_list = os.listdir('/dev/input')
            if input_list != self.prev_input_list:
                # Needs to refresh
                needs_refresh = True
                self.prev_input_list = input_list
                self.available_joysticks = []
                # Rescan the joystick list
                for input_dev in input_list:
                    if input_dev.startswith("js"):
                        input_dev = "/dev/input/" + input_dev
                        self.available_joysticks.append(input_dev)

                        # Do a search for target_index to ensure the selected entry doesn't change
                        if input_dev == current_selection:
                            target_index = len(self.available_joysticks) - 1

            # Refresh control if needed
            if needs_refresh:
                if len(self.available_joysticks) == 0:
                    self._joystick_selector.setEnabled(False)
                    self._joystick_selector.clear()
                    self._joystick_selector.addItem("No Joysticks")
                    self._joystick_selector.setCurrentIndex(0)

                    # Disable button if no joysticks found
                    self._start_button.setEnabled(False)
                else:
                    if not self._joystick_selector.isEnabled():
                        self._joystick_selector.setEnabled(True)
                    self._joystick_selector.clear()
                    self._joystick_selector.addItems(self.available_joysticks)
                    self._joystick_selector.setCurrentIndex(target_index)

                    # Only enable start button if process is not running, there's joysticks availble, and the teleop is running
                    if teleop_node_running:
                        self._start_button.setEnabled(True)


    def set_namespace(self, namespace):
        self._cleanup_topics()
        self.namespace = namespace

        self._init_topics()
        self._status_label.setText("(Loading)")
        self._status_label.setToolTip("")
        self._status_label.setStyleSheet(self.STYLE_NORMAL)
        self._start_button.setEnabled(False)
        self._stop_button.setEnabled(False)

