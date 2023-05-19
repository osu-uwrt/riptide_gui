import os
import math
import rclpy
import threading
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from transforms3d.euler import euler2quat, quat2euler

from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Empty, Bool
from nav_msgs.msg import Odometry
import riptide_msgs2.action
from riptide_msgs2.msg import KillSwitchReport, ControllerCommand

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMessageBox, QWidget, QVBoxLayout, QLineEdit, QPushButton, QDoubleSpinBox, QLabel, QRadioButton, QHBoxLayout
from python_qt_binding.QtCore import QTimer, Slot

from .action_widget import ActionWidget
from .joystick_widget import PS3TeleopWidget

package_share_dir = get_package_share_directory('riptide_rqt_plugins2')

class OrientationControlsManager:
    def __init__(self, widget: 'ControllersWidget'):
        self._widget = widget

        # Load in all controls
        self._orientation_x_label = widget.findChild(QLabel, "orientationXLabel")
        self._orientation_x = widget.findChild(QDoubleSpinBox, "orientationXValue")
        self._orientation_y_label = widget.findChild(QLabel, "orientationYLabel")
        self._orientation_y = widget.findChild(QDoubleSpinBox, "orientationYValue")
        self._orientation_z_label = widget.findChild(QLabel, "orientationZLabel")
        self._orientation_z = widget.findChild(QDoubleSpinBox, "orientationZValue")
        self._orientation_w_label = widget.findChild(QLabel, "orientationWLabel")
        self._orientation_w = widget.findChild(QDoubleSpinBox, "orientationWValue")

        self._angle_mode_btn = widget.findChild(QPushButton, "angleModeButton")
        self._euler_mode_radio = widget.findChild(QRadioButton, "eulerModeRadio")
        self._quaternion_mode_radio = widget.findChild(QRadioButton, "quaternionModeRadio")

        self._angle_mode_btn.clicked.connect(self.toggle_angle)
        self._euler_mode_radio.clicked.connect(self.set_euler_mode)
        self._quaternion_mode_radio.clicked.connect(self.set_quaternion_mode)

        # Load initial defaults
        self.in_euler_mode = False      # Set initial mode to quaternion since that is what the UI file loads into
        self.in_degree_mode = False
        self.set_orientation(Quaternion(w=1.0, x=0.0, y=0.0, z=0.0))
        self.set_euler_mode()
        self.set_degrees_mode()

    ########################################
    # Private Control Update Functions
    ########################################

    def _update_field_mode(self, field: QDoubleSpinBox, half_range: bool):
        if self.in_euler_mode:
            if self.in_degree_mode:
                if half_range:
                    max_value = 90.0
                else:
                    max_value = 180.0
                
                field.setMaximum(max_value)
                field.setMinimum(-max_value)
                field.setSingleStep(5.0)
                field.setDecimals(1)
            else:
                if half_range:
                    max_value = math.pi/2
                else:
                    max_value = math.pi
                
                field.setMaximum(max_value)
                field.setMinimum(-max_value)
                field.setSingleStep(0.05)
                field.setDecimals(3)
        else:
            field.setMaximum(1.0)
            field.setMinimum(-1.0)
            field.setSingleStep(0.05)
            field.setDecimals(3)

    ########################################
    # Settings Restoring/Saving
    ########################################

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.contains('orientationMode'):
            mode = instance_settings.value('orientationMode')
            if mode == "quaternion":
                self.set_quaternion_mode()
            elif mode == "euler":
                self.set_euler_mode()
            else:
                self._widget._node.get_logger().warn("Invalid Orientation Mode '{}' in saved configuration".format(mode))
        
        if instance_settings.contains('orientationAngleSystem'):
            mode = instance_settings.value('orientationAngleSystem')
            if mode == "degrees":
                self.set_degrees_mode()
            elif mode == "radians":
                self.set_radians_mode()
            else:
                self._widget._node.get_logger().warn("Invalid Angle System '{}' in saved configuration".format(mode))

        target_orientation = self.get_orientation()
        if instance_settings.contains('orientationX'):
            target_orientation.x = float(instance_settings.value('orientationX'))
        if instance_settings.contains('orientationY'):
            target_orientation.y = float(instance_settings.value('orientationY'))
        if instance_settings.contains('orientationZ'):
            target_orientation.z = float(instance_settings.value('orientationZ'))
        if instance_settings.contains('orientationW'):
            target_orientation.w = float(instance_settings.value('orientationW'))
        self.set_orientation(target_orientation)

    def save_settings(self, plugin_settings, instance_settings):
        current_orientation = self.get_orientation()
        instance_settings.set_value('orientationX', current_orientation.x)
        instance_settings.set_value('orientationY', current_orientation.y)
        instance_settings.set_value('orientationZ', current_orientation.z)
        instance_settings.set_value('orientationW', current_orientation.w)
        
        instance_settings.set_value('orientationMode', "euler" if self.in_euler_mode else "quaternion")
        instance_settings.set_value('orientationAngleSystem', "degrees" if self.in_degree_mode else "radians")

    ########################################
    # Mode Change Functions
    ########################################

    def set_quaternion_mode(self):
        current_orientation = self.get_orientation()

        self.in_euler_mode = False
        self._quaternion_mode_radio.setChecked(True)
        self._angle_mode_btn.setVisible(False)
        self._orientation_x_label.setText("X:")
        self._orientation_y_label.setText("Y:")
        self._orientation_z_label.setText("Z:")
        self._orientation_w_label.setText("W:")

        self._orientation_w_label.setVisible(True)
        self._orientation_w.setVisible(True)

        self._update_field_mode(self._orientation_x, False)
        self._update_field_mode(self._orientation_y, True)
        self._update_field_mode(self._orientation_z, False)
        self._update_field_mode(self._orientation_w, False)

        self.set_orientation(current_orientation)

    def set_euler_mode(self):
        current_orientation = self.get_orientation()
        
        self.in_euler_mode = True
        self._euler_mode_radio.setChecked(True)
        self._angle_mode_btn.setVisible(True)
        self._orientation_x_label.setText("R:")
        self._orientation_y_label.setText("P:")
        self._orientation_z_label.setText("Y:")
        self._orientation_w_label.setVisible(False)
        self._orientation_w.setVisible(False)

        self._update_field_mode(self._orientation_x, False)
        self._update_field_mode(self._orientation_y, True)
        self._update_field_mode(self._orientation_z, False)
        self._update_field_mode(self._orientation_w, False)

        self.set_orientation(current_orientation)

    def set_degrees_mode(self):
        current_orientation = self.get_orientation()

        self.in_degree_mode = True
        self._angle_mode_btn.setText("Degrees")
        self._update_field_mode(self._orientation_x, False)
        self._update_field_mode(self._orientation_y, True)
        self._update_field_mode(self._orientation_z, False)
        self._update_field_mode(self._orientation_w, False)

        self.set_orientation(current_orientation)

    def set_radians_mode(self):
        current_orientation = self.get_orientation()

        self.in_degree_mode = False
        self._angle_mode_btn.setText("Radians")
        self._update_field_mode(self._orientation_x, False)
        self._update_field_mode(self._orientation_y, True)
        self._update_field_mode(self._orientation_z, False)
        self._update_field_mode(self._orientation_w, False)

        self.set_orientation(current_orientation)

    def toggle_angle(self):
        if self.in_degree_mode:
            self.set_radians_mode()
        else:
            self.set_degrees_mode()

    ########################################
    # Field to Quaternion Conversions
    ########################################

    def set_orientation(self, orientation: Quaternion):
        xValue = orientation.x
        yValue = orientation.y
        zValue = orientation.z
        wValue = orientation.w

        if self.in_euler_mode:
            xValue, yValue, zValue = quat2euler((wValue, xValue, yValue, zValue), 'sxyz')

            if self.in_degree_mode:
                xValue *= 180.0/math.pi
                yValue *= 180.0/math.pi
                zValue *= 180.0/math.pi

        self._orientation_w.setValue(wValue)
        self._orientation_x.setValue(xValue)
        self._orientation_y.setValue(yValue)
        self._orientation_z.setValue(zValue)

    def get_orientation(self) -> Quaternion:
        xValue=self._orientation_x.value()
        yValue=self._orientation_y.value()
        zValue=self._orientation_z.value()
        wValue=self._orientation_w.value()

        if self.in_euler_mode:
            if self.in_degree_mode:
                xValue *= math.pi/180.0
                yValue *= math.pi/180.0
                zValue *= math.pi/180.0
            
            wValue, xValue, yValue, zValue = euler2quat(xValue, yValue, zValue, axes='sxyz')

        return Quaternion(x=xValue, y=yValue, z=zValue, w=wValue)

class ControllersWidget(QWidget):
    namespace = ""

    LIGHT_STYLE_OFF = "QLabel{ color: rgb(186, 189, 182) }"
    LIGHT_STYLE_RED = "QLabel{ color: rgb(239, 41, 41) }"
    LIGHT_STYLE_GREEN = "QLabel{ color: rgb(78, 154, 6) }"

    stopping = False
    steady_light_data = False
    conflicting_publisher_light_data = False
    kill_switch_killed = False

    confirm_unkill = None

    last_odom_message = None

    CONFLICT_EXPIRATION_TIME = Duration(seconds=30)
    CONFLICT_SEQUENCE_TIME = Duration(seconds=10)

    def __init__(self, node: 'rclpy.Node'):
        super(ControllersWidget, self).__init__()
        self._node: 'rclpy.Node' = node
    
        # Load UI
        ui_file = os.path.join(package_share_dir, 'resource', 'ControllersPlugin.ui')
        loadUi(ui_file, self)
        self.setObjectName('ControllersPluginUi')

        # Get all UI elements loaded into class and connected to callbacks
        
        # Configure all actions to be used
        self._actions_layout = self.findChild(QVBoxLayout, "actionsVerticalLayout")
        self._actions = []
        self.add_action("Calibrate Buoyancy", "calibrate_buoyancy", riptide_msgs2.action.CalibrateBuoyancy, has_results=True)
        self.add_action("Calibrate Drag", "calibrate_drag", riptide_msgs2.action.CalibrateDrag, has_results=True)
        self.add_action("Thruster Test", "thruster_test", riptide_msgs2.action.ThrusterTest, has_results=False)

        self._teleop_widget = PS3TeleopWidget(self.namespace, self._node)
        self._teleop_widget_layout = self.findChild(QVBoxLayout, "teleopControlLayout")
        self._teleop_widget_layout.addWidget(self._teleop_widget)

        # Namespace config
        self._namespace_text = self.findChild(QLineEdit, "namespaceEdit")
        self._namespace_apply = self.findChild(QPushButton, "applyNamespaceButton")
        self._namespace_apply.clicked.connect(self._namespace_apply_callback)

        # Status Labels
        self._steady_light = self.findChild(QLabel, "steadyLabel")
        self._conflicting_publisher_light = self.findChild(QLabel, "conflictingPublisherLabel")
        self._conflicting_publisher_light.setVisible(False)

        # Target Values
        self._linear_target_title = self.findChild(QLabel, "linearTargetTitle")
        self._angular_target_title = self.findChild(QLabel, "angularTargetTitle")
        self._linear_target_label = self.findChild(QLabel, "linearTargetLabel")
        self._angular_target_label = self.findChild(QLabel, "angularTargetLabel")

        # Add lock for competing publishers since multiple publishers share the code to check
        # if there's a competing publisher and race conditions occur without it
        self._competing_publishing_lock = threading.Lock()

        # Controller Position Publishing
        self._position_x = self.findChild(QDoubleSpinBox, "positionXValue")
        self._position_y = self.findChild(QDoubleSpinBox, "positionYValue")
        self._position_z = self.findChild(QDoubleSpinBox, "positionZValue")
        self._orientation_controls_manager = OrientationControlsManager(self)

        self._load_current_position = self.findChild(QPushButton, "loadCurrentPosition")
        self._load_current_orientation = self.findChild(QPushButton, "loadCurrentOrientation")
        self._load_current_position.clicked.connect(self._load_current_position_callback)
        self._load_current_orientation.clicked.connect(self._load_current_orientation_callback)

        self._publish_position = self.findChild(QPushButton, "publishPositionButton")
        self._stop_controller = self.findChild(QPushButton, "stopControllerButton")
        self._publish_position.clicked.connect(self._publish_position_callback)
        self._stop_controller.clicked.connect(self._stop_controller_callback)

        # Software Kill
        self._software_kill = self.findChild(QPushButton, "softwareKillButton")
        self._software_kill_enable = self.findChild(QPushButton, "softwareKillEnableButton")
        self._software_kill_require_ping = self.findChild(QPushButton, "softwareKillRequirePingButton")
        self._killed_label = self.findChild(QLabel, "killedLabel")
        self._software_kill.clicked.connect(self._software_kill_callback)
        self._software_kill_enable.clicked.connect(self._software_kill_enabled_callback)

        # Initialize kill switch message with node info
        self.switch_msg = KillSwitchReport()
        self.switch_msg.sender_id = self._node.get_namespace() + '/' + self._node.get_name()
        self.switch_msg.switch_needs_update = False
        self.switch_msg.kill_switch_id = KillSwitchReport.KILL_SWITCH_RQT_CONTROLLER

        # Setup timers
        self._software_kill_pub_timer = QTimer(self)
        self._software_kill_pub_timer.timeout.connect(self._software_kill_pub_cb)
        self._tick_timer = QTimer(self)
        self._tick_timer.timeout.connect(self.timer_tick)

        # Finally configure window to reset state
        self._reset_state()

    def _reset_state(self):
        # Resets the state of the ui to loading
        # This is the default state after any reload, such as loading or changing namespace

        self._load_current_position.setEnabled(False)
        self._load_current_orientation.setEnabled(False)
        self.last_odom_message = None

        self._publish_position.setEnabled(False)
        self._stop_controller.setEnabled(False)

        self.kill_switch_killed = False
        self._killed_label.setStyleSheet(self.LIGHT_STYLE_OFF)
        self._software_kill_enable.setEnabled(False)
        self._software_kill_enable.setChecked(False)
        self._software_kill.setEnabled(False)
        self._software_kill.setChecked(False)
        self._software_kill_require_ping.setEnabled(False)
        self._software_kill_require_ping.setChecked(False)
        self._software_kill.setVisible(False)
        self._software_kill_require_ping.setVisible(False)
        self._killed_label.setVisible(False)

        self._linear_target_title.setText("Position:")
        self._linear_target_label.setText("Loading")
        self._linear_target_label.setToolTip("")
        self._angular_target_title.setText("Orientation:")
        self._angular_target_label.setText("Loading")
        self._angular_target_label.setToolTip("")

        self._linear_target_data = ["Position:", "No Data", None, None]
        self._angular_target_data = ["Orientation:", "No Data", None, None]

        self._steady_light.setStyleSheet(self.LIGHT_STYLE_OFF)
        self._conflicting_publisher_light.setStyleSheet(self.LIGHT_STYLE_OFF)
        self.steady_light_data = False
        self.conflicting_publisher_light_data = False
        with self._competing_publishing_lock:
            self._controller_publish_history = [[],[]]

    ########################################
    # Topic Management
    ########################################

    def _init_topics(self):
        self._position_pub = self._node.create_publisher(ControllerCommand, self.namespace + "/controller/linear", qos_profile_system_default)
        self._orientation_pub = self._node.create_publisher(ControllerCommand, self.namespace + "/controller/angular", qos_profile_system_default)

        self._software_kill_pub = self._node.create_publisher(KillSwitchReport, self.namespace + "/control/software_kill", qos_profile_sensor_data)

        self._odom_sub = self._node.create_subscription(Odometry, self.namespace + "/odometry/filtered", self._odom_callback, 1)
        self._position_sub = self._node.create_subscription(ControllerCommand, self.namespace + "/controller/linear", self._linear_callback, qos_profile_system_default)
        self._orientation_sub = self._node.create_subscription(ControllerCommand, self.namespace + "/controller/angular", self._angular_callback, qos_profile_system_default)
        self._steady_sub = self._node.create_subscription(Bool, self.namespace + "/controller/steady", self._steady_callback, qos_profile_system_default)
        self._kill_switch_sub = self._node.create_subscription(Bool, self.namespace + "/state/kill", self._kill_switch_callback, qos_profile_sensor_data)

    def _cleanup_topics(self):
        if self._software_kill_pub_timer.isActive():
            self._software_kill_pub_timer.stop()
        
        self._node.destroy_publisher(self._position_pub)
        self._node.destroy_publisher(self._orientation_pub)
        self._node.destroy_publisher(self._software_kill_pub)

        self._node.destroy_subscription(self._odom_sub)
        self._node.destroy_subscription(self._position_sub)
        self._node.destroy_subscription(self._orientation_sub)
        self._node.destroy_subscription(self._steady_sub)
        self._node.destroy_subscription(self._kill_switch_sub)

    def _odom_callback(self, msg: Odometry):
        self.last_odom_message = msg

    def _track_publisher(self, node):
        with self._competing_publishing_lock:
            current_time = self._node.get_clock().now()
            # Format for _controller_publish_history
            # First list is a list of previously published nodes
            # Second list is an ordered list of times, whose index correspond to the nodes in the first list

            # First cleanup any expired messages
            while len(self._controller_publish_history[0]) > 0 and (current_time - self._controller_publish_history[1][0]) > self.CONFLICT_EXPIRATION_TIME:
                tnode = self._controller_publish_history[0].pop(0)
                self._controller_publish_history[1].pop(0)

            # Then check for if the message has been published within expiration time
            if node in self._controller_publish_history[0]:
                if self._controller_publish_history[0][-1] != node and (current_time - self._controller_publish_history[1][-1]) <= self.CONFLICT_SEQUENCE_TIME:
                    self.conflicting_publisher_light_time = current_time
                    self.conflicting_nodes = [node, self._controller_publish_history[0][-1]]
                    self.conflicting_publisher_light_data = True
                prev_index = self._controller_publish_history[0].index(node)
                tnode = self._controller_publish_history[0].pop(prev_index)
                self._controller_publish_history[1].pop(prev_index)
            
            self._controller_publish_history[0].append(node)
            self._controller_publish_history[1].append(current_time)

    def _linear_callback(self, msg: ControllerCommand):
        #self._track_publisher(msg._connection_header['callerid'])
        if msg.mode == ControllerCommand.POSITION:
            self._linear_target_data[0] = "Position:"
            self._linear_target_data[1] = "({0:.2f}, {1:.2f}, {2:.2f})".format(msg.setpoint_vect.x, msg.setpoint_vect.y, msg.setpoint_vect.z)
            #self._linear_target_data[2] = msg._connection_header['callerid']
            self._linear_target_data[3] = self._node.get_clock().now()
        elif msg.mode == ControllerCommand.VELOCITY:
            self._linear_target_data[0] = "Lin Vel:"
            self._linear_target_data[1] = "({0:.2f}, {1:.2f}, {2:.2f})".format(msg.setpoint_vect.x, msg.setpoint_vect.y, msg.setpoint_vect.z)
            #self._linear_target_data[2] = msg._connection_header['callerid']
            self._linear_target_data[3] = self._node.get_clock().now()
        elif msg.mode == ControllerCommand.FEEDFORWARD:
            self._linear_target_data[0] = "Linear:"
            self._linear_target_data[1] = "Feed Forward"
            #self._linear_target_data[2] = msg._connection_header['callerid']
            self._linear_target_data[3] = self._node.get_clock().now()
        elif msg.mode == ControllerCommand.DISABLED:
            self._linear_target_data = ["Position:", "No Data", None, None]
        else:
            self._linear_target_data = ["Position:", "Invalid Cmd", None, None]

    def _angular_callback(self, msg: ControllerCommand):
        #self._track_publisher(msg._connection_header['callerid'])
        if msg.mode == ControllerCommand.POSITION:
            self._angular_target_data[0] = "Orientation:"
            self._angular_target_data[1] = "({0:.2f}, {1:.2f}, {2:.2f}. {3:.2f})".format(msg.setpoint_quat.x, msg.setpoint_quat.y, msg.setpoint_quat.z, msg.setpoint_quat.w)
            #self._angular_target_data[2] = msg._connection_header['callerid']
            self._angular_target_data[3] = self._node.get_clock().now()
        elif msg.mode == ControllerCommand.VELOCITY:
            self._angular_target_data[0] = "Ang Vel:"
            self._angular_target_data[1] = "({0:.2f}, {1:.2f}, {2:.2f})".format(msg.setpoint_vect.x, msg.setpoint_vect.y, msg.setpoint_vect.z)
            #self._angular_target_data[2] = msg._connection_header['callerid']
            self._angular_target_data[3] = self._node.get_clock().now()
        elif msg.mode == ControllerCommand.FEEDFORWARD:
            self._angular_target_data[0] = "Angular:"
            self._angular_target_data[1] = "Feed Forward"
            #self._linear_target_data[2] = msg._connection_header['callerid']
            self._angular_target_data[3] = self._node.get_clock().now()
        elif msg.mode == ControllerCommand.DISABLED:
            self._angular_target_data = ["Orientation:", "No Data", None, None]
        else:
            self._angular_target_data = ["Orientation:", "Invalid", None, None]

    def _steady_callback(self, msg):
        self.steady_light_data = msg.data

    def _kill_switch_callback(self, msg: Bool):
        # If the kill switch was just inserted, clear target data
        # Not going to continuously clear the data so it'll be clear if topics are still publishing after kill
        if not msg.data and not self.kill_switch_killed:
            self._linear_target_data = ["Position:", "No Data", None, None]
            self._angular_target_data = ["Orientation:", "No Data", None, None]

        self.kill_switch_killed = not msg.kill_switch_inserted

    ########################################
    # Button Callbacks
    ########################################
    
    @Slot()
    def _namespace_apply_callback(self):
        # Fix up entered namespace, and hide trailing slash for aesthetics
        namespace = self._namespace_text.text().rstrip("/")
        
        if len(namespace) != 0 and namespace[0] != "/":
            namespace = "/" + namespace
        
        if len(namespace) == 0:
            self._namespace_text.setText('/')    
        else:
            self._namespace_text.setText(namespace)

        if self.namespace != namespace:
            self.namespace = namespace
            self.update_namespace()

    def _confim_unkill_btn_callback(self, i):
        if i.text() == "&Yes":
            self._software_kill.setChecked(False)

    @Slot()
    def _software_kill_enabled_callback(self):
        self._software_kill_enable.setChecked(True)
        self._software_kill_enable.setEnabled(False)
        self._software_kill.setVisible(True)
        self._software_kill_require_ping.setVisible(True)
        self._killed_label.setVisible(True)

        self._software_kill_pub_timer.start(200)

    @Slot()
    def _software_kill_pub_cb(self):
        self.switch_msg.switch_asserting_kill = self._software_kill.isChecked()
        self.switch_msg.switch_needs_update = self._software_kill_require_ping.isChecked()
        self._software_kill_pub.publish(self.switch_msg)

    @Slot()
    def _software_kill_callback(self):        
        # Show message box if controller isn't stopped and trying to unkill
        if not self._software_kill.isChecked() and self._linear_target_data[3] is not None or self._angular_target_data[3] is not None:
            if self.confirm_unkill is None:
                self.confirm_unkill = QMessageBox()
                self.confirm_unkill.setIcon(QMessageBox.Critical)
                self.confirm_unkill.setText("The controller is still active!\nIf higher level code is still publishing position when the robot is unkilled, it will unexpectedly move.\nIt is recommended that you press Stop Controllers to ensure nothing is still publishing.\n\nAre you STILL sure you want to unkill the robot?")
                self.confirm_unkill.setWindowTitle("Are you sure?")
                self.confirm_unkill.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
                self.confirm_unkill.buttonClicked.connect(self._confim_unkill_btn_callback)
            self.confirm_unkill.show()
            self._software_kill.setChecked(True)
        else:
            self._software_kill_pub_cb()

    @Slot()
    def _load_current_position_callback(self):
        if self.last_odom_message is not None:
            self._position_x.setValue(self.last_odom_message.pose.pose.position.x)
            self._position_y.setValue(self.last_odom_message.pose.pose.position.y)
            self._position_z.setValue(self.last_odom_message.pose.pose.position.z)

    @Slot()
    def _load_current_orientation_callback(self):
        if self.last_odom_message is not None:
            self._orientation_controls_manager.set_orientation(self.last_odom_message.pose.pose.orientation)

    @Slot()
    def _publish_position_callback(self):
        linear_cmd = ControllerCommand()
        linear_cmd.mode = ControllerCommand.POSITION
        linear_cmd.setpoint_vect = Vector3(x=self._position_x.value(), y=self._position_y.value(), z=self._position_z.value())

        angular_cmd = ControllerCommand()
        angular_cmd.mode = ControllerCommand.POSITION
        angular_cmd.setpoint_quat = self._orientation_controls_manager.get_orientation()

        self._position_pub.publish(linear_cmd)
        self._orientation_pub.publish(angular_cmd)
    
    @Slot()
    def _stop_controller_callback(self):
        off_cmd = ControllerCommand()
        off_cmd.mode = ControllerCommand.DISABLED
        self._position_pub.publish(off_cmd)
        self._orientation_pub.publish(off_cmd)

    ########################################
    # Lifetime Management
    ########################################

    def start(self):
        """
        This method needs to be called to start updating topic pane.
        """
        self._init_topics()
        self._tick_timer.start(1000)

    def shutdown_plugin(self):
        self.stopping = True
        self._tick_timer.stop()
        self._cleanup_topics()
        self._teleop_widget.cleanup()

        for action in self._actions:
            action.cleanup_topics()

        if self.confirm_unkill is not None:
            self.confirm_unkill.destroy(destroyWindow=True)
            self.confirm_unkill = None

    @Slot()
    def timer_tick(self):
        if self.stopping:
            return

        # Tick child widgets
        self.tick_actions()
        self._teleop_widget.tick()

        # Update button status
        # We are subscribing to these topics as well, so there must be another subscriber for controllers to be present
        self._load_current_position.setEnabled(self.last_odom_message is not None)
        self._load_current_orientation.setEnabled(self.last_odom_message is not None)
        self._publish_position.setEnabled((self._position_pub.get_subscription_count() > 1) and (self._orientation_pub.get_subscription_count() > 1))
        self._stop_controller.setEnabled((self._position_pub.get_subscription_count()) > 1 or (self._orientation_pub.get_subscription_count() > 1))

        kill_publisher_available = self._software_kill_pub.get_subscription_count() > 0
        self._software_kill.setEnabled(kill_publisher_available)
        self._software_kill_require_ping.setEnabled(kill_publisher_available)
        if not self._software_kill_enable.isChecked():
            self._software_kill_enable.setEnabled(kill_publisher_available)
        
        if self.kill_switch_killed:
            self._killed_label.setStyleSheet(self.LIGHT_STYLE_RED)
        else:
            self._killed_label.setStyleSheet(self.LIGHT_STYLE_OFF)

        # Update linear and angular targets for the controller
        current_time = self._node.get_clock().now()
        linear_time_diff = ""
        linear_origin_node = ""
        if self._linear_target_data[3] is not None:
            linear_time_diff = "{0} Secs. Ago".format((current_time - self._linear_target_data[3]).nanoseconds // 1e9)
        if self._linear_target_data[2] is not None:
            linear_origin_node = "Origin: " + self._linear_target_data[2]
        self._linear_target_title.setText(self._linear_target_data[0])
        self._linear_target_label.setText(self._linear_target_data[1])
        self._linear_target_label.setToolTip(linear_origin_node + linear_time_diff)

        angular_time_diff = ""
        angular_origin_node = ""
        if self._angular_target_data[3] is not None:
            angular_time_diff = "{0} Secs. Ago".format((current_time - self._angular_target_data[3]).nanoseconds // 1e9)
        if self._angular_target_data[2] is not None:
            angular_origin_node = "Origin: " + self._angular_target_data[2]
        self._angular_target_title.setText(self._angular_target_data[0])
        self._angular_target_label.setText(self._angular_target_data[1])
        self._angular_target_label.setToolTip(angular_origin_node + angular_time_diff)

        # Update the status "lights"
        if self.steady_light_data:
            self._steady_light.setStyleSheet(self.LIGHT_STYLE_GREEN)
        else:
            self._steady_light.setStyleSheet(self.LIGHT_STYLE_OFF)

        if self.conflicting_publisher_light_data and (self._node.get_clock().now() - self.conflicting_publisher_light_time) <= self.CONFLICT_EXPIRATION_TIME:
            self._conflicting_publisher_light.setStyleSheet(self.LIGHT_STYLE_RED)
            self._conflicting_publisher_light.setToolTip("Conflicts: {0} and {1}".format(self.conflicting_nodes[0], self.conflicting_nodes[1]))
        else:
            if self.conflicting_publisher_light_data:
                self.conflicting_publisher_light_data = False
                self.conflicting_publisher_light_time = None
            self._conflicting_publisher_light.setStyleSheet(self.LIGHT_STYLE_OFF)
            self._conflicting_publisher_light.setToolTip("")

    def save_settings(self, plugin_settings, instance_settings):
        # Save occurs before shutdown
        instance_settings.set_value('namespace', self.namespace)
        instance_settings.set_value('positionX', self._position_x.value())
        instance_settings.set_value('positionY', self._position_y.value())
        instance_settings.set_value('positionZ', self._position_z.value())
        self._orientation_controls_manager.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore occurs after init
        if instance_settings.contains('namespace'):
            self.namespace = instance_settings.value('namespace')
            if len(self.namespace) == 0:
                self._namespace_text.setText('/')    
            else:
                self._namespace_text.setText(self.namespace)
            self.update_namespace()
        
        if instance_settings.contains('positionX'):
            self._position_x.setValue(float(instance_settings.value('positionX')))
        if instance_settings.contains('positionY'):
            self._position_y.setValue(float(instance_settings.value('positionY')))
        if instance_settings.contains('positionZ'):
            self._position_z.setValue(float(instance_settings.value('positionZ')))
        self._orientation_controls_manager.restore_settings(plugin_settings, instance_settings)

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    ########################################
    # Actions Management
    ########################################
    def add_action(self, name, topic, spec, has_results=False):
        action = ActionWidget(self._node, name, self.namespace, topic, spec, self._actions_layout, has_results)
        self._actions.append(action)

    def update_namespace(self):
        for action in self._actions:
            action.set_namespace(self.namespace)
        self._teleop_widget.set_namespace(self.namespace)
        self._cleanup_topics()
        self._reset_state()
        self._init_topics()


    def tick_actions(self):
        # Call periodic update on all actions
        # Typically called for timer callback for updating state without callbacks (like a disappearing topic)
        for action in self._actions:
            action.tick()

class ControllersPlugin(Plugin):

    def __init__(self, context):
        super(ControllersPlugin, self).__init__(context)
        self._node = context.node

        # Give QObjects reasonable names
        self.setObjectName('ControllersPlugin')

        self._widget = ControllersWidget(self._node)
        self._widget.start()
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
    
    def shutdown_plugin(self):
        self._widget.shutdown_plugin()

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)