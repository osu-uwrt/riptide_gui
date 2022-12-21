# riptide_rviz

This is a ROS2 package designed around controlling the OSU-UWRT Ritpide software stack onboard our robot. 
The plugin set is able to start code on remote computers, control the state of the vehicle, manipulate onboard actuators, 
visualize and execute autonomous behavior, and display important telemetry in a centralized location for an operator.

This is the configuration that is saved with the repo. This configurations in only reccomended, but can be modified as needed.
<img width="1512" alt="image" src="https://user-images.githubusercontent.com/5054270/194714880-4972024b-ce7f-43ab-8435-2ca03f239d07.png">


## Control Panel
<img width="389" alt="image" src="https://user-images.githubusercontent.com/5054270/194715024-bd59b701-42c5-4737-a28b-004843d4dd3e.png">


The control panel focuses on commanding the vehicle in the water. It has the following functions:

1. Software kill management: The enable and disable buttons, in tandem with the require kill check-box govern the kill state of the system. When the disable button is illuminated, the system is active, and the robot may move. When the enable button is illminated, the robot is in a killed state and cannot move. The require kill checkbox allows the robot to come untehered from the system without the kill switch timeout mechanism triggering. this should only be used for runs where the riptide vehicle will not be tethered during its run.

2. Control mode management: The 4 controller state buttons in tandem govern the next control mode to be requested of the robot. For the velocity and position modes, the information in the text entry fields below is used to command the robot. For Teleoperation, and Feedforward, simply the state set command is used.

3. Control command management: The current command in both velocity and position control modes is grabbed from the text entry fields before sending the command. Should any of the fields fail to be parsed, the send command button will disable and become red for 1 seccond to indicate a parse failure. Addtionally, the current button can be used while connected to the robot, to load the current odometry readout (shown in the text boxes above the command) into the command field. This allows for easy updates to the robotcs command from its current location.

4. Special features: The plugin also has a dive in place button that is only activated when the vehicle is located within a half meter of the surface (z=0.0). When pressed, it will send a position command to the robot to tell it to dive 0.75m depth in the same x and y position while also preserving the yaw angle. Roll and pitch will become zeroed out so the vehicle sits level at the end of the move.


## Actuator Panel
<img width="389" alt="image" src="https://user-images.githubusercontent.com/5054270/194715620-13e3a139-847a-4013-b8fe-c76e54ed8806.png">

The Actuator panel focuses on the control of actuation systems on the vehicle. It has control over the claw, dropper and torpedo systems currently.


For the claw system, the buttons can be used to command an open or a closed state.


For the dropper and torpedo systems that store energy, the syetems must be armed first. When the operator wishes to actuate them, they should press the arm button first. The button will become yellow during the arming process. At any time after pressing, the user can press the arm button again to disarm the system. Once the system is fully armed, the fire or drop buttons will become enabled and active. Pressing the button will trigger the actuation, then safely disarm the system. The armed button will again return to green when the system has been disarmed succesfully.


## Bringup Panel
<img width="389" alt="image" src="https://user-images.githubusercontent.com/5054270/194715864-f34cca7c-6532-4a1e-adea-9c51650e9862.png">

The Bringup panel is focused around starting and stopinng code execution on a remote system. 


It uses the ros2 package ros2_launch_system in order to control bringing up and down a launch file configuration from a specific package. In order to start, make sure the launch service is installed on the target computer, and enabled. If the computer list remains empty, click refresh to trigger a re-scan for the service. Once detected select the target computer in the list. Then select the file you wish to launch. When ready click the start button to launch the file on the remote computer. Once the request is sent, the stop button will illuminate, and the start button will disable. If the launch crashes, or the user presses stop, the launch will be shut down and the start button re-illuminated. 

## Mission Panel
<img width="348" alt="image" src="https://user-images.githubusercontent.com/5054270/194717198-68556710-ce69-4975-9961-7b2b006d2ca9.png">

The Mission panel controls the autonomy action server. It is capable of selecting, starting and stopping the action server in riptide_autonomy. 

The upper portion of the panel is used to control the action server. If autonomy has not been started before the rviz plugin, the user can press refresh to get the current list of trees. Once the list is populated, the default option will become none selected. When the selection is a valid tree, the user can then press start to begin running the tree. When started, the start button will become disabled, and the stop button will illuminate. When / if the user wishes to stop the tree, they can press stop. 

While a tree is running, a special topic becomes active with the execution stack. this stack is rendered in the tree view panel below. It will always contain the last stack recieved by the autonomy server. Should an error cause the exectuion to abort. The tree view will turn red for one second to indicate a tree failure and subsequent abort.
