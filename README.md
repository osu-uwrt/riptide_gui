# riptide_gui
The riptide GUI repository contains ros packages pertianing to visual control of our AUV platforms. 

|            |              |
|------------|--------------|
| OS Distro  | Ubuntu 22.04 |
| ROS Distro | ROS2 Humble  |

## riptide_rviz
Riptide RVIZ is the primary visual control system for the riptide AUV's. It provides data visualization in 3d as well as control functionality over onboard systems. It also is capable of displaying the vehicle status to the user in real time as the information comes in. It contains several plugins that are run on the operator computer to provide full command and control functonality. 

## riptide_rqt_plugins
Riptide RQT Plugins provides an older vehicle control interface used for commanding the riptide AUV's. It provides some test functionality for the vehicle that is not present in the more polished plugin system created by Riptide RVIZ. 