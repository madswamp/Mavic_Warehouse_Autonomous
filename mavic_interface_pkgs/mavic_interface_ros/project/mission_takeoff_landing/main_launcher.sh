#!/bin/bash

NUMID_DRONE=0
export AEROSTACK_PROJECT=${MAVIC_NAV}/src/mavic_interface_pkgs/mavic_interface_ros/project/mission_takeoff_landing

. ${MAVIC_NAV}/src/mavic_interface_pkgs/mavic_interface_ros/config/mission/setup.sh

#---------------------------------------------------------------------------------------------
# INTERNAL PROCESSES
#---------------------------------------------------------------------------------------------
gnome-terminal  \
`#---------------------------------------------------------------------------------------------` \
`# Basic Behaviors                                                                             ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Basic Behaviors" --command "bash -c \"
roslaunch basic_quadrotor_behaviors basic_quadrotor_behaviors.launch --wait \
    namespace:=drone$NUMID_DRONE;
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Quadrotor Motion With Platform Control                                                           ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Quadrotor Motion With PID Control" --command "bash -c \"
roslaunch quadrotor_motion_with_pid_control quadrotor_motion_with_pid_control.launch --wait \
    namespace:=drone$NUMID_DRONE \
    robot_config_path:=${AEROSTACK_PROJECT}/configs/drone$NUMID_DRONE;
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Mavic Interface                                                                          ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Mavic Interface" --command "bash -c \"
roslaunch mavic_interface_ros mavic.launch --wait \
    namespace:=drone$NUMID_DRONE;
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Belief Memory Viewer                                                                        ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Belief memory Viewer" --command "bash -c \"
roslaunch belief_memory_viewer belief_memory_viewer.launch --wait \
  robot_namespace:=drone$NUMID_DRONE \
  drone_id:=$NUMID_DRONE;
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Execution Viewer                                                                   ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior Execution Viewer" --command "bash -c \"
roslaunch behavior_execution_viewer behavior_execution_viewer.launch --wait \
  robot_namespace:=drone$NUMID_DRONE \
  drone_id:=$NUMID_DRONE \
  catalog_path:=${AEROSTACK_PROJECT}/configs/mission/behavior_catalog.yaml;
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Belief Manager                                                                              ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Belief Manager" --command "bash -c \"
roslaunch belief_manager_process belief_manager_process.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE \
    config_path:=${AEROSTACK_PROJECT}/configs/mission;
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Belief Updater                                                                              ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Belief Updater" --command "bash -c \"
roslaunch belief_updater_process belief_updater_process.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    drone_id:=$NUMID_DRONE;
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Manager                                                                       ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior manager" --command "bash -c \" sleep 2;
roslaunch behavior_manager behavior_manager.launch --wait \
  robot_namespace:=drone$NUMID_DRONE \
  catalog_path:=${AEROSTACK_PROJECT}/configs/mission/behavior_catalog.yaml;
exec bash\"" &

#---------------------------------------------------------------------------------------------
# USER INTERFACE PROCESSES
#---------------------------------------------------------------------------------------------
gnome-terminal  \
`#---------------------------------------------------------------------------------------------` \
`# Alphanumeric Viewer                                                                         ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Alphanumeric Viewer"  --command "bash -c \"
roslaunch alphanumeric_viewer alphanumeric_viewer.launch  --wait \
    drone_id_namespace:=drone$NUMID_DRONE;
exec bash\""  &
sleep 3
gnome-terminal \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Tree Interpreter                                                                   ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior Tree Interpreter" --command "bash -c \"
roslaunch behavior_tree_interpreter behavior_tree_interpreter.launch --wait \
  robot_namespace:=drone$NUMID_DRONE \
  drone_id:=$NUMID_DRONE \
  mission_configuration_folder:=${AEROSTACK_PROJECT}/configs/mission \
  catalog_path:=${AEROSTACK_PROJECT}/configs/mission/behavior_catalog.yaml;
exec bash\"" &
