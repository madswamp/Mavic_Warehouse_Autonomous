#!/bin/bash

NUMID_DRONE=0
DRONE_SWARM_ID=0
export AEROSTACK_PROJECT=${AEROSTACK_STACK}/projects/mission_test

. ${AEROSTACK_STACK}/config/mission/setup.sh

gnome-terminal \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Tree Editor                                                                        ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior Tree Editor" --command "bash -c \"
roslaunch behavior_tree_editor behavior_tree_editor.launch --wait \
  robot_namespace:=drone$NUMID_DRONE \
  drone_id:=$NUMID_DRONE \
  mission_config_path:=${AEROSTACK_PROJECT}/configs/mission \
  catalog_path:=${AEROSTACK_PROJECT}/configs/mission/behavior_catalog.yaml;
exec bash\""  &