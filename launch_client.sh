#!/bin/bash

spawn_config="spawn_configs/test3.json"
# scenario_config="scenario_configs/pedestrian.json"
# scenario_config="scenario_configs/carcrash.json"
scenario_config="scenario_configs/bike.json"
scenario_flag=false
record_flag=false
npc_flag=false

for (( i=1; i <= $#; i=i+1 )); do
	let next=$i+1
	#echo ${i} ${!i} ${!next}
	if [ ${!i} == '-s' ]; then
		#echo "cars" ${!next}
		scenario_flag=true
  elif [ ${!i} == '-r' ]; then
    record_flag=true
  elif [ ${!i} == '-npc' ]; then
    npc_flag=true
  fi
done

launch_command="python synchronous_client.py -sp "
launch_command+=$spawn_config
launch_command+=" -sc "
launch_command+=$scenario_config
if [ "$scenario_flag" = true ]; then
  launch_command+=" -s True"
fi
if [ "$record_flag" = true ]; then
  launch_command+=" -r True"
fi
launch_command+="; read line"
#echo $launch_command
gnome-terminal -- bash -c "$launch_command"

if [ "$npc_flag" = true ]; then
  npc_command="python npc_manager.py -sp "
  npc_command+=$spawn_config
  sleep 7
  gnome-terminal -- bash -c "$npc_command"
fi
