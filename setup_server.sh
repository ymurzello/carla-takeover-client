#!/bin/bash

carlaPath="$HOME/Carla99"

gnome-terminal -- $carlaPath/CarlaUE4.sh

#default values
cars=30
walkers=30
record=false
map="Town03"

for (( i=1; i <= $#; i=i+2 )); do
	let next=$i+1
	#echo ${i} ${!i} ${!next}
	if [ ${!i} == '-n' ]
	then
		#echo "cars" ${!next}
		let cars=${!next}
		#echo $cars
	elif [ ${!i} == '-w' ]
	then
		#echo "walkers" ${!next}
		let walkers=${!next}
		#echo $walkers
	elif [ ${!i} == '-r' ] || [ ${!i} == '--record' ]
	then
		#set record
		if [ ${!next} == 't' ] || [ ${!next} == 'true' ]
		then
			record=true
		fi
		#echo $record
	elif [ ${!i} == '-m' ] || [ ${!i} == '--map' ]
	then
		#double brackets because we need arithmetic evaluation
		if [[ ${!next} != "Town0"[1-7] ]]
		then
			echo "Invalid map name:" ${!next}
			exit
		fi
		map=${!next}
		#echo $map
	fi
done

sleep 7
if [ $map != "Town03" ]
then
	gnome-terminal -- python $carlaPath/PythonAPI/util/config.py -m $map
fi
#gnome-terminal -- python $carlaPath/PythonAPI/examples/dynamic_weather.py
#gnome-terminal -- python $carlaPath/PythonAPI/examples/tm_spawn_npc.py -n $cars -w $walkers
