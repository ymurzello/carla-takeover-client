# recorder-client

### Usage
CARLA works on a client server model. To run the client, the server must be launched first. Open up a terminal, activate the conda environment, then run the command below to launch the client.
```bash
#enable recording, scenario, and npcs
./launch_client.sh -r -s -npc
```
or
```bash
#to drive around without recording, scenario, and npcs
./launch_client.sh
```

### Controls
On the G29 steering wheel, press x to toggle autopilot and press right paddle to toggle reverse. To use a different controller, use the examples in wheel_config.ini as templates to add your own.

### NPC spawning
NPC spawn information (along with ego car spawn info) are saved as a configuration in spawn_configs. To generate new configs, call spawn_locations.py in the terminal:
```bash
#the conda environment must be activated and the server must be launched before running this script
python spawn_locations.py
```
It will generate a new config called test.json in the root directory. Rename it and move it to the spawn_configs directory. To use this new config, modify the ```spawn_config``` string in launch_client.sh.

### Dependencies
This client was built around Carla 0.9.9. Create a folder named Carla99 in your Linux home directory and install it there.

The Python environment named ```carla99``` used by the client is given as a .txt and .yml file, either of which can be used to create the environment on your own computer. Please see Conda's [docs](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html#id2) for instructions. Once it has been created, activate it by this command:
```bash
conda activate carla99
```
