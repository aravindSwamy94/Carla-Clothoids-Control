
## Requirements
* Carla - 0.9.6 - Download the version from the above link (https://github.com/carla-simulator/carla/releases/tag/0.9.6)
* python 3.5 (Strict Requirement)
* numpy
* pygame
* Clothoids - Download and installation instructions from link (https://github.com/ebertolazzi/Clothoids). Clothoids library location to be added in local_planner.py inside the framework_fns/ folder


## Files/Folders Included

### multibash.sh 
This bash script helps to open five terminals for easily navigating the python scripts.

* Terminal 1- To launch the Carla Server
* Terminal 2- To change the map configuration to Town01
* Terminal 3- To launch the manual control version of carla 
* Terminal 4- To launch the Scenario 
* Terminal 5- To launch the AD alogrithms to control the ego vehicle


## Files Organization

* **Download Carla**: Download the Carla version from the link mentioned above and place it in Desktop (Or in any preferred location. Here on I assume that Carla version is placed in Desktop). NOTE: Download the compiled version. Please dont download the source code.

* **Download this repo code**: Download the code and the contents are to be copied in different locations which is explained below
 
* **multibash placement**: Place the _multibash.sh_ file in the Carla folder. i.e. /home/${USER}/Desktop/CARLA_0.9.6/

* **Motion_planning_Framework/ placement**: Place the Motion_planning_Framework/ folder inside the PythonAPI folder of Carla base. i.e./home/${USER}/Desktop/CARLA_0.9.6/PythonAPI/

## Steps to run Framework:
There are common steps to run different scenarios. 

* Step 1: open Terminal 1 and start the server 
```
./CarlaUE4.sh -ResX=1 -ResY=1 -carla-port=2000
```
* Step 2: open Terminal 2 and config Town01
```
python config.py -m Town01
```
Note: This is a one step process. This can be done only once for every time you run the server.
* Step 3: Open Terminal 3 and launch the manu control
```
python manual_control.py --filter vehicle.tesla.model* --res 720x560
```

* Step 4: Set the start of ego position to the desired location of the scenario. Open terminal 5
```
python set_ego_pos.py
```

### LANE_FOLLOW:
* Step 5: open Terminal 5 and launch the main code.
```
python launch_framework.py
```

### LANE_CHANGE
* Step 4: open Terminal 4 and launch code for getting Obstacle Vehicle:
```
python spawn_actor_Stop.py
```
* Step 5: open Terminal 5 and launch the main code.
```
python launch_framework.py
```
* Step 6: After the scenario is over . Please Dont forget to destroy the Obstacle actor . This can be done by
```
python destroy_actor_Stop.py
```
### STOP AND GO
* Step 4: open Terminal 4 and launch code for getting Obstacle Vehicle:
```
python spawn_actor_DK.py
```
* Step 5: open Terminal 5 and launch the main code.
```
python launch_framework.py
```
This actors cannot be destryoed. Please restart the whole simulation after the execution
