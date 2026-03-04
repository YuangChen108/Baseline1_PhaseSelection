# ReadMe



## 1. Simulation of Aerial Tracking and Landing

**[NOTE]** You can choose use CUDA or not
Use CUDA:
- set "use_depth" true in `src/mapping/launch/mapping.launch` 
- set "ENABLE_CUDA" true in `src/uav_simulator/local_sensing/CMakeLists.txt`


>Preparation and visualization:
```
cd Elastic-Tracker
catkin_make
source devel/setup.zsh
chmod +x sh_utils/pub_triger.sh
chmod +x sh_utils/land_triger.sh
roslaunch mapping rviz_sim.launch
```

>A small drone with the global map as the chasing target:
```
roslaunch fake_planning fake_boat_target.launch
```

>Start the elastic tracker:
```
roslaunch planning simulation_tl.launch
```

> Triger the drone to track and land on the moving vehicle:
```
./sh_utils/auto_land.sh
```
<p align="center">
    <img src="figs/sim_landing.gif" width="500"/>
</p>

## 2. Config
- **Change obstacle number**
  In 'src/uav_simulator/uav_simulator/config/mockamap.yaml', change obstacle_number
- **Change drone initial position**
  In 'src/uav_simulator/uav_simulator/launch/uav_simulator.launch', change init_x_, init_y_, init_z_
- **Change landing attitude**
  In 'src/planning/planning/config/config.yaml', change land_pitch(=-0.6)

## 3. Acknowledgement
We use [**MINCO**](https://github.com/ZJU-FAST-Lab/GCOPTER) as our trajectory representation.