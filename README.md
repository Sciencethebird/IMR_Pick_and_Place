Panda Robot Arm Pick and Place Demo
===

## About

Pick and Place with 6D pose estimation network (DenseFusion) and Panda robot arm simulator (ROS+GAZEBO)

## Environment
-    Ubuntu 16.04
-    ROS Kinetic
-    Gazebo 7.0
## Installation: Panda Robot Arm Simulator 
-    Basically follow the installation guide on this [repo](https://github.com/justagist/panda_simulator/tree/kinetic-devel)
### Install panda simulator
1.  install libfranka ( [install from source](https://frankaemika.github.io/docs/installation_linux.html#building-from-source) recommended )
2.  franka-ros [v0.6.0]( https://github.com/frankaemika/franka_ros/commit/49e5ac1055e332581b4520a1bd9ac8aaf4580fb1) ( [install from source](https://frankaemika.github.io/docs/installation_linux.html#building-from-source) )
3.  clone this [repo](https://github.com/justagist/panda_simulator/tree/kinetic-devel), make sure using the kinetic-devel branch 
```sh
cd <catkin_ws>/src
git clone https://github.com/justagist/panda_simulator -b kinetic-devel
```
4.  run `./build_ws.sh` from <catkin_ws>/src/panda_simulator
### Install python dependencies

- panda_robot: add this [repo](https://github.com/justagist/panda_robot) to your <catkin_ws> and build it
- quaternion
`pip install numpy-quaternion`
- pyquaternion
`pip install pyquaternion`
- numba (optional)
	```sh
	python -m pip install llvmlite==0.31.0
	pip install numba
	```
- scipy (optional)
    ```sh
    pip install scipy
    ```
### ROS world and launch files
- put IMR_PICK_AND_PLACE.launch under 
`<catkin_ws>/src/panda_simulator/panda_gazebo/launch`
- put IMR_PICK_AND_PLACE.world under your `$GAZEBO_RESOURCE_PATH`
### Some problem you may encounted
1. catkin build problem 
    
    - this repo uses `catkin build` instead of `catkin_make` , you need to delete /build and /devel folder in your <catkin_ws> or simply `catkin clean` if you're originally using catkin_make to build the package.
2. franka-ros problem

    - this repo uses franka-ros 0.6.0, check the version before moving on to further step, you may need to reinstall libfranka, follow [build from source](https://frankaemika.github.io/docs/installation_linux.html#building-the-ros-packages)
    - in the install guide,  use `catkin build` instead of `catkin_make`
        
        ```sh
        rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys libfranka
        catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
        ```
3. libfranka problem
        - problem: apt-get installs a non-working version of libfranka
        - I use [libfranka 0.7.0](https://github.com/frankaemika/libfranka) build from source, make sure the /common folder is not empty before building libfranka


## Demo 1: Simple Pick and Place
- demo goal: using franka robot arm to grab linemod-dataset-objects with kinect_ros camera attach to the gripper of franka robot arm
![](https://i.imgur.com/dWnZgO7.gif)


    1. Start demo world
    	```sh
    	roslaunch panda_gazebo IMR_PICK_AND_PLACE.launch
    	```
    2. Start the object-robot interface
        ```sh
    	python object_interface_server.py
    	```
    3. Attach the camera to the robot arm
		```sh
		python attach_camera.py
        ```
    4. Start camera viewer
        ```sh
        python img_saver.py
        ```   
    5. Run pick and place demo
        ```sh
        python pick_and_place_demo.py
        ```
    
## Demo 2:Pick and Place using 6D pose estimation
![](https://i.imgur.com/O0RjQBr.gif)


## Code Explain


###### tags: `GAZEBO` `ROS` `IMR` `Documentation`
