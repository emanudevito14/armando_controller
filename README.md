In ros2_ws/src
```bash
git clone https://github.com/emanudevito14/armando_controller.git
```

In ros2_ws, open a terminal and build the package:
```bash
colcon build --packages-select armando_controller
```
Open a new terminal in ros2_ws and source the setup script:
```bash
source install/setup.bash
```
Open another terminal in ros2_ws and launch the armando_gazebo.launch.py file:
```bash
ros2 launch armando_controller armando_gazebo.launch.py control_type:=position
```
or 
```bash
ros2 launch armando_controller armando_gazebo.launch.py control_type:=trajectory
```
