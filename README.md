### run in ros2 humble and ubuntu22.04


# Clone the repository and cd the workspace
open a cmd
```bash
git clone https://github.com/woody63/plant_pot.git
```
```bash
cd ~/plant_pot
```

# Install ROS 2 dependencies via rosdep


```bash
rosdep install --from-paths src --ignore-src -r -y
```


# Build the specific package
```bash
colcon build --symlink-install 
```


# Source the newly built workspace
```bash
source install/setup.bash
```


# Run the gazebo&rviz2 node 
```bash
ros2 launch plant spawn_turntable.launch.py
```
# Run the capture_images node 
open a new cmd window
```bash
cd ~/plant_pot
```
```bash
source install/setup.bash
```
```bash
ros2 run plant image_capturer_node
```
