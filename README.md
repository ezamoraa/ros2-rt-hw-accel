# ROS 2 real-time and hardware acceleration experiments

## Setup ROS 2 workspace on KR260 board

### Host workspace (native)

```bash
mkdir src
vcs import src --recursive < kr260.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --merge-install
```

### KR260 workspace (cross-compiled)

```bash
source install/setup.bash
source /tools/Xilinx/Vitis/2023.1/settings64.sh
export PATH="/usr/bin":$PATH
export LD_PRELOAD=/lib/x86_64-linux-gnu/libudev.so.1
colcon acceleration select kr260
COLCON_DEFAULTS_FILE="" colcon build --build-base=build-kr260 --install-base=install-kr260-ubuntu --merge-install --mixin kr260 --packages-select ament_acceleration ament_vitis vitis_common ros2acceleration offloaded_doublevadd_publisher
```
