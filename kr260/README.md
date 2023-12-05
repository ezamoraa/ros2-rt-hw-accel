# ROS 2 real-time and HW acceleration on Kria KR260

## Dependencies

TODO: Xilinx Vitis installation

## Setup Petalinux build on KR260

- Download and install Petalinux SDK

```bash
PETALINUX_INSTALL=/tools/Xilinx/Petalinux/2023.1/
mkdir -p $PETALINUX_INSTALL
./petalinux-v2023.1-05012318-installer.run -d $PETALINUX_INSTALL
```

- Source Petalinux SDK environment

```bash
source /tools/Xilinx/Petalinux/2023.1/settings.sh
```

- Build Petalinux project

```bash
petalinux-build
```

- Build BOOT.BIN FSBL image (QSPI)

```bash
petalinux-package --boot --u-boot --force
```

- Create WIC image (SD card)

```bash
petalinux-package --wic --wks project-spec/image/rootfs.wks --bootfiles "ramdisk.cpio.gz.u-boot boot.scr Image system.dtb"
```

- Flash WIC to SD card using a tool like Balena Etcher

## Setup ROS 2 workspace on KR260

### Setup development container

```bash
vagrant up
vagrant ssh
```

### Build KR260 host workspace

This is the host native ROS 2 workspace used to prepare the KR260 cross-compilation environment

On the dev container:

```bash
mkdir -p ros_ws
cd ros_ws
mkdir src
# TODO: Update to support ros-acceleration forks
vcs import src --recursive < ../krs.repos
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --merge-install
```

### Build KR260 target workspace

This is the cross-compiled ROS 2 workspace with packages to be run in the KR260 (ARM64 binaries)

On the dev container (ros_ws):

```bash
source install/setup.bash
source /tools/Xilinx/Vitis/2023.1/settings64.sh
export PATH="/usr/bin":$PATH
export LD_PRELOAD=/lib/x86_64-linux-gnu/libudev.so.1
colcon acceleration select kr260
```

```bash
colcon build --build-base=build-kr260 --install-base=install-kr260-ubuntu --merge-install --mixin kr260 --packages-select ament_acceleration ament_vitis vitis_common ros2acceleration offloaded_doublevadd_publisher
```

```bash
# TODO: Check if we still needs this hack
# This is a hack to add libclass_loader dependency to RPATH and build image_transport (manually copied to src from image_common)
cp ./aarch64-xilinx-linux/opt/ros/humble/lib/libclass_loader.so ./aarch64-xilinx-linux/usr/lib/

colcon build --build-base=build-kr260 --install-base=install-kr260-ubuntu --merge-install --mixin kr260 --packages-select ament_acceleration ament_vitis vitis_common ros2acceleration tracetools_image_pipeline image_proc image_transport perception_2nodes perception_3nodes
```

```bash
colcon build --build-base=build-kr260 --install-base=install-kr260-ubuntu --merge-install --mixin kr260 --packages-select ament_acceleration ament_vitis vitis_common ros2acceleration tracetools_image_pipeline image_proc image_transport rt_hw_accel_msgs rt_hw_accel_demo
```

### Set up KR260 target workspace

On the KR260:

To create the path to the workspace installation directory:

```bash
mkdir -p ~/ros_ws/install
```

To copy the workspace to the KR260:

On the dev machine (ros_ws):

```bash
scp -r install-kr260-ubuntu/* root@192.168.1.10:~/ros_ws/install
```

On the KR260:

Hack to fix mismatch between OpenCV Petalinux and acceleration_firmware_kr260 versions:

```bash
ln -s /usr/lib/libopencv_core.so.4.6.0 /usr/lib/libopencv_core.so.4.5d
ln -s /usr/lib/libopencv_imgproc.so.4.6.0 /usr/lib/libopencv_imgproc.so.4.5d
```

To enable the workspace:

```bash
sudo su  # use root to facilitate loading acceleration kernels
source /usr/bin/ros_setup.sh
COLCON_CURRENT_PREFIX=/home/root/ros_ws/install . /home/root/ros_ws/install/local_setup.sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/root/ros_ws/install/lib/sysroot:/home/root/ros_ws/install/lib/sysroot/aarch64-linux-gnu/
```

### Load acceleration kernel

To reload the FPGA apps:

```bash
ros2 acceleration stop; ros2 acceleration start
ros2 acceleration list
```

To run FPGA demo nodes:

```bash
ros2 acceleration select rt_hw_accel_demo
cd /home/root/ros_ws/install/lib/rt_hw_accel_demo/
ros2 run rt_hw_accel_demo fpga_graph
```

<!-- ```bash
ros2 acceleration select offloaded_doublevadd_publisher
cd /home/root/ros_ws/install/lib/offloaded_doublevadd_publisher/
ros2 run offloaded_doublevadd_publisher offloaded_doublevadd_publisher
``` -->

### Set up micro-ROS workspace

Refer to [Micro-ROS first application on Zephyr tutorial](https://micro.ros.org/docs/tutorials/core/first_application_rtos/zephyr/)

On the dev machine (microros_ws):

Create the setup workspace:

<!-- TODO: Automate setup workspace creation and add instructions -->

To create the firmware workspace for the KR260 (same as KV260 in Zephyr):

```bash
ros2 run micro_ros_setup create_firmware_ws.sh zephyr kv260
```

To configure the firmware with the demo application:

```bash
ros2 run micro_ros_setup configure_firmware.sh rt_hw_accel_demo --transport serial
```

To build the configured firmware:

```bash
ros2 run micro_ros_setup build_firmware.sh
```

### Set up micro-ROS agent

To download the micro-ROS agent packages:

```bash
ros2 run micro_ros_setup create_agent_ws.sh
```

To build the micro-ROS agent:

```bash
ros2 run micro_ros_setup build_agent.sh
```

To run the micro-ROS agent with the serial transport to the KR260:

```bash
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1
```

### Run micro-ROS firmware

On the dev machine (ros_ws):

To copy the Zephyr firmware image to the KR260:

```bash
scp firmware/build/zephyr/zephyr.elf root@192.168.1.10:/lib/firmware/
```

On the KR260:

To set the firmware image name in remoteproc (OpenAMP):

```bash
echo zephyr.elf > /sys/class/remoteproc/remoteproc0/firmware
```

To start the firmware in remoteproc:

```bash
echo start > /sys/class/remoteproc/remoteproc0/state
```

To stop the firmware in remoteproc:

```bash
echo stop > /sys/class/remoteproc/remoteproc0/state
```

## Serial console to KR260

```bash
# NOTE: Make sure that the hardware flow control is disabled in minicom
sudo minicom -D /dev/ttyUSB1 -b 115200
```

## Vitis platform update process

<!-- TODO: Document

- Generate HW platform in Vivado (XSA)
- Generate platform in Vitis
- Reconfigure Petalinux project -->

```bash
petalinux-config --get-hw-description <xsa-file-path>
```
