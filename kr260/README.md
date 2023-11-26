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

### Host workspace build (native)

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

### KR260 workspace build (cross-compile)

On the dev container (ros_ws):

```bash
source install/setup.bash
source /tools/Xilinx/Vitis/2023.1/settings64.sh
export PATH="/usr/bin":$PATH
export LD_PRELOAD=/lib/x86_64-linux-gnu/libudev.so.1
colcon acceleration select kr260
colcon build --build-base=build-kr260 --install-base=install-kr260-ubuntu --merge-install --mixin kr260 --packages-select ament_acceleration ament_vitis vitis_common ros2acceleration offloaded_doublevadd_publisher
```

```bash
# TODO: Add image_transport to Petalinux image
# This is a hack to add libclass_loader dependency to RPATH and build image_transport (manually copied to src from image_common)
cp ./aarch64-xilinx-linux/opt/ros/humble/lib/libclass_loader.so ./aarch64-xilinx-linux/usr/lib/

colcon build --build-base=build-kr260 --install-base=install-kr260-ubuntu --merge-install --mixin kr260 --packages-select ament_acceleration ament_vitis vitis_common ros2acceleration tracetools_image_pipeline image_proc image_transport perception_2nodes perception_3nodes
```

### KR260 workspace setup

On the KR260:

```bash
mkdir -p ~/ros_ws/install
```

On the dev machine (ros_ws):

```bash
scp -r install-kr260-ubuntu/* root@192.168.1.10:~/ros_ws/install
# Hack to install missing system libraries (should add to Petalinux image)
rsync --progress -avhe ssh ./firmware/kr260/sysroots/aarch64-xilinx-linux/usr/lib/{libgdal*,libogdi*,libmfhd*,libdfalt*,libarmadillo*,aarch64-linux-gnu/*.so*,aarch64-linux-gnu/blas/libblas*,aarch64-linux-gnu/lapack/liblapack*,../../opt/ros/humble/lib/*} --exclude libc.so*  root@192.168.1.10:~/ros_ws/install/lib/sysroot/
```

On the KR260:

```bash
# Hack to fix symlinks from copied missing system libraries (should add to Petalinux image)
rm /home/root/ros_ws/install/lib/sysroot/liblapack.so.3 /home/root/ros_ws/install/lib/sysroot/libblas.so.3
ln -s /home/root/ros_ws/install/lib/sysroot/libblas.so.3.10.0 /home/root/ros_ws/install/lib/sysroot/libblas.so.3
ln -s /home/root/ros_ws/install/lib/sysroot/liblapack.so.3.10.0 /home/root/ros_ws/install/lib/sysroot/liblapack.so.3
```

```bash
sudo su  # use root to facilitate loading acceleration kernels
source /usr/bin/ros_setup.sh
COLCON_CURRENT_PREFIX=/home/root/ros_ws/install . /home/root/ros_ws/install/local_setup.sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/root/ros_ws/install/lib/sysroot:/home/root/ros_ws/install/lib/sysroot/aarch64-linux-gnu/
```

### Load acceleration kernel

```bash
ros2 acceleration stop; ros2 acceleration start
ros2 acceleration list
```

```bash
ros2 acceleration select offloaded_doublevadd_publisher
cd /home/root/ros_ws/install/lib/offloaded_doublevadd_publisher/
ros2 run offloaded_doublevadd_publisher offloaded_doublevadd_publisher
```

```bash
ros2 acceleration select perception_3nodes
cd /home/root/ros_ws/install/lib/perception_3nodes/
ros2 run perception_3nodes perception_3nodes_fpga
```

```bash
ros2 acceleration select rt_hw_accel_demo
cd /home/root/ros_ws/install/lib/rt_hw_accel_demo/
ros2 run rt_hw_accel_demo fpga_graph
```

## Vitis platform update process

TODO: Document

- Generate HW platform in Vivado (XSA)
- Generate platform in Vitis
- Reconfigure Petalinux project

```bash
petalinux-config --get-hw-description <xsa-file-path>
```
