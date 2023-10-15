# ROS 2 real-time and hardware acceleration on Kria KR260

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

- Flash WIC to SD card

```bash
sudo dd if=images/linux/petalinux-sdimage.wic of=/dev/<sd-card-device> conv=fsync status=progress
```

## Setup ROS 2 workspace on KR260 board

### Host workspace (native build)

```bash
mkdir -p ros_ws
cd ros_ws
mkdir src
vcs import src --recursive < ../krs.repos
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --merge-install
```

### KR260 workspace (cross-compile build)

```bash
source install/setup.bash
source /tools/Xilinx/Vitis/2023.1/settings64.sh
export PATH="/usr/bin":$PATH
export LD_PRELOAD=/lib/x86_64-linux-gnu/libudev.so.1
colcon acceleration select kr260
colcon build --build-base=build-kr260 --install-base=install-kr260-ubuntu --merge-install --mixin kr260 --packages-select ament_acceleration ament_vitis vitis_common ros2acceleration offloaded_doublevadd_publisher
```
