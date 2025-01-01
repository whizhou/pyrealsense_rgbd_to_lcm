# Republish RGBD Data from Realsense to LCM

+ 本项目通过 python 代码直接调用 realsense 深度相机，获取 RGB 帧和 Depth帧

+ 将 RGBD 数据打包为 LabelFusion 所需要格式的 lcm 消息并发布到 `OPENNI_FRAME` 通道

+ 使用 `lcm-logger` 收集原始数据，用于 LabelFusion 6D物体位姿标注

## Environmen Setup

### 一、Realsense 深度相机配置

1. 安装 librealsense 从而使用 `realsense-viewer` 检测相机

    ```shell
    sudo mkdir -p /etc/apt/keyrings
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list
    sudo apt-get update

    sudo apt-get install librealsense2-dkms
    sudo apt-get install librealsense2-utils
    sudo apt-get install librealsense2-dev
    sudo apt-get install librealsense2-dbg
    ```

    打开终端，运行 `realsense-viewer` 命令检测相机。

2. Python Packages

    ```shell
    pip install pyrealsense2
    ```

### 二、安装 LCM (via pip)

LCM can be installed via the Python package manager (pip) on many systems. To do so, run:

```shell
pip3 install lcm
```

This package contains:

+ The LCM Python module

+ LCM executables (for example, lcm-logplayer)

    Note: Java-based executables (like lcm-logplayer-gui) are not included for musl-based linux distributions

+ Development files (headers and libraries)

Note: this package has a hard runtime dependency on GLib 2.0. If you have not already, please install this dependency before using the Python package.

### 三、安装 Python Packages

```shell
pip install numpy opencv-python
```

## 收集原始数据

1. 运行 `scripts/lcm_republisher.py`，启动相机发布 lcm 消息

2. 启动终端运行以下命令：

    ```shell
    cd /path/to/data-folder
    lcm-logger
    ```

    Ctrl + C 结束记录日志

## 物体 6D 位姿标注

1. Install [Nvidia Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit): [install-guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

2. `git clone https://github.com/Y-pandaman/labelfusion-cuda11.3.git`

3. `LabelFusion/docker/docker_run.sh /path/to/data-folder`

    > The `docker_run.sh` script calls `nvidia-docker` to start the LabelFusion Docker container with an interactive bash session. The first time it runs the LabelFusion image will be downloaded from DockerHub automatically.

    命令会将本地的 `LabelFusion` 源目录和数据目录（如果提供了路径）挂载为 Docker 内的卷。Docker 容器内的路径为：

    ~/labelfusion <-- the mounted LabelFusion directory

    ~/labelfusion/data <-- the mounted data directory

4. 数据标注流程参考 [Quick Pipeline Instructions](https://github.com/RobotLocomotion/LabelFusion?tab=readme-ov-file#quick-pipeline-instructions-for-making-new-labeled-data-with-labelfusion)

    请仔细阅读 [Data Organization](https://github.com/RobotLocomotion/LabelFusion/blob/master/docs/data_organization.rst) 参考文档，其中提供了 Object meshes 的存放位置及所需信息 object_data.yaml 的组织方式。

    请注意 lcmlog 的存放位置：

    ```
    ~/labelfusion/data/logs/
        logs/
            2025-01-01-01/
                original_log.lcmlog
    ```

    缺少子文件夹可能导致运行命令 `run_alignment_tool` 时找不到所需文件。
