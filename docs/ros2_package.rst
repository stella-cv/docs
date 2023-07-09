.. _chapter-ros-package:

============
ROS2 Package
============

.. _section-installation:

Installation
============

.. _section-prerequisites:

Prerequisites
^^^^^^^^^^^^^

Tested for **Ubuntu 20.04**.

Please install the following dependencies.

* ROS2 : ``foxy`` or later. Please follow `Installation of ROS2 <https://index.ros.org/doc/ros2/Installation/>`_.

* `image_tools <https://index.ros.org/p/image_tools/#dashing>`_ : An optional requirement to use USB cameras.

Build Instructions
^^^^^^^^^^^^^^^^^^

(If using Pangolin)

.. code-block:: bash

    sudo apt install -y libglew-dev
    git clone https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    git checkout ad8b5f83
    sed -i -e "193,198d" ./src/utils/file_utils.cpp
    mkdir -p build
    cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_PANGOLIN_DEPTHSENSE=OFF \
        -DBUILD_PANGOLIN_FFMPEG=OFF \
        -DBUILD_PANGOLIN_LIBDC1394=OFF \
        -DBUILD_PANGOLIN_LIBJPEG=OFF \
        -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
        -DBUILD_PANGOLIN_LIBPNG=OFF \
        -DBUILD_PANGOLIN_LIBREALSENSE=OFF \
        -DBUILD_PANGOLIN_LIBREALSENSE2=OFF \
        -DBUILD_PANGOLIN_LIBTIFF=OFF \
        -DBUILD_PANGOLIN_LIBUVC=OFF \
        -DBUILD_PANGOLIN_LZ4=OFF \
        -DBUILD_PANGOLIN_OPENNI=OFF \
        -DBUILD_PANGOLIN_OPENNI2=OFF \
        -DBUILD_PANGOLIN_PLEORA=OFF \
        -DBUILD_PANGOLIN_PYTHON=OFF \
        -DBUILD_PANGOLIN_TELICAM=OFF \
        -DBUILD_PANGOLIN_TOON=OFF \
        -DBUILD_PANGOLIN_UVC_MEDIAFOUNDATION=OFF \
        -DBUILD_PANGOLIN_V4L=OFF \
        -DBUILD_PANGOLIN_VIDEO=OFF \
        -DBUILD_PANGOLIN_ZSTD=OFF \
        -DBUILD_PYPANGOLIN_MODULE=OFF \
        ..
    make -j$(($(nproc) / 2))
    make install

(If using SocketViewer)

.. code-block:: bash

    sudo apt install -y autogen autoconf libtool
    cd /tmp
    git clone https://github.com/shinsumicco/socket.io-client-cpp.git
    cd socket.io-client-cpp
    git submodule init
    git submodule update
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_UNIT_TESTS=OFF \
        ..
    make -j4
    sudo make install
    sudo apt install -y libprotobuf-dev protobuf-compiler
    wget -q https://github.com/google/protobuf/archive/v3.6.1.tar.gz
    tar xf v3.6.1.tar.gz
    cd protobuf-3.6.1
    ./autogen.sh
    ./configure \
        --prefix=/usr/local \
        --enable-static=no
    make -j4
    sudo make install

.. code-block:: bash

    rosdep update
    sudo apt update
    mkdir -p ~/lib
    cd ~/lib
    git clone --recursive --depth 1 https://github.com/stella-cv/stella_vslam.git
    rosdep install -y -i --from-paths ~/lib
    cd ~/lib/stella_vslam
    mkdir -p ~/lib/stella_vslam/build
    cd ~/lib/stella_vslam/build
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
    make -j
    sudo make install

    # When building with support for PangolinViewer
    cd ~/lib
    git clone -b 0.0.1 --recursive https://github.com/stella-cv/pangolin_viewer.git
    mkdir -p pangolin_viewer/build
    cd pangolin_viewer/build
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
    make -j
    sudo make install

    # When building with support for SocketViewer
    cd ~/lib
    git clone -b 0.0.1 --recursive https://github.com/stella-cv/socket_publisher.git
    mkdir -p socket_publisher/build
    cd socket_publisher/build
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
    make -j
    sudo make install

    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone --recursive -b ros2 --depth 1 https://github.com/stella-cv/stella_vslam_ros.git
    cd ~/ros2_ws
    rosdep install -y -i --from-paths ~/ros2_ws/src --skip-keys=stella_vslam
    colcon build --symlink-install

For using USB cam as a image source, download a repository of ``demos`` and pick ``image_tools`` module.

.. code-block:: bash

    cd ~/ros2_ws
    git clone https://github.com/ros2/demos.git
    cp -r demos/image_tools src/
    rm -rf demos

Examples
========

Publisher
^^^^^^^^^

Publish Images by a video
-------------------------

.. code-block:: bash

    ros2 run image_publisher image_publisher_node ./aist_living_lab_1/video.mp4 --ros-args --remap /image_raw:=/camera/image_raw

Publish Images Captured by a USB Camera
---------------------------------------

For using a standard USB camera for visual SLAM or localization.

.. code-block:: bash

    ros2 run image_tools cam2image

Republish the ROS topic to ``/camera/image_raw``.

.. code-block:: bash

    ros2 run image_transport republish \
        raw in:=image raw out:=/camera/image_raw

Subscriber
^^^^^^^^^^

Subscribers continually receive images.
Please execute one of the following command snippets in the new terminal.

.. NOTE ::

    Option arguments are the same as :ref:`the examples of stella_vslam <chapter-example>`.

Tracking and Mapping
--------------------

We provide an example snippet for visual SLAM.
The source code is placed at ``stella_vslam_ros/src/run_slam.cc``.

.. code-block:: bash

    source ~/ros2_ws/install/setup.bash
    ros2 run stella_vslam_ros run_slam \
        -v /path/to/orb_vocab.fbow \
        -c /path/to/config.yaml \
        --map-db-out /path/to/map.msg \
        --ros-args -p publish_tf:=false

Localization
------------

We provide an example snippet for localization based on a prebuilt map.
The source code is placed at ``stella_vslam_ros/src/run_slam.cc``.

.. code-block:: bash

    source ~/ros2_ws/install/setup.bash
    ros2 run stella_vslam_ros run_slam \
        --disable-mapping \
        -v /path/to/orb_vocab.fbow \
        -c /path/to/config.yaml \
        --map-db-in /path/to/map.msg \
        --ros-args -p publish_tf:=false

.. _section-offline-slam:

Offline SLAM
------------

We provide an example snippet for localization based on a prebuilt map.
The source code is placed at ``stella_vslam_ros/src/run_slam.cc``.

.. code-block:: bash

    source ~/ros2_ws/install/setup.bash
    ros2 run stella_vslam_ros run_slam_offline \
        -b /path/to/bagfile.bag2 \
        -v /path/to/orb_vocab.fbow \
        -c /path/to/config.yaml \
        -o /path/to/map.msg \
        --camera=your_camera_topic_namespace \
        --storage-id=sqlite3 \
        --ros-args -p publish_tf:=false
