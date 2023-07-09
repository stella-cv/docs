.. _chapter-ros-package:

===========
ROS Package
===========

.. _section-installation:

Installation
============

.. _section-prerequisites:

Prerequisites
^^^^^^^^^^^^^

Tested for **Ubuntu 20.04**.

Please install the following dependencies.

* ROS : ``noetic``. Please follow `Installation of ROS <http://wiki.ros.org/ROS/Installation>`_.

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
    sudo make install

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

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone --recursive -b ros --depth 1 https://github.com/stella-cv/stella_vslam_ros.git
    cd ~/catkin_ws/
    rosdep install -y -i --from-paths ~/catkin_ws/src --skip-keys=stella_vslam
    catkin_make -j

Examples
========

Run the core program required for ROS-based system in advance.

.. code-block:: bash

    roscore

.. NOTE ::

    Please leave the **roscore** run.

Publisher
^^^^^^^^^

Publish Images by a video
-------------------------

.. code-block:: bash

    rosrun image_publisher image_publisher ./aist_living_lab_1/video.mp4 /image_raw:=/camera/image_raw

Publish Images of a USB Camera
------------------------------

For using a standard USB camera for visual SLAM or localization.

.. code-block:: bash

    apt install ros-${ROS_DISTRO}-usb-cam

.. code-block:: bash

    rosparam set usb_cam/pixel_format yuyv
    rosrun usb_cam usb_cam_node

Republish the ROS topic to ``/camera/image_raw``.

.. code-block:: bash

    rosrun image_transport republish \
        raw in:=/usb_cam/image_raw raw out:=/camera/image_raw

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

    source ~/catkin_ws/devel/setup.bash
    rosrun stella_vslam_ros run_slam \
        -v /path/to/orb_vocab.fbow \
        -c /path/to/config.yaml \
        --map-db-out /path/to/map.msg

Localization
------------

We provide an example snippet for localization based on a prebuilt map.
The source code is placed at ``stella_vslam_ros/src/run_slam.cc``.

.. code-block:: bash

    source ~/catkin_ws/devel/setup.bash
    rosrun stella_vslam_ros run_slam \
        --disable-mapping \
        -v /path/to/orb_vocab.fbow \
        -c /path/to/config.yaml \
        --map-db-in /path/to/map.msg
