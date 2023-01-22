.. _chapter-installation:

============
Installation
============


.. _section-get-source:

Source code
===========

The source code can be viewed from this `GitHub repository <https://github.com/stella-cv/stella_vslam>`_.

Cloning the repository:

.. code-block:: bash

       git clone --recursive https://github.com/stella-cv/stella_vslam.git

If you are Windows 10 user, please install the dependencies and stella_vslam with :ref:`SocketViewer support <subsection-dependencies-socketviewer>` on `Windows Subsystem for Linux (WSL) <https://en.wikipedia.org/wiki/Windows_Subsystem_for_Linux>`__.

:ref:`Docker <chapter-docker>` systems can be used instead of preparing the dependencies manually.

.. _section-dependencies:

Dependencies
============

stella_vslam requires a **C++11-compliant** compiler.
It relies on several open-source libraries as shown below.

Requirements for stella_vslam
^^^^^^^^^^^^^^^^^^^^^^^^^^

* `Eigen <http://eigen.tuxfamily.org/>`_ : version 3.3.0 or later.

* `g2o <https://github.com/RainerKuemmerle/g2o>`_ : Please use the latest release. Tested on commit ID `ed40a5b <https://github.com/RainerKuemmerle/g2o/tree/ed40a5bb028566fd56a78fd7b04921b613492d6f>`_.

* `SuiteSparse <http://faculty.cse.tamu.edu/davis/suitesparse.html>`_ : Required by g2o.

* `FBoW <https://github.com/stella-cv/FBoW>`_ : **Please use the custom version of FBoW** released in `https://github.com/stella-cv/FBoW <https://github.com/stella-cv/FBoW>`_.

* `yaml-cpp <https://github.com/jbeder/yaml-cpp>`_ : version 0.6.0 or later.

* `OpenCV <https://opencv.org/>`_ : version 3.3.1 or later.

.. NOTE ::

    OpenCV with GUI support is necessary for using the built-in viewer (Pangolin Viewer).
    OpenCV with video support is necessary if you plan on using video files (e.g. ``.mp4``) as inputs.
    If your CPU has many cores, it is recommended to enable TBB.

Requirements for PangolinViewer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| We provided an OpenGL-based simple viewer.
| This viewer is implemented with `Pangolin <https://github.com/stevenlovegrove/Pangolin>`_. Thus, we call it **PangolinViewer**.
| Please install the following dependencies if you plan on using PangolinViewer.

* `Pangolin <https://github.com/stevenlovegrove/Pangolin>`_ : Please use the latest release. Tested on commit ID `eab3d34 <https://github.com/stevenlovegrove/Pangolin/tree/eab3d3449a33a042b1ee7225e1b8b593b1b21e3e>`_.

* `GLEW <http://glew.sourceforge.net/>`_ : Required by Pangolin.

.. NOTE ::

    If Pangolin version 0.7 or higher, C++17 is required.

.. _subsection-dependencies-socketviewer:

Requirements for SocketViewer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| We provided an WebGL-based simple viewer running on web browsers.
| The SLAM systems publish the map and the frame to the server implemented with `Node.js <https://nodejs.org/>`_ via WebSocket. Thus, we call it **SocketViewer**.
| Please install the following dependencies if you plan on using SocketViewer.

* `socket.io-client-cpp <https://github.com/shinsumicco/socket.io-client-cpp>`_ : **Please use the custom version of socket.io-client-cpp** released in `https://github.com/shinsumicco/socket.io-client-cpp <https://github.com/shinsumicco/socket.io-client-cpp>`_.

* `Protobuf <https://github.com/protocolbuffers/protobuf>`_ : version 3 or later.

The following libraries are the dependencies for the server.

* `Node.js <https://nodejs.org/>`_ : version 6 or later.

* `npm <https://www.npmjs.com/>`_ : Tested on version 3.5.2.

Recommended
^^^^^^^^^^^

* `backward-cpp <https://github.com/bombela/backward-cpp>`_ : Used for stack-trace logger.


.. _section-prerequisites-unix:

Prerequisites for Unix
======================

.. NOTE ::

    If your PC is frozen during the build, please reduce the number of parallel compile jobs when executing ``make`` (e.g. ``make -j2``).

.. _section-linux:

Installing for Linux
^^^^^^^^^^^^^^^^^^^^

Tested for **Ubuntu 18.04**.

Install the dependencies via ``apt``.

.. code-block:: bash

    apt update -y
    apt upgrade -y --no-install-recommends
    # basic dependencies
    apt install -y build-essential pkg-config cmake git wget curl unzip
    # g2o dependencies
    apt install -y libatlas-base-dev libsuitesparse-dev
    # OpenCV dependencies
    apt install -y libgtk-3-dev ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavresample-dev libtbb-dev
    # eigen dependencies
    apt install -y gfortran
    # backward-cpp dependencies (optional)
    apt install -y binutils-dev
    # other dependencies
    apt install -y libyaml-cpp-dev libgflags-dev sqlite3 libsqlite3-dev

    # (if you plan on using PangolinViewer)
    # Pangolin dependencies
    apt install -y libglew-dev

    # (if you plan on using SocketViewer)
    # Protobuf dependencies
    apt install -y autogen autoconf libtool
    # Node.js
    curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
    apt install -y nodejs

Download and install Eigen from source.

.. code-block:: bash

    cd /path/to/working/dir
    wget -q https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.bz2
    tar xf eigen-3.3.7.tar.bz2 && rm -rf eigen-3.3.7.tar.bz2
    cd eigen-3.3.7
    mkdir -p build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        ..
    make -j4 && make install

Download, build and install OpenCV from source.

.. code-block:: bash

    cd /path/to/working/dir
    # Download OpenCV
    wget -q https://github.com/opencv/opencv/archive/4.5.5.zip
    unzip -q 4.5.5.zip && rm -rf 4.5.5.zip
    # Download aruco module (optional)
    wget -q https://github.com/opencv/opencv_contrib/archive/refs/tags/4.5.5.zip -O opencv_contrib-4.5.5.zip
    unzip -q opencv_contrib-4.5.5.zip && rm -rf opencv_contrib-4.5.5.zip
    mkdir -p extra && mv opencv_contrib-4.5.5/modules/aruco extra
    rm -rf opencv_contrib-4.5.5
    # Build and install OpenCV
    cd opencv-4.5.5
    mkdir -p build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_DOCS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_JASPER=OFF \
        -DBUILD_OPENEXR=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_TESTS=OFF \
        -DBUILD_PROTOBUF=OFF \
        -DBUILD_opencv_apps=OFF \
        -DBUILD_opencv_dnn=OFF \
        -DBUILD_opencv_ml=OFF \
        -DBUILD_opencv_python_bindings_generator=OFF \
        -DENABLE_CXX11=ON \
        -DENABLE_FAST_MATH=ON \
        -DWITH_EIGEN=ON \
        -DWITH_FFMPEG=ON \
        -DWITH_TBB=ON \
        -DWITH_OPENMP=ON \
        -DOPENCV_EXTRA_MODULES_PATH=/tmp/extra \
        ..
    make -j4 && make install

Jump to :ref:`Common Installation Instructions <subsection-common-linux-macos>` for the next step.

.. _section-macos:

Installing for macOS
^^^^^^^^^^^^^^^^^^^^

Tested for **macOS High Sierra**.

Install the dependencies via ``brew``.

.. code-block:: bash

    brew update
    # basic dependencies
    brew install pkg-config cmake git
    # g2o dependencies
    brew install suite-sparse
    # OpenCV dependencies and OpenCV
    brew install eigen ffmpeg opencv
    # other dependencies
    brew install yaml-cpp glog gflags sqlite3

    # (if you plan on using PangolinViewer)
    # Pangolin dependencies
    brew install glew

    # (if you plan on using SocketViewer)
    # Protobuf dependencies
    brew install automake autoconf libtool
    # Node.js
    brew install node

Jump to :ref:`Common Installation Instructions <subsection-common-linux-macos>` for the next step.

.. _subsection-common-linux-macos:

Common Installation Instructions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Download, build and install **the custom FBoW** from source.

.. code-block:: bash

    cd /path/to/working/dir
    git clone https://github.com/stella-cv/FBoW.git
    cd FBoW
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        ..
    make -j4 && make install

Download, build and install g2o.

.. code-block:: bash

    cd /path/to/working/dir
    git clone https://github.com/RainerKuemmerle/g2o.git
    cd g2o
    git checkout ed40a5bb028566fd56a78fd7b04921b613492d6f
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_UNITTESTS=OFF \
        -DG2O_USE_CHOLMOD=OFF \
        -DG2O_USE_CSPARSE=ON \
        -DG2O_USE_OPENGL=OFF \
        -DG2O_USE_OPENMP=OFF \
        -DG2O_BUILD_APPS=OFF \
        -DG2O_BUILD_EXAMPLES=OFF \
        -DG2O_BUILD_LINKED_APPS=OFF \
        ..
    make -j4 && make install

| (**if you plan on using PangolinViewer**)
| Download, build and install Pangolin from source.

.. code-block:: bash

    cd /path/to/working/dir
    git clone https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    git checkout eab3d3449a33a042b1ee7225e1b8b593b1b21e3e
    mkdir build && cd build
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_PANGOLIN_DEPTHSENSE=OFF \
        -DBUILD_PANGOLIN_FFMPEG=OFF \
        -DBUILD_PANGOLIN_LIBDC1394=OFF \
        -DBUILD_PANGOLIN_LIBJPEG=OFF \
        -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
        -DBUILD_PANGOLIN_LIBPNG=OFF \
        -DBUILD_PANGOLIN_LIBTIFF=OFF \
        -DBUILD_PANGOLIN_LIBUVC=OFF \
        -DBUILD_PANGOLIN_LZ4=OFF \
        -DBUILD_PANGOLIN_OPENNI=OFF \
        -DBUILD_PANGOLIN_OPENNI2=OFF \
        -DBUILD_PANGOLIN_PLEORA=OFF \
        -DBUILD_PANGOLIN_PYTHON=OFF \
        -DBUILD_PANGOLIN_TELICAM=OFF \
        -DBUILD_PANGOLIN_UVC_MEDIAFOUNDATION=OFF \
        -DBUILD_PANGOLIN_V4L=OFF \
        -DBUILD_PANGOLIN_ZSTD=OFF \
        ..
    make -j4 && make install

| (**if you plan on using SocketViewer**)
| Download, build and install socket.io-client-cpp from source.

.. code-block:: bash

    cd /path/to/working/dir
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
    make install

| (**if you plan on using SocketViewer**)
| Install Protobuf.

If you use Ubuntu 18.04 or macOS, Protobuf 3.x can be installed via ``apt`` or ``brew``.

.. code-block:: bash

    # for Ubuntu 18.04 (or later)
    apt install -y libprotobuf-dev protobuf-compiler
    # for macOS
    brew install protobuf

Otherwise, please download, build and install Protobuf from source.

.. code-block:: bash

    wget -q https://github.com/google/protobuf/archive/v3.6.1.tar.gz
    tar xf v3.6.1.tar.gz
    cd protobuf-3.6.1
    ./autogen.sh
    ./configure \
        --prefix=/usr/local \
        --enable-static=no
    make -j4
    make install

.. _section-build-unix:

Build Instructions
==================

When building with support for PangolinViewer, please specify the following cmake options: ``-DUSE_PANGOLIN_VIEWER=ON`` and ``-DUSE_SOCKET_PUBLISHER=OFF``.

.. code-block:: bash

    cd /path/to/stella_vslam
    mkdir build && cd build
    cmake \
        -DUSE_STACK_TRACE_LOGGER=ON \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DUSE_PANGOLIN_VIEWER=ON \
        -DINSTALL_PANGOLIN_VIEWER=ON \
        -DUSE_SOCKET_PUBLISHER=OFF \
        -DBUILD_TESTS=OFF \
        -DBUILD_EXAMPLES=ON \
        ..
    make -j4 && make install

When building with support for SocketViewer, please specify the following cmake options: ``-DUSE_PANGOLIN_VIEWER=OFF`` and ``-DUSE_SOCKET_PUBLISHER=ON``.

.. code-block:: bash

    cd /path/to/stella_vslam
    mkdir build && cd build
    cmake \
        -DUSE_STACK_TRACE_LOGGER=ON \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DUSE_PANGOLIN_VIEWER=OFF \
        -DUSE_SOCKET_PUBLISHER=ON \
        -DINSTALL_SOCKET_PUBLISHER=ON \
        -DBUILD_TESTS=OFF \
        -DBUILD_EXAMPLES=ON \
        ..
    make -j4 && make install

After building, check to see if it was successfully built by executing ``./run_kitti_slam -h``.

.. code-block:: bash

    $ ./run_kitti_slam -h
    Allowed options:
    -h, --help               produce help message
    -v, --vocab arg          vocabulary file path
    -d, --data-dir arg       directory path which contains dataset
    -c, --config arg         config file path
    --frame-skip arg (=1)    interval of frame skip
    --no-sleep               not wait for next frame in real time
    --auto-term              automatically terminate the viewer
    --log-level arg (=info)  log level


.. _section-viewer-setup:

Server Setup for SocketViewer
=============================

If you plan on using SocketViewer, please setup the environment for the server with ``npm``.

.. code-block:: bash

    $ cd /path/to/stella_vslam/viewer
    $ ls
    Dockerfile  app.js  package.json  public  views
    $ npm install
    added 88 packages from 60 contributors and audited 204 packages in 2.105s
    found 0 vulnerabilities
    $ ls
    Dockerfile  app.js  node_modules  package-lock.json  package.json  public  views

Then, launch the server with ``node app.js``.

.. code-block:: bash

    $ cd /path/to/stella_vslam/viewer
    $ ls
    Dockerfile  app.js  node_modules  package-lock.json  package.json  public  views
    $ node app.js
    WebSocket: listening on *:3000
    HTTP server: listening on *:3001

After launching, please access to ``http://localhost:3001/`` to check whether the server is correctly launched.

.. image:: ./img/browser_viewer_default.png
    :width: 800px
    :align: center

.. NOTE ::

    When you try :ref:`the tutotial <chapter-simple-tutorial>` and :ref:`the examples <chapter-example>` with SocketViewer, please launch the server in the other terminal and access to it with the web browser **in advance**.
