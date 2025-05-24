.. _chapter-docker:

=================
Running on Docker
=================


.. _section-instructions-for-iridescenceviewer:

Instructions for IridescenceViewer
==================================

``Dockerfile.iridescense`` can be used for easy installation.
This chapter provides instructions on building and running examples with IridescenceViewer support using Docker.

The instructions are tested on Ubuntu 20.04.

Building Docker Image
^^^^^^^^^^^^^^^^^^^^^

Execute the following commands:

.. code-block:: bash

    git clone --recursive https://github.com/stella-cv/stella_vslam.git
    cd stella_vslam
    docker build -t stella_vslam-iridescence -f Dockerfile.iridescence .


You can accelerate the build of the docker image with ``--build-arg NUM_THREADS=<number of parallel builds>`` option. For example:

.. code-block:: bash

    # building the docker image with four threads
    docker build -t stella_vslam-iridescense -f Dockerfile.iridescense . --build-arg NUM_THREADS=`expr $(nproc) - 1`

Starting Docker Container
^^^^^^^^^^^^^^^^^^^^^^^^^

In order to enable X11 forwarding, supplemental options (``-e DISPLAY=$DISPLAY`` and ``-v /tmp/.X11-unix/:/tmp/.X11-unix:ro``) are needed for ``docker run``.

.. code-block:: bash

    # before launching the container, allow display access from local users
    xhost +local:
    # launch the container
    docker run -it --rm --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro stella_vslam-iridescense

.. NOTE ::

    Additional option ``--gpus all`` is needed if you use NVIDIA graphics card(s).


After launching the container, the shell interface will be launched in the docker container.
See :ref:`Tutorial <chapter-simple-tutorial>` to run SLAM examples in the container.

If you need to access to any files and directories on a host machine from the container, :ref:`bind directories <section-directory-binding>` between the host and the container.


.. _section-instructions-for-pangolinviewer:

Instructions for PangolinViewer
===============================

``Dockerfile.desktop`` can be used for easy installation.
This chapter provides instructions on building and running examples with PangolinViewer support using Docker.

The instructions are tested on Ubuntu 20.04.
Docker for Mac are NOT supported due to OpenGL forwarding. Please :ref:`install the dependencies manually <chapter-installation>` or use :ref:`the docker images for SocketViewer <section-instructions-for-socketviewer>`.

.. NOTE ::
    If you plan on using a machine with NVIDIA graphics card(s), install nvidia-docker2 and the version 390 or later of NVIDIA driver.

Building Docker Image
^^^^^^^^^^^^^^^^^^^^^

Execute the following commands:

.. code-block:: bash

    git clone --recursive https://github.com/stella-cv/stella_vslam.git
    cd stella_vslam
    docker build -t stella_vslam-desktop -f Dockerfile.desktop .


You can accelerate the build of the docker image with ``--build-arg NUM_THREADS=<number of parallel builds>`` option. For example:

.. code-block:: bash

    # building the docker image with four threads
    docker build -t stella_vslam-desktop -f Dockerfile.desktop . --build-arg NUM_THREADS=`expr $(nproc) - 1`

Starting Docker Container
^^^^^^^^^^^^^^^^^^^^^^^^^

In order to enable X11 forwarding, supplemental options (``-e DISPLAY=$DISPLAY`` and ``-v /tmp/.X11-unix/:/tmp/.X11-unix:ro``) are needed for ``docker run``.

.. code-block:: bash

    # before launching the container, allow display access from local users
    xhost +local:
    # launch the container
    docker run -it --rm --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro stella_vslam-desktop

.. NOTE ::

    Additional option ``--gpus all`` is needed if you use NVIDIA graphics card(s).


After launching the container, the shell interface will be launched in the docker container.
See :ref:`Tutorial <chapter-simple-tutorial>` to run SLAM examples in the container.

.. NOTE ::

    If the viewer does not work, see :ref:`Trouble Shooting <section-trouble-viewer>`

If you need to access to any files and directories on a host machine from the container, :ref:`bind directories <section-directory-binding>` between the host and the container.


.. _section-instructions-for-socketviewer:

Instructions for SocketViewer
=============================

``Dockerfile.socket`` and ``Dockerfile`` in `socket_viewer <https://github.com/stella-cv/socket_viewer>`__ can be used for easy installation.
This chapter provides instructions on building and running examples with SocketViewer support using Docker.

Building Docker Images
^^^^^^^^^^^^^^^^^^^^^^

Docker Image of stella_vslam
`````````````````````````

Execute the following commands:

.. code-block:: bash

    cd /path/to/stella_vslam
    docker build -t stella_vslam-socket -f Dockerfile.socket .


You can accelerate the build of the docker image with ``--build-arg NUM_THREADS=<number of parallel builds>`` option. For example:

.. code-block:: bash

    # building the docker image with four threads
    docker build -t stella_vslam-socket -f Dockerfile.socket . --build-arg NUM_THREADS=`expr $(nproc) - 1`

Docker Image of Server
``````````````````````

Execute the following commands:

.. code-block:: bash

    git clone --recursive https://github.com/stella-cv/socket_viewer.git
    cd socket_viewer
    docker build -t stella_vslam-viewer .

Starting Docker Containers
^^^^^^^^^^^^^^^^^^^^^^^^^^

On Linux
`````````````````````

Launch the server container and access to it with the web browser in advance.
Please specify ``--net=host`` in order to share the network with the host machine.

.. code-block:: bash

    $ docker run --rm -it --name stella_vslam-viewer --net=host stella_vslam-viewer
    WebSocket: listening on *:3000
    HTTP server: listening on *:3001

After launching, access to ``http://localhost:3001/`` with the web browser.

Next, launch the container of stella_vslam.
The shell interface will be launched in the docker container.

.. code-block:: bash

    $ docker run --rm -it --name stella_vslam-socket --net=host stella_vslam-socket
    root@hostname:/stella_vslam_examples/build#

See :ref:`Tutorial <chapter-simple-tutorial>` to run SLAM examples in the container.

If you need to access to any files and directories on a host machine from the container, :ref:`bind directories <section-directory-binding>` between the host and the container.

On macOS
`````````````````````

Launch the server container and access to it with the web browser in advance.
Please specify ``-p 3001:3001`` for port-forwarding.

.. code-block:: bash

    $ docker run --rm -it --name stella_vslam-viewer -p 3001:3001 stella_vslam-viewer
    WebSocket: listening on *:3000
    HTTP server: listening on *:3001

After launching, access to ``http://localhost:3001/`` with the web browser.

Then, inspect the container's IP address and append the ``SocketPublisher.server_uri`` entry to the YAML config file of stella_vslam.

.. code-block:: bash

    # inspect the server's IP address
    $ docker inspect stella_vslam-viewer | grep -m 1 \"IPAddress\" | sed 's/ //g' | sed 's/,//g'
    "IPAddress": "172.17.0.2"

.. code-block:: yaml

    # config file of stella_vslam

    ...

    #============================#
    # SocketPublisher Parameters #
    #============================#

    # append this entry
    SocketPublisher.server_uri: "http://172.17.0.2:3000"

Next, launch the container of stella_vslam.
The shell interface will be launched in the docker container.

.. code-block:: bash

    $ docker run --rm -it --name stella_vslam-socket stella_vslam-socket
    root@hostname:/stella_vslam_examples/build#

| See :ref:`Tutorial <chapter-simple-tutorial>` to run SLAM examples in the container.
| Please don't forget to append ``SocketPublisher.server_uri`` entry to the ``config.yaml`` if you use the downloaded datasets in the tutorial.

If you need to access to any files and directories on a host machine from the container, :ref:`bind directories <section-directory-binding>` between the host and the container.

.. _section-directory-binding:

Bind of Directories
===================

If you need to access to any files and directories on a host machine from the container, bind directories between the host and the container using ``--volume`` or ``--mount`` option.
(See `the docker documentataion <https://docs.docker.com/engine/reference/commandline/run/>`_.)

For example:

.. code-block:: bash

    # launch a container of stella_vslam-desktop with --volume option
    $ docker run -it --rm --runtime=nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro \
        --volume /path/to/dataset/dir/:/dataset:ro \
        --volume /path/to/vocab/dir:/vocab:ro \
        stella_vslam-desktop
    # dataset/ and vocab/ are found at the root directory in the container
    root@0c0c9f115d74:/# ls /
    ...   dataset/   vocab/   ...

.. code-block:: bash

    # launch a container of stella_vslam-socket with --volume option
    $ docker run --rm -it --name stella_vslam-socket --net=host \
        --volume /path/to/dataset/dir/:/dataset:ro \
        --volume /path/to/vocab/dir:/vocab:ro \
        stella_vslam-socket
    # dataset/ and vocab/ are found at the root directory in the container
    root@0c0c9f115d74:/# ls /
    ...   dataset/   vocab/   ...
