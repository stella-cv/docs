.. _chapter-trouble-shooting:

================
Trouble Shooting
================


.. _section-trouble-build:

For building
============

#. stella_vslam terminates abnormaly soon after **launching** or **optimization with g2o**.

    Please configure and rebuild g2o and stella_vslam with ``-DBUILD_WITH_MARCH_NATIVE=OFF`` option for ``cmake``.

.. _section-trouble-viewer:

For Viewer
==========

#.  (on Docker) Additional option ``--gpus all`` is needed if you use NVIDIA graphics card(s).  
    Please see `here <https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(Native-GPU-Support)#usage>`_ for more details.
#.  If the viewer does not work, try :ref:`the docker images for SocketViewer <section-instructions-for-socketviewer>` instead.


.. _section-trouble-slam:

For SLAM
========
