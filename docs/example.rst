.. _chapter-example:

=======
Example
=======

We provided example code snippets for running stella_vslam with variety of datasets.

.. NOTE ::

    (For ROS2 users) run_slam_offline node allows you to run SLAM by reading rosbag2 file directly. See :ref:`here <section-offline-slam>`.

.. _section-example-video:

SLAM with Video Files
=====================

We provide an example snippet for using video files (e.g. ``.mp4``) for visual SLAM.
The source code is placed at ``stella_vslam_examples/src/run_video_slam.cc``.

| The camera that captures the video file must be calibrated. Create a config file (``.yaml``) according to the camera parameters.
| We provided a vocabulary file for FBoW at `here <https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow>`__.

You can create a map database file by running one of the ``run_****_slam`` executables with ``--map-db-out map_file_name.msg`` option.

.. _section-example-image-sequence:

SLAM with Image Sequences
=========================

We provided an example snippet for using image sequences for visual SLAM.
The source code is placed at ``stella_vslam_examples/src/run_image_slam.cc``.

| The camera that captures the video file must be calibrated. Create a config file (``.yaml``) according to the camera parameters.
| We provided a vocabulary file for FBoW at `here <https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow>`__.

You can create a map database file by running one of the ``run_****_slam`` executables with ``--map-db-out map_file_name.msg`` option.

.. _section-example-standard-datasets:

SLAM with Standard Datasets
===========================

.. _subsection-example-kitti:

KITTI Odometry dataset
^^^^^^^^^^^^^^^^^^^^^^

`KITTI Odometry dataset <http://www.cvlibs.net/datasets/kitti/>`_ is a benchmarking dataset for monocular and stereo visual odometry and lidar odometry that is captured from car-mounted devices.
We provided an example source code for running monocular and stereo visual SLAM with this dataset.
The source code is placed at ``stella_vslam_examples/src/run_kitti_slam.cc``.

Start by downloading the dataset from `here <http://www.cvlibs.net/datasets/kitti/eval_odometry.php>`__.
Download the grayscale set (``data_odometry_gray.zip``).

After downloading and uncompressing it, you will find several sequences under the ``sequences/`` directory.

.. code-block:: bash

    $ ls sequences/
    00  01  02  03  04  05  06  07  08  09  10  11  12  13  14  15  16  17  18  19  20  21

In addition, download a vocabulary file for FBoW from `here <https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow>`__.

A configuration file for each sequence is contained under ``./example/kitti/``.

.. code-block:: bash

    # at the build directory of stella_vslam
    # monocular SLAM with sequence 00
    $ ./run_kitti_slam \
        -v /path/to/orb_vocab/orb_vocab.fbow \
        -d /path/to/KITTI/Odometry/sequences/00/ \
        -c ~/lib/stella_vslam/example/kitti/KITTI_mono_00-02.yaml
    # stereo SLAM with sequence 05
    $ ./run_kitti_slam \
        -v /path/to/orb_vocab/orb_vocab.fbow \
        -d /path/to/KITTI/Odometry/sequences/05/ \
        -c ~/lib/stella_vslam/example/kitti/KITTI_stereo_04-12.yaml

.. _subsection-example-euroc:

EuRoC MAV dataset
^^^^^^^^^^^^^^^^^

`EuRoC MAV dataset <https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets>`_ is a benchmarking dataset for monocular and stereo visual odometry that is captured from drone-mounted devices.
We provide an example source code for running monocular and stereo visual SLAM with this dataset.
The source code is placed at ``stella_vslam_examples/src/run_euroc_slam.cc``.

Start by downloading the dataset from `here <http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/>`__.
Download the ``.zip`` file of a dataset you plan on using.

After downloading and uncompressing it, you will find several directories under the ``mav0/`` directory.

.. code-block:: bash

    $ ls mav0/
    body.yaml  cam0  cam1  imu0  leica0  state_groundtruth_estimate0

In addition, download a vocabulary file for FBoW from `here <https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow>`__.

We provided the two config files for EuRoC, ``~/lib/stella_vslam/example/euroc/EuRoC_mono.yaml`` for monocular and ``~/lib/stella_vslam/example/euroc/EuRoC_stereo.yaml`` for stereo.

.. code-block:: bash

    # at the build directory of stella_vslam
    # monocular SLAM with any EuRoC sequence
    $ ./run_euroc_slam \
        -v /path/to/orb_vocab/orb_vocab.fbow \
        -d /path/to/EuRoC/MAV/mav0/ \
        -c ~/lib/stella_vslam/example/euroc/EuRoC_mono.yaml
    # stereo SLAM with any EuRoC sequence
    $ ./run_euroc_slam \
        -v /path/to/orb_vocab/orb_vocab.fbow \
        -d /path/to/EuRoC/MAV/mav0/ \
        -c ~/lib/stella_vslam/example/euroc/EuRoC_stereo.yaml

.. _subsection-example-tum-rgbd:

TUM RGBD dataset
^^^^^^^^^^^^^^^^

`TUM RGBD dataset <https://vision.in.tum.de/data/datasets/rgbd-dataset>`_ is a benchmarking dataset fcontaining RGB-D data and ground-truth data with the goal to establish a novel benchmark for the evaluation of visual odometry and visual SLAM systems.
The source code is placed at ``stella_vslam_examples/src/run_tum_rgbd_slam.cc``.

Start by downloading the various dataset from `here <https://vision.in.tum.de/data/datasets/rgbd-dataset/download>`__. 
One of many example datasets can be found from  `here <https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_calibration_rgb_depth.tgz>`__. 
Download the ``.tgz`` file of a dataset you plan on using.

After downloading and uncompressing it, you will find two directories and few text files under the ``rgbd_dataset_freiburg3_calibration_rgb_depth/`` directory.

.. code-block:: bash

    $ ls rgbd_dataset_freiburg3_calibration_rgb_depth
    accelerometer.txt  depth  depth.txt  groundtruth.txt  rgb  rgb.txt

If you would like to preprocess dataset then you can usee tool from `here <https://vision.in.tum.de/data/datasets/rgbd-dataset/tools>`__.

In addition, download a vocabulary file for FBoW from `here <https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow>`__.

We provided the config files for RGBD dataset at, ``./example/tum_rgbd``.

For above specific example we shall use two config files, ``~/lib/stella_vslam/example/tum_rgbd/TUM_RGBD_mono_3.yaml`` for monocular and ``~/lib/stella_vslam/example/tum_rgbd/TUM_RGBD_rgbd_3.yaml`` for RGBD.

Tracking and Mapping
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    # at the build directory of stella_vslam
    # monocular SLAM with rgbd_dataset_freiburg3_calibration_rgb_depth
    $ ./run_tum_rgbd_slam \
        -v /path/to/orb_vocab/orb_vocab.fbow \
        -d /path/to/rgbd_dataset_freiburg3_calibration_rgb_depth/ \
        -c ~/lib/stella_vslam/example/tum_rgbd/TUM_RGBD_mono_3.yaml \
        --no-sleep \
        --auto-term \
        --map-db-out fr3_slam_mono.msg

    # RGBD SLAM with rgbd_dataset_freiburg3_calibration_rgb_depth
    $ ./run_tum_rgbd_slam \
        -v /path/to/orb_vocab/orb_vocab.fbow \
        -d /path/to/rgbd_dataset_freiburg3_calibration_rgb_depth/ \
        -c ~/lib/stella_vslam/example/tum_rgbd/TUM_RGBD_rgbd_3.yaml \
        --no-sleep \
        --auto-term \
        --map-db-out fr3_slam_rgbd.msg

Localization
^^^^^^^^^^^^

.. code-block:: bash

    # at the build directory of stella_vslam
    # monocular localization with rgbd_dataset_freiburg3_calibration_rgb_depth
    $ ./run_tum_rgbd_slam --disable-mapping \
        -v /path/to/orb_vocab/orb_vocab.fbow \
        -d /path/to/rgbd_dataset_freiburg3_calibration_rgb_depth/ \
        -c ~/lib/stella_vslam/example/tum_rgbd/TUM_RGBD_mono_3.yaml \
        --no-sleep \
        --auto-term \
        --map-db-in fr3_slam_mono.msg

    # RGBD SLAM with rgbd_dataset_freiburg3_calibration_rgb_depth
    $ ./run_tum_rgbd_slam --disable-mapping \
        -v /path/to/orb_vocab/orb_vocab.fbow \
        -d /path/to/rgbd_dataset_freiburg3_calibration_rgb_depth/ \
        -c ~/lib/stella_vslam/example/tum_rgbd/TUM_RGBD_rgbd_3.yaml \
        --no-sleep \
        --auto-term \
        --map-db-in fr3_slam_rgbd.msg

Localization with temporal mapping based odometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This feature can be used to add keyframes to stabilize localization results.

.. code-block:: bash

    # at the build directory of stella_vslam
    # monocular localization with rgbd_dataset_freiburg3_calibration_rgb_depth
    $ ./run_tum_rgbd_slam --temporal-mapping \
        -v /path/to/orb_vocab/orb_vocab.fbow \
        -d /path/to/rgbd_dataset_freiburg3_calibration_rgb_depth/ \
        -c ~/lib/stella_vslam/example/tum_rgbd/TUM_RGBD_mono_3.yaml \
        --no-sleep \
        --auto-term \
        --map-db-in fr3_slam_mono.msg

    # RGBD SLAM with rgbd_dataset_freiburg3_calibration_rgb_depth
    $ ./run_tum_rgbd_slam --temporal-mapping \
        -v /path/to/orb_vocab/orb_vocab.fbow \
        -d /path/to/rgbd_dataset_freiburg3_calibration_rgb_depth/ \
        -c ~/lib/stella_vslam/example/tum_rgbd/TUM_RGBD_rgbd_3.yaml \
        --no-sleep \
        --auto-term \
        --map-db-in fr3_slam_rgbd.msg

* If run with ``--temporal-mapping``, loaded keyframes are prioritized for localization/localBA.
* If set parameter ``erase_temporal_keyframes`` to true, it will remove keyframes older than ``num_temporal_keyframes``.
* If set parameter ``enable_temporal_keyframe_only_tracking`` to true, then tracking with only temporal keyframes will not be treated as Lost. If ``--temporal-mapping`` is not set, ``enable_temporal_keyframe_only_tracking`` will be ignored.
* Enabling all three of the above will run Visual SLAM with a limited number of keyframes

.. _section-example-uvc-camera:

SLAM with UVC camera
=========================

Tracking and Mapping
^^^^^^^^^^^^^^^^^^^^

We provided an example snippet for using a UVC camera, which is often called a webcam, for visual SLAM.
The source code is placed at ``stella_vslam_examples/src/run_camera_slam.cc``.

| Please specify the camera number you want to use by ``-n`` option.
| The camera must be calibrated. Create a config file (``.yaml``) according to the camera parameters.
| You can scale input images to the performance of your machine by ``-s`` option. Please modify the config accordingly.
| We provided a vocabulary file for FBoW at `here <https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow>`__.
