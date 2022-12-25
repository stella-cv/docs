.. _chapter-parameters:

==========
Parameters
==========


.. _section-parameters-system:

System
======

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - map_format
      - msgpack or sqlite3
    * - min_num_shared_lms
      - minimum number of shared landmarks for covisibility graph connection. Keyframes that exceed the threshold are considered to share a sufficient number of landmarks and are used for local BA, loop detection and graph optimization.

.. _section-parameters-camera:

Camera
======

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - name
      - It is used by the camera database to identify the camera.
    * - setup
      - monocular, stereo, RGBD
    * - model
      - perspective, fisheye, equirectangular, radial_division (note: If you want to use stereo_rectifier, you need to specify perspective.)
    * - fx, fy
      - Focal length (pixel)
    * - cx, cy
      - Principal point (pixel)
    * - k1, k2, p1, p2, k3
      - Distortion parameters for perspective camera. When using StereoRectifier, there is no distortion after stereo rectification.
    * - k1, k2, k3, k4
      - Distortion parameters for fisheye camera
    * - distortion
      - Distortion parameters for radial_division camera
    * - fps
      - Framerate of input images
    * - cols, rows
      - Resolution (pixel)
    * - color_order
      - Gray, RGB, RGBA, BGR, BGRA
    * - focal_x_baseline
      - For stereo cameras, it is the value of the baseline between the left and right cameras multiplied by the focal length fx.
        For RGBD cameras, if the measurement method is stereo, set it based on its baseline. If the measurement method is other than that, set the appropriate value based on the relationship between depth accuracy and baseline.
    * - depth_threshold
      - The ratio used to determine the depth threshold.

.. _section-parameters-feature:

Feature
=======

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - name
      - name of ORB feature extraction model (id for saving)
    * - scale_factor
      - Scale of the image pyramid
    * - num_levels
      - Number of levels of in the image pyramid
    * - ini_fast_threshold
      - FAST threshold for try first
    * - min_fast_threshold
      - FAST threshold for try second time

.. _section-parameters-preprocessing:

Preprocessing
=============

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - min_size
      - Size of node occupied by one feature point. The larger this value, the fewer feature points are extracted.
    * - depthmap_factor
      - The ratio used to convert depth image pixel values to distance.

.. _section-parameters-tracking:

Tracking
========

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - reloc_distance_threshold
      - Maximum distance threshold (in meters) where close keyframes could be found when doing a relocalization by pose.
    * - reloc_angle_threshold
      - Maximum angle threshold (in radians) between given pose and close keyframes when doing a relocalization by pose.
    * - enable_auto_relocalization
      - If true, automatically try to relocalize when lost.
    * - use_robust_matcher_for_relocalization_request
      - If true, use robust_matcher for relocalization request.
    * - max_num_local_keyfrms
      - Max number of local keyframes for tracking.
    * - backend
      - g2o or gtsam

.. _section-parameters-mapping:

Mapping
=======

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - baseline_dist_thr_ratio
      - For two frames of baseline below the threshold, no triangulation will be performed. In the monocular case, the scale is indefinite, so relative values are recommended.
        Either baseline_dist_thr or this one should be specified. If not specified, baseline_dist_thr_ratio will be used.
    * - baseline_dist_thr
      - For two frames of baseline below the threshold, no triangulation will be performed.
    * - redundant_obs_ratio_thr
      - 
    * - observed_ratio_thr
      - 
    * - num_reliable_keyfrms
      - 
    * - enable_interruption_of_landmark_generation
      - If true, enable interruption of landmark generation
    * - enable_interruption_before_local_BA
      - If true, enable interruption before local BA
    * - backend
      - g2o or gtsam

.. _section-parameters-stereo-rectifier:

StereoRectifier
===============

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - model
      - camera model type before rectification. The option is perspective or fisheye. (note: If you want to use fisheye model for stereo_rectifier, you need to specify Camera::model to perspective.)
    * - K_left, K_right
      - Intrinsic parameters. The 3x3 matrix are written in row-major order.
    * - D_left, D_right
      - Distortion parameters. The 5 parameters are k1, k2, p1, p2, k3.
    * - R_left, R_right
      - Stereo-recitification parameters. The 3x3 matrix are written in row-major order.


.. _section-parameters-initializer:

Initializer
===========

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - num_ransac_iterations
      - max number of iterations of RANSAC (only for monocular initializer)
    * - min_num_valid_pts
      - min number of valid pts (It should be greater than or equal to min_num_triangulated_)
    * - min_num_triangulated_pts
      - Minimum number of triangulated points
    * - parallax_deg_threshold
      - min parallax (only for monocular initializer)
    * - reprojection_error_threshold
      - reprojection error threshold (only for monocular initializer)
    * - num_ba_iterations
      - max number of iterations of BA (only for monocular initializer)
    * - scaling_factor
      - initial scaling factor (only for monocular initializer)
    * - use_fixed_seed
      - Use fixed random seed for RANSAC if true

.. _section-parameters-relocalizer:

Relocalizer
===========

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - bow_match_lowe_ratio
      - 
    * - proj_match_lowe_ratio
      - 
    * - min_num_bow_matches
      - 
    * - min_num_valid_obs
      - 

.. _section-parameters-keyframe-inserter:

KeyframeInserter
================

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - max_interval
      - max interval to insert keyframe
    * - min_interval
      - 
    * - max_distance
      - 
    * - enough_lms_thr
      - 
    * - lms_ratio_thr_almost_all_lms_are_tracked
      - Threshold at which we consider that we are tracking almost all landmarks. Ratio-threshold of "the number of 3D points observed in the current frame" / "that of 3D points observed in the last keyframe"
    * - lms_ratio_thr_view_changed
      - Threshold at which we consider the view to have changed. Ratio-threshold of "the number of 3D points observed in the current frame" / "that of 3D points observed in the last keyframe"

.. _section-parameters-pangolin:

PangolinViewer
==============

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - keyframe_size
      - 
    * - keyframe_line_width
      - 
    * - graph_line_width
      - 
    * - point_size
      - 
    * - camera_size
      - 
    * - camera_line_width
      - 
    * - menu_width
      - 
    * - viewpoint_x, viewpoint_y, viewpoint_z, viewpoint_f
      - 

.. _section-parameters-socket-publisher:

SocketPublisher
===============

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - emitting_interval
      - 
    * - image_quality
      - 
    * - server_uri
      - 
    * - max_num_keyframes
      - Limit the number of keyframes transferred at one time. This avoids disconnections when loading large maps.
    * - max_num_landmarks
      - Limit the number of landmarks transferred at one time. This avoids disconnections when loading large maps.
    * - publish_points
      - If true, pointcloud transfer is enabled. The default is true. Pointcloud transfer is slow, so disabling pointcloud transfer may be useful to improve performance of SocketViewer.

.. _section-parameters-loop-detector:

LoopDetector
============

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - enabled
      - flag which indicates the loop detector is enabled or not
    * - num_final_matches_threshold
      - the threshold of the number of mutual matches after the Sim3 estimation
    * - min_continuity
      - the threshold of the continuity of continuously detected keyframe set
    * - reject_by_graph_distance
      - If true, reject by distance on essential graph
    * - loop_min_distance_on_graph
      - Minimum distance to allow for loop candidates
    * - top_n_covisibilities_to_search
      - Top n covisibilities to search (0 means disabled)
    * - num_matches_thr
      - Minimum number of matches to allow for loop candidates
    * - num_matches_thr_brute_force
      - Minimum number of matches to allow for loop candidates after brute force matching. (0 means disabled)
    * - num_optimized_inliers_thr
      - Minimum number of matches to allow for loop candidates after optimization by transform_optimizer
    * - backend
      - g2o or gtsam

.. _section-parameters-marker-model:

MarkerModel
===========

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - type
      - Only "aruco" is a valid value
    * - width
      - Physical size of marker
    * - marker_size
      - 4, 5, 6, 7. See https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html.
    * - max_markers
      - 50, 100, 250, 1000. See https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html.
