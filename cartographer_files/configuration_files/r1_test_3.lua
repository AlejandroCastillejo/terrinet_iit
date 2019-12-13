-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"



options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.15,   --1.
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

---------------------- system settings --------------------------------------------------

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4 -- Increase up to number of cores
--TRAJECTORY_BUILDER.pure_localization = false     -- set to true for pure localization
--TRAJECTORY_BUILDER_2D.missing_data_ray_length = 15.

TRAJECTORY_BUILDER_2D.use_imu_data = false

------------------------------------
TRAJECTORY_BUILDER_2D = {
  use_imu_data = false,     				--- was true in default
  min_range = 0.3,
  max_range = 15.,
  min_z = -0.8,
  max_z = 2.,
  missing_data_ray_length = 5.,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.025,

  adaptive_voxel_filter = {
    max_length = 0.5,
    min_num_points = 200,
    max_range = 50.,
  },

  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },

  use_online_correlative_scan_matching = true,      --- it was false in default
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.1,
    angular_search_window = math.rad(20.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

  ceres_scan_matcher = {
    occupied_space_weight = 1.,
    translation_weight = 10.,
    rotation_weight = 40.,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },

  motion_filter = {
    max_time_seconds = 5.,
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),
  },

  imu_gravity_time_constant = 10.,

  submaps = {
    num_range_data = 90,
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
      resolution = 0.05,
    },
    range_data_inserter = {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      probability_grid_range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
      tsdf_range_data_inserter = {
        truncation_distance = 0.3,
        maximum_weight = 10.,
        update_free_space = false,
        normal_estimation_options = {
          num_normal_samples = 4,
          sample_radius = 0.5,
        },
        project_sdf_distance_to_scan_normal = true,
        update_weight_range_exponent = 0,
        update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
        update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
      },
    },
  },
}

POSE_GRAPH = {
  optimize_every_n_nodes = 90,
  constraint_builder = {
    sampling_ratio = 0.3,
    max_constraint_distance = 15.,
    min_score = 0.55,
    global_localization_min_score = 0.6,
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1e5,
    log_matches = true,
    fast_correlative_scan_matcher = {
      linear_search_window = 7.,
      angular_search_window = math.rad(30.),
      branch_and_bound_depth = 7,
    },
    ceres_scan_matcher = {
      occupied_space_weight = 20.,
      translation_weight = 10.,
      rotation_weight = 1.,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      linear_xy_search_window = 5.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(15.),
    },
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 5.,
      occupied_space_weight_1 = 30.,
      translation_weight = 10.,
      rotation_weight = 1.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },
  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3,
  optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1e3,
    rotation_weight = 3e5,
    local_slam_pose_translation_weight = 1e5,
    local_slam_pose_rotation_weight =1e5,
    odometry_translation_weight = 1e5,
    odometry_rotation_weight = 1e3,
    fixed_frame_pose_translation_weight = 1e1,
    fixed_frame_pose_rotation_weight = 1e2,
    log_solver_summary = false,
    use_online_imu_extrinsics_in_3d = true,
    fix_z_in_3d = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },
  max_num_final_iterations = 200,
  global_sampling_ratio = 0.003,
  log_residual_histograms = true,
  global_constraint_search_after_n_seconds = 10.,
  --  overlapping_submaps_trimmer_2d = {
  --    fresh_submaps_count = 1,
  --    min_covered_area = 2,
  --    min_added_submaps_count = 5,
  --  },
}





--POSE_GRAPH = {
--	optimization_problem ={
--		local_slam_pose_translation_weight = 
--		local_slam_pose_rotation_weight = 
--		odometry_translation_weight =
--		odometry_rotation_weight =
--	}
--}



-----------------  system settings ---------------------
	--MAP_BUILDER.use_trajectory_builder_2d = true
	--MAP_BUILDER.num_background_threads = 4 -- Increase up to number of cores
--TRAJECTORY_BUILDER.pure_localization = false     -- set to true for pure localization

--TRAJECTORY_BUILDER_2D.missing_data_ray_length = 15.
	--TRAJECTORY_BUILDER_2D.use_imu_data = false
	--TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10.
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 4e2

--POSE_GRAPH.optimization_problem.huber_scale = 1e2

-----------------TUNE THESE PARAMETERS -------------------------------



------------Global SLAM------------
--POSE_GRAPH.optimize_every_n_nodes = 1 -- Decrease
--POSE_GRAPH.global_sampling_ratio = 0.00001 -- Decrease
--POSE_GRAPH.constraint_builder.sampling_ratio = 0.0001 -- Decrease
--POSE_GRAPH.constraint_builder.min_score = 0.75 -- Increase
--POSE_GRAPH.global_constraint_search_after_n_seconds = 20 -- Increase
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 5 -- Decrease

---------Global/Local SLAM---------
--TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05 									-- To reduce latency: Increase - the smaller the size and the higher the number of laser point is used  
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100 					-- To reduce latency: Decrease
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 10. 						-- To reduce latency: Decrease
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 1.0 						-- To reduce latency: Increase
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 50 		-- To reduce latency: Decrease
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 10. 			-- To reduce latency: Decrease
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 1.8 		-- To reduce latency: Increase
--TRAJECTORY_BUILDER_2D.submaps.num_range_data = 1 -- Decrease

----------- Local SLAM --------------
--POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight
--POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight
--POSE_GRAPH.optimization_problem.odometry_translation_weight
--POSE_GRAPH.optimization_problem.odometry_rotation_weight = 


return options
