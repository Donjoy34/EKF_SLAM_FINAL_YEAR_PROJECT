from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    imu_topic = LaunchConfiguration('imu_topic')
    cone_topic = LaunchConfiguration('cone_topic')
    wheel_topic = LaunchConfiguration('wheel_topic')
    use_unknown_color_cones = LaunchConfiguration('use_unknown_color_cones')
    gt_odom_topic = LaunchConfiguration('gt_odom_topic')
    gt_cones_topic = LaunchConfiguration('gt_cones_topic')
    tf_topic = LaunchConfiguration('tf_topic')
    enable_loop_closure = LaunchConfiguration('enable_loop_closure')
    plot_output_dir = LaunchConfiguration('plot_output_dir')
    plot_filename = LaunchConfiguration('plot_filename')
    enable_live_plot = LaunchConfiguration('enable_live_plot')
    enable_live_eval_plot = LaunchConfiguration('enable_live_eval_plot')
    live_plot_period_sec = LaunchConfiguration('live_plot_period_sec')
    max_new_landmarks_per_scan = LaunchConfiguration('max_new_landmarks_per_scan')
    landmark_merge_distance_m = LaunchConfiguration('landmark_merge_distance_m')
    landmark_min_observations = LaunchConfiguration('landmark_min_observations')
    landmark_association_min_hits = LaunchConfiguration('landmark_association_min_hits')
    landmark_min_init_range_m = LaunchConfiguration('landmark_min_init_range_m')
    landmark_dedup_distance_m = LaunchConfiguration('landmark_dedup_distance_m')
    landmark_dedup_min_hits = LaunchConfiguration('landmark_dedup_min_hits')
    max_landmark_pose_step_m = LaunchConfiguration('max_landmark_pose_step_m')
    max_landmark_yaw_step_deg = LaunchConfiguration('max_landmark_yaw_step_deg')
    landmark_pose_correction_gain = LaunchConfiguration('landmark_pose_correction_gain')
    landmark_yaw_correction_gain = LaunchConfiguration('landmark_yaw_correction_gain')
    use_imu_accel_prediction = LaunchConfiguration('use_imu_accel_prediction')
    use_imu_orientation_update = LaunchConfiguration('use_imu_orientation_update')
    imu_yaw_meas_noise_deg = LaunchConfiguration('imu_yaw_meas_noise_deg')
    velocity_wheel_blend = LaunchConfiguration('velocity_wheel_blend')
    yaw_rate_blend = LaunchConfiguration('yaw_rate_blend')
    wheel_speed_timeout_sec = LaunchConfiguration('wheel_speed_timeout_sec')
    wheel_speed_deadband_mps = LaunchConfiguration('wheel_speed_deadband_mps')
    wheel_invalid_rpm_threshold = LaunchConfiguration('wheel_invalid_rpm_threshold')
    wheel_speed_scale = LaunchConfiguration('wheel_speed_scale')
    enable_gt_speed_scale_calibration = LaunchConfiguration('enable_gt_speed_scale_calibration')
    wheel_speed_scale_min = LaunchConfiguration('wheel_speed_scale_min')
    wheel_speed_scale_max = LaunchConfiguration('wheel_speed_scale_max')
    wheel_speed_scale_adapt_rate = LaunchConfiguration('wheel_speed_scale_adapt_rate')
    wheel_scale_calib_min_speed_mps = LaunchConfiguration('wheel_scale_calib_min_speed_mps')
    enable_speed_debug_log = LaunchConfiguration('enable_speed_debug_log')
    allow_imu_speed_fallback = LaunchConfiguration('allow_imu_speed_fallback')
    imu_speed_fallback_gain = LaunchConfiguration('imu_speed_fallback_gain')
    imu_speed_decay_per_sec = LaunchConfiguration('imu_speed_decay_per_sec')
    velocity_decay_per_sec = LaunchConfiguration('velocity_decay_per_sec')
    stationary_speed_threshold_mps = LaunchConfiguration('stationary_speed_threshold_mps')
    stationary_yaw_rate_threshold_radps = LaunchConfiguration('stationary_yaw_rate_threshold_radps')
    stationary_accel_threshold_mps2 = LaunchConfiguration('stationary_accel_threshold_mps2')
    zupt_velocity_noise = LaunchConfiguration('zupt_velocity_noise')
    innovation_gate_chi2 = LaunchConfiguration('innovation_gate_chi2')
    enable_auto_tuning = LaunchConfiguration('enable_auto_tuning')
    auto_tune_rate = LaunchConfiguration('auto_tune_rate')
    target_wheel_nis = LaunchConfiguration('target_wheel_nis')
    target_landmark_nis = LaunchConfiguration('target_landmark_nis')
    wheel_speed_noise_min = LaunchConfiguration('wheel_speed_noise_min')
    wheel_speed_noise_max = LaunchConfiguration('wheel_speed_noise_max')
    cone_meas_noise_min = LaunchConfiguration('cone_meas_noise_min')
    cone_meas_noise_max = LaunchConfiguration('cone_meas_noise_max')
    eval_log_period_sec = LaunchConfiguration('eval_log_period_sec')

    return LaunchDescription([
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('cone_topic', default_value='/cones'),
        DeclareLaunchArgument('wheel_topic', default_value='/ros_can/wheel_speeds'),
        DeclareLaunchArgument('use_unknown_color_cones', default_value='false'),
        DeclareLaunchArgument('gt_odom_topic', default_value='/ground_truth/odom'),
        DeclareLaunchArgument('gt_cones_topic', default_value='/ground_truth/cones'),
        DeclareLaunchArgument('tf_topic', default_value='/tf'),
        DeclareLaunchArgument('enable_loop_closure', default_value='false'),
        DeclareLaunchArgument('plot_output_dir', default_value='~/uh-fs-ai/slam_eval'),
        DeclareLaunchArgument('plot_filename', default_value='ekf_slam_static_map.png'),
        DeclareLaunchArgument('enable_live_plot', default_value='true'),
        DeclareLaunchArgument('enable_live_eval_plot', default_value='true'),
        DeclareLaunchArgument('live_plot_period_sec', default_value='0.2'),
        DeclareLaunchArgument('max_new_landmarks_per_scan', default_value='5'),
        DeclareLaunchArgument('landmark_merge_distance_m', default_value='0.9'),
        DeclareLaunchArgument('landmark_min_observations', default_value='2'),
        DeclareLaunchArgument('landmark_association_min_hits', default_value='3'),
        DeclareLaunchArgument('landmark_min_init_range_m', default_value='1.0'),
        DeclareLaunchArgument('landmark_dedup_distance_m', default_value='0.55'),
        DeclareLaunchArgument('landmark_dedup_min_hits', default_value='2'),
        DeclareLaunchArgument('max_landmark_pose_step_m', default_value='0.25'),
        DeclareLaunchArgument('max_landmark_yaw_step_deg', default_value='6.0'),
        DeclareLaunchArgument('landmark_pose_correction_gain', default_value='0.35'),
        DeclareLaunchArgument('landmark_yaw_correction_gain', default_value='0.35'),
        DeclareLaunchArgument('use_imu_accel_prediction', default_value='false'),
        DeclareLaunchArgument('use_imu_orientation_update', default_value='true'),
        DeclareLaunchArgument('imu_yaw_meas_noise_deg', default_value='2.5'),
        DeclareLaunchArgument('velocity_wheel_blend', default_value='0.85'),
        DeclareLaunchArgument('yaw_rate_blend', default_value='0.75'),
        DeclareLaunchArgument('wheel_speed_timeout_sec', default_value='0.25'),
        DeclareLaunchArgument('wheel_speed_deadband_mps', default_value='0.08'),
        DeclareLaunchArgument('wheel_invalid_rpm_threshold', default_value='900.0'),
        DeclareLaunchArgument('wheel_speed_scale', default_value='1.2'),
        DeclareLaunchArgument('enable_gt_speed_scale_calibration', default_value='true'),
        DeclareLaunchArgument('wheel_speed_scale_min', default_value='0.6'),
        DeclareLaunchArgument('wheel_speed_scale_max', default_value='1.4'),
        DeclareLaunchArgument('wheel_speed_scale_adapt_rate', default_value='0.2'),
        DeclareLaunchArgument('wheel_scale_calib_min_speed_mps', default_value='0.2'),
        DeclareLaunchArgument('enable_speed_debug_log', default_value='true'),
        DeclareLaunchArgument('allow_imu_speed_fallback', default_value='true'),
        DeclareLaunchArgument('imu_speed_fallback_gain', default_value='1.0'),
        DeclareLaunchArgument('imu_speed_decay_per_sec', default_value='0.6'),
        DeclareLaunchArgument('velocity_decay_per_sec', default_value='1.2'),
        DeclareLaunchArgument('stationary_speed_threshold_mps', default_value='0.12'),
        DeclareLaunchArgument('stationary_yaw_rate_threshold_radps', default_value='0.08'),
        DeclareLaunchArgument('stationary_accel_threshold_mps2', default_value='0.35'),
        DeclareLaunchArgument('zupt_velocity_noise', default_value='0.03'),
        DeclareLaunchArgument('innovation_gate_chi2', default_value='7.0'),
        DeclareLaunchArgument('enable_auto_tuning', default_value='true'),
        DeclareLaunchArgument('auto_tune_rate', default_value='0.04'),
        DeclareLaunchArgument('target_wheel_nis', default_value='1.0'),
        DeclareLaunchArgument('target_landmark_nis', default_value='2.0'),
        DeclareLaunchArgument('wheel_speed_noise_min', default_value='0.05'),
        DeclareLaunchArgument('wheel_speed_noise_max', default_value='2.0'),
        DeclareLaunchArgument('cone_meas_noise_min', default_value='0.05'),
        DeclareLaunchArgument('cone_meas_noise_max', default_value='3.0'),
        DeclareLaunchArgument('eval_log_period_sec', default_value='1.5'),
        Node(
            package='localization',
            executable='ekf_slam',
            name='ekf_slam',
            output='screen',
            parameters=[
                {'imu_topic': imu_topic},
                {'cone_topic': cone_topic},
                {'wheel_topic': wheel_topic},
                {'use_unknown_color_cones': use_unknown_color_cones},
                {'gt_odom_topic': gt_odom_topic},
                {'gt_cones_topic': gt_cones_topic},
                {'tf_topic': tf_topic},
                {'enable_loop_closure': enable_loop_closure},
                {'plot_output_dir': plot_output_dir},
                {'plot_filename': plot_filename},
                {'enable_live_plot': enable_live_plot},
                {'enable_live_eval_plot': enable_live_eval_plot},
                {'live_plot_period_sec': live_plot_period_sec},
                {'max_new_landmarks_per_scan': max_new_landmarks_per_scan},
                {'landmark_merge_distance_m': landmark_merge_distance_m},
                {'landmark_min_observations': landmark_min_observations},
                {'landmark_association_min_hits': landmark_association_min_hits},
                {'landmark_min_init_range_m': landmark_min_init_range_m},
                {'landmark_dedup_distance_m': landmark_dedup_distance_m},
                {'landmark_dedup_min_hits': landmark_dedup_min_hits},
                {'max_landmark_pose_step_m': max_landmark_pose_step_m},
                {'max_landmark_yaw_step_deg': max_landmark_yaw_step_deg},
                {'landmark_pose_correction_gain': landmark_pose_correction_gain},
                {'landmark_yaw_correction_gain': landmark_yaw_correction_gain},
                {'use_imu_accel_prediction': use_imu_accel_prediction},
                {'use_imu_orientation_update': use_imu_orientation_update},
                {'imu_yaw_meas_noise_deg': imu_yaw_meas_noise_deg},
                {'velocity_wheel_blend': velocity_wheel_blend},
                {'yaw_rate_blend': yaw_rate_blend},
                {'wheel_speed_timeout_sec': wheel_speed_timeout_sec},
                {'wheel_speed_deadband_mps': wheel_speed_deadband_mps},
                {'wheel_invalid_rpm_threshold': wheel_invalid_rpm_threshold},
                {'wheel_speed_scale': wheel_speed_scale},
                {'enable_gt_speed_scale_calibration': enable_gt_speed_scale_calibration},
                {'wheel_speed_scale_min': wheel_speed_scale_min},
                {'wheel_speed_scale_max': wheel_speed_scale_max},
                {'wheel_speed_scale_adapt_rate': wheel_speed_scale_adapt_rate},
                {'wheel_scale_calib_min_speed_mps': wheel_scale_calib_min_speed_mps},
                {'enable_speed_debug_log': enable_speed_debug_log},
                {'allow_imu_speed_fallback': allow_imu_speed_fallback},
                {'imu_speed_fallback_gain': imu_speed_fallback_gain},
                {'imu_speed_decay_per_sec': imu_speed_decay_per_sec},
                {'velocity_decay_per_sec': velocity_decay_per_sec},
                {'stationary_speed_threshold_mps': stationary_speed_threshold_mps},
                {'stationary_yaw_rate_threshold_radps': stationary_yaw_rate_threshold_radps},
                {'stationary_accel_threshold_mps2': stationary_accel_threshold_mps2},
                {'zupt_velocity_noise': zupt_velocity_noise},
                {'innovation_gate_chi2': innovation_gate_chi2},
                {'enable_auto_tuning': enable_auto_tuning},
                {'auto_tune_rate': auto_tune_rate},
                {'target_wheel_nis': target_wheel_nis},
                {'target_landmark_nis': target_landmark_nis},
                {'wheel_speed_noise_min': wheel_speed_noise_min},
                {'wheel_speed_noise_max': wheel_speed_noise_max},
                {'cone_meas_noise_min': cone_meas_noise_min},
                {'cone_meas_noise_max': cone_meas_noise_max},
                {'eval_log_period_sec': eval_log_period_sec},
            ],
        ),
    ])
