python setup_ycb_scene.py --index 161 --datadir /root/Datasets/benchmarking/final_scenes/metadata/




poseCNN:

object 0, class 035_power_drill, z 0.724933385848999, z new 1.111829400062561
object 1, class 011_banana, z 0.8830955028533936, z new 0.40575963258743286
object 2, class 005_tomato_soup_can, z 0.961146354675293, z new 0.8840349912643433
object 3, class 024_bowl, z 0.763563871383667, z new 1.1272146701812744
object 4, class 004_sugar_box, z 0.6299688220024109, z new 1.1976063251495361
object 5, class 036_wood_block, z 0.4996073246002197, z new 0.8253579139709473
sdf 0 points for object 0, class 14 035_power_drill, no refinement
sdf 0 points for object 1, class 9 011_banana, no refinement
sdf 701 points for object 2, class 3 005_tomato_soup_can
sdf 0 points for object 3, class 12 024_bowl, no refinement
sdf 0 points for object 4, class 2 004_sugar_box, no refinement
sdf 11700 points for object 5, class 15 036_wood_block
sdf with 12401 points
sdf refinement iterations 50, time 0.090665
pose refine time 0.177455
035_power_drill 0.9958774 14
011_banana 0.99562246 9
005_tomato_soup_can 0.9885634 3
024_bowl 0.95793414 12
004_sugar_box 0.8178627 2
036_wood_block 0.38784295 15
cudaGraphicsGLRegisterImage failed: 304
cudaGraphicsMapResources failed: 400
cudaGraphicsSubResourceGetMappedArray failed: 400
cudaMemcpy2DFromArray failed: 709
cudaGraphicsUnmapResources failed: 400
cudaGraphicsGLRegisterImage failed: 304
cudaGraphicsMapResources failed: 400
cudaGraphicsSubResourceGetMappedArray failed: 400
cudaMemcpy2DFromArray failed: 709
cudaGraphicsUnmapResources failed: 400
cudaGraphicsGLRegisterImage failed: 304
cudaGraphicsMapResources failed: 400
cudaGraphicsSubResourceGetMappedArray failed: 400
cudaMemcpy2DFromArray failed: 709
cudaGraphicsUnmapResources failed: 400
cudaGraphicsGLRegisterImage failed: 304
cudaGraphicsMapResources failed: 400
cudaGraphicsSubResourceGetMappedArray failed: 400
cudaMemcpy2DFromArray failed: 709
cudaGraphicsUnmapResources failed: 400
--- 0.7388684749603271 seconds ---
cudaGraphicsGLRegisterImage failed: 304
cudaGraphicsMapResources failed: 400
cudaGraphicsSubResourceGetMappedArray failed: 400
cudaMemcpy2DFromArray failed: 709
cudaGraphicsUnmapResources failed: 400
cudaGraphicsGLRegisterImage failed: 304
cudaGraphicsMapResources failed: 400
cudaGraphicsSubResourceGetMappedArray failed: 400
cudaMemcpy2DFromArray failed: 709


UnseenObjectClusting, run pip install requirement:
ERROR: jupyter-packaging 0.12.3 has requirement setuptools>=60.2.0, but you'll have setuptools 45.2.0 which is incompatible.
ERROR: notebook 6.5.7 has requirement jupyter-client<8,>=5.3.4, but you'll have jupyter-client 8.6.3 which is incompatible.
ERROR: open3d 0.13.0 has requirement pillow>=8.2.0, but you'll have pillow 7.0.0 which is incompatible.
ERROR: open3d 0.13.0 has requirement pyyaml>=5.4.1, but you'll have pyyaml 5.3.1 which is incompatible.
ERROR: imageio 2.35.1 has requirement pillow>=8.3.2, but you'll have pillow 7.0.0 which is incompatible.


=====

running model-free grasp with UnseenObjectClustering and contact_graspnet, seeing in the terminal running just_robot:
[ERROR] [1753393692.713512, 844.292000]: bad callback: <bound method Subscriber.callback of <message_filters.Subscriber object at 0x7e021d02f450>>
Traceback (most recent call last):
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1365, in _do_call
    return fn(*args)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1350, in _run_fn
    target_list, run_metadata)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1443, in _call_tf_sessionrun
    run_metadata)
tensorflow.python.framework.errors_impl.ResourceExhaustedError: 2 root error(s) found.
  (0) Resource exhausted: OOM when allocating tensor with shape[4,2048,128,128] and type float on /job:localhost/replica:0/task:0/device:GPU:0 by allocator GPU_0_bfc
	 [[{{node layer1/conv2_2/bn/cond/output/_242-0-0-TransposeNCHWToNHWC-LayoutOptimizer}}]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

  (1) Resource exhausted: OOM when allocating tensor with shape[4,2048,128,128] and type float on /job:localhost/replica:0/task:0/device:GPU:0 by allocator GPU_0_bfc
	 [[{{node layer1/conv2_2/bn/cond/output/_242-0-0-TransposeNCHWToNHWC-LayoutOptimizer}}]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

	 [[Sigmoid/_553]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

0 successful operations.
0 derived errors ignored.

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/opt/ros/noetic/lib/python3/dist-packages/message_filters/__init__.py", line 76, in callback
    self.signalMessage(msg)
  File "/opt/ros/noetic/lib/python3/dist-packages/message_filters/__init__.py", line 58, in signalMessage
    cb(*(msg + args))
  File "/opt/ros/noetic/lib/python3/dist-packages/message_filters/__init__.py", line 330, in add
    self.signalMessage(*msgs)
  File "/opt/ros/noetic/lib/python3/dist-packages/message_filters/__init__.py", line 58, in signalMessage
    cb(*(msg + args))
  File "./ros/test_model_ros.py", line 220, in callback_points
    self.run_network(viz=False)
  File "./ros/test_model_ros.py", line 288, in run_network
    forward_passes=self.forward_passes,
  File "/root/compare_SceneReplica/src/contact_graspnet/ros/../contact_graspnet/contact_grasp_estimator.py", line 254, in predict_scene_grasps
    pred_grasps_cam[k], scores[k], contact_pts[k], gripper_openings[k] = self.predict_grasps(sess, pc_region, convert_cam_coords=True, forward_passes=forward_passes)
  File "/root/compare_SceneReplica/src/contact_graspnet/ros/../contact_graspnet/contact_grasp_estimator.py", line 192, in predict_grasps
    pred_grasps_cam, pred_scores, pred_points, offset_pred = sess.run(self.inference_ops, feed_dict=feed_dict)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 958, in run
    run_metadata_ptr)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1181, in _run
    feed_dict_tensor, options, run_metadata)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1359, in _do_run
    run_metadata)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1384, in _do_call
    raise type(e)(node_def, op, message)
tensorflow.python.framework.errors_impl.ResourceExhaustedError: 2 root error(s) found.
  (0) Resource exhausted: OOM when allocating tensor with shape[4,2048,128,128] and type float on /job:localhost/replica:0/task:0/device:GPU:0 by allocator GPU_0_bfc
	 [[{{node layer1/conv2_2/bn/cond/output/_242-0-0-TransposeNCHWToNHWC-LayoutOptimizer}}]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

  (1) Resource exhausted: OOM when allocating tensor with shape[4,2048,128,128] and type float on /job:localhost/replica:0/task:0/device:GPU:0 by allocator GPU_0_bfc
	 [[{{node layer1/conv2_2/bn/cond/output/_242-0-0-TransposeNCHWToNHWC-LayoutOptimizer}}]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

	 [[Sigmoid/_553]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

0 successful operations.
0 derived errors ignored.

---
- warning in other terminals running launch files:
[WARN] [1753393898.212546246, 977.001000000]: TF_REPEATED_DATA ignoring data with redundant timestamp for frame base_link (parent odom) at time 976.995000 according to authority unknown_publisher
[WARN] [1753393899.040372955, 977.631000000]: TF_REPEATED_DATA ignoring data with redundant timestamp for frame base_link (parent odom) at time 977.630000 according to authority unknown_publisher
[WARN] [1753393899.040994350, 977.631000000]: TF_REPEATED_DATA ignoring data with redundant timestamp for frame base_link (parent odom) at time 977.630000 according to authority unknown_publisher
[WARN] [1753393930.563567104, 998.392000000]: TF_REPEATED_DATA ignoring data with redundant timestamp for frame base_link (parent odom) at time 998.385000 according to authority unknown_publisher

---
output in motion planning python script:

7 objects segmented
save data to /root/compare_SceneReplica/src/data/experiments/bench_6dof_seg_cg/25-07-24_T214727_grasp-contact_gnet_seg-uois_scene-10_ord-random/listener_0724T214747/meta-000000.mat
Num: Labels: 8 | GT: 6
COST SHAPE: (6, 8)
Target maskid_gt: 8 | LABELS_GT: [  1   2   8  10  12 101]
Pred Label ids: [0 1 2 3 4 5 6 7]
Assignmnet: [(0, 7), (1, 4), (2, 1), (3, 5), (4, 6), (5, 0)]
*******object order: ['011_banana', '035_power_drill', '005_tomato_soup_can', '024_bowl', '004_sugar_box'] ************
Step: 0 | grasp object: 011_banana | maskid_label_GT: 8 | maskid_label_Pred: 1
(2319, 3)
(2319, 3)
Obtained target object points, publishing....
PC_ALL_SCENE shape: (269194, 3)
Publishing the PointCloud message!
Publishing the PointCloud message!
Waiting for grasp pose array message...


------

For MSMFormer:

- when running: (rgb)
~/compare_SceneReplica/src/UnseenObjectsWithMeanShift# ./tools/test_image_with_ms_transformer.py  --imgdir data/demo   --color *-color.png   --depth *-depth.png --pretrained data/checkpoints/rgb_pretrain/norm_RGB_pretrained.pth --pretrained_crop data/checkpoints/rgb_pretrain/crop_RGB_pretrained.pth --network_cfg MSMFormer/configs/mixture_UCN.yaml  --network_crop_cfg MSMFormer/configs/crop_mixture_UCN.yaml
- or simply: (rgbd)
~/compare_SceneReplica/src/UnseenObjectsWithMeanShift# ./experiments/scripts/demo_msmformer_rgbd.sh 
- got errors similar to:
torch.OutOfMemoryError: CUDA out of memory. Tried to allocate 938.00 MiB. GPU 0 has a total capacity of 5.60 GiB of which 49.44 MiB is free. Process 12622 has 948.00 MiB memory in use. Including non-PyTorch memory, this process has 4.06 GiB memory in use. Of the allocated memory 3.73 GiB is allocated by PyTorch, and 233.73 MiB is reserved by PyTorch but unallocated. If reserved but unallocated memory is large try setting PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True to avoid fragmentation.  See documentation for Memory Management  (https://pytorch.org/docs/stable/notes/cuda.html#environment-variables)
root@nerve-desktop-6:~/compare_SceneReplica/src/UnseenObjectsWithMeanShift# 


-------

ROS Topic List when running gazebo option:

root@nerve-desktop-6:~/compare_SceneReplica/src/UnseenObjectsWithMeanShift# rostopic list 
/arm_controller/follow_joint_trajectory/cancel
/arm_controller/follow_joint_trajectory/feedback
/arm_controller/follow_joint_trajectory/goal
/arm_controller/follow_joint_trajectory/result
/arm_controller/follow_joint_trajectory/status
/arm_with_torso_controller/follow_joint_trajectory/cancel
/arm_with_torso_controller/follow_joint_trajectory/feedback
/arm_with_torso_controller/follow_joint_trajectory/goal
/arm_with_torso_controller/follow_joint_trajectory/result
/arm_with_torso_controller/follow_joint_trajectory/status
/attached_collision_object
/base_controller/command
/base_scan
/clock
/cmd_vel
/cmd_vel_mux/selected
/collision_object
/execute_trajectory/cancel
/execute_trajectory/feedback
/execute_trajectory/goal
/execute_trajectory/result
/execute_trajectory/status
/gazebo/bellows_joint/position/parameter_descriptions
/gazebo/bellows_joint/position/parameter_updates
/gazebo/bellows_joint/velocity/parameter_descriptions
/gazebo/bellows_joint/velocity/parameter_updates
/gazebo/elbow_flex_joint/position/parameter_descriptions
/gazebo/elbow_flex_joint/position/parameter_updates
/gazebo/elbow_flex_joint/velocity/parameter_descriptions
/gazebo/elbow_flex_joint/velocity/parameter_updates
/gazebo/forearm_roll_joint/position/parameter_descriptions
/gazebo/forearm_roll_joint/position/parameter_updates
/gazebo/forearm_roll_joint/velocity/parameter_descriptions
/gazebo/forearm_roll_joint/velocity/parameter_updates
/gazebo/head_pan_joint/position/parameter_descriptions
/gazebo/head_pan_joint/position/parameter_updates
/gazebo/head_pan_joint/velocity/parameter_descriptions
/gazebo/head_pan_joint/velocity/parameter_updates
/gazebo/head_tilt_joint/position/parameter_descriptions
/gazebo/head_tilt_joint/position/parameter_updates
/gazebo/head_tilt_joint/velocity/parameter_descriptions
/gazebo/head_tilt_joint/velocity/parameter_updates
/gazebo/l_gripper_finger_joint/position/parameter_descriptions
/gazebo/l_gripper_finger_joint/position/parameter_updates
/gazebo/l_gripper_finger_joint/velocity/parameter_descriptions
/gazebo/l_gripper_finger_joint/velocity/parameter_updates
/gazebo/l_wheel_joint/position/parameter_descriptions
/gazebo/l_wheel_joint/position/parameter_updates
/gazebo/l_wheel_joint/velocity/parameter_descriptions
/gazebo/l_wheel_joint/velocity/parameter_updates
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/r_gripper_finger_joint/position/parameter_descriptions
/gazebo/r_gripper_finger_joint/position/parameter_updates
/gazebo/r_gripper_finger_joint/velocity/parameter_descriptions
/gazebo/r_gripper_finger_joint/velocity/parameter_updates
/gazebo/r_wheel_joint/position/parameter_descriptions
/gazebo/r_wheel_joint/position/parameter_updates
/gazebo/r_wheel_joint/velocity/parameter_descriptions
/gazebo/r_wheel_joint/velocity/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/gazebo/shoulder_lift_joint/position/parameter_descriptions
/gazebo/shoulder_lift_joint/position/parameter_updates
/gazebo/shoulder_lift_joint/velocity/parameter_descriptions
/gazebo/shoulder_lift_joint/velocity/parameter_updates
/gazebo/shoulder_pan_joint/position/parameter_descriptions
/gazebo/shoulder_pan_joint/position/parameter_updates
/gazebo/shoulder_pan_joint/velocity/parameter_descriptions
/gazebo/shoulder_pan_joint/velocity/parameter_updates
/gazebo/torso_lift_joint/position/parameter_descriptions
/gazebo/torso_lift_joint/position/parameter_updates
/gazebo/torso_lift_joint/velocity/parameter_descriptions
/gazebo/torso_lift_joint/velocity/parameter_updates
/gazebo/upperarm_roll_joint/position/parameter_descriptions
/gazebo/upperarm_roll_joint/position/parameter_updates
/gazebo/upperarm_roll_joint/velocity/parameter_descriptions
/gazebo/upperarm_roll_joint/velocity/parameter_updates
/gazebo/wrist_flex_joint/position/parameter_descriptions
/gazebo/wrist_flex_joint/position/parameter_updates
/gazebo/wrist_flex_joint/velocity/parameter_descriptions
/gazebo/wrist_flex_joint/velocity/parameter_updates
/gazebo/wrist_roll_joint/position/parameter_descriptions
/gazebo/wrist_roll_joint/position/parameter_updates
/gazebo/wrist_roll_joint/velocity/parameter_descriptions
/gazebo/wrist_roll_joint/velocity/parameter_updates
/gripper_controller/gripper_action/cancel
/gripper_controller/gripper_action/feedback
/gripper_controller/gripper_action/goal
/gripper_controller/gripper_action/result
/gripper_controller/gripper_action/status
/head_camera/crop_decimate/parameter_descriptions
/head_camera/crop_decimate/parameter_updates
/head_camera/depth_downsample/camera_info
/head_camera/depth_downsample/image_raw
/head_camera/depth_downsample/image_raw/compressed
/head_camera/depth_downsample/image_raw/compressed/parameter_descriptions
/head_camera/depth_downsample/image_raw/compressed/parameter_updates
/head_camera/depth_downsample/image_raw/compressedDepth
/head_camera/depth_downsample/image_raw/compressedDepth/parameter_descriptions
/head_camera/depth_downsample/image_raw/compressedDepth/parameter_updates
/head_camera/depth_downsample/image_raw/theora
/head_camera/depth_downsample/image_raw/theora/parameter_descriptions
/head_camera/depth_downsample/image_raw/theora/parameter_updates
/head_camera/depth_downsample/points
/head_camera/depth_registered/camera_info
/head_camera/depth_registered/image_raw
/head_camera/depth_registered/points
/head_camera/head_camera_nodelet_manager/bond
/head_camera/parameter_descriptions
/head_camera/parameter_updates
/head_camera/rgb/camera_info
/head_camera/rgb/image_raw
/head_camera/rgb/image_raw/compressed
/head_camera/rgb/image_raw/compressed/parameter_descriptions
/head_camera/rgb/image_raw/compressed/parameter_updates
/head_camera/rgb/image_raw/compressedDepth
/head_camera/rgb/image_raw/compressedDepth/parameter_descriptions
/head_camera/rgb/image_raw/compressedDepth/parameter_updates
/head_camera/rgb/image_raw/theora
/head_camera/rgb/image_raw/theora/parameter_descriptions
/head_camera/rgb/image_raw/theora/parameter_updates
/head_controller/follow_joint_trajectory/cancel
/head_controller/follow_joint_trajectory/feedback
/head_controller/follow_joint_trajectory/goal
/head_controller/follow_joint_trajectory/result
/head_controller/follow_joint_trajectory/status
/head_controller/point_head/cancel
/head_controller/point_head/feedback
/head_controller/point_head/goal
/head_controller/point_head/result
/head_controller/point_head/status
/joint_states
/move_group/cancel
/move_group/display_contacts
/move_group/display_cost_sources
/move_group/display_grasp_markers
/move_group/display_planned_path
/move_group/feedback
/move_group/goal
/move_group/monitored_planning_scene
/move_group/motion_plan_request
/move_group/ompl/parameter_descriptions
/move_group/ompl/parameter_updates
/move_group/plan_execution/parameter_descriptions
/move_group/plan_execution/parameter_updates
/move_group/planning_scene_monitor/parameter_descriptions
/move_group/planning_scene_monitor/parameter_updates
/move_group/result
/move_group/sense_for_plan/parameter_descriptions
/move_group/sense_for_plan/parameter_updates
/move_group/status
/move_group/trajectory_execution/parameter_descriptions
/move_group/trajectory_execution/parameter_updates
/odom
/pickup/cancel
/pickup/feedback
/pickup/goal
/pickup/result
/pickup/status
/place/cancel
/place/feedback
/place/goal
/place/result
/place/status
/planning_scene
/planning_scene_world
/query_controller_states/cancel
/query_controller_states/feedback
/query_controller_states/goal
/query_controller_states/result
/query_controller_states/status
/recognized_object_array
/rosout
/rosout_agg
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full
/rviz_nerve_desktop_6_15535_8238869937198517476/motionplanning_planning_scene_monitor/parameter_descriptions
/rviz_nerve_desktop_6_15535_8238869937198517476/motionplanning_planning_scene_monitor/parameter_updates
/teleop/cmd_vel
/tf
/tf_static
/torso_controller/follow_joint_trajectory/cancel
/torso_controller/follow_joint_trajectory/feedback
/torso_controller/follow_joint_trajectory/goal
/torso_controller/follow_joint_trajectory/result
/torso_controller/follow_joint_trajectory/status
/trajectory_execution_event
/visualization_marker_array


rosservice list 
/VizSceneSim/get_loggers
/VizSceneSim/set_logger_level
/apply_planning_scene
/base_controller/command_select
/check_state_validity
/clear_octomap
/cmd_vel_mux/add
/cmd_vel_mux/delete
/cmd_vel_mux/get_loggers
/cmd_vel_mux/list
/cmd_vel_mux/select
/cmd_vel_mux/set_logger_level
/compute_cartesian_path
/compute_fk
/compute_ik
/gazebo/apply_body_wrench
/gazebo/apply_joint_effort
/gazebo/bellows_joint/position/set_parameters
/gazebo/bellows_joint/velocity/set_parameters
/gazebo/clear_body_wrenches
/gazebo/clear_joint_forces
/gazebo/delete_light
/gazebo/delete_model
/gazebo/elbow_flex_joint/position/set_parameters
/gazebo/elbow_flex_joint/velocity/set_parameters
/gazebo/forearm_roll_joint/position/set_parameters
/gazebo/forearm_roll_joint/velocity/set_parameters
/gazebo/get_joint_properties
/gazebo/get_light_properties
/gazebo/get_link_properties
/gazebo/get_link_state
/gazebo/get_loggers
/gazebo/get_model_properties
/gazebo/get_model_state
/gazebo/get_physics_properties
/gazebo/get_world_properties
/gazebo/head_pan_joint/position/set_parameters
/gazebo/head_pan_joint/velocity/set_parameters
/gazebo/head_tilt_joint/position/set_parameters
/gazebo/head_tilt_joint/velocity/set_parameters
/gazebo/l_gripper_finger_joint/position/set_parameters
/gazebo/l_gripper_finger_joint/velocity/set_parameters
/gazebo/l_wheel_joint/position/set_parameters
/gazebo/l_wheel_joint/velocity/set_parameters
/gazebo/pause_physics
/gazebo/r_gripper_finger_joint/position/set_parameters
/gazebo/r_gripper_finger_joint/velocity/set_parameters
/gazebo/r_wheel_joint/position/set_parameters
/gazebo/r_wheel_joint/velocity/set_parameters
/gazebo/reset_simulation
/gazebo/reset_world
/gazebo/set_joint_properties
/gazebo/set_light_properties
/gazebo/set_link_properties
/gazebo/set_link_state
/gazebo/set_logger_level
/gazebo/set_model_configuration
/gazebo/set_model_state
/gazebo/set_parameters
/gazebo/set_physics_properties
/gazebo/shoulder_lift_joint/position/set_parameters
/gazebo/shoulder_lift_joint/velocity/set_parameters
/gazebo/shoulder_pan_joint/position/set_parameters
/gazebo/shoulder_pan_joint/velocity/set_parameters
/gazebo/spawn_sdf_model
/gazebo/spawn_urdf_model
/gazebo/torso_lift_joint/position/set_parameters
/gazebo/torso_lift_joint/velocity/set_parameters
/gazebo/unpause_physics
/gazebo/upperarm_roll_joint/position/set_parameters
/gazebo/upperarm_roll_joint/velocity/set_parameters
/gazebo/wrist_flex_joint/position/set_parameters
/gazebo/wrist_flex_joint/velocity/set_parameters
/gazebo/wrist_roll_joint/position/set_parameters
/gazebo/wrist_roll_joint/velocity/set_parameters
/gazebo_gui/get_loggers
/gazebo_gui/set_logger_level
/get_planner_params
/get_planning_scene
/head_camera/crop_decimate/get_loggers
/head_camera/crop_decimate/set_logger_level
/head_camera/crop_decimate/set_parameters
/head_camera/depth_downsample/image_raw/compressed/set_parameters
/head_camera/depth_downsample/image_raw/compressedDepth/set_parameters
/head_camera/depth_downsample/image_raw/theora/set_parameters
/head_camera/depth_downsample/points_downsample/get_loggers
/head_camera/depth_downsample/points_downsample/set_logger_level
/head_camera/head_camera_nodelet_manager/get_loggers
/head_camera/head_camera_nodelet_manager/list
/head_camera/head_camera_nodelet_manager/load_nodelet
/head_camera/head_camera_nodelet_manager/set_logger_level
/head_camera/head_camera_nodelet_manager/unload_nodelet
/head_camera/rgb/image_raw/compressed/set_parameters
/head_camera/rgb/image_raw/compressedDepth/set_parameters
/head_camera/rgb/image_raw/theora/set_parameters
/head_camera/set_camera_info
/head_camera/set_parameters
/move_group/get_loggers
/move_group/load_map
/move_group/ompl/set_parameters
/move_group/plan_execution/set_parameters
/move_group/planning_scene_monitor/set_parameters
/move_group/save_map
/move_group/sense_for_plan/set_parameters
/move_group/set_logger_level
/move_group/trajectory_execution/set_parameters
/move_group_commander_wrappers_1753738710001817523/get_loggers
/move_group_commander_wrappers_1753738710001817523/set_logger_level
/plan_kinematic_path
/query_planner_interface
/ramp_grasping/get_loggers
/ramp_grasping/set_logger_level
/ramp_grasping/tf2_frames
/robot_state_publisher/get_loggers
/robot_state_publisher/set_logger_level
/rosout/get_loggers
/rosout/set_logger_level
/rviz_nerve_desktop_6_15535_8238869937198517476/get_loggers
/rviz_nerve_desktop_6_15535_8238869937198517476/load_config
/rviz_nerve_desktop_6_15535_8238869937198517476/load_config_discarding_changes
/rviz_nerve_desktop_6_15535_8238869937198517476/motionplanning_planning_scene_monitor/set_parameters
/rviz_nerve_desktop_6_15535_8238869937198517476/reload_shaders
/rviz_nerve_desktop_6_15535_8238869937198517476/save_config
/rviz_nerve_desktop_6_15535_8238869937198517476/set_logger_level
/set_planner_params

-----

rosservice info /gazebo/get_model_state 
Node: /gazebo
URI: rosrpc://nerve-desktop-6:43765
Type: gazebo_msgs/GetModelState
Args: model_name relative_entity_name


-----

using poseCNN:

----

root@nerve-desktop-6:~# rostopic list 
/arm_controller/follow_joint_trajectory/cancel
/arm_controller/follow_joint_trajectory/feedback
/arm_controller/follow_joint_trajectory/goal
/arm_controller/follow_joint_trajectory/result
/arm_controller/follow_joint_trajectory/status
/arm_with_torso_controller/follow_joint_trajectory/cancel
/arm_with_torso_controller/follow_joint_trajectory/feedback
/arm_with_torso_controller/follow_joint_trajectory/goal
/arm_with_torso_controller/follow_joint_trajectory/result
/arm_with_torso_controller/follow_joint_trajectory/status
/attached_collision_object
/base_controller/command
/base_scan
/clicked_point
/clock
/cmd_vel
/cmd_vel_mux/selected
/collision_object
/deepim_pose_image_00/mouse_click
/execute_trajectory/cancel
/execute_trajectory/feedback
/execute_trajectory/goal
/execute_trajectory/result
/execute_trajectory/status
/gazebo/bellows_joint/position/parameter_descriptions
/gazebo/bellows_joint/position/parameter_updates
/gazebo/bellows_joint/velocity/parameter_descriptions
/gazebo/bellows_joint/velocity/parameter_updates
/gazebo/elbow_flex_joint/position/parameter_descriptions
/gazebo/elbow_flex_joint/position/parameter_updates
/gazebo/elbow_flex_joint/velocity/parameter_descriptions
/gazebo/elbow_flex_joint/velocity/parameter_updates
/gazebo/forearm_roll_joint/position/parameter_descriptions
/gazebo/forearm_roll_joint/position/parameter_updates
/gazebo/forearm_roll_joint/velocity/parameter_descriptions
/gazebo/forearm_roll_joint/velocity/parameter_updates
/gazebo/head_pan_joint/position/parameter_descriptions
/gazebo/head_pan_joint/position/parameter_updates
/gazebo/head_pan_joint/velocity/parameter_descriptions
/gazebo/head_pan_joint/velocity/parameter_updates
/gazebo/head_tilt_joint/position/parameter_descriptions
/gazebo/head_tilt_joint/position/parameter_updates
/gazebo/head_tilt_joint/velocity/parameter_descriptions
/gazebo/head_tilt_joint/velocity/parameter_updates
/gazebo/l_gripper_finger_joint/position/parameter_descriptions
/gazebo/l_gripper_finger_joint/position/parameter_updates
/gazebo/l_gripper_finger_joint/velocity/parameter_descriptions
/gazebo/l_gripper_finger_joint/velocity/parameter_updates
/gazebo/l_wheel_joint/position/parameter_descriptions
/gazebo/l_wheel_joint/position/parameter_updates
/gazebo/l_wheel_joint/velocity/parameter_descriptions
/gazebo/l_wheel_joint/velocity/parameter_updates
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/r_gripper_finger_joint/position/parameter_descriptions
/gazebo/r_gripper_finger_joint/position/parameter_updates
/gazebo/r_gripper_finger_joint/velocity/parameter_descriptions
/gazebo/r_gripper_finger_joint/velocity/parameter_updates
/gazebo/r_wheel_joint/position/parameter_descriptions
/gazebo/r_wheel_joint/position/parameter_updates
/gazebo/r_wheel_joint/velocity/parameter_descriptions
/gazebo/r_wheel_joint/velocity/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/gazebo/shoulder_lift_joint/position/parameter_descriptions
/gazebo/shoulder_lift_joint/position/parameter_updates
/gazebo/shoulder_lift_joint/velocity/parameter_descriptions
/gazebo/shoulder_lift_joint/velocity/parameter_updates
/gazebo/shoulder_pan_joint/position/parameter_descriptions
/gazebo/shoulder_pan_joint/position/parameter_updates
/gazebo/shoulder_pan_joint/velocity/parameter_descriptions
/gazebo/shoulder_pan_joint/velocity/parameter_updates
/gazebo/torso_lift_joint/position/parameter_descriptions
/gazebo/torso_lift_joint/position/parameter_updates
/gazebo/torso_lift_joint/velocity/parameter_descriptions
/gazebo/torso_lift_joint/velocity/parameter_updates
/gazebo/upperarm_roll_joint/position/parameter_descriptions
/gazebo/upperarm_roll_joint/position/parameter_updates
/gazebo/upperarm_roll_joint/velocity/parameter_descriptions
/gazebo/upperarm_roll_joint/velocity/parameter_updates
/gazebo/wrist_flex_joint/position/parameter_descriptions
/gazebo/wrist_flex_joint/position/parameter_updates
/gazebo/wrist_flex_joint/velocity/parameter_descriptions
/gazebo/wrist_flex_joint/velocity/parameter_updates
/gazebo/wrist_roll_joint/position/parameter_descriptions
/gazebo/wrist_roll_joint/position/parameter_updates
/gazebo/wrist_roll_joint/velocity/parameter_descriptions
/gazebo/wrist_roll_joint/velocity/parameter_updates
/gripper_controller/gripper_action/cancel
/gripper_controller/gripper_action/feedback
/gripper_controller/gripper_action/goal
/gripper_controller/gripper_action/result
/gripper_controller/gripper_action/status
/head_camera/crop_decimate/parameter_descriptions
/head_camera/crop_decimate/parameter_updates
/head_camera/depth_downsample/camera_info
/head_camera/depth_downsample/image_raw
/head_camera/depth_downsample/image_raw/compressed
/head_camera/depth_downsample/image_raw/compressed/parameter_descriptions
/head_camera/depth_downsample/image_raw/compressed/parameter_updates
/head_camera/depth_downsample/image_raw/compressedDepth
/head_camera/depth_downsample/image_raw/compressedDepth/parameter_descriptions
/head_camera/depth_downsample/image_raw/compressedDepth/parameter_updates
/head_camera/depth_downsample/image_raw/theora
/head_camera/depth_downsample/image_raw/theora/parameter_descriptions
/head_camera/depth_downsample/image_raw/theora/parameter_updates
/head_camera/depth_downsample/points
/head_camera/depth_registered/camera_info
/head_camera/depth_registered/image_raw
/head_camera/depth_registered/image_raw/mouse_click
/head_camera/depth_registered/points
/head_camera/head_camera_nodelet_manager/bond
/head_camera/parameter_descriptions
/head_camera/parameter_updates
/head_camera/rgb/camera_info
/head_camera/rgb/image_raw
/head_camera/rgb/image_raw/compressed
/head_camera/rgb/image_raw/compressed/parameter_descriptions
/head_camera/rgb/image_raw/compressed/parameter_updates
/head_camera/rgb/image_raw/compressedDepth
/head_camera/rgb/image_raw/compressedDepth/parameter_descriptions
/head_camera/rgb/image_raw/compressedDepth/parameter_updates
/head_camera/rgb/image_raw/mouse_click
/head_camera/rgb/image_raw/theora
/head_camera/rgb/image_raw/theora/parameter_descriptions
/head_camera/rgb/image_raw/theora/parameter_updates
/head_controller/follow_joint_trajectory/cancel
/head_controller/follow_joint_trajectory/feedback
/head_controller/follow_joint_trajectory/goal
/head_controller/follow_joint_trajectory/result
/head_controller/follow_joint_trajectory/status
/head_controller/point_head/cancel
/head_controller/point_head/feedback
/head_controller/point_head/goal
/head_controller/point_head/result
/head_controller/point_head/status
/initialpose
/joint_states
/move_base_simple/goal
/move_group/cancel
/move_group/display_contacts
/move_group/display_cost_sources
/move_group/display_grasp_markers
/move_group/display_planned_path
/move_group/feedback
/move_group/goal
/move_group/monitored_planning_scene
/move_group/motion_plan_request
/move_group/ompl/parameter_descriptions
/move_group/ompl/parameter_updates
/move_group/plan_execution/parameter_descriptions
/move_group/plan_execution/parameter_updates
/move_group/planning_scene_monitor/parameter_descriptions
/move_group/planning_scene_monitor/parameter_updates
/move_group/result
/move_group/sense_for_plan/parameter_descriptions
/move_group/sense_for_plan/parameter_updates
/move_group/status
/move_group/trajectory_execution/parameter_descriptions
/move_group/trajectory_execution/parameter_updates
/objects/prior_pose/00_banana
/objects/prior_pose/00_bleach_cleanser
/objects/prior_pose/00_bowl
/objects/prior_pose/00_cracker_box
/objects/prior_pose/00_extra_large_clamp
/objects/prior_pose/00_foam_brick
/objects/prior_pose/00_gelatin_box
/objects/prior_pose/00_large_marker
/objects/prior_pose/00_master_chef_can
/objects/prior_pose/00_mug
/objects/prior_pose/00_mustard_bottle
/objects/prior_pose/00_pitcher_base
/objects/prior_pose/00_potted_meat_can
/objects/prior_pose/00_power_drill
/objects/prior_pose/00_pudding_box
/objects/prior_pose/00_scissors
/objects/prior_pose/00_sugar_box
/objects/prior_pose/00_tomato_soup_can
/objects/prior_pose/00_tuna_fish_can
/objects/prior_pose/00_wood_block
/odom
/pickup/cancel
/pickup/feedback
/pickup/goal
/pickup/result
/pickup/status
/place/cancel
/place/feedback
/place/goal
/place/result
/place/status
/planning_scene
/planning_scene_world
/posecnn_label_00
/posecnn_label_00/mouse_click
/posecnn_mask_00
/posecnn_pose_00
/posecnn_pose_00/mouse_click
/posecnn_pose_refined_00
/posecnn_pose_refined_00/mouse_click
/query_controller_states/cancel
/query_controller_states/feedback
/query_controller_states/goal
/query_controller_states/result
/query_controller_states/status
/recognized_object_array
/rosout
/rosout_agg
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full
/rviz_nerve_desktop_6_15535_8238869937198517476/motionplanning_planning_scene_monitor/parameter_descriptions
/rviz_nerve_desktop_6_15535_8238869937198517476/motionplanning_planning_scene_monitor/parameter_updates
/teleop/cmd_vel
/tf
/tf_static
/torso_controller/follow_joint_trajectory/cancel
/torso_controller/follow_joint_trajectory/feedback
/torso_controller/follow_joint_trajectory/goal
/torso_controller/follow_joint_trajectory/result
/torso_controller/follow_joint_trajectory/status
/trajectory_execution_event
/visualization_marker_array

----
rostopic echo: 
/posecnn_pose_refined_00, /posecnn_label_00, etc., are actively publishing

----
----

Add to .sh: export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
Add to .py: import torch
torch.cuda.empty_cache()


modify cuda torch dtype: from dtype=torch.float32 to dtype=torch.float16 (25/07/31)

compare_SceneReplica/src/PoseCNN-PyTorch-NV-Release/lib/sdf/test_sdf_optimizer.py
compare_SceneReplica/src/PoseCNN-PyTorch-NV-Release/lib/sdf/multi_sdf_optimizer.py
compare_SceneReplica/src/PoseCNN-PyTorch-NV-Release/lib/sdf/sdf_optimizer.py [reverted back due to lib/layers/sdf_matching_loss.py error]
compare_SceneReplica/src/PoseCNN-PyTorch-NV-Release/lib/sdf/sdf_utils.py
compare_SceneReplica/src/PoseCNN-PyTorch-NV-Release/lib/fcn/test_common.py [reverted back due to lib/layers/sdf_matching_loss.py error]

- Error with sdf_matching_loss: "outputs = posecnn_cuda.sdf_loss_forward(pose_delta, pose_init, sdf_grids, sdf_limits, points, regularization) RuntimeError: expected scalar type Float but found Half"

compare_SceneReplica/src/UnseenObjectsWithMeanShift/MSMFormer/meanshiftformer/modeling/transformer_decoder/position_encoding.py
compare_SceneReplica/src/UnseenObjectsWithMeanShift/MSMFormer/meanshiftformer/modeling/pixel_decoder/ops/modules/ms_deform_attn.py
compare_SceneReplica/src/UnseenObjectsWithMeanShift/MSMFormer/meanshiftformer/modeling/pixel_decoder/msdeformattn.py

NOT YET CHANGED:
src/UnseenObjectsWithMeanShift/MSMFormer/meanshiftformer/modeling/pixel_decoder/ops/build/lib.linux-x86_64-3.8/modules/ms_deform_attn.py
src/UnseenObjectsWithMeanShift/MSMFormer/meanshiftformer/modeling/pixel_decoder/ops/build/lib.linux-x86_64-cpython-38/modules/ms_deform_attn.py

-------

- check cuda memory dynamically:
watch -n 1 nvidia-smi

-------

25/07/31 cuda out of memory error occurs when running uois with contact-graspnet:

[LISTENER] Running network...
[LISTENER] Scaling Input Points
[LISTENER] Filtering point cloud based on Z range
[LISTENER] Predicting Grasps....
Extracted Region Cube Size:  0.3
2025-07-31 20:22:30.409789: I tensorflow/stream_executor/platform/default/dso_loader.cc:44] Successfully opened dynamic library libcudnn.so.7
2025-07-31 20:22:32.417959: I tensorflow/stream_executor/platform/default/dso_loader.cc:44] Successfully opened dynamic library libcublas.so.10
2025-07-31 20:22:32.859693: I tensorflow/stream_executor/cuda/cuda_driver.cc:763] failed to allocate 735.56M (771293184 bytes) from device: CUDA_ERROR_OUT_OF_MEMORY: out of memory
2025-07-31 20:22:32.860073: I tensorflow/stream_executor/cuda/cuda_driver.cc:763] failed to allocate 662.01M (694163968 bytes) from device: CUDA_ERROR_OUT_OF_MEMORY: out of memory
2025-07-31 20:22:32.860266: I tensorflow/stream_executor/cuda/cuda_driver.cc:763] failed to allocate 595.81M (624747776 bytes) from device: CUDA_ERROR_OUT_OF_MEMORY: out of memory
2025-07-31 20:22:32.897727: W tensorflow/core/common_runtime/bfc_allocator.cc:311] Garbage collection: deallocate free memory regions (i.e., allocations) so that we can re-allocate a larger region to avoid OOM due to memory fragmentation. If you see this message frequently, you are running near the threshold of the available device memory and re-allocation may incur great performance overhead. You may try smaller batch sizes to observe the performance impact. Set TF_ENABLE_GPU_GARBAGE_COLLECTION=false if you'd like to disable this feature.
2025-07-31 20:22:32.901607: I tensorflow/stream_executor/cuda/cuda_driver.cc:763] failed to allocate 343.34M (360015104 bytes) from device: CUDA_ERROR_OUT_OF_MEMORY: out of memory
2025-07-31 20:22:32.901720: W tensorflow/core/common_runtime/bfc_allocator.cc:245] Allocator (GPU_0_bfc) ran out of memory trying to allocate 272.00MiB with freed_by_count=0. The caller indicates that this is not a failure, but may mean that there could be performance gains if more memory were available.
2025-07-31 20:22:32.901916: I tensorflow/stream_executor/cuda/cuda_driver.cc:763] failed to allocate 343.34M (360015104 bytes) from device: CUDA_ERROR_OUT_OF_MEMORY: out of memory
2025-07-31 20:22:32.901934: W tensorflow/core/common_runtime/bfc_allocator.cc:245] Allocator (GPU_0_bfc) ran out of memory trying to allocate 272.00MiB with freed_by_count=0. The caller indicates that this is not a failure, but may mean that there could be performance gains if more memory were available.
2025-07-31 20:22:32.902040: W tensorflow/core/common_runtime/bfc_allocator.cc:245] Allocator (GPU_0_bfc) ran out of memory trying to allocate 356.80MiB with freed_by_count=0. The caller indicates that this is not a failure, but may mean that there could be performance gains if more memory were available.
2025-07-31 20:22:32.902083: W tensorflow/core/common_runtime/bfc_allocator.cc:245] Allocator (GPU_0_bfc) ran out of memory trying to allocate 356.80MiB with freed_by_count=0. The caller indicates that this is not a failure, but may mean that there could be performance gains if more memory were available.
2025-07-31 20:22:32.958489: W tensorflow/core/common_runtime/bfc_allocator.cc:245] Allocator (GPU_0_bfc) ran out of memory trying to allocate 425.06MiB with freed_by_count=0. The caller indicates that this is not a failure, but may mean that there could be performance gains if more memory were available.
2025-07-31 20:22:32.958539: W tensorflow/core/common_runtime/bfc_allocator.cc:245] Allocator (GPU_0_bfc) ran out of memory trying to allocate 425.06MiB with freed_by_count=0. The caller indicates that this is not a failure, but may mean that there could be performance gains if more memory were available.
2025-07-31 20:22:32.958761: W tensorflow/core/common_runtime/bfc_allocator.cc:245] Allocator (GPU_0_bfc) ran out of memory trying to allocate 528.00MiB with freed_by_count=0. The caller indicates that this is not a failure, but may mean that there could be performance gains if more memory were available.
2025-07-31 20:22:32.958777: W tensorflow/core/kernels/gpu_utils.cc:49] Failed to allocate memory for convolution redzone checking; skipping this check. This is benign and only means that we won't check cudnn for out-of-bounds reads and writes. This message will only be printed once.
2025-07-31 20:22:32.991305: W tensorflow/core/common_runtime/bfc_allocator.cc:245] Allocator (GPU_0_bfc) ran out of memory trying to allocate 400.00MiB with freed_by_count=0. The caller indicates that this is not a failure, but may mean that there could be performance gains if more memory were available.
2025-07-31 20:22:32.991418: W tensorflow/core/common_runtime/bfc_allocator.cc:245] Allocator (GPU_0_bfc) ran out of memory trying to allocate 400.00MiB with freed_by_count=0. The caller indicates that this is not a failure, but may mean that there could be performance gains if more memory were available.
2025-07-31 20:22:32.991477: W tensorflow/core/common_runtime/bfc_allocator.cc:245] Allocator (GPU_0_bfc) ran out of memory trying to allocate 493.59MiB with freed_by_count=0. The caller indicates that this is not a failure, but may mean that there could be performance gains if more memory were available.
2025-07-31 20:22:33.007814: W tensorflow/core/common_runtime/bfc_allocator.cc:311] Garbage collection: deallocate free memory regions (i.e., allocations) so that we can re-allocate a larger region to avoid OOM due to memory fragmentation. If you see this message frequently, you are running near the threshold of the available device memory and re-allocation may incur great performance overhead. You may try smaller batch sizes to observe the performance impact. Set TF_ENABLE_GPU_GARBAGE_COLLECTION=false if you'd like to disable this feature.
2025-07-31 20:22:33.035692: I tensorflow/stream_executor/cuda/cuda_driver.cc:763] failed to allocate 663.34M (695559424 bytes) from device: CUDA_ERROR_OUT_OF_MEMORY: out of memory
2025-07-31 20:22:33.038419: I tensorflow/stream_executor/cuda/cuda_driver.cc:763] failed to allocate 663.34M (695559424 bytes) from device: CUDA_ERROR_OUT_OF_MEMORY: out of memory
2025-07-31 20:22:43.038719: I tensorflow/stream_executor/cuda/cuda_driver.cc:763] failed to allocate 663.34M (695559424 bytes) from device: CUDA_ERROR_OUT_OF_MEMORY: out of memory
2025-07-31 20:22:43.038933: I tensorflow/stream_executor/cuda/cuda_driver.cc:763] failed to allocate 663.34M (695559424 bytes) from device: CUDA_ERROR_OUT_OF_MEMORY: out of memory
2025-07-31 20:22:43.038955: W tensorflow/core/common_runtime/bfc_allocator.cc:434] Allocator (GPU_0_bfc) ran out of memory trying to allocate 512.00MiB (rounded to 536870912)
Current allocation summary follows.
2025-07-31 20:22:43.038974: I tensorflow/core/common_runtime/bfc_allocator.cc:934] BFCAllocator dump for GPU_0_bfc
2025-07-31 20:22:43.038984: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (256): 	Total Chunks: 59, Chunks in use: 58. 14.8KiB allocated for chunks. 14.5KiB in use in bin. 10.1KiB client-requested in use in bin.
2025-07-31 20:22:43.038992: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (512): 	Total Chunks: 114, Chunks in use: 111. 57.5KiB allocated for chunks. 55.5KiB in use in bin. 55.0KiB client-requested in use in bin.
2025-07-31 20:22:43.039000: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (1024): 	Total Chunks: 44, Chunks in use: 43. 45.5KiB allocated for chunks. 44.2KiB in use in bin. 44.0KiB client-requested in use in bin.
2025-07-31 20:22:43.039007: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (2048): 	Total Chunks: 5, Chunks in use: 5. 10.0KiB allocated for chunks. 10.0KiB in use in bin. 10.0KiB client-requested in use in bin.
2025-07-31 20:22:43.039015: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (4096): 	Total Chunks: 7, Chunks in use: 6. 29.0KiB allocated for chunks. 25.0KiB in use in bin. 25.0KiB client-requested in use in bin.
2025-07-31 20:22:43.039024: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (8192): 	Total Chunks: 1, Chunks in use: 0. 8.5KiB allocated for chunks. 0B in use in bin. 0B client-requested in use in bin.
2025-07-31 20:22:43.039033: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (16384): 	Total Chunks: 4, Chunks in use: 2. 72.2KiB allocated for chunks. 32.0KiB in use in bin. 32.0KiB client-requested in use in bin.
2025-07-31 20:22:43.039043: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (32768): 	Total Chunks: 4, Chunks in use: 2. 144.8KiB allocated for chunks. 64.0KiB in use in bin. 64.0KiB client-requested in use in bin.
2025-07-31 20:22:43.039052: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (65536): 	Total Chunks: 12, Chunks in use: 11. 857.2KiB allocated for chunks. 760.5KiB in use in bin. 720.8KiB client-requested in use in bin.
2025-07-31 20:22:43.039061: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (131072): 	Total Chunks: 10, Chunks in use: 10. 1.44MiB allocated for chunks. 1.44MiB in use in bin. 1.44MiB client-requested in use in bin.
2025-07-31 20:22:43.039070: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (262144): 	Total Chunks: 5, Chunks in use: 4. 1.50MiB allocated for chunks. 1.25MiB in use in bin. 1.25MiB client-requested in use in bin.
2025-07-31 20:22:43.039080: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (524288): 	Total Chunks: 4, Chunks in use: 4. 2.54MiB allocated for chunks. 2.54MiB in use in bin. 2.25MiB client-requested in use in bin.
2025-07-31 20:22:43.039088: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (1048576): 	Total Chunks: 3, Chunks in use: 3. 4.74MiB allocated for chunks. 4.74MiB in use in bin. 3.88MiB client-requested in use in bin.
2025-07-31 20:22:43.039097: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (2097152): 	Total Chunks: 5, Chunks in use: 3. 13.43MiB allocated for chunks. 7.60MiB in use in bin. 6.62MiB client-requested in use in bin.
2025-07-31 20:22:43.039116: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (4194304): 	Total Chunks: 1, Chunks in use: 0. 6.00MiB allocated for chunks. 0B in use in bin. 0B client-requested in use in bin.
2025-07-31 20:22:43.039126: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (8388608): 	Total Chunks: 0, Chunks in use: 0. 0B allocated for chunks. 0B in use in bin. 0B client-requested in use in bin.
2025-07-31 20:22:43.039134: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (16777216): 	Total Chunks: 1, Chunks in use: 0. 16.12MiB allocated for chunks. 0B in use in bin. 0B client-requested in use in bin.
2025-07-31 20:22:43.039143: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (33554432): 	Total Chunks: 0, Chunks in use: 0. 0B allocated for chunks. 0B in use in bin. 0B client-requested in use in bin.
2025-07-31 20:22:43.039150: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (67108864): 	Total Chunks: 1, Chunks in use: 1. 64.00MiB allocated for chunks. 64.00MiB in use in bin. 64.00MiB client-requested in use in bin.
2025-07-31 20:22:43.039158: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (134217728): 	Total Chunks: 1, Chunks in use: 0. 192.00MiB allocated for chunks. 0B in use in bin. 0B client-requested in use in bin.
2025-07-31 20:22:43.039178: I tensorflow/core/common_runtime/bfc_allocator.cc:941] Bin (268435456): 	Total Chunks: 2, Chunks in use: 2. 792.22MiB allocated for chunks. 792.22MiB in use in bin. 768.00MiB client-requested in use in bin.
2025-07-31 20:22:43.039200: I tensorflow/core/common_runtime/bfc_allocator.cc:957] Bin for 512.00MiB was 256.00MiB, Chunk State: 
2025-07-31 20:22:43.039212: I tensorflow/core/common_runtime/bfc_allocator.cc:970] Next region of size 562273024
2025-07-31 20:22:43.039228: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc12a000000 of size 562273024 next 18446744073709551615
2025-07-31 20:22:43.039240: I tensorflow/core/common_runtime/bfc_allocator.cc:970] Next region of size 536870912
2025-07-31 20:22:43.039253: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc16c000000 of size 67108864 next 207
2025-07-31 20:22:43.039266: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc170000000 of size 268435456 next 179
2025-07-31 20:22:43.039278: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc180000000 of size 201326592 next 18446744073709551615
2025-07-31 20:22:43.039291: I tensorflow/core/common_runtime/bfc_allocator.cc:970] Next region of size 33554432
2025-07-31 20:22:43.039306: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc24c000000 of size 99072 next 276
2025-07-31 20:22:43.039318: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc24c018300 of size 786432 next 293
2025-07-31 20:22:43.039330: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc24c0d8300 of size 2784512 next 283
2025-07-31 20:22:43.039343: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc24c380000 of size 1572864 next 284
2025-07-31 20:22:43.039355: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc24c500000 of size 6291456 next 289
2025-07-31 20:22:43.039367: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc24cb00000 of size 1572864 next 296
2025-07-31 20:22:43.039382: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc24cc80000 of size 3145728 next 297
2025-07-31 20:22:43.039396: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc24cf80000 of size 393216 next 298
2025-07-31 20:22:43.039409: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc24cfe0000 of size 16908288 next 18446744073709551615
2025-07-31 20:22:43.039421: I tensorflow/core/common_runtime/bfc_allocator.cc:970] Next region of size 1048576
2025-07-31 20:22:43.039440: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad200000 of size 1280 next 1
2025-07-31 20:22:43.039515: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad200500 of size 1024 next 2
2025-07-31 20:22:43.039525: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad200900 of size 4096 next 3
2025-07-31 20:22:43.039532: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad201900 of size 1024 next 4
2025-07-31 20:22:43.039539: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad201d00 of size 512 next 5
2025-07-31 20:22:43.039545: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad201f00 of size 512 next 6
2025-07-31 20:22:43.039551: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad202100 of size 512 next 7
2025-07-31 20:22:43.039556: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad202300 of size 512 next 8
2025-07-31 20:22:43.039563: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad202500 of size 512 next 9
2025-07-31 20:22:43.039568: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad202700 of size 512 next 10
2025-07-31 20:22:43.039574: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad202900 of size 512 next 11
2025-07-31 20:22:43.039579: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad202b00 of size 256 next 12
2025-07-31 20:22:43.039584: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad202c00 of size 512 next 13
2025-07-31 20:22:43.039589: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad202e00 of size 256 next 14
2025-07-31 20:22:43.039594: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad202f00 of size 512 next 15
2025-07-31 20:22:43.039600: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad203100 of size 512 next 16
2025-07-31 20:22:43.039606: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad203300 of size 1024 next 17
2025-07-31 20:22:43.039612: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad203700 of size 512 next 18
2025-07-31 20:22:43.039618: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad203900 of size 512 next 19
2025-07-31 20:22:43.039624: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad203b00 of size 256 next 20
2025-07-31 20:22:43.039630: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad203c00 of size 256 next 21
2025-07-31 20:22:43.039636: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad203d00 of size 1024 next 22
2025-07-31 20:22:43.039642: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad204100 of size 512 next 23
2025-07-31 20:22:43.039649: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad204300 of size 512 next 24
2025-07-31 20:22:43.039655: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad204500 of size 512 next 25
2025-07-31 20:22:43.039660: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad204700 of size 256 next 26
2025-07-31 20:22:43.039665: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad204800 of size 512 next 27
2025-07-31 20:22:43.039670: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad204a00 of size 512 next 28
2025-07-31 20:22:43.039675: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad204c00 of size 256 next 29
2025-07-31 20:22:43.039680: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad204d00 of size 512 next 30
2025-07-31 20:22:43.039685: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad204f00 of size 512 next 31
2025-07-31 20:22:43.039691: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad205100 of size 256 next 32
2025-07-31 20:22:43.039695: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad205200 of size 512 next 33
2025-07-31 20:22:43.039700: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad205400 of size 256 next 34
2025-07-31 20:22:43.039706: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad205500 of size 512 next 35
2025-07-31 20:22:43.039711: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad205700 of size 512 next 36
2025-07-31 20:22:43.039716: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad205900 of size 256 next 37
2025-07-31 20:22:43.039721: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad205a00 of size 256 next 38
2025-07-31 20:22:43.039726: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad205b00 of size 1024 next 39
2025-07-31 20:22:43.039731: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad205f00 of size 512 next 40
2025-07-31 20:22:43.039737: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad206100 of size 1024 next 41
2025-07-31 20:22:43.039741: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad206500 of size 256 next 42
2025-07-31 20:22:43.039747: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad206600 of size 512 next 43
2025-07-31 20:22:43.039752: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad206800 of size 512 next 44
2025-07-31 20:22:43.039791: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad206a00 of size 1024 next 45
2025-07-31 20:22:43.039814: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad206e00 of size 512 next 46
2025-07-31 20:22:43.039839: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad207000 of size 512 next 47
2025-07-31 20:22:43.039861: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad207200 of size 1024 next 48
2025-07-31 20:22:43.039882: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad207600 of size 512 next 49
2025-07-31 20:22:43.039902: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad207800 of size 512 next 50
2025-07-31 20:22:43.039923: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad207a00 of size 512 next 51
2025-07-31 20:22:43.039944: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad207c00 of size 512 next 52
2025-07-31 20:22:43.039990: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad207e00 of size 512 next 53
2025-07-31 20:22:43.040078: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad208000 of size 512 next 54
2025-07-31 20:22:43.040101: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad208200 of size 512 next 55
2025-07-31 20:22:43.040113: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad208400 of size 512 next 56
2025-07-31 20:22:43.040124: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad208600 of size 1024 next 57
2025-07-31 20:22:43.040134: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad208a00 of size 512 next 58
2025-07-31 20:22:43.040145: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad208c00 of size 256 next 59
2025-07-31 20:22:43.040155: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad208d00 of size 512 next 60
2025-07-31 20:22:43.040166: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad208f00 of size 512 next 61
2025-07-31 20:22:43.040176: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad209100 of size 512 next 62
2025-07-31 20:22:43.040187: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad209300 of size 512 next 63
2025-07-31 20:22:43.040197: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad209500 of size 512 next 64
2025-07-31 20:22:43.040208: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad209700 of size 512 next 65
2025-07-31 20:22:43.040267: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad209900 of size 512 next 66
2025-07-31 20:22:43.040280: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad209b00 of size 512 next 67
2025-07-31 20:22:43.040291: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad209d00 of size 1024 next 68
2025-07-31 20:22:43.040302: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20a100 of size 1024 next 69
2025-07-31 20:22:43.040312: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20a500 of size 512 next 70
2025-07-31 20:22:43.040323: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20a700 of size 512 next 71
2025-07-31 20:22:43.040333: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20a900 of size 512 next 72
2025-07-31 20:22:43.040343: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20ab00 of size 512 next 73
2025-07-31 20:22:43.040353: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20ad00 of size 1024 next 74
2025-07-31 20:22:43.040377: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20b100 of size 256 next 75
2025-07-31 20:22:43.040389: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20b200 of size 512 next 76
2025-07-31 20:22:43.040400: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20b400 of size 512 next 77
2025-07-31 20:22:43.040410: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20b600 of size 512 next 78
2025-07-31 20:22:43.040421: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20b800 of size 512 next 79
2025-07-31 20:22:43.040431: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20ba00 of size 512 next 80
2025-07-31 20:22:43.040441: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20bc00 of size 512 next 81
2025-07-31 20:22:43.040452: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20be00 of size 256 next 82
2025-07-31 20:22:43.040462: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20bf00 of size 256 next 83
2025-07-31 20:22:43.040472: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20c000 of size 1024 next 84
2025-07-31 20:22:43.040482: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20c400 of size 1024 next 85
2025-07-31 20:22:43.040493: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20c800 of size 512 next 86
2025-07-31 20:22:43.040509: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20ca00 of size 512 next 87
2025-07-31 20:22:43.040519: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20cc00 of size 512 next 88
2025-07-31 20:22:43.040530: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20ce00 of size 256 next 89
2025-07-31 20:22:43.040540: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20cf00 of size 256 next 90
2025-07-31 20:22:43.040550: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20d000 of size 256 next 91
2025-07-31 20:22:43.040560: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20d100 of size 1024 next 92
2025-07-31 20:22:43.040571: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20d500 of size 1024 next 93
2025-07-31 20:22:43.040581: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20d900 of size 512 next 94
2025-07-31 20:22:43.040591: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20db00 of size 256 next 95
2025-07-31 20:22:43.040601: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20dc00 of size 256 next 96
2025-07-31 20:22:43.040611: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20dd00 of size 512 next 97
2025-07-31 20:22:43.040622: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20df00 of size 256 next 98
2025-07-31 20:22:43.040632: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20e000 of size 512 next 99
2025-07-31 20:22:43.040642: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20e200 of size 256 next 100
2025-07-31 20:22:43.040653: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20e300 of size 256 next 101
2025-07-31 20:22:43.040663: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20e400 of size 256 next 102
2025-07-31 20:22:43.040673: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20e500 of size 256 next 103
2025-07-31 20:22:43.040683: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20e600 of size 256 next 104
2025-07-31 20:22:43.040693: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20e700 of size 256 next 105
2025-07-31 20:22:43.040704: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20e800 of size 256 next 106
2025-07-31 20:22:43.040714: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20e900 of size 256 next 107
2025-07-31 20:22:43.040724: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20ea00 of size 256 next 108
2025-07-31 20:22:43.040735: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20eb00 of size 256 next 109
2025-07-31 20:22:43.040745: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20ec00 of size 256 next 110
2025-07-31 20:22:43.040755: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20ed00 of size 256 next 111
2025-07-31 20:22:43.040765: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20ee00 of size 256 next 112
2025-07-31 20:22:43.040776: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20ef00 of size 512 next 113
2025-07-31 20:22:43.040787: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20f100 of size 512 next 114
2025-07-31 20:22:43.040797: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20f300 of size 256 next 115
2025-07-31 20:22:43.040807: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20f400 of size 1024 next 116
2025-07-31 20:22:43.040818: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20f800 of size 256 next 117
2025-07-31 20:22:43.040828: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20f900 of size 512 next 118
2025-07-31 20:22:43.040839: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20fb00 of size 256 next 119
2025-07-31 20:22:43.040849: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20fc00 of size 512 next 120
2025-07-31 20:22:43.040860: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad20fe00 of size 1024 next 121
2025-07-31 20:22:43.040870: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad210200 of size 512 next 122
2025-07-31 20:22:43.040880: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad210400 of size 512 next 123
2025-07-31 20:22:43.040890: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad210600 of size 512 next 124
2025-07-31 20:22:43.040901: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad210800 of size 1024 next 125
2025-07-31 20:22:43.040911: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad210c00 of size 256 next 126
2025-07-31 20:22:43.040922: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad210d00 of size 256 next 127
2025-07-31 20:22:43.040932: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad210e00 of size 512 next 128
2025-07-31 20:22:43.040943: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad211000 of size 512 next 129
2025-07-31 20:22:43.040953: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad211200 of size 512 next 130
2025-07-31 20:22:43.040962: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad211400 of size 512 next 131
2025-07-31 20:22:43.040969: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad211600 of size 256 next 132
2025-07-31 20:22:43.040974: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad211700 of size 1024 next 133
2025-07-31 20:22:43.040980: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad211b00 of size 512 next 134
2025-07-31 20:22:43.040985: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad211d00 of size 512 next 135
2025-07-31 20:22:43.040990: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad211f00 of size 1024 next 136
2025-07-31 20:22:43.040995: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad212300 of size 512 next 137
2025-07-31 20:22:43.041000: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad212500 of size 1024 next 138
2025-07-31 20:22:43.041006: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad212900 of size 1024 next 139
2025-07-31 20:22:43.041011: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad212d00 of size 1024 next 140
2025-07-31 20:22:43.041017: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad213100 of size 2048 next 141
2025-07-31 20:22:43.041023: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad213900 of size 512 next 142
2025-07-31 20:22:43.041028: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad213b00 of size 1024 next 143
2025-07-31 20:22:43.041033: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad213f00 of size 512 next 144
2025-07-31 20:22:43.041038: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad214100 of size 512 next 145
2025-07-31 20:22:43.041044: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad214300 of size 1024 next 146
2025-07-31 20:22:43.041049: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad214700 of size 1024 next 147
2025-07-31 20:22:43.041054: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad214b00 of size 2048 next 148
2025-07-31 20:22:43.041059: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad215300 of size 4096 next 149
2025-07-31 20:22:43.041064: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad216300 of size 1024 next 150
2025-07-31 20:22:43.041069: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad216700 of size 512 next 151
2025-07-31 20:22:43.041075: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad216900 of size 512 next 152
2025-07-31 20:22:43.041080: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad216b00 of size 1024 next 153
2025-07-31 20:22:43.041085: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad216f00 of size 1024 next 154
2025-07-31 20:22:43.041090: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad217300 of size 2048 next 155
2025-07-31 20:22:43.041096: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad217b00 of size 4096 next 156
2025-07-31 20:22:43.041101: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad218b00 of size 512 next 157
2025-07-31 20:22:43.041107: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad218d00 of size 1024 next 158
2025-07-31 20:22:43.041112: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad219100 of size 512 next 159
2025-07-31 20:22:43.041117: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad219300 of size 512 next 160
2025-07-31 20:22:43.041122: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad219500 of size 1024 next 161
2025-07-31 20:22:43.041127: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad219900 of size 2048 next 162
2025-07-31 20:22:43.041132: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad21a100 of size 4096 next 163
2025-07-31 20:22:43.041138: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad21b100 of size 512 next 164
2025-07-31 20:22:43.041143: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad21b300 of size 512 next 165
2025-07-31 20:22:43.041149: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad21b500 of size 256 next 166
2025-07-31 20:22:43.041154: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad21b600 of size 256 next 167
2025-07-31 20:22:43.041159: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad21b700 of size 256 next 168
2025-07-31 20:22:43.041164: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad21b800 of size 256 next 169
2025-07-31 20:22:43.041169: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad21b900 of size 256 next 170
2025-07-31 20:22:43.041174: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad21ba00 of size 256 next 171
2025-07-31 20:22:43.041180: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad21bb00 of size 131072 next 172
2025-07-31 20:22:43.041185: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad23bb00 of size 256 next 173
2025-07-31 20:22:43.041190: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad23bc00 of size 256 next 174
2025-07-31 20:22:43.041195: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad23bd00 of size 256 next 175
2025-07-31 20:22:43.041201: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad23be00 of size 256 next 176
2025-07-31 20:22:43.041206: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad23bf00 of size 165376 next 177
2025-07-31 20:22:43.041212: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad264500 of size 131072 next 178
2025-07-31 20:22:43.041217: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc2ad284500 of size 8704 next 180
2025-07-31 20:22:43.041238: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad286700 of size 229376 next 181
2025-07-31 20:22:43.041251: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad2be700 of size 256 next 182
2025-07-31 20:22:43.041262: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc2ad2be800 of size 768 next 183
2025-07-31 20:22:43.041273: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad2beb00 of size 512 next 184
2025-07-31 20:22:43.041284: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc2ad2bed00 of size 256 next 185
2025-07-31 20:22:43.041294: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad2bee00 of size 512 next 186
2025-07-31 20:22:43.041305: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad2bf000 of size 512 next 187
2025-07-31 20:22:43.041316: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad2bf200 of size 512 next 188
2025-07-31 20:22:43.041327: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc2ad2bf400 of size 24576 next 189
2025-07-31 20:22:43.041370: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad2c5400 of size 65536 next 190
2025-07-31 20:22:43.041381: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad2d5400 of size 512 next 191
2025-07-31 20:22:43.041392: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad2d5600 of size 65536 next 192
2025-07-31 20:22:43.041403: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc2ad2e5600 of size 1280 next 194
2025-07-31 20:22:43.041413: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad2e5b00 of size 512 next 195
2025-07-31 20:22:43.041424: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc2ad2e5d00 of size 512 next 196
2025-07-31 20:22:43.041435: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad2e5f00 of size 512 next 197
2025-07-31 20:22:43.041446: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad2e6100 of size 106240 next 18446744073709551615
2025-07-31 20:22:43.041458: I tensorflow/core/common_runtime/bfc_allocator.cc:970] Next region of size 2097152
2025-07-31 20:22:43.041470: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad800000 of size 512 next 199
2025-07-31 20:22:43.041481: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc2ad800200 of size 16640 next 201
2025-07-31 20:22:43.041492: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad804300 of size 65536 next 202
2025-07-31 20:22:43.041507: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad814300 of size 512 next 203
2025-07-31 20:22:43.041519: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad814500 of size 131072 next 204
2025-07-31 20:22:43.041529: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad834500 of size 1024 next 205
2025-07-31 20:22:43.041540: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc2ad834900 of size 49920 next 209
2025-07-31 20:22:43.041551: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad840c00 of size 1536 next 210
2025-07-31 20:22:43.041595: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad841200 of size 1024 next 211
2025-07-31 20:22:43.041609: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ad841600 of size 1829376 next 18446744073709551615
2025-07-31 20:22:43.041620: I tensorflow/core/common_runtime/bfc_allocator.cc:970] Next region of size 4194304
2025-07-31 20:22:43.041631: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc2ada00000 of size 768 next 214
2025-07-31 20:22:43.041642: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ada00300 of size 165376 next 215
2025-07-31 20:22:43.041653: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ada28900 of size 1024 next 216
2025-07-31 20:22:43.041664: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc2ada28d00 of size 32768 next 217
2025-07-31 20:22:43.041675: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ada30d00 of size 65536 next 218
2025-07-31 20:22:43.041685: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2ada40d00 of size 524288 next 219
2025-07-31 20:22:43.041696: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2adac0d00 of size 256 next 220
2025-07-31 20:22:43.041707: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2adac0e00 of size 65536 next 221
2025-07-31 20:22:43.041718: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2adad0e00 of size 131072 next 222
2025-07-31 20:22:43.041729: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2adaf0e00 of size 82688 next 223
2025-07-31 20:22:43.041740: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2adb05100 of size 512 next 224
2025-07-31 20:22:43.041750: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2adb05300 of size 65536 next 225
2025-07-31 20:22:43.041761: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2adb15300 of size 1024 next 226
2025-07-31 20:22:43.041771: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2adb15700 of size 256 next 227
2025-07-31 20:22:43.041782: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2adb15800 of size 329216 next 228
2025-07-31 20:22:43.041797: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2adb65e00 of size 2728448 next 18446744073709551615
2025-07-31 20:22:43.041804: I tensorflow/core/common_runtime/bfc_allocator.cc:970] Next region of size 8388608
2025-07-31 20:22:43.041811: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af000000 of size 512 next 230
2025-07-31 20:22:43.041817: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af000200 of size 16384 next 231
2025-07-31 20:22:43.041823: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af004200 of size 65536 next 232
2025-07-31 20:22:43.041830: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af014200 of size 2097152 next 233
2025-07-31 20:22:43.041836: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af214200 of size 1024 next 234
2025-07-31 20:22:43.041841: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af214600 of size 512 next 235
2025-07-31 20:22:43.041847: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af214800 of size 256 next 236
2025-07-31 20:22:43.041853: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af214900 of size 512 next 237
2025-07-31 20:22:43.042039: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af214b00 of size 512 next 238
2025-07-31 20:22:43.042139: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af214d00 of size 4096 next 239
2025-07-31 20:22:43.042205: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af215d00 of size 512 next 240
2025-07-31 20:22:43.042370: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af215f00 of size 65536 next 241
2025-07-31 20:22:43.042450: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af225f00 of size 512 next 242
2025-07-31 20:22:43.042544: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af226100 of size 131072 next 243
2025-07-31 20:22:43.042602: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc2af246100 of size 4096 next 244
2025-07-31 20:22:43.042610: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af247100 of size 256 next 245
2025-07-31 20:22:43.042617: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af247200 of size 32768 next 246
2025-07-31 20:22:43.042622: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af24f200 of size 2048 next 247
2025-07-31 20:22:43.042627: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af24fa00 of size 32768 next 248
2025-07-31 20:22:43.042633: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af257a00 of size 329216 next 249
2025-07-31 20:22:43.042638: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af2a8000 of size 262144 next 250
2025-07-31 20:22:43.042644: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af2e8000 of size 256 next 251
2025-07-31 20:22:43.042649: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af2e8100 of size 1536 next 252
2025-07-31 20:22:43.042655: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af2e8700 of size 164608 next 253
2025-07-31 20:22:43.042660: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af310a00 of size 512 next 254
2025-07-31 20:22:43.042666: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af310c00 of size 1024 next 255
2025-07-31 20:22:43.042671: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af311000 of size 512 next 256
2025-07-31 20:22:43.042676: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af311200 of size 256 next 257
2025-07-31 20:22:43.042682: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af311300 of size 16384 next 258
2025-07-31 20:22:43.042687: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af315300 of size 5120 next 259
2025-07-31 20:22:43.042694: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af316700 of size 658432 next 260
2025-07-31 20:22:43.042699: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af3b7300 of size 256 next 261
2025-07-31 20:22:43.042704: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af3b7400 of size 512 next 262
2025-07-31 20:22:43.042709: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af3b7600 of size 1024 next 263
2025-07-31 20:22:43.042715: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af3b7a00 of size 65536 next 264
2025-07-31 20:22:43.042720: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc2af3c7a00 of size 262144 next 294
2025-07-31 20:22:43.042726: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af407a00 of size 697856 next 265
2025-07-31 20:22:43.042731: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af4b2000 of size 512 next 266
2025-07-31 20:22:43.042736: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af4b2200 of size 512 next 267
2025-07-31 20:22:43.042741: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af4b2400 of size 512 next 268
2025-07-31 20:22:43.042747: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af4b2600 of size 131072 next 269
2025-07-31 20:22:43.042752: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af4d2600 of size 1024 next 270
2025-07-31 20:22:43.042757: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af4d2a00 of size 512 next 271
2025-07-31 20:22:43.042762: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af4d2c00 of size 512 next 286
2025-07-31 20:22:43.042768: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af4d2e00 of size 512 next 287
2025-07-31 20:22:43.042773: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af4d3000 of size 512 next 288
2025-07-31 20:22:43.042778: I tensorflow/core/common_runtime/bfc_allocator.cc:990] InUse at 7bc2af4d3200 of size 512 next 279
2025-07-31 20:22:43.042783: I tensorflow/core/common_runtime/bfc_allocator.cc:990] Free  at 7bc2af4d3400 of size 3329024 next 18446744073709551615
2025-07-31 20:22:43.042789: I tensorflow/core/common_runtime/bfc_allocator.cc:995]      Summary of in-use Chunks by size: 
2025-07-31 20:22:43.042804: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 58 Chunks of size 256 totalling 14.5KiB
2025-07-31 20:22:43.042811: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 111 Chunks of size 512 totalling 55.5KiB
2025-07-31 20:22:43.042818: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 40 Chunks of size 1024 totalling 40.0KiB
2025-07-31 20:22:43.042824: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 1280 totalling 1.2KiB
2025-07-31 20:22:43.042830: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 2 Chunks of size 1536 totalling 3.0KiB
2025-07-31 20:22:43.042836: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 5 Chunks of size 2048 totalling 10.0KiB
2025-07-31 20:22:43.042842: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 5 Chunks of size 4096 totalling 20.0KiB
2025-07-31 20:22:43.042849: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 5120 totalling 5.0KiB
2025-07-31 20:22:43.042855: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 2 Chunks of size 16384 totalling 32.0KiB
2025-07-31 20:22:43.042861: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 2 Chunks of size 32768 totalling 64.0KiB
2025-07-31 20:22:43.042868: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 9 Chunks of size 65536 totalling 576.0KiB
2025-07-31 20:22:43.042874: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 82688 totalling 80.8KiB
2025-07-31 20:22:43.042881: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 106240 totalling 103.8KiB
2025-07-31 20:22:43.042887: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 6 Chunks of size 131072 totalling 768.0KiB
2025-07-31 20:22:43.042893: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 164608 totalling 160.8KiB
2025-07-31 20:22:43.042899: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 2 Chunks of size 165376 totalling 323.0KiB
2025-07-31 20:22:43.042906: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 229376 totalling 224.0KiB
2025-07-31 20:22:43.042912: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 262144 totalling 256.0KiB
2025-07-31 20:22:43.042918: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 2 Chunks of size 329216 totalling 643.0KiB
2025-07-31 20:22:43.042924: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 393216 totalling 384.0KiB
2025-07-31 20:22:43.042931: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 524288 totalling 512.0KiB
2025-07-31 20:22:43.042937: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 658432 totalling 643.0KiB
2025-07-31 20:22:43.042943: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 697856 totalling 681.5KiB
2025-07-31 20:22:43.042949: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 786432 totalling 768.0KiB
2025-07-31 20:22:43.042955: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 2 Chunks of size 1572864 totalling 3.00MiB
2025-07-31 20:22:43.042962: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 1829376 totalling 1.74MiB
2025-07-31 20:22:43.042968: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 2097152 totalling 2.00MiB
2025-07-31 20:22:43.042974: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 2728448 totalling 2.60MiB
2025-07-31 20:22:43.042980: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 3145728 totalling 3.00MiB
2025-07-31 20:22:43.042986: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 67108864 totalling 64.00MiB
2025-07-31 20:22:43.042992: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 268435456 totalling 256.00MiB
2025-07-31 20:22:43.042998: I tensorflow/core/common_runtime/bfc_allocator.cc:998] 1 Chunks of size 562273024 totalling 536.22MiB
2025-07-31 20:22:43.043004: I tensorflow/core/common_runtime/bfc_allocator.cc:1002] Sum Total of in-use chunks: 874.79MiB
2025-07-31 20:22:43.043010: I tensorflow/core/common_runtime/bfc_allocator.cc:1004] total_region_allocated_bytes_: 1148427008 memory_limit_: 1843986432 available bytes: 695559424 curr_region_allocation_bytes_: 2147483648
2025-07-31 20:22:43.043019: I tensorflow/core/common_runtime/bfc_allocator.cc:1010] Stats: 
Limit:                  1843986432
InUse:                   917285632
MaxInUse:               1420726272
NumAllocs:                     455
MaxAllocSize:            562273024

2025-07-31 20:22:43.043037: W tensorflow/core/common_runtime/bfc_allocator.cc:439] ***********************************************x*******************************________________*****
2025-07-31 20:22:43.043072: W tensorflow/core/framework/op_kernel.cc:1753] OP_REQUIRES failed at transpose_op.cc:198 : Resource exhausted: OOM when allocating tensor with shape[4,2048,128,128] and type float on /job:localhost/replica:0/task:0/device:GPU:0 by allocator GPU_0_bfc
[ERROR] [1753993363.049300, 147.613000]: bad callback: <bound method Subscriber.callback of <message_filters.Subscriber object at 0x7bc3224848d0>>
Traceback (most recent call last):
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1365, in _do_call
    return fn(*args)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1350, in _run_fn
    target_list, run_metadata)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1443, in _call_tf_sessionrun
    run_metadata)
tensorflow.python.framework.errors_impl.ResourceExhaustedError: 2 root error(s) found.
  (0) Resource exhausted: OOM when allocating tensor with shape[4,2048,128,128] and type float on /job:localhost/replica:0/task:0/device:GPU:0 by allocator GPU_0_bfc
	 [[{{node layer1/conv2_2/bn/cond/output/_242-0-0-TransposeNCHWToNHWC-LayoutOptimizer}}]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

	 [[Sigmoid/_553]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

  (1) Resource exhausted: OOM when allocating tensor with shape[4,2048,128,128] and type float on /job:localhost/replica:0/task:0/device:GPU:0 by allocator GPU_0_bfc
	 [[{{node layer1/conv2_2/bn/cond/output/_242-0-0-TransposeNCHWToNHWC-LayoutOptimizer}}]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

0 successful operations.
0 derived errors ignored.

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/opt/ros/noetic/lib/python3/dist-packages/message_filters/__init__.py", line 76, in callback
    self.signalMessage(msg)
  File "/opt/ros/noetic/lib/python3/dist-packages/message_filters/__init__.py", line 58, in signalMessage
    cb(*(msg + args))
  File "/opt/ros/noetic/lib/python3/dist-packages/message_filters/__init__.py", line 330, in add
    self.signalMessage(*msgs)
  File "/opt/ros/noetic/lib/python3/dist-packages/message_filters/__init__.py", line 58, in signalMessage
    cb(*(msg + args))
  File "./ros/test_model_ros.py", line 220, in callback_points
    self.run_network(viz=False)
  File "./ros/test_model_ros.py", line 288, in run_network
    forward_passes=self.forward_passes,
  File "/root/compare_SceneReplica/src/contact_graspnet/ros/../contact_graspnet/contact_grasp_estimator.py", line 254, in predict_scene_grasps
    pred_grasps_cam[k], scores[k], contact_pts[k], gripper_openings[k] = self.predict_grasps(sess, pc_region, convert_cam_coords=True, forward_passes=forward_passes)
  File "/root/compare_SceneReplica/src/contact_graspnet/ros/../contact_graspnet/contact_grasp_estimator.py", line 192, in predict_grasps
    pred_grasps_cam, pred_scores, pred_points, offset_pred = sess.run(self.inference_ops, feed_dict=feed_dict)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 958, in run
    run_metadata_ptr)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1181, in _run
    feed_dict_tensor, options, run_metadata)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1359, in _do_run
    run_metadata)
  File "/root/miniconda3/envs/contact_graspnet_env/lib/python3.7/site-packages/tensorflow/python/client/session.py", line 1384, in _do_call
    raise type(e)(node_def, op, message)
tensorflow.python.framework.errors_impl.ResourceExhaustedError: 2 root error(s) found.
  (0) Resource exhausted: OOM when allocating tensor with shape[4,2048,128,128] and type float on /job:localhost/replica:0/task:0/device:GPU:0 by allocator GPU_0_bfc
	 [[{{node layer1/conv2_2/bn/cond/output/_242-0-0-TransposeNCHWToNHWC-LayoutOptimizer}}]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

	 [[Sigmoid/_553]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

  (1) Resource exhausted: OOM when allocating tensor with shape[4,2048,128,128] and type float on /job:localhost/replica:0/task:0/device:GPU:0 by allocator GPU_0_bfc
	 [[{{node layer1/conv2_2/bn/cond/output/_242-0-0-TransposeNCHWToNHWC-LayoutOptimizer}}]]
Hint: If you want to see a list of allocated tensors when OOM happens, add report_tensor_allocations_upon_oom to RunOptions for current allocation info.

0 successful operations.
0 derived errors ignored.

^C^C[INFO] Exiting Contact GraspNet generation ROS Node

