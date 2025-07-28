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

def get_pose(object_name: str, pose_method: str):
    """
    Calls the suitable function depending on pose method
    
    Input:
    - object_name (str) : name of the object mode, e.g '003_cracker_box'
    - pose_method (str) : name of the model based pose method, e.g. 'poserbpf'
        - options : {'gazebo', 'posecnn', 'poserbpf'}
    
    Returns:
    - RT_object (4,4 np.ndarray) : 4x4 transform of the object in robot's base_link frame
    """

if pose_method == "gazebo":
        return get_pose_gazebo(object_name)

def get_pose_gazebo(model_name, relative_entity_name=""):
    import roslib

    roslib.load_manifest("gazebo_msgs")
    from gazebo_msgs.srv import GetModelState

    def gms_client(model_name, relative_entity_name):
        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            resp1 = gms(model_name, relative_entity_name)
            return resp1
        except (rospy.ServiceException, e):
            print("Service call failed: %s" % e)

    res = gms_client(model_name, relative_entity_name)
    RT_obj = ros_pose_to_rt(res.pose)

    # query fetch base line
    res = gms_client(model_name="fetch", relative_entity_name="base_link")
    RT_base = ros_pose_to_rt(res.pose)

    # object pose in robot base
    RT = np.matmul(np.linalg.inv(RT_base), RT_obj)
    return RT

def get_pose_posecnn(object_name: str):
    """
    Queries the PoseCNN topic for the given YCB `object_name` and returns
    a 4x4 transform for its pose
    """
    # Example: "posecnn/00_cracker_box_01_refined"
    # NOTE: The object name appears without the ycbid, "cracker_box" instead of "003_cracker_box"
    posecnn_topic_name = f"posecnn/00_{object_name[4:]}_01_refined"
    RT_obj = get_tf_pose(posecnn_topic_name, 'base_link')
    return RT_obj
    
    
def setup_moveit_scene(
    scene,
    table_height,
    object_names,
    models_path,
    pose_method,
    gt_metadata,
    gt_file_path,
    table_position=(0.8, 0, 0)
):
    for obj_name in object_names:
        RT_obj = get_pose(obj_name, pose_method)
        gt_metadata["estimated_poses"][obj_name] = RT_obj
        
        
        
 def sort_and_filter_grasps(RT_obj, RT_gripper, RT_grasps, table_height: float):
    # translate all RT graspits grasps using the object mean
    # transform grasps to robot base
    n = RT_grasps.shape[0]
    # RT_grasps_base = np.zeros_like(RT_grasps)
    distances = np.zeros((n,), dtype=np.float32)
    RT_grasps_base = []
    distances = []
    for i in range(n):
        RT_g = RT_grasps[i]
        # transform grasp to robot base
        RT = RT_obj @ RT_g
        trans = RT[:3, 3]
        if trans[-1] > (table_height + 0.02):  # 2cm offset above table surface
            RT_grasps_base.append(RT)
            d = np.linalg.norm(RT_gripper[:3, 3] - RT[:3, 3])
            distances.append(d)
    final_grasp_len = len(RT_grasps_base)
    pruned_ratio = (n - final_grasp_len) / n
    print(f"Filter ratio: {pruned_ratio}")
    if pruned_ratio == 1.0:
        print(f"returning all none")
        return None, None, None
    RT_grasps_base = np.asarray(RT_grasps_base)
    distances = np.asarray(distances, dtype=np.float32)
    index = np.argsort(distances)
    RT_grasps_base = RT_grasps_base[index]
    return RT_grasps_base, index, pruned_ratio


 if __name__ == "__main__":
    for obj_i, object_to_grasp in enumerate(object_order):
        grasp_num, trajectory_standoff, trajectory_final = None, None, None
        direct_topdown = False
        if not direct_topdown:
	    RT_gripper = get_gripper_rt(tf_buffer)
	    print("RT_gripper", RT_gripper)
	    # Using Graspit generated grasp to test the Pose Detection (model based grasping)
	    grasp_file = os.path.join(grasp_dir, f"fetch_gripper-{object_to_grasp}.json")
	    RT_grasps = parse_grasps(grasp_file)
	    RT_grasps_base, grasp_index, pruned_ratio = sort_and_filter_grasps(
		RT_obj, RT_gripper, RT_grasps, table_height
	    )

	    if (grasp_index is None ):
		direct_topdown=True
	    else:    
		successful_grasps = set(success_grasp_info[scene_idx][object_to_grasp])
		# grasp planning
		RT_grasp, grasp_num, trajectory_standoff, trajectory_final = plan_grasp(
		    group,
		    scene,
		    display_trajectory_publisher,
		    RT_grasps_base,
		    grasp_index,
		    object_to_grasp,
		    RT_obj,
		    model_dir,
		    successful_grasps,
		)
    if direct_topdown or (grasp_num == -1 or (not trajectory_standoff) or (not trajectory_final)):
            print("No plans found for direct grasping, trying TOP-DOWN!")
            logger.warning("No plans found for direct grasping, trying TOP-DOWN!")
            mesh_p = os.path.join(model_dir, object_to_grasp, "textured_simple.obj")
            obj_pts = get_object_verts(mesh_p, pose=RT_obj)
            RT_grasp, g_width = model_based_top_down_grasp(obj_pts)
            print(f"Gripper Width: {g_width}")
            if object_to_grasp in {"052_extra_large_clamp", "025_mug"}:
                g_width = -1
            if g_width < (0.1 - 0.002):
                grasp_with_rt(
                    gripper,
                    group,
                    scene,
                    object_to_grasp,
                    display_trajectory_publisher,
                    RT_grasp,
                )
                logger.inform("TOP DOWN SUCCESSFUL")
            else:
                print("TOP DOWN FAILED!! Object too wide.")
                logger.error("TOP DOWN FAILED!! Object too wide.")
        elif trajectory_standoff and trajectory_final and grasp_num != -1:
            grasp_with_rt(
                gripper,
                group,
                scene,
                object_to_grasp,
                display_trajectory_publisher,
                RT_grasp,
            )
        logger.inform(f"{object_to_grasp} reached successfully")

