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

Attempts to resolve cuda out of memory using smaller dtype
----

Add to .sh: export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True [from chatgpt; yet Gram suggested setting to False]
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
...
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




-------

Added cuda default dtype setup from Downltest_cuda_torch_sam to: MSMFormer/meanshiftformer/modeling/pixel_decoder/msdeformattn.py:

import os
import gc
import torchvision

os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "max_split_size_mb:256"
os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:False"
os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "roundup_power2_divisions:256" # 17.17 vs 24.00 vs 20.00 vs 18.00

print("PyTorch version:", torch.__version__)
print("Torchvision version:", torchvision.__version__)
print("CUDA is available:", torch.cuda.is_available())

torch.cuda.memory._record_memory_history()
gc.collect() # collect garbage
torch.cuda.empty_cache()

FLOAT_PRECISION = "float16"  # bfloat16 is recommended for training and inference on GPUs with Ampere architecture or later

if torch.cuda.is_available():
    device = torch.device("cuda")
else:
    device = torch.device("cpu")
print("Using device:", device)
if device.type == "cuda":
    # colab has everything on bfloat16
    if FLOAT_PRECISION == "bfloat16":
        torch.set_default_dtype(torch.bfloat16)
        print("Using bfloat16 precision")
    elif FLOAT_PRECISION == "float32":
        torch.set_default_dtype(torch.float32)
        print("Using float32 precision")
    elif FLOAT_PRECISION == "float16":
        torch.set_default_dtype(torch.float16)
        print("Using float16 precision")
    else:
        raise ValueError("Unsupported FLOAT_PRECISION: {}".format(FLOAT_PRECISION))

    # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
    if torch.cuda.get_device_properties(0).major >= 8:
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
elif device.type == "cpu":
    # turn on bfloat16 for CPU
    torch.set_float32_matmul_precision("high")
    torch.set_default_dtype(torch.bfloat16)
else:
    raise RuntimeError("Unsupported device type: {}".format(device.type))

-----

Output:
Traceback (most recent call last):
  File "./ros/test_images_segmentation_transformer.py", line 343, in <module>
    listener.run_network()
  File "./ros/test_images_segmentation_transformer.py", line 180, in run_network
    out_label, out_label_refined, out_score, bbox = test_sample_crop_nolabel(self.cfg_transformer, sample, self.predictor, self.predictor_crop, visualization=False, topk=False, confident_score=0.2, print_result=True)
  File "/root/compare_SceneReplica/src/UnseenObjectsWithMeanShift/ros/../lib/fcn/test_utils.py", line 356, in test_sample_crop_nolabel
    outputs = predictor(sample)
  File "/root/compare_SceneReplica/src/UnseenObjectsWithMeanShift/ros/../lib/fcn/test_utils.py", line 165, in __call__
    predictions = self.model([sample])[0]
  File "/usr/local/lib/python3.8/dist-packages/torch/nn/modules/module.py", line 1553, in _wrapped_call_impl
    return self._call_impl(*args, **kwargs)
  File "/usr/local/lib/python3.8/dist-packages/torch/nn/modules/module.py", line 1562, in _call_impl
    return forward_call(*args, **kwargs)
  File "/root/compare_SceneReplica/src/UnseenObjectsWithMeanShift/ros/../lib/fcn/../../MSMFormer/meanshiftformer/pretrained_meanshiftformer_model.py", line 291, in forward
    features = self.pretrained_backbone(images.tensor, None, images_depth.tensor) #.detach() # if use detach(), backbone is not trained
  File "/usr/local/lib/python3.8/dist-packages/torch/nn/modules/module.py", line 1553, in _wrapped_call_impl
    return self._call_impl(*args, **kwargs)
  File "/usr/local/lib/python3.8/dist-packages/torch/nn/modules/module.py", line 1562, in _call_impl
    return forward_call(*args, **kwargs)
  File "/root/compare_SceneReplica/src/UnseenObjectsWithMeanShift/ros/../lib/networks/SEG.py", line 105, in forward
    features = self.fcn(img)
  File "/usr/local/lib/python3.8/dist-packages/torch/nn/modules/module.py", line 1553, in _wrapped_call_impl
    return self._call_impl(*args, **kwargs)
  File "/usr/local/lib/python3.8/dist-packages/torch/nn/modules/module.py", line 1562, in _call_impl
    return forward_call(*args, **kwargs)
  File "/root/compare_SceneReplica/src/UnseenObjectsWithMeanShift/ros/../lib/networks/resnet_dilated.py", line 323, in forward
    x = self.resnet34_8s(x)
  File "/usr/local/lib/python3.8/dist-packages/torch/nn/modules/module.py", line 1553, in _wrapped_call_impl
    return self._call_impl(*args, **kwargs)
  File "/usr/local/lib/python3.8/dist-packages/torch/nn/modules/module.py", line 1562, in _call_impl
    return forward_call(*args, **kwargs)
  File "/root/compare_SceneReplica/src/UnseenObjectsWithMeanShift/ros/../lib/networks/resnet.py", line 237, in forward
    x = self.conv1(x)
  File "/usr/local/lib/python3.8/dist-packages/torch/nn/modules/module.py", line 1553, in _wrapped_call_impl
    return self._call_impl(*args, **kwargs)
  File "/usr/local/lib/python3.8/dist-packages/torch/nn/modules/module.py", line 1562, in _call_impl
    return forward_call(*args, **kwargs)
  File "/usr/local/lib/python3.8/dist-packages/torch/nn/modules/conv.py", line 458, in forward
    return self._conv_forward(input, self.weight, self.bias)
  File "/usr/local/lib/python3.8/dist-packages/torch/nn/modules/conv.py", line 454, in _conv_forward
    return F.conv2d(input, weight, bias, self.stride,
RuntimeError: Input type (torch.cuda.FloatTensor) and weight type (torch.cuda.HalfTensor) should be the same

-------

update: 
- also added torch dtype constrains of:
	(~/compare_SceneReplica/src/UnseenObjectsWithMeanShift/)ros/test_images_segmentation_transformer.py [main]
	(~/compare_SceneReplica/src/UnseenObjectsWithMeanShift/)MSMFormer/meanshiftformer/pretrained_meanshiftformer_model.py
	(~/compare_SceneReplica/src/UnseenObjectsWithMeanShift/)lib/fcn/test_utils.py
	(~/compare_SceneReplica/src/UnseenObjectsWithMeanShift/)lib/networks/resnet_dilated.py
	
- found following comments inside /usr/local/lib/python3.8/dist-packages/torch/nn/modules/conv.py:
    This module supports :ref:`TensorFloat32<tf32_on_ampere>`.
    On certain ROCm devices, when using float16 inputs this module will use :ref:`different precision<fp16_on_mi200>` for backward.

Reverted after error of persistently mismatched input and weight size

----

Regarding:
usr/local/lib/python3.8/dist-packages/fvcore/common/checkpoint.py:252: FutureWarning: You are using `torch.load` with `weights_only=False` (the current default value), which uses the default pickle module implicitly. It is possible to construct malicious pickle data which will execute arbitrary code during unpickling (See https://github.com/pytorch/pytorch/blob/main/SECURITY.md#untrusted-models for more details). In a future release, the default value for `weights_only` will be flipped to `True`. This limits the functions that could be executed during unpickling. Arbitrary objects will no longer be allowed to be loaded via this mode unless they are explicitly allowlisted by the user via `torch.serialization.add_safe_globals`. We recommend you start setting `weights_only=True` for any use case where you don't have full control of the loaded file. Please open an issue on GitHub for any issues related to this experimental feature.
  return torch.load(f, map_location=torch.device("cpu"))
/root/compare_SceneReplica/src/UnseenObjectsWithMeanShift/ros/../lib/fcn/../../lib/fcn/get_network_crop.py:81: FutureWarning: You are using `torch.load` with `weights_only=False` (the current default value), which uses the default pickle module implicitly. It is possible to construct malicious pickle data which will execute arbitrary code during unpickling (See https://github.com/pytorch/pytorch/blob/main/SECURITY.md#untrusted-models for more details). In a future release, the default value for `weights_only` will be flipped to `True`. This limits the functions that could be executed during unpickling. Arbitrary objects will no longer be allowed to be loaded via this mode unless they are explicitly allowlisted by the user via `torch.serialization.add_safe_globals`. We recommend you start setting `weights_only=True` for any use case where you don't have full control of the loaded file. Please open an issue on GitHub for any issues related to this experimental feature.
  network_data_crop = torch.load(pretrained_crop)

Added 'torch.load` with `weights_only=False` for: 
/root/compare_SceneReplica/src/UnseenObjectsWithMeanShift/ros/../lib/fcn/../../lib/fcn/get_network_crop.py
/usr/local/lib/python3.8/dist-packages/fvcore/common/checkpoint.py


-----

with torch.autocast(device_type=device.type, dtype=torch.float16):
   with torch.no_grad():
   
- before adding to ./ros/test_images_segmentation_transformer.py:
Tried to allocate 938.00 MiB. GPU 0 has a total capacity of 5.60 GiB of which 549.19 MiB is free.
- after:
Tried to allocate 938.00 MiB. GPU 0 has a total capacity of 5.60 GiB of which 683.38 MiB is free.

-----

running poserbpf under posecnn-pytorch, when only posecnn is taking effect:

024_bowl
005_tomato_soup_can
035_power_drill
011_banana
004_sugar_box
[004_sugar_box] points : 10902
[004_sugar_box] detection score: 0.799198
[004_sugar_box] location mean: 0.368687, 0.267487, 0.810683
[004_sugar_box] location mean on table: 0.386015, 0.288044, 0.055452
-------------------------------------------
[035_power_drill] points : 11032
[035_power_drill] detection score: 0.995783
[035_power_drill] location mean: 0.480357, 0.089962, 0.783328
[035_power_drill] location mean on table: 0.518232, 0.098236, 0.082026
-------------------------------------------
[005_tomato_soup_can] points : 3331
[005_tomato_soup_can] detection score: 0.998381
[005_tomato_soup_can] location mean: 0.699173, -0.007104, 0.791082
[005_tomato_soup_can] location mean on table: 0.727574, -0.008531, 0.042656
-------------------------------------------
[011_banana] points : 2200
[011_banana] detection score: 0.980949
[011_banana] location mean: 0.658497, -0.098329, 0.769730
[011_banana] location mean on table: 0.698406, -0.107803, 0.064698
-------------------------------------------
[024_bowl] points : 12379
[024_bowl] detection score: 0.999525
[024_bowl] location mean: 0.493304, -0.178501, 0.763270
[024_bowl] location mean on table: 0.522025, -0.195610, 0.063862
-------------------------------------------


